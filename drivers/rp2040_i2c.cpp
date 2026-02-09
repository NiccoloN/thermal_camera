#include "rp2040_i2c.h"
#include "applicationui.h"
#include "kernel/lock.h"
#include "kernel/logging.h"
#include "CMSIS/Device/RaspberryPi/RP2040/Include/RP2040.h"
#include "board_settings.h"
#include "drivers/rp2040_dma.h"
#include "rp2040/hardware/regs/dma.h"
#include "rp2040/hardware/regs/i2c.h"
#include "rp2040/hardware/structs/resets.h"
#include <cstdint>
#include <interfaces/interrupts.h>
#include "config/miosix_settings.h"
#include "interfaces/arch_registers.h"
#include "interfaces/gpio.h"
#include "kernel/thread.h"
#include "kernel/lock.h"
#include <main.h>

using namespace miosix;

namespace miosix {

// I2C driver implementation for rp2040 (master only)
RP2040I2C1Master::RP2040I2C1Master(GpioPin sda, GpioPin scl, int frequency)
{
    iprintf("Initializing I2C...\n");
    volatile uint32_t var;
    {
        GlobalIrqLock lock;
        IRQn_Type irqn;

        irqn = I2C1_IRQ_IRQn;
        clocks_hw->wake_en0|=CLOCKS_WAKE_EN0_CLK_SYS_I2C1_BITS;
        clocks_hw->sleep_en0|=CLOCKS_SLEEP_EN0_CLK_SYS_I2C1_BITS;
        unreset_block_wait(RESETS_RESET_I2C1_BITS);

        IRQregisterIrq(lock,irqn,&RP2040I2C1Master::IRQhandleInterrupt,this);
        txDmaCh=RP2040Dma::IRQregisterChannel(lock,&RP2040I2C1Master::IRQhandleDmaInterrupt,this);
        rxDmaCh=RP2040Dma::IRQregisterChannel(lock,&RP2040I2C1Master::IRQhandleDmaInterrupt,this);

        i2c = i2c1_hw;
        
        //Disable the peripheral in order to write the control register IC_CON
        i2c->enable = I2C_IC_ENABLE_RESET;

        /*
         * Bit 11 ("SPECIAL"): 0 => normal operation (NO START BYTE NOR GENERAL CALL;
         * Bit 10 ("GC_OR_START"): 0 BUT it is ignored because SPECIAL is set to 1;
         * Bit 9:0: 0..0 => where the target address will be placed;
         */
        i2c->tar = I2C_IC_TAR_RESET;

        /*
         * Number of elements at or above which the {R,T}X_FULL interrupt is fired.
         * Since the I2C controller can hold at most 16 entries, we'll set it to 15.
         */
        i2c->rx_tl = 15;
        i2c->tx_tl = 15;

        var = i2c->clr_intr;
        
        /*
         * On reset:
         *  - slave issues STOP_DET intr always;
         *  - slave mode is disabled;
         *  - master restart is enabled;
         *  - master 7 bit addressing mode;
         *  - slave 7 bit addressing mode;
         *  - fast (<= 1000Kbit/s) speed;
         *  - master mode enabled.
         */
        i2c->con = I2C_IC_CON_RESET;
        i2c->intr_mask = 0; //NOTE: TO CHECK

        i2c->dma_cr = I2C_IC_DMA_CR_TDMAE_BITS | I2C_IC_DMA_CR_RDMAE_BITS;

        sda.function(Function::I2C1); sda.fast(); //sda.mode(Mode::INPUT_SCHMITT_TRIG_PULL_UP);   //sda.fast(); 
        scl.function(Function::I2C1_SCL); scl.fast(); //scl.mode(Mode::INPUT_SCHMITT_TRIG_PULL_UP); //scl.fast();

        setBitrate(frequency);

        //Bus clear feature? (4.3.13.1)


        i2c->enable |= 0x00000001;

    }
    iprintf("I2C initialized!\n");
    iprintf(">>> %d\n", var);
    
}

// TODO: check speed calculation
// See reference @ 4.3.14.2
// To be called ONLY when the device is disabled
void RP2040I2C1Master::setBitrate(int frequency){
    unsigned long oscfreq = cpuFrequency; //ic_clk frequency (Hz)
    unsigned long MIN_SCL_HIGHtime; //minimum high period, ns
    unsigned long MIN_SCL_LOWtime; //minimum low period, ns
                                   //
    //TODO: variable spike suppression, verify
    //minimum 50ns spike suppression for SS and FS[+] modes
    unsigned long spike_suppression_period = 50;

    if(frequency <= 100){
        i2c->con ^= 3<<1; // Set bits 2:1 to "01" for SS mode in one operation, from default "10"
        MIN_SCL_HIGHtime = 4000;
        MIN_SCL_LOWtime = 4700;
    }else if(frequency <= 400){
        MIN_SCL_HIGHtime = 600;
        MIN_SCL_LOWtime = 1300;
    }else{
        MIN_SCL_HIGHtime = 260;
        MIN_SCL_LOWtime = 500;
    }

    unsigned char fs_spklen = ((unsigned long)spike_suppression_period * oscfreq)/1000000000;

    i2c->fs_spklen = fs_spklen;

    //TODO: round-up (ceiling) in an elegant and efficient way
    // Counters for fastest possible speed in the current speed configuration (SS, FS, FS+)
    // in order to be compliant with the protocol specification.
    // Effectively, being a counter of clock ticks, it acts as a lower bound for the counter settings,
    // taking into account the minimum values defined @ 4.3.14.1
    uint32_t max_speed_hcnt = ((unsigned long) oscfreq * MIN_SCL_HIGHtime) / 1000000000;
    uint32_t max_speed_lcnt = ((unsigned long) oscfreq * MIN_SCL_LOWtime) / 1000000000;

    // See reference @ 4.3.14.1
    uint32_t hcnt = ((unsigned long) oscfreq / (frequency * 1000))/2 - fs_spklen - 7;
    uint32_t lcnt = ((unsigned long) oscfreq / (frequency * 1000))/2 - 1;

    if(frequency <= 100){
        // Minimum timing requirements with respect to fs_spklen
        i2c->ss_scl_hcnt = hcnt >= max_speed_hcnt ? hcnt : max_speed_hcnt;
        i2c->ss_scl_lcnt = lcnt >= max_speed_lcnt ? lcnt : max_speed_lcnt;
    }else{
        // Minimum timing requirements with respect to fs_spklen
        i2c->fs_scl_hcnt &= 0xFFFF0000;
        i2c->fs_scl_hcnt |= (hcnt >= max_speed_hcnt ? hcnt : max_speed_hcnt) & 0x0000FFFF;
        i2c->fs_scl_lcnt &= 0xFFFF0000;
        i2c->fs_scl_lcnt |= (lcnt >= max_speed_lcnt ? lcnt : max_speed_lcnt) & 0x0000FFFF;
    }
}

// NOTE: devAddr is passed right-shifted by one
void RP2040I2C1Master::setTarget(unsigned char devAddr){
    // Check if lower 8 bits of the previous target device are the same as the provided one, shifted by 1
    if(((devAddr>>1) & 0xFF) != (i2c->tar & 0xFF)){
        FastGlobalIrqLock lock;
        uint32_t prev_dma_cr = i2c->dma_cr; // Memorize previous dma control settings
        i2c->dma_cr &= ~(1<<I2C_IC_DMA_CR_TDMAE_LSB);
        i2c->enable |= I2C_IC_ENABLE_ABORT_BITS;

        while(i2c->enable & I2C_IC_ENABLE_ABORT_BITS)
            ;
        (void) i2c->clr_tx_abrt;

        i2c->dma_cr = prev_dma_cr;
        i2c->enable = 0;
        while(i2c->enable_status & I2C_IC_ENABLE_STATUS_IC_EN_BITS)
            ;
        i2c->tar = devAddr>>1;
        i2c->enable = 1;
        while(!(i2c->enable_status & I2C_IC_ENABLE_STATUS_IC_EN_BITS))
            ;
    }
}

bool RP2040I2C1Master::recv(unsigned char address, void *data, int len)
{
    //return false;
    //iprintf("[i2c] Receiving %d bytes from '%x'...\n", len, address);
    setTarget(address);

    unsigned int datasz=DMA_CH1_CTRL_TRIG_DATA_SIZE_VALUE_SIZE_BYTE;
    unsigned int rxDreq=i2c==i2c0_hw?33:35;
    unsigned int txDreq=i2c==i2c0_hw?32:34;

    unsigned short sendDummy = 0x01aa;

    //i2c->data_cmd = 0x01aa; //Send read command without DMA
                            //this will send the address and the R/W bit (set to read obviously)

    dma_hw->ch[txDmaCh].read_addr=reinterpret_cast<unsigned int>(&sendDummy);
    dma_hw->ch[txDmaCh].write_addr=reinterpret_cast<unsigned int>(&i2c->data_cmd);
    dma_hw->ch[txDmaCh].transfer_count=len;
    dma_hw->ch[txDmaCh].al1_ctrl=(txDreq<<DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB)
                                |(txDmaCh<<DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB) // disable chaining!!!!
                                |(DMA_CH1_CTRL_TRIG_DATA_SIZE_VALUE_SIZE_HALFWORD<<DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB)
                                |DMA_CH0_CTRL_TRIG_EN_BITS;

    dma_hw->ch[rxDmaCh].read_addr=reinterpret_cast<unsigned int>(&i2c->data_cmd);
    dma_hw->ch[rxDmaCh].write_addr=reinterpret_cast<unsigned int>(data);
    dma_hw->ch[rxDmaCh].transfer_count=len;
    dma_hw->ch[rxDmaCh].al1_ctrl=(rxDreq<<DMA_CH1_CTRL_TRIG_TREQ_SEL_LSB)
                                |(rxDmaCh<<DMA_CH1_CTRL_TRIG_CHAIN_TO_LSB) // disable chaining!!!!
                                |DMA_CH1_CTRL_TRIG_INCR_WRITE_BITS
                                |(datasz<<DMA_CH1_CTRL_TRIG_DATA_SIZE_LSB)
                                |DMA_CH1_CTRL_TRIG_HIGH_PRIORITY_BITS
                                |DMA_CH1_CTRL_TRIG_EN_BITS;
                                
    i2c->dma_cr = I2C_IC_DMA_CR_TDMAE_BITS | I2C_IC_DMA_CR_RDMAE_BITS;
    {
        FastGlobalIrqLock lock;
        dma_hw->multi_channel_trigger=(1U<<txDmaCh)|(1U<<rxDmaCh);
        while(true)
        {
            // TODO: Check for possible errors
            if(!(dma_hw->ch[rxDmaCh].al1_ctrl & DMA_CH1_CTRL_TRIG_BUSY_BITS)&&!(dma_hw->ch[txDmaCh].al1_ctrl & DMA_CH1_CTRL_TRIG_BUSY_BITS)) break;
            waiting = Thread::IRQgetCurrentThread();
            while(waiting) Thread::IRQglobalIrqUnlockAndWait(lock);
        }
        dma_hw->ch[txDmaCh].al1_ctrl=0;
        dma_hw->ch[rxDmaCh].al1_ctrl=0;
    }
    i2c->data_cmd = 0x0300;
    if((i2c->raw_intr_stat & I2C_IC_RAW_INTR_STAT_TX_ABRT_BITS)
            && ((i2c->tx_abrt_source & (I2C_IC_TX_ABRT_SOURCE_ABRT_7B_ADDR_NOACK_BITS | I2C_IC_TX_ABRT_SOURCE_ABRT_TXDATA_NOACK_BITS)))){
        (void)i2c->clr_intr;
        return false;
    }
    (void) i2c->clr_intr;
    //iprintf("Recvd!\n");
    return true;
}

bool RP2040I2C1Master::send(unsigned char address, const void *data, int len, bool sendStop)
{
    setTarget(address);

    unsigned int datasz=DMA_CH0_CTRL_TRIG_DATA_SIZE_VALUE_SIZE_BYTE;
    unsigned int txDreq=i2c==i2c0_hw?32:34;
    dma_hw->ch[txDmaCh].read_addr=reinterpret_cast<unsigned int>(data);
    dma_hw->ch[txDmaCh].write_addr=reinterpret_cast<unsigned int>(&i2c->data_cmd);
    dma_hw->ch[txDmaCh].transfer_count=len;
    i2c->dma_cr |= I2C_IC_DMA_CR_TDMAE_BITS | I2C_IC_DMA_CR_RDMAE_BITS;
    {
        FastGlobalIrqLock lock;
        dma_hw->ch[txDmaCh].ctrl_trig=(txDreq<<DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB)
                               |(txDmaCh<<DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB) // disable chaining!!!!
                               |DMA_CH0_CTRL_TRIG_INCR_READ_BITS
                               |(datasz<<DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB)
                               |DMA_CH0_CTRL_TRIG_EN_BITS;
        while((dma_hw->ch[txDmaCh].al1_ctrl)&DMA_CH0_CTRL_TRIG_BUSY_BITS)
        {
            waiting=Thread::IRQgetCurrentThread();
            while(waiting) Thread::IRQglobalIrqUnlockAndWait(lock);
        }
        dma_hw->ch[txDmaCh].al1_ctrl=0;
    }

    if(sendStop)
        stop();
    
    //TODO: THE FOLLOWING PART
    // Wait until "Transmit Fifo [completely] Empty"
    while(!(i2c->status & I2C_IC_STATUS_TFE_BITS));

    if((i2c->raw_intr_stat & I2C_IC_RAW_INTR_STAT_TX_ABRT_BITS)
            && ((i2c->tx_abrt_source & (I2C_IC_TX_ABRT_SOURCE_ABRT_7B_ADDR_NOACK_BITS | I2C_IC_TX_ABRT_SOURCE_ABRT_TXDATA_NOACK_BITS)))){
        (void)i2c->clr_intr;
        return false;
    }

    //Clear all interrupts
    (void)i2c->clr_intr;
    //iprintf("Sent!\n");
    return true;
}

RP2040I2C1Master::~RP2040I2C1Master()
{
    //TODO: see 4.3.10.3 "Disabling DW_apb_i2c"
    {
        GlobalIrqLock lock;

        i2c->enable = 0;

        uint32_t var = i2c->clr_intr;
        
        reset_block(RESETS_RESET_I2C1_BITS);
        IRQunregisterIrq(lock,I2C1_IRQ_IRQn,&RP2040I2C1Master::IRQhandleInterrupt,this);
        RP2040Dma::IRQunregisterChannel(lock,txDmaCh,&RP2040I2C1Master::IRQhandleDmaInterrupt,this);
        RP2040Dma::IRQunregisterChannel(lock,rxDmaCh,&RP2040I2C1Master::IRQhandleDmaInterrupt,this);

    }
}

void RP2040I2C1Master::stop()
{
    i2c->data_cmd = 1<<I2C_IC_DATA_CMD_STOP_LSB;
}

void RP2040I2C1Master::IRQhandleDmaInterrupt()
{
    uint32_t a = i2c->raw_intr_stat;
    uint32_t b = dma_hw->ints0 | dma_hw->ints1;
    eq.IRQpost([=]{ printf("In IRQhandleDmaInterrupt\nraw_intr_stat: 0x%08x\ndma intr: 0x%08x\n", a, b);});
    (void) i2c->clr_intr;
    //i2c->intr_mask = 0;
    //dma_hw->intr = dma_hw->intr;
    if(waiting)
    {
        waiting->IRQwakeup();
        waiting=nullptr;
    }
}

void RP2040I2C1Master::IRQhandleInterrupt() noexcept
{
    FastGlobalLockFromIrq lock;
    volatile uint32_t interrupts = i2c->raw_intr_stat;
    (void) i2c->clr_intr;
    //eq.IRQpost([=]{ printf("In IRQhandleInterrupt\nTriggered by: 0x%x\n", interrupts);});
    if(waiting)
    {
        waiting->IRQwakeup();
        waiting=nullptr;
    }
}

} //namespace miosix

