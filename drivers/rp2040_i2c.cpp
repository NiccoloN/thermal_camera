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
         * Bit 11 ("SPECIAL"): 0 => normal operation (NO START BYTE NOR GENERAL CALL);
         * Bit 10 ("GC_OR_START"): 0 BUT it is ignored because SPECIAL is set to 1;
         * Bit 9:0: 0..0 => where the target address will be placed;
         */
        i2c->tar = I2C_IC_TAR_RESET;

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
        i2c->con |= 1<<I2C_IC_CON_IC_RESTART_EN_LSB;
        i2c->con |= 1<<I2C_IC_CON_TX_EMPTY_CTRL_LSB;
        i2c->intr_mask = 0;

        i2c->dma_cr = I2C_IC_DMA_CR_TDMAE_BITS | I2C_IC_DMA_CR_RDMAE_BITS;

        sda.function(Function::I2C1); //sda.fast(); //sda.mode(Mode::INPUT_SCHMITT_TRIG_PULL_UP);   //sda.fast(); 
        scl.function(Function::I2C1_SCL); //sda.fast(); //scl.mode(Mode::INPUT_SCHMITT_TRIG_PULL_UP); //scl.fast();

        setBitrate(frequency);

        //Bus clear feature? (4.3.13.1)


        i2c->enable |= 0x00000001;

    }
    iprintf("I2C initialized!\n");
    
}

// TODO: check speed calculation
// See reference @ 4.3.14.2
// To be called ONLY when the device is disabled
void RP2040I2C1Master::setBitrate(int frequency){
    unsigned long long oscfreq = cpuFrequency; //ic_clk frequency (Hz)
    unsigned long long MIN_SCL_HIGHtime; //minimum high period, ns
    unsigned long long MIN_SCL_LOWtime; //minimum low period, ns
                                   //
    //TODO: variable spike suppression, verify
    //minimum 50ns spike suppression for SS and FS[+] modes
    unsigned long spike_suppression_period = 50;

    i2c->con &= ~I2C_IC_CON_SPEED_BITS;

    if(frequency <= 100){
        i2c->con |= (1 << I2C_IC_CON_SPEED_LSB);
        MIN_SCL_HIGHtime = 4000;
        MIN_SCL_LOWtime = 4700;
    }else if(frequency <= 400){
        i2c->con |= (2 << I2C_IC_CON_SPEED_LSB);
        MIN_SCL_HIGHtime = 600;
        MIN_SCL_LOWtime = 1300;
    }else{
        i2c->con |= (2 << I2C_IC_CON_SPEED_LSB);
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
    uint32_t max_speed_hcnt = (oscfreq * MIN_SCL_HIGHtime) / 1000000000;
    uint32_t max_speed_lcnt = (oscfreq * MIN_SCL_LOWtime) / 1000000000;

    // See reference @ 4.3.14.1
    uint32_t hcnt = ((unsigned long) oscfreq / (frequency * 1000))/2 - fs_spklen - 7;
    uint32_t lcnt = ((unsigned long) oscfreq / (frequency * 1000))/2 - 1;

    if(frequency <= 100){
        // Minimum timing requirements with respect to fs_spklen
        //i2c->ss_scl_hcnt = hcnt >= max_speed_hcnt ? hcnt : max_speed_hcnt;
        //i2c->ss_scl_lcnt = lcnt >= max_speed_lcnt ? lcnt : max_speed_lcnt;
        i2c->ss_scl_hcnt = max_speed_hcnt;
        i2c->ss_scl_lcnt = max_speed_lcnt;
    }else{
        // Minimum timing requirements with respect to fs_spklen
        //i2c->fs_scl_hcnt = (hcnt >= max_speed_hcnt ? hcnt : max_speed_hcnt) & 0x0000FFFF;
        //i2c->fs_scl_lcnt = (lcnt >= max_speed_lcnt ? lcnt : max_speed_lcnt) & 0x0000FFFF;
        i2c->fs_scl_hcnt = max_speed_hcnt;
        i2c->fs_scl_lcnt = max_speed_lcnt;
    }
}

// NOTE: devAddr is passed right-shifted by one
void RP2040I2C1Master::setTarget(unsigned char devAddr){
    // Check if lower 8 bits of the previous target device are the same as the provided one, shifted by 1
    if(((devAddr>>1) & 0xFF) != (i2c->tar & 0xFF)){
        FastGlobalIrqLock lock;
        i2c->enable = 0;
        while(i2c->enable_status & I2C_IC_ENABLE_STATUS_IC_EN_BITS); // Wait for the controller to be disabled
        i2c->tar = (devAddr>>1) & 0xFF;
        i2c->enable = 1;
        while(!(i2c->enable_status & I2C_IC_ENABLE_STATUS_IC_EN_BITS)); // Wait for the controller to be enabled
    }
}

bool RP2040I2C1Master::recv(unsigned char address, void *data, int len)
{
    //iprintf("R 0x%x %d\n", address, len);
    setTarget(address);

    unsigned short sendDummy = 0x015d;

    if(len == 1){
        i2c->data_cmd = sendDummy | 1<<I2C_IC_DATA_CMD_STOP_LSB;
        while(!(i2c->status & I2C_IC_STATUS_RFNE_BITS))
            ;
        unsigned char *dest = static_cast<unsigned char*>(data);
        *dest = i2c->data_cmd;
        return true;
    }

    unsigned short reqs[len];
    int i;
    for(i=0;i<len-1;i++)reqs[i]=sendDummy;
    reqs[i]=sendDummy | 1<<I2C_IC_DATA_CMD_STOP_LSB;

    unsigned int datasz=DMA_CH1_CTRL_TRIG_DATA_SIZE_VALUE_SIZE_BYTE;
    unsigned int rxDreq=i2c==i2c0_hw?33:35;
    unsigned int txDreq=i2c==i2c0_hw?32:34;
    
    i2c->dma_cr = I2C_IC_DMA_CR_TDMAE_BITS | I2C_IC_DMA_CR_RDMAE_BITS;
    dma_hw->ch[txDmaCh].read_addr=reinterpret_cast<unsigned int>(reqs);
    dma_hw->ch[txDmaCh].write_addr=reinterpret_cast<unsigned int>(&i2c->data_cmd);
    dma_hw->ch[txDmaCh].transfer_count=len;
    dma_hw->ch[txDmaCh].al1_ctrl=(txDreq<<DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB)
                                |(txDmaCh<<DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB) // disable chaining!!!!
                                |DMA_CH1_CTRL_TRIG_INCR_READ_BITS
                                |(DMA_CH1_CTRL_TRIG_DATA_SIZE_VALUE_SIZE_HALFWORD<<DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB) // Send 16 bits, to send CMD=1 too
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
                                
    {
        FastGlobalIrqLock lock;
        dma_hw->multi_channel_trigger|=(1U<<txDmaCh)|(1U<<rxDmaCh);
        while(true)
        {
            // TODO: Check for possible errors
            if(!(dma_hw->ch[txDmaCh].al1_ctrl & DMA_CH1_CTRL_TRIG_BUSY_BITS)){
                //while(!(i2c->status & I2C_IC_STATUS_TFNF_BITS));
                //i2c->data_cmd = 0x0311; // Stop and read last;
                /*
                while(!(i2c->status & I2C_IC_STATUS_TFNF_BITS));
                i2c->data_cmd = 0x0311; // Stop and read last;
                while(!(i2c->status & I2C_IC_STATUS_RFNE_BITS));
                *((uint8_t *)data) = i2c->data_cmd;
                */
            }
            if(!(dma_hw->ch[rxDmaCh].al1_ctrl & DMA_CH1_CTRL_TRIG_BUSY_BITS)) break;
            waiting = Thread::IRQgetCurrentThread();
            while(waiting) Thread::IRQglobalIrqUnlockAndWait(lock);
        }
        //i2c->data_cmd = 0x0300; // Stop and read last;
        (void) i2c->clr_intr;
        dma_hw->ch[txDmaCh].al1_ctrl=0;
        dma_hw->ch[rxDmaCh].al1_ctrl=0;
    }

    //while((i2c->status & I2C_IC_STATUS_RFNE_BITS)); // while it's not empty, wait for it to empty

    bool aborted = (i2c->raw_intr_stat & I2C_IC_RAW_INTR_STAT_TX_ABRT_BITS)
            && ((i2c->tx_abrt_source & (I2C_IC_TX_ABRT_SOURCE_ABRT_7B_ADDR_NOACK_BITS | I2C_IC_TX_ABRT_SOURCE_ABRT_TXDATA_NOACK_BITS)));

    if(aborted) iprintf("Recv aborted. Reason: 0x%08lx\n", i2c->tx_abrt_source);

    //iprintf("R!\n");

    (void) i2c->clr_intr;
    return !aborted;
}

bool RP2040I2C1Master::send(unsigned char address, const void *data, int len, bool sendStop)
{
    //iprintf("S 0x%x %d (stop=%d)\n", address, len, sendStop);
    setTarget(address);
    
    if(len == 1){
        uint32_t dataToBeSent = 0;
        dataToBeSent = (sendStop ? 1<<I2C_IC_DATA_CMD_STOP_LSB : 0) | (*((uint8_t *)data) & 0xFF);
        i2c->data_cmd = dataToBeSent;
    }else{
        unsigned int datasz=DMA_CH0_CTRL_TRIG_DATA_SIZE_VALUE_SIZE_BYTE;
        unsigned int txDreq=i2c==i2c0_hw?32:34;
        dma_hw->ch[txDmaCh].read_addr=reinterpret_cast<unsigned int>(data);
        dma_hw->ch[txDmaCh].write_addr=reinterpret_cast<unsigned int>(&i2c->data_cmd);
        dma_hw->ch[txDmaCh].transfer_count= (sendStop ? len-1 : len);
        {
            FastGlobalIrqLock lock;
            dma_hw->ch[txDmaCh].ctrl_trig=(txDreq<<DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB)
                                   |(txDmaCh<<DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB) // disable chaining!!!!
                                   |DMA_CH0_CTRL_TRIG_INCR_READ_BITS
                                   |(datasz<<DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB)
                                   |DMA_CH0_CTRL_TRIG_EN_BITS;
            while(true)
            {
                if(!((dma_hw->ch[txDmaCh].al1_ctrl)&DMA_CH0_CTRL_TRIG_BUSY_BITS)){
                    if(sendStop){
                        while (!(i2c->status & I2C_IC_STATUS_TFNF_BITS));
                        i2c->data_cmd = 1<<I2C_IC_DATA_CMD_STOP_LSB | ((uint8_t *)data)[len-1];
                    }
                    break;
                }

                waiting=Thread::IRQgetCurrentThread();
                while(waiting) Thread::IRQglobalIrqUnlockAndWait(lock);
            }
            dma_hw->ch[txDmaCh].al1_ctrl=0;
        }
    }
    
    // Wait until "Transmit Fifo [completely] Empty"
    while(!(i2c->status & I2C_IC_STATUS_TFE_BITS));

    bool aborted = (i2c->raw_intr_stat & I2C_IC_RAW_INTR_STAT_TX_ABRT_BITS)
            && ((i2c->tx_abrt_source & (I2C_IC_TX_ABRT_SOURCE_ABRT_7B_ADDR_NOACK_BITS | I2C_IC_TX_ABRT_SOURCE_ABRT_TXDATA_NOACK_BITS)));

    if(aborted) iprintf("Send aborted. Reason: 0x%08lx\n", i2c->tx_abrt_source);

    //iprintf("S!\n");

    //Clear all interrupts
    (void)i2c->clr_intr;
    return !aborted;
}

RP2040I2C1Master::~RP2040I2C1Master()
{
    //TODO: see 4.3.10.3 "Disabling DW_apb_i2c"
    {
        GlobalIrqLock lock;

        i2c->enable = 0;

        (void) i2c->clr_intr;
        
        reset_block(RESETS_RESET_I2C1_BITS);
        clocks_hw->wake_en0&=~(CLOCKS_WAKE_EN0_CLK_SYS_I2C1_BITS);
        clocks_hw->sleep_en0&=~(CLOCKS_SLEEP_EN0_CLK_SYS_I2C1_BITS);
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
    //uint32_t a = i2c->raw_intr_stat;
    //uint32_t b = dma_hw->ints0 | dma_hw->ints1;
    //eq.IRQpost([=]{ printf("In IRQhandleDmaInterrupt\nraw_intr_stat: 0x%08x\ndma intr: 0x%08x\n", a, b);});
    //(void) i2c->clr_intr;
    //dma_hw->intr = dma_hw->intr;
    dma_hw->ints0 = (1U << txDmaCh) | (1U << rxDmaCh);
    dma_hw->ints1 = (1U << txDmaCh) | (1U << rxDmaCh);
    if(waiting)
    {
        waiting->IRQwakeup();
        waiting=nullptr;
    }
}

void RP2040I2C1Master::IRQhandleInterrupt() noexcept
{
    FastGlobalLockFromIrq lock;
    (void) i2c->clr_intr;
    if(waiting)
    {
        waiting->IRQwakeup();
        waiting=nullptr;
    }
}

} //namespace miosix

