#include "rp2040_i2c.h"
#include "kernel/lock.h"
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

using namespace miosix;

namespace miosix {

// I2C driver implementation for rp2040 (master only)
RP2040I2C1Master::RP2040I2C1Master(GpioPin sda, GpioPin scl, int frequency)
{
    {
        GlobalIrqLock lock;
        IRQn_Type irqn;

        irqn = I2C1_IRQ_IRQn;
        clocks_hw->wake_en0|=CLOCKS_WAKE_EN0_CLK_SYS_I2C1_BITS;
        clocks_hw->sleep_en0|=CLOCKS_SLEEP_EN0_CLK_SYS_I2C1_BITS;
        unreset_block_wait(RESETS_RESET_I2C1_BITS);

        IRQregisterIrq(lock,irqn,&RP2040I2C1Master::IRQhandleInterrupt,this);

        // Used for sending read reqs
        txDmaCh=RP2040Dma::IRQregisterChannel(lock,&RP2040I2C1Master::IRQhandleDmaInterrupt,this); 
 
        // Used to send the finishing byte w/ the stop bit
        finisherDmaCh=RP2040Dma::IRQregisterChannel(lock,&RP2040I2C1Master::IRQhandleDmaInterrupt,this); 
        
        // Used for actually reading the incoming data
        rxDmaCh=RP2040Dma::IRQregisterChannel(lock,&RP2040I2C1Master::IRQhandleDmaInterrupt,this);

        i2c = i2c1_hw;
        
        //Disable the peripheral in order to write the control register IC_CON
        i2c->enable = I2C_IC_ENABLE_RESET;
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
        i2c->con = I2C_IC_CON_RESET | 1<<I2C_IC_CON_IC_RESTART_EN_LSB;
        i2c->intr_mask = 0;
        i2c->dma_cr = I2C_IC_DMA_CR_TDMAE_BITS | I2C_IC_DMA_CR_RDMAE_BITS;
        sda.function(Function::I2C1); //sda.fast(); //sda.mode(Mode::INPUT_SCHMITT_TRIG_PULL_UP);   //sda.fast(); 
        scl.function(Function::I2C1_SCL); //sda.fast(); //scl.mode(Mode::INPUT_SCHMITT_TRIG_PULL_UP); //scl.fast();

    }
    setBitrate(frequency);
    i2c->enable |= 0x00000001;
}

// To be called ONLY when the device is disabled
void RP2040I2C1Master::setBitrate(int frequency){
    unsigned long long oscfreq = cpuFrequency; //ic_clk frequency (Hz)
    unsigned long long MIN_SCL_HIGHtime; //minimum high period, ns
    unsigned long long MIN_SCL_LOWtime; //minimum low period, ns

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

    // Minimum 50ns spike suppression for SS and FS[+] modes
    unsigned long spike_suppression_period = 50;
    unsigned char fs_spklen = ((unsigned long)spike_suppression_period * oscfreq)/1000000000;

    i2c->fs_spklen = fs_spklen;

    uint32_t max_speed_hcnt = (oscfreq * MIN_SCL_HIGHtime) / 1000000000;
    uint32_t max_speed_lcnt = (oscfreq * MIN_SCL_LOWtime) / 1000000000;

    if(frequency <= 100){
        i2c->ss_scl_hcnt = max_speed_hcnt;
        i2c->ss_scl_lcnt = max_speed_lcnt;
    }else{
        i2c->fs_scl_hcnt = max_speed_hcnt;
        i2c->fs_scl_lcnt = max_speed_lcnt;
    }
}

// NOTE: devAddr is passed right-shifted by one because of an old API inherited by the STM32
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
    setTarget(address);

    unsigned short sendDummy = 0x0111;
    unsigned short stopDummy = 0x0311;

    if(len == 1){
        i2c->data_cmd = sendDummy | 1<<I2C_IC_DATA_CMD_STOP_LSB;
        while(!(i2c->status & I2C_IC_STATUS_RFNE_BITS))
            ;
        unsigned char *dest = static_cast<unsigned char*>(data);
        *dest = i2c->data_cmd;
        return true;
    }

    unsigned int datasz=DMA_CH1_CTRL_TRIG_DATA_SIZE_VALUE_SIZE_BYTE;
    unsigned int rxDreq=i2c==i2c0_hw?33:35;
    unsigned int txDreq=i2c==i2c0_hw?32:34;
    
    i2c->dma_cr = I2C_IC_DMA_CR_TDMAE_BITS | I2C_IC_DMA_CR_RDMAE_BITS;
    dma_hw->ch[txDmaCh].read_addr=reinterpret_cast<unsigned int>(&sendDummy);
    dma_hw->ch[txDmaCh].write_addr=reinterpret_cast<unsigned int>(&i2c->data_cmd);
    dma_hw->ch[txDmaCh].transfer_count=len-1;
    dma_hw->ch[txDmaCh].al1_ctrl=(txDreq<<DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB)
                                |(finisherDmaCh<<DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB) // chain to finisher
                                |(DMA_CH1_CTRL_TRIG_DATA_SIZE_VALUE_SIZE_HALFWORD<<DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB) // Send 16 bits, to send CMD=1 too
                                |DMA_CH0_CTRL_TRIG_EN_BITS;

    dma_hw->ch[finisherDmaCh].read_addr=reinterpret_cast<unsigned int>(&stopDummy);
    dma_hw->ch[finisherDmaCh].write_addr=reinterpret_cast<unsigned int>(&i2c->data_cmd);
    dma_hw->ch[finisherDmaCh].transfer_count=1;
    dma_hw->ch[finisherDmaCh].al1_ctrl=(txDreq<<DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB)
                                |(finisherDmaCh<<DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB) // disable chaining!!!!
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
        dma_hw->multi_channel_trigger=(1U<<txDmaCh)|(1U<<rxDmaCh);
        while(true)
        {
            if(!(dma_hw->ch[rxDmaCh].al1_ctrl & DMA_CH1_CTRL_TRIG_BUSY_BITS)) break;
            waiting = Thread::IRQgetCurrentThread();
            while(waiting) Thread::IRQglobalIrqUnlockAndWait(lock);
        }
        dma_hw->ch[txDmaCh].al1_ctrl=0;
        dma_hw->ch[finisherDmaCh].al1_ctrl=0;
        dma_hw->ch[rxDmaCh].al1_ctrl=0;
    }

    bool aborted = (i2c->raw_intr_stat & I2C_IC_RAW_INTR_STAT_TX_ABRT_BITS)
            && ((i2c->tx_abrt_source & (I2C_IC_TX_ABRT_SOURCE_ABRT_7B_ADDR_NOACK_BITS | I2C_IC_TX_ABRT_SOURCE_ABRT_TXDATA_NOACK_BITS)));

    (void) i2c->clr_intr;

    return !aborted;
}

/*
 * This driver is not implemented with DMA support because the integration between
 * the DMA and the I2C controller is poorly designed.
 * Turns out that, even if the DMA were programmed to read and write just one byte
 * at a time, the I2C controller would read 16 bits anyway: so if it were sending the
 * n-th byte and the (n+1)th byte were odd, the write operation would magically become
 * a read operation because it would interpret the LSB of the (n+1)th byte as bit 9
 * of byte n, making it a read.
 */
bool RP2040I2C1Master::send(unsigned char address, const void *data, int len, bool sendStop)
{
    setTarget(address);
    
    if(len == 1){
        uint32_t dataToBeSent = (sendStop ? 1 : 0)<<I2C_IC_DATA_CMD_STOP_LSB  | (*((uint8_t *)data) & 0xFF);
        i2c->data_cmd = dataToBeSent;
    }else{
        FastGlobalIrqLock lock;

        /*
         * Too high a number, and the interrupt would be fired too often;
         * too low, and a restart could be issued between one byte and another because too much time
         * would pass between the end of the FIFO and the interrupt, which would cause a restart
         * on the next byte.
         */
        i2c->tx_tl = 3;
        i2c->intr_mask |= 1<<I2C_IC_INTR_MASK_M_TX_EMPTY_LSB; // Allow only one type of interrupt
        int curr = 0;
        while(true)
        {
            while(i2c->status & I2C_IC_STATUS_TFNF_BITS && curr < len){
                if(curr < len-1)
                    i2c->data_cmd = ((uint8_t *)data)[curr];
                else
                    i2c->data_cmd = (sendStop ? 1 : 0)<<I2C_IC_DATA_CMD_STOP_LSB | ((uint8_t *)data)[len-1];
                curr++;
            }
            if(curr == len) break;
            waiting=Thread::IRQgetCurrentThread();
            while(waiting) Thread::IRQglobalIrqUnlockAndWait(lock);
        }
        i2c->intr_mask &= ~(1<<I2C_IC_INTR_MASK_M_TX_EMPTY_LSB);
        i2c->tx_tl = 0;
    }
    
    // Wait until "Transmit Fifo [completely] Empty"
    while(!(i2c->status & I2C_IC_STATUS_TFE_BITS));

    bool aborted = (i2c->raw_intr_stat & I2C_IC_RAW_INTR_STAT_TX_ABRT_BITS)
            && ((i2c->tx_abrt_source & (I2C_IC_TX_ABRT_SOURCE_ABRT_7B_ADDR_NOACK_BITS | I2C_IC_TX_ABRT_SOURCE_ABRT_TXDATA_NOACK_BITS)));

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
        RP2040Dma::IRQunregisterChannel(lock,finisherDmaCh,&RP2040I2C1Master::IRQhandleDmaInterrupt,this);
    }
}

void RP2040I2C1Master::stop()
{
    i2c->data_cmd = 1<<I2C_IC_DATA_CMD_STOP_LSB;
}

void RP2040I2C1Master::IRQhandleDmaInterrupt()
{
    //GIL taken in recv function
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

