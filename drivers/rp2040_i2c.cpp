#include "rp2040_i2c.h"
#include "CMSIS/Device/RaspberryPi/RP2040/Include/RP2040.h"
#include "board_settings.h"
#include "drivers/rp2040_dma.h"
#include <interfaces/interrupts.h>

using namespace miosix;

namespace miosix {

// I2C driver implementation for rp2040 (master only)
RP2040I2C1Master::RP2040I2C1Master(GpioPin sda, GpioPin scl, int frequency)
{
    {
        GlobalIrqLock lock;
        IRQn_Type irqn;
        
        i2c = i2c1_hw;
        irqn = I2C1_IRQ_IRQn;

        //TODO: interrupts
        txDmaCh=RP2040Dma::IRQregisterChannel(lock,&RP2040I2C1Master::I2C1txDmaHandler,this);
        rxDmaCh=RP2040Dma::IRQregisterChannel(lock,&RP2040I2C1Master::I2C1rxDmaHandler,this);
    }

    //Disable the peripheral in order to write the control register IC_CON
    i2c->enable = I2C_IC_ENABLE_RESET;

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
    i2c->con |= I2C_IC_CON_RESET;

    setBitrate(frequency);

    i2c->tar = I2C_IC_TAR_RESET;
    //TODO General call or start byte???
    
    sda.function(Function::I2C1); //??? sda.fast();
    scl.function(Function::SPI); scl.mode(Mode::OUTPUT); //??? scl.fast();


    //Bus clear feature? (4.3.13.1)
    
    


    // Enable I2C1 controller
    i2c->enable |= I2C_IC_ENABLE_ENABLE_VALUE_ENABLED;
}

void RP2040I2C1Master::setBitrate(int frequency){
    // See reference @ 4.3.14.2
    unsigned int oscfreq = cpuFrequency; //ic_clk frequency (Hz)
    unsigned int MIN_SCL_HIGHtime; //minimum high period, ns
    unsigned int MIN_SCL_LOWtime; //minimum low period, ns
                                   //
    //TODO: variable spike suppression, verify
    //minimum 50ns spike suppression for SS and FS[+] modes
    unsigned int spike_suppression_period = 50;

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

    unsigned char fs_spklen = (spike_suppression_period * oscfreq)/1000000000;
    i2c->fs_spklen = fs_spklen;

    //TODO: round-up (ceiling) in an elegant and efficient way
    // Counters for fastest possible speed in the current speed configuration (SS, FS, FS+)
    // in order to be compliant with the protocol specification.
    // Effectively, being a counter of clock ticks, it acts as a lower bound for the counter settings,
    // taking into account the minimum values defined @ 4.3.14.1
    unsigned short max_speed_hcnt = (oscfreq * MIN_SCL_HIGHtime) / 1000000000;
    unsigned short max_speed_lcnt = (oscfreq * MIN_SCL_LOWtime) / 1000000000;

    // See reference @ 4.3.14.1
    unsigned short hcnt = (oscfreq / frequency)/2 - fs_spklen - 7;
    unsigned short lcnt = (oscfreq / frequency)/2 - 1;

    if(frequency <= 100){
        // Minimum timing requirements with respect to fs_spklen
        i2c->ss_scl_hcnt = hcnt >= max_speed_hcnt ? hcnt : max_speed_hcnt;
        i2c->ss_scl_lcnt = lcnt >= max_speed_lcnt ? lcnt : max_speed_lcnt;
    }else{
        // Minimum timing requirements with respect to fs_spklen
        i2c->fs_scl_hcnt = hcnt >= max_speed_hcnt ? hcnt : max_speed_hcnt;
        i2c->fs_scl_lcnt = lcnt >= max_speed_lcnt ? lcnt : max_speed_lcnt;
    }
}

bool RP2040I2C1Master::recv(unsigned char address, void *data, int len)
{
    i2c->tar = address;
    //TODO
    for(int i = 0; i < len; i++)
        *((char *)data) = 0;
    return true;
}

bool RP2040I2C1Master::send(unsigned char address, const void *data, int len, bool sendStop)
{
    i2c->tar = address;
    //TODO
    return true;
}

RP2040I2C1Master::~RP2040I2C1Master()
{
    //TODO: see 4.3.10.3 "Disabling DW_apb_i2c"


    GlobalIrqLock lock;
    // Set IC_ENABLE.ABORT to initiate controller shutdown
    i2c->enable = I2C_IC_ENABLE_ABORT_VALUE_ENABLED;

}

bool RP2040I2C1Master::start(unsigned char address)
{
    //TODO
    return true;
}

bool RP2040I2C1Master::startWorkaround(unsigned char address, int len)
{
    //TODO
    return true;
}

bool RP2040I2C1Master::waitStatus1()
{
    //TODO
    return true;
}

void RP2040I2C1Master::stop()
{
    //TODO
}

void RP2040I2C1Master::I2C1rxDmaHandler()
{
    //TODO
    waiting=nullptr;
}

void RP2040I2C1Master::I2C1txDmaHandler()
{
    //TODO
}

void RP2040I2C1Master::I2C1IrqHandler()
{
    //TODO
    waiting=nullptr;
}

void RP2040I2C1Master::I2C1errIrqHandler()
{
    //TODO
    waiting=nullptr;
}

} //namespace miosix

