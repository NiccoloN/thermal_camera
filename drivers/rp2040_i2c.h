#pragma once

#include <miosix.h>
#include <interfaces/gpio.h>

namespace miosix {

/**
 * Driver for the I2C peripheral in RP2040 under Miosix
 */
class RP2040I2C1Master
{
public:
    /**
     * Constructor. This driver only supports master mode with 7bit address.
     * NOTE: do not create multiple instances of this class.
     * \param sda SDA GPIO pin, the constructor configures the pin
     * \param scl SCL GPIO pin, the constructor configures the pin
     * \param frequency I2C bus frequency, in kHz.
     */
    RP2040I2C1Master(GpioPin sda, GpioPin scl, int frequency=400);

    /**
     * Send data
     * - send START condition
     * - send address
     * - send data
     * - send STOP condition
     * 
     * \param address device address, stored in bits 7 to 1. Bit 0 is ignored
     * \param data pointer with data to send
     * \param len length of data to send
     * \return true on success, false on failure
     */
    bool send(unsigned char address, const void *data, int len)
    {
        return send(address,data,len,true);
    }

    /**
     * Purely receive data
     * - send START condition
     * - send address
     * - receive data
     * - send STOP condition
     * 
     * \param address device address, stored in bits 7 to 1. Bit 0 is ignored
     * \param data pointer to a buffer where data will be received
     * \param len length of data to receive
     * \return true on success, false on failure
     */
    bool recv(unsigned char address, void *data, int len);

    /**
     * Send and receive data, with a repeated START between send and receive
     * - send START condition
     * - send address
     * - send data
     * - send repeated START
     * - send address
     * - receive data
     * - send STOP condition
     * 
     * \param address device address, stored in bits 7 to 1.
     * \param txData data to transmit, set to nullptr if none
     * \param txLen number of bytes to transmit, set to 0 if none
     * \param rxData data to receive, set to nullptr if none
     * \param rxLen number of bytes to receive, set to 0 if none
     */
    bool sendRecv(unsigned char address, const void *txData, int txLen,
                  void *rxData, int rxLen)
    {
        if(send(address,txData,txLen,false)==false) return false;
        return recv(address,rxData,rxLen);
    }

    /**
     * Probe if a device is on the bus
     * - send START condition
     * - send address
     * - send STOP condition
     * \return true if the address was acknowledged on the bus
     */
    bool probe(unsigned char address)
    {
        return true;
        char data = 0;
        return send(address, &data, 1, true);
    }
    
    /**
     * Internal version of send, able to omit the final STOP
     * \param address device address (bit 0 is forced at 0)
     * \param data pointer with data to send
     * \param len length of data to send
     * \param sendStop if set to false disables the sending of a STOP condition
     * after data transmission has finished
     * \return true on success, false on failure
     */
    bool send(unsigned char address, const void *data, int len, bool sendStop);
    
    /**
     * Wait until until an interrupt occurs during the send start bit and
     * send address phases of the i2c communication.
     * \return true if the operation was successful, false on error
     */
    //bool waitStatus1();
    
    /**
     * Send a stop condition, waiting for its completion
     */
    void stop();
    /**
     * Destructor
     */
    ~RP2040I2C1Master();
    
private:
    RP2040I2C1Master(const RP2040I2C1Master&);
    RP2040I2C1Master& operator=(const RP2040I2C1Master&);

    /**
     * DMA I2C interrupt handler
     */
    void IRQhandleDmaInterrupt();

    void IRQhandleInterrupt() noexcept;

    /**
     * Sets the I2C's controller to the specified bitrate (in kHz)
     */
    void setBitrate(int bitrate);

    void setTarget(unsigned char targetAddr);

    volatile bool error;     ///< Set to true by IRQ on error
    miosix::Thread *waiting=nullptr; ///< Thread waiting for an operation to complete
    i2c_hw_t *i2c;
    unsigned char txDmaCh, rxDmaCh;
};

} //namespace miosix
