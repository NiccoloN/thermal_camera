/***************************************************************************
 *   Copyright (C) 2012-2022 by Terraneo Federico                          *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   As a special exception, if other files instantiate templates or use   *
 *   macros or inline functions from this file, or you compile this file   *
 *   and link it with other works to produce a work based on this file,    *
 *   this file does not by itself cause the resulting work to be covered   *
 *   by the GNU General Public License. However the source code for this   *
 *   file must still be made available in accordance with the GNU General  *
 *   Public License. This exception does not invalidate any other reasons  *
 *   why a work based on this file might be covered by the GNU General     *
 *   Public License.                                                       *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, see <http://www.gnu.org/licenses/>   *
 ***************************************************************************/

#pragma once

#include <mutex>
#include "rp2040_spi.h"
#include "hwmapping.h"

/**
 * Class to access a MRAM memory
 */
class MRAM
{
public:
    MRAM();
    
    /**
     * \return the MRAM's size in bytes
     */
    unsigned int size() const { return ramSize; }
    
    /**
     * Write a block of data into the MRAM.
     * \param addr start address into the MRAM where the data block will be
     * written
     * \param data data block
     * \param size data block size
     * \return true on success, false on failure
     */
    bool write(unsigned int addr, const void *data, int size);
    
    /**
     * Read a block of data from the MRAM
     * \param addr start address into the MRAM where the data block will be read
     * \param data data block
     * \param size data block size
     * \return true on success, false on failure
     */
    bool read(unsigned int addr, void *data, int size);
    
private:
    MRAM(const MRAM&)=delete;
    MRAM& operator= (const MRAM&)=delete;

    /**
     * Send the write enable command, required before writing/erasing.
     * Requires the mutex to be locked.
     */
    void writeEnable();

    /**
     * Read one of the status registers.
     */
    unsigned char readStatus(int reg=0);

    void setConfig(unsigned char c1, unsigned char c2,
                   unsigned char c3, unsigned char c4);
    
    miosix::RP2040PL022DmaSpi spi;
    unsigned int ramSize;
};
