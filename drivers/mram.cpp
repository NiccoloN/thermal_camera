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

#include "mram.h"
#include <cstdio>
#include <miosix.h>
#include <interfaces/delays.h>

using namespace std;
using namespace miosix;

MRAM::MRAM() : spi(1,40*1000000,false,false,mram_miso::getPin(),mram_mosi::getPin(),mram_sck::getPin(),GpioPin())
{
    iprintf("MRAM\n");
    mram_cs::function(Function::GPIO);
    mram_cs::mode(Mode::OUTPUT);
    mram_cs::fast();

    iprintf("device ID:\n");
    mram_cs::low();
    spi.sendRecv(0x9f);
    unsigned int deviceId=0;
    for(int i=0;i<4;i++) deviceId=(deviceId<<8)|spi.sendRecv(0xff);
    mram_cs::high();
    iprintf("  vendor   =0x%02x\n",(deviceId>>24)&0xff);
    iprintf("  interface=%d\n",(deviceId>>20)&0xf);
    unsigned int voltage=(deviceId>>16)&0xf;
    iprintf("  voltage  =%d (%s)\n",voltage,voltage==1?"3V":"1.8V");
    iprintf("  temp     =%d\n",(deviceId>>12)&0xf);
    unsigned int size=(deviceId>>8)&0xf;
    iprintf("  size     =%d (%d Mb)\n",size,1U<<size);
    unsigned int frequency=(deviceId>>0)&0xff;
    iprintf("  frequency=0x%02x (%d MHz)\n",frequency,frequency==1?108:54);
    if(size==0) errorHandler(Error::UNEXPECTED);
    ramSize=(1U<<size)*1024*(1024/8);
    
    setConfig(0,0,0x60,0x5);

    for(int i=0;i<5;i++)
        iprintf("status%d=0x%x\n",i,readStatus(i));

    constexpr int testSize=256, testNum=10;
    int failures=0;
    unsigned char testWrite[testSize];
    unsigned char testRead[testSize];
    for(int i=0;i<testNum;i++)
    {
        iprintf("testing MRAM; test %d\n", i);
        for(int j=0;j<testSize;j++) testWrite[j]=rand()%256;
        write(0,testWrite,testSize);
        read(0,testRead,testSize);
        for(int j=0;j<testSize;j++) if(testWrite[j]!=testRead[j])
        {
            iprintf("failed at index %d (%02x read != %02x written)\n",j,testRead[j],testWrite[j]);
            failures++;
        }
        if(failures!=0)
        {
            iprintf("mram test failed\n");
            break;
        }
    }

}

bool MRAM::write(unsigned int addr, const void *data, int size)
{
    if(addr>=this->size() || addr+size>this->size()) return false;
    spi.setBitrate(108000000);
    writeEnable();
    mram_cs::low();
    spi.sendRecv(0x02);
    spi.sendRecv((addr>>16) & 0xff);
    spi.sendRecv((addr>>8) & 0xff);
    spi.sendRecv(addr & 0xff);
    spi.send(reinterpret_cast<const unsigned char *>(data),size);
    mram_cs::high();
    spi.setBitrate(40000000);
    return true;
}

bool MRAM::read(unsigned int addr, void *data, int size)
{
    if(addr>=this->size() || addr+size>this->size()) return false;
    mram_cs::low();
    spi.sendRecv(0x03);
    spi.sendRecv((addr>>16) & 0xff);
    spi.sendRecv((addr>>8) & 0xff);
    spi.sendRecv(addr & 0xff);
    spi.recv(reinterpret_cast<unsigned char *>(data),size);
    mram_cs::high();
    return true;
}

void MRAM::writeEnable()
{
    mram_cs::low();
    spi.sendRecv(0x06); //Write enable
    mram_cs::high();
}

unsigned char MRAM::readStatus(int reg)
{
    static const unsigned char statusRegs[5]={0x05,0x35,0x3f,0x44,0x45};
    if(reg>=5) return 0;
    mram_cs::low();
    spi.sendRecv(statusRegs[reg]);
    unsigned int res=spi.sendRecv(0xff);
    mram_cs::high();
    return res;
}

void MRAM::setConfig(unsigned char c1, unsigned char c2,
                     unsigned char c3, unsigned char c4)
{
    writeEnable();
    mram_cs::low();
    spi.sendRecv(0x87);
    spi.sendRecv(c1);
    spi.sendRecv(c2);
    spi.sendRecv(c3);
    spi.sendRecv(c4);
    mram_cs::high();
}
