/***************************************************************************
 *   Copyright (C) 2021 by Terraneo Federico                               *
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

#include "display_er_oledm015.h"
#include <cstdint>
#include <miosix.h>
#include <interfaces/endianness.h>
#include <algorithm>
#include <line.h>
#include "drivers/rp2040_gpio.h"
#include "hwmapping.h"
#include "kernel/lock.h"
#include "rp2040_spi.h"

using namespace std;
using namespace miosix;
using namespace mxgui;

//Display connection

using cs   = oled_cs;
using sck  = oled_sck;  //Used as HW SPI
using mosi = oled_mosi; //Used as HW SPI
using dc   = oled_dc;
using res  = oled_res;

/**
 * Send a command to the display
 * \param c command
 */
void DisplayErOledm015::cmd(unsigned char c)
{
    dc::low();
    cs::low();
    spiController.sendRecv(c);
    cs::high();
    delayUs(1);
}

/**
 * Send data to the display
 * \param d data
 */
void DisplayErOledm015::dat(unsigned char d)
{
    dc::high();
    cs::low();
    spiController.sendRecv(d);
    cs::high();
    delayUs(1);
}

static const int width=128, height=128; ///< Display size

/**
 * Set cursor to desired location
 * \param point where to set cursor (0<=x<128, 0<=y<128)
 */
void DisplayErOledm015::setCursor(Point p)
{
    #ifdef MXGUI_ORIENTATION_VERTICAL
    cmd(0x15); dat(p.x()); dat(0x7f); // Set column address
    cmd(0x75); dat(p.y()); dat(0x7f); // Set row address
    #else //MXGUI_ORIENTATION_HORIZONTAL
    cmd(0x15); dat(p.y()); dat(0x7f); // Set column address
    cmd(0x75); dat(p.x()); dat(0x7f); // Set row address
    #endif //Hardware doesn't seem to support mirroring
}

/**
 * Set a hardware window on the screen, optimized for writing text.
 * The GRAM increment will be set to up-to-down first, then left-to-right which
 * is the correct increment to draw fonts
 * \param p1 upper left corner of the window
 * \param p2 lower right corner of the window
 */
void DisplayErOledm015::textWindow(Point p1, Point p2)
{
    #ifdef MXGUI_ORIENTATION_VERTICAL
    cmd(0x15); dat(p1.x()); dat(p2.x()); // Set column address
    cmd(0x75); dat(p1.y()); dat(p2.y()); // Set row address
    cmd(0xa0); dat(0x67);
    #else //MXGUI_ORIENTATION_HORIZONTAL
    cmd(0x15); dat(p1.y()); dat(p2.y()); // Set column address
    cmd(0x75); dat(p1.x()); dat(p2.x()); // Set row address
    cmd(0xa0); dat(0x64);
    #endif //Hardware doesn't seem to support mirroring
}

/**
 * Set a hardware window on the screen, optimized for drawing images.
 * The GRAM increment will be set to left-to-right first, then up-to-down which
 * is the correct increment to draw images
 * \param p1 upper left corner of the window
 * \param p2 lower right corner of the window
 */
inline void DisplayErOledm015::imageWindow(Point p1, Point p2)
{
    #ifdef MXGUI_ORIENTATION_VERTICAL
    cmd(0x15); dat(p1.x()); dat(p2.x()); // Set column address
    cmd(0x75); dat(p1.y()); dat(p2.y()); // Set row address
    cmd(0xa0); dat(0x66);
    #else //MXGUI_ORIENTATION_HORIZONTAL
    cmd(0x15); dat(p1.y()); dat(p2.y()); // Set column address
    cmd(0x75); dat(p1.x()); dat(p2.x()); // Set row address
    cmd(0xa0); dat(0x65);
    #endif //Hardware doesn't seem to support mirroring
}

//
// class DisplayErOledm015
//

namespace mxgui {

DisplayErOledm015::DisplayErOledm015() : buffer(nullptr), buffer2(nullptr),
spiController(
    0,
    15000000,
    true,
    true,
    Gpio<P0, 15>::getPin(), //random high pin
    oled_mosi::getPin(),
    oled_sck::getPin(),
    Gpio<P0, 16>::getPin() //random high pin
)
{
    cs::function(Function::GPIO);
    cs::mode(Mode::OUTPUT); cs::getPin().fast();

    dc::function(Function::GPIO);
    dc::mode(Mode::OUTPUT); dc::getPin().fast();

    res::high();
    Thread::sleep(1);
    res::low();
    delayUs(100);
    res::high();
    delayUs(100);

    static const unsigned char grayTable[]=
    {
          0,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15, 16,
         17, 18, 19, 21, 23, 25, 27, 29, 31, 33, 35, 37, 39, 42, 45, 48,
         51, 54, 57, 60, 63, 66, 69, 72, 76, 80, 84, 88, 92, 96,100,104,
        108,112,116,120,125,130,135,140,145,150,155,160,165,170,175,180
    };
    
    cmd(0xfd); dat(0x12);                       // Disable command lock
    cmd(0xfd); dat(0xb1);                       // Enable all commands
    cmd(0xae);                                  // Display OFF
    cmd(0xa1); dat(0x00);                       // Set display start line
    cmd(0xa2); dat(0x00);                       // Set display offset
    cmd(0xa6);                                  // Normal display mode
    cmd(0xab); dat(0x01);                       // 8bit iface, 2.5V regulator ON
    cmd(0xb1); dat(0x32);                       // Precharge phase2=3 phase1=5
    cmd(0xb3); dat(0xf1);                       // Oscillator 0xf divide by 2
    cmd(0xb4); dat(0xa0); dat(0xb5); dat(0x55); // External VSL
    cmd(0xb6); dat(0x01);                       // Second precharge 1 DCLKS
    cmd(0xb8);                                  // Set gray table
    for(unsigned int i=0;i<sizeof(grayTable);i++) dat(grayTable[i]);
    cmd(0xbb); dat(0x17);                       // Precharge voltage ~0.5VCC
    cmd(0xbe); dat(0x05);                       // VCOMH
    cmd(0xc1); dat(0x88); dat(0x70); dat(0x88); // A B C brightness
    cmd(0xc7); dat(0x0f);                       // Master brightness
    cmd(0xca); dat(0x7f);                       // Duty 1:128
    clear(0);
    cmd(0xaf);                                  // Display ON
    setTextColor(make_pair(Color(0xffff),Color(0x0)));
}

void DisplayErOledm015::doTurnOn()
{
    cmd(0xaf);
}

void DisplayErOledm015::doTurnOff()
{
    cmd(0xae);
}

void DisplayErOledm015::doSetBrightness(int brt)
{
    cmd(0xc7); dat(max(0,min(15,brt/6)));
}

pair<short int, short int> DisplayErOledm015::doGetSize() const
{
    return make_pair(height,width);
}

void DisplayErOledm015::write(Point p, const char *text)
{
    font.draw(*this,textColor,p,text);
}

void DisplayErOledm015::clippedWrite(Point p, Point a, Point b, const char *text)
{
    font.clippedDraw(*this,textColor,p,a,b,text);
}

void DisplayErOledm015::clear(Color color)
{
    clear(Point(0,0),Point(width-1,height-1),color);
}

void DisplayErOledm015::clear(Point p1, Point p2, Color color)
{
    imageWindow(p1,p2);
    doBeginPixelWrite();
    int numPixels=(p2.x()-p1.x()+1)*(p2.y()-p1.y()+1);
    for(int i=0;i<numPixels;i++) doWritePixel(color);
    doEndPixelWrite();
}

void DisplayErOledm015::beginPixel() {}

void DisplayErOledm015::setPixel(Point p, Color color)
{
    //Can't move boilerplate to beginPixel, as can't do setCursor in between
    setCursor(p);
    doBeginPixelWrite();
    doWritePixel(color);
    doEndPixelWrite();
}

void DisplayErOledm015::line(Point a, Point b, Color color)
{
    //Horizontal line speed optimization
    if(a.y()==b.y())
    {
        imageWindow(Point(min(a.x(),b.x()),a.y()),
                    Point(max(a.x(),b.x()),a.y()));
        doBeginPixelWrite();
        int numPixels=abs(a.x()-b.x());
        for(int i=0;i<=numPixels;i++) doWritePixel(color);
        doEndPixelWrite();
        return;
    }
    //Vertical line speed optimization
    if(a.x()==b.x())
    {
        textWindow(Point(a.x(),min(a.y(),b.y())),
                    Point(a.x(),max(a.y(),b.y())));
        doBeginPixelWrite();
        int numPixels=abs(a.y()-b.y());
        for(int i=0;i<=numPixels;i++) doWritePixel(color);
        doEndPixelWrite();
        return;
    }
    //General case
    Line::draw(*this,a,b,color);
}

void DisplayErOledm015::scanLine(Point p, const Color *colors, unsigned short length)
{
    if(buffer2==nullptr) buffer2=new Color[buffer2Size];
    length=min<unsigned short>(length,width-p.x());
    imageWindow(p,Point(length-1,p.y()));
    cmd(0x5c);
    dc::high();
    cs::low();
    for(int i=0;i<length;i++) buffer2[i]=toBigEndian16(colors[i]);
    unsigned char *toSend = (unsigned char *)buffer2;
    spiController.send(toSend, 2*length, 8);
    cs::high();
    delayUs(1);
}

Color *DisplayErOledm015::getScanLineBuffer()
{
    //getWidth() would be enough as size, but we reuse the buffer for DMA
    if(buffer==nullptr) buffer=new Color[getWidth()];
    return buffer;
}

void DisplayErOledm015::scanLineBuffer(Point p, unsigned short length)
{
    scanLine(p,buffer,length);
}

void DisplayErOledm015::drawImage(Point p, const ImageBase& img)
{
    const Color *imgData=img.getData();
    if(imgData!=0)
    {
        if(buffer2==nullptr) buffer2=new Color[buffer2Size];
        short int xEnd=p.x()+img.getWidth()-1;
        short int yEnd=p.y()+img.getHeight()-1;
        imageWindow(p,Point(xEnd,yEnd));
        cmd(0x5c);
        dc::high();
        cs::low();
        //Unfortunately the DMA requires the endianness to be swapped, the
        //pointer we get is read-only (can be in flash), and we may not have
        //enough memory to allocate a large enough buffer to hold the entire
        //image, so we'll have to split it in chunks
        int imgSize=img.getHeight()*img.getWidth();
        unsigned char *toSend = (unsigned char *)buffer2;
        while(imgSize>0)
        {
            int chunkSize=min(imgSize,buffer2Size);
            for(int i=0;i<chunkSize;i++) buffer2[i]=toBigEndian16(imgData[i]);
            spiController.send(toSend, 2*chunkSize, 8);
            imgSize-=chunkSize;
            imgData+=chunkSize;
        }
        cs::high();
        delayUs(1);
    } else img.draw(*this,p);
}

void DisplayErOledm015::clippedDrawImage(Point p, Point a, Point b, const ImageBase& img)
{
    img.clippedDraw(*this,p,a,b);
}

void DisplayErOledm015::drawRectangle(Point a, Point b, Color c)
{
    line(a,Point(b.x(),a.y()),c);
    line(Point(b.x(),a.y()),b,c);
    line(b,Point(a.x(),b.y()),c);
    line(Point(a.x(),b.y()),a,c);
}

DisplayErOledm015::pixel_iterator DisplayErOledm015::begin(Point p1, Point p2,
        IteratorDirection d)
{
    if(p1.x()<0 || p1.y()<0 || p2.x()<0 || p2.y()<0) return pixel_iterator(0, this);
    if(p1.x()>=width || p1.y()>=height || p2.x()>=width || p2.y()>=height)
        return pixel_iterator();
    if(p2.x()<p1.x() || p2.y()<p1.y()) return pixel_iterator(0, this);
 
    if(d==DR) textWindow(p1,p2);
    else imageWindow(p1,p2);
    doBeginPixelWrite();

    unsigned int numPixels=(p2.x()-p1.x()+1)*(p2.y()-p1.y()+1);
    return pixel_iterator(numPixels, this);
}

DisplayErOledm015::~DisplayErOledm015() {}

void DisplayErOledm015::doBeginPixelWrite()
{
    cmd(0x5c);
    dc::high();
    cs::low();
}
    
void DisplayErOledm015::doWritePixel(Color c)
{
    uint16_t toSend = (c>>8 & 0x00FF)|(c<<8 & 0xFF00);
    unsigned char *temp = (unsigned char *)&toSend;
    spiController.send(temp, 2, 8);
}
    
void DisplayErOledm015::doEndPixelWrite()
{
    cs::high();
    delayUs(1);
}

} //namespace mxgui
