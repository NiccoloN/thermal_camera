/***************************************************************************
 *   Copyright (C) 2022 by Terraneo Federico                               *
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

#include <cstdio>
#include "drivers/mlx90640.h"
#include "drivers/rp2040_i2c.h"
#include "miosix.h"
#include <drivers/misc.h>
#include <application.h>
#include <mxgui/display.h>
#include <drivers/display_er_oledm015.h>
#include <e20/e20.h>
#include <thread>
#include <drivers/hwmapping.h>
#include <drivers/mlx90640.h>
#include <memory>
#include <miosix.h>

using namespace std;
using namespace mxgui;
using namespace miosix;

namespace mxgui {
void registerDisplayHook(DisplayManager& dm)
{
    dm.registerDisplay(new DisplayErOledm015);
}
} //namespace mxgui

#ifdef WITH_CPU_TIME_COUNTER
void *profilerMain(void *)
{
    CPUProfiler::thread(5000000000LL);
    return nullptr;
}
#endif

FixedEventQueue<20> eq;

int main()
{
    //thread t([]{ eq.run(); });
    //t.detach();
    iprintf("iprintf-o, ergo sum!\n");

    /*
    RP2040I2C1Master i2c(sen_sda::getPin(), sen_scl::getPin(), 400);

    //Thread::sleep(10);

    char data[6] = "Dario";
    //i2c.stop();
    //i2c.send(0x33, data, 1, false);
    //i2c.send(0x33, &data[1], 1, false);
    //i2c.send(0x33, data, 1, true);
    i2c.send(0x33, data, 5, true);

    for(;;){
        iprintf("End!\n");
        Thread::sleep(1000);
    }
    //--------------------------
    */

    initializeBoard();
    
    #ifdef WITH_CPU_TIME_COUNTER
    Thread *profiler = Thread::create(profilerMain, 2048U, Priority(0), nullptr, Thread::DETACHED);
    #endif

    auto& display=DisplayManager::instance().getDisplay();
    
    try {
        Application app(display);
        app.run();
    } catch(exception& e) {
        iprintf("exception: %s\n",e.what());
    }
    
    display.turnOff();
    #ifdef WITH_CPU_TIME_COUNTER
    profiler->terminate();
    #endif
    shutdownBoard();
}
