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

#pragma once

#include <miosix.h>

namespace miosix {

//Display
using oled_sck    = Gpio<P0,6>;  //SPI0
using oled_mosi   = Gpio<P0,7>;  //SPI0
using oled_cs     = Gpio<P0,9>;
using oled_dc     = Gpio<P0,4>;
using oled_res    = NullGpio;

//Sensor
using sen_scl     = Gpio<P0,3>;  //I2C1
using sen_sda     = Gpio<P0,2>;  //I2C1

//Buttons and power management
using up_btn      = Gpio<P0,19>;
using on_btn      = Gpio<P0,18>;
using keep_on     = NullGpio;
using vbatt_sense = NullGpio;

//USB
/*
using usb_dm      = Gpio<GPIOA_BASE,11>; //OTG_FS_DM
using usb_dp      = Gpio<GPIOA_BASE,12>; //OTG_FS_DP
using usb_dm_2    = Gpio<GPIOC_BASE,8>;
using usb_dp_2    = Gpio<GPIOC_BASE,7>;
*/

//FLASH
/*
using flash_cs    = Gpio<GPIOB_BASE,12>; //SPI2
using flash_sck   = Gpio<GPIOB_BASE,13>; //SPI2
using flash_miso  = Gpio<GPIOB_BASE,14>; //SPI2
using flash_mosi  = Gpio<GPIOB_BASE,15>; //SPI2
*/

//Serial
//using boot_tx     = Gpio<GPIOA_BASE,9>;  //USART1
//using boot_rx     = Gpio<GPIOA_BASE,10>; //USART1

} //namespace miosix
