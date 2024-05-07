/* Kaleidoscope - Firmware for computer input devices
 * Copyright (C) 2024-2025 Keyboard.io, inc.
 *
 *
 * This program is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, version 3.
 *
 * Additional Permissions:
 * As an additional permission under Section 7 of the GNU General Public
 * License Version 3, you may link this software against a Vendor-provided
 * Hardware Specific Software Module under the terms of the MCU Vendor
 * Firmware Library Additional Permission Version 1.0.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include "kaleidoscope/driver/ble/Bluefruit.h"  // for Base

namespace kaleidoscope {
namespace driver {
namespace ble {

#ifdef ARDUINO_ARCH_NRF52
bool BLEBluefruit::adv_started = false;
bool BLEBluefruit::bonded      = false;
BLEDis BLEBluefruit::bledis;
BLEBas BLEBluefruit::blebas;
BLEUart BLEBluefruit::bleuart;
// Move this to HID driver later
BLEHidAdafruit BLEBluefruit::blehid;
#endif

}  // namespace ble
}  // namespace driver
}  // namespace kaleidoscope
