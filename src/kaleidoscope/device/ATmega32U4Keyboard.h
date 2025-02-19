/* Kaleidoscope - Firmware for computer input devices
 * Copyright (C) 2019-2025 Keyboard.io, inc.
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

#pragma once

#if defined(__AVR__) || defined(KALEIDOSCOPE_VIRTUAL_BUILD)

// IWYU pragma: begin_exports
#include "kaleidoscope/device/Base.h"                           // for BaseProps
#include "kaleidoscope/driver/hid/Keyboardio.h"                 // for Keyboardio, KeyboardioProps
#include "kaleidoscope/driver/mcu/ATmega32U4.h"                 // for ATmega32U4, ATmega32U4Props
#include "kaleidoscope/driver/storage/ATmega32U4EEPROMProps.h"  // for ATmega32U4EEPROMProps
#include "kaleidoscope/driver/storage/AVREEPROM.h"              // for AVREEPROM
// IWYU pragma: end_exports

namespace kaleidoscope {
namespace device {

struct ATmega32U4KeyboardProps : kaleidoscope::device::BaseProps {
  typedef kaleidoscope::driver::hid::KeyboardioProps HIDProps;
  typedef kaleidoscope::driver::hid::Keyboardio<HIDProps> HID;
  typedef kaleidoscope::driver::mcu::ATmega32U4Props MCUProps;
  typedef kaleidoscope::driver::mcu::ATmega32U4<MCUProps> MCU;
  typedef kaleidoscope::driver::storage::ATmega32U4EEPROMProps StorageProps;
  typedef kaleidoscope::driver::storage::AVREEPROM<StorageProps> Storage;
};

#ifndef KALEIDOSCOPE_VIRTUAL_BUILD
template<typename _DeviceProps>
class ATmega32U4Keyboard : public kaleidoscope::device::Base<_DeviceProps> {
 public:
  auto serialPort() -> decltype(Serial) & {
    return Serial;
  }
  void initSerial() {
    Serial.begin(9600);
  }
};
#else   // ifndef KALEIDOSCOPE_VIRTUAL_BUILD
#endif  // ifndef KALEIDOSCOPE_VIRTUAL_BUILD

}  // namespace device
}  // namespace kaleidoscope

#endif
