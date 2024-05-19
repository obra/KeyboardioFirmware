/* Kaleidoscope-Hardware-Keyboardio-Alto -- Keyboardio Alto hardware support for Kaleidoscope
 * Copyright 2017-2025 Keyboard.io, inc.
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

#ifdef ARDUINO_keyboardio_alto

#include <Arduino.h>

#define CRGB(r, g, b) \
  (cRGB) {            \
    b, g, r           \
  }

struct cRGB {
  uint8_t b;
  uint8_t g;
  uint8_t r;
};


#include "kaleidoscope/device/Base.h"
#include "kaleidoscope/driver/bootloader/gd32/Base.h"
#include "kaleidoscope/driver/hid/Base.h"
#include "kaleidoscope/driver/hid/Keyboardio.h"
#include "kaleidoscope/driver/keyscanner/Simple.h"
#include "kaleidoscope/driver/led/Base.h"
#include "kaleidoscope/driver/mcu/GD32.h"
#include "kaleidoscope/driver/storage/GD32Flash.h"


namespace kaleidoscope {
namespace device {
namespace keyboardio {

struct AltoStorageProps : public kaleidoscope::driver::storage::GD32FlashProps {
  static constexpr uint16_t length = 16384;
};

struct AltoKeyScannerProps : public kaleidoscope::driver::keyscanner::SimpleProps {

  static constexpr uint8_t matrix_rows    = 4;
  static constexpr uint8_t matrix_columns = 16;
  typedef MatrixAddr<matrix_rows, matrix_columns> KeyAddr;
#ifndef KALEIDOSCOPE_VIRTUAL_BUILD
  static constexpr uint8_t matrix_row_pins[matrix_rows]    = {PB15, PB14, PB13, PB12};
  static constexpr uint8_t matrix_col_pins[matrix_columns] = {PB11, PB10, PB9, PB8, PB7, PB6, PB5, PC15, PC14, PC13, PA0, PA1, PA2, PA3, PA4, PA5};

#endif  // ifndef KALEIDOSCOPE_VIRTUAL_BUILD
};


// If we need to override HID props:
struct AltoHIDProps : public kaleidoscope::driver::hid::KeyboardioProps {
  //typedef kaleidoscope::driver::hid::base::AbsoluteMouseProps AbsoluteMouseProps;
  //typedef kaleidoscope::driver::hid::base::AbsoluteMouse<AbsoluteMouseProps> AbsoluteMouse;
};


struct AltoProps : public kaleidoscope::device::BaseProps {
  typedef AltoHIDProps HIDProps;
  typedef kaleidoscope::driver::hid::Keyboardio<HIDProps> HID;

  typedef AltoKeyScannerProps KeyScannerProps;
  typedef kaleidoscope::driver::keyscanner::Simple<KeyScannerProps> KeyScanner;

  typedef AltoStorageProps StorageProps;
  typedef kaleidoscope::driver::storage::GD32Flash<StorageProps> Storage;

  typedef kaleidoscope::driver::bootloader::gd32::Base Bootloader;
  static constexpr const char *short_name = "alto";

  typedef kaleidoscope::driver::mcu::GD32Props MCUProps;
  typedef kaleidoscope::driver::mcu::GD32<MCUProps> MCU;
};

#ifndef KALEIDOSCOPE_VIRTUAL_BUILD

class Alto : public kaleidoscope::device::Base<AltoProps> {
 public:
  auto serialPort() -> decltype(Serial) & {
    return Serial;
  }
  void initSerial() {
    Serial.begin(9600);
  }
  static void rebootBootloader();
};

#endif  // ifndef KALEIDOSCOPE_VIRTUAL_BUILD

}  // namespace keyboardio
}  // namespace device

EXPORT_DEVICE(kaleidoscope::device::keyboardio::Alto)

}  // namespace kaleidoscope

// clang-format off


#define PER_KEY_DATA(dflt,                                                                                 \
   r0c0, r0c1, r0c2, r0c3, r0c4, r0c5, r0c6, r0c7, r0c8, r0c9, r0c10, r0c11, r0c12, r0c13, r0c14, r0c15, \
   r1c0, r1c1, r1c2, r1c3, r1c4, r1c5, r1c6, r1c7, r1c8, r1c9, r1c10, r1c11, r1c12, r1c13, r1c14, r1c15, \
   r2c0, r2c1, r2c2, r2c3, r2c4, r2c5, r2c6, r2c7, r2c8, r2c9, r2c10, r2c11, r2c12, r2c14,  \
   r3c0, r3c1, r3c2, r3c3, r3c4, r3c5, r3c6, r3c8, r3c9, r3c10, r3c11, r3c12, r3c13, r3c14, \
    						r3c7, ...)\
    r0c0, r0c1, r0c2, r0c3, r0c4, r0c5, r0c6, r0c7, r0c8, r0c9, r0c10, r0c11, r0c12, r0c13, r0c14, r0c15, \
    r1c0, r1c1, r1c2, r1c3, r1c4, r1c5, r1c6, r1c7, r1c8, r1c9, r1c10, r1c11, r1c12, r1c13, r1c14, r1c15, \
    r2c0, r2c1, r2c2, r2c3, r2c4, r2c5, r2c6, r2c7, r2c8, r2c9, r2c10, r2c11, r2c12, XXX, r2c14, XXX, \
    r3c0, r3c1, r3c2, r3c3, r3c4, r3c5, r3c6, r3c7, r3c8, r3c9, r3c10, r3c11, r3c12, r3c13, RESTRICT_ARGS_COUNT((r3c14), 61, KEYMAP, ##__VA_ARGS__)

#endif
