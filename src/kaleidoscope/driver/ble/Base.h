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

#pragma once

#include <Arduino.h>

namespace kaleidoscope {
namespace driver {
namespace ble {

class Base {
 public:
  Base() {}

  void setup() {}
  void startDiscoverableAdvertising() {}
  void startAdvConn() {}
  void stopAdv() {}
  bool connected() {
    return false;
  }
  Stream &serialPort() {
    return noserial_;
  }

 private:
  class NoSerial : public Stream {
    int available() {
      return 0;
    }
    int read() {
      return 0;
    }
    int peek() {
      return 0;
    }
    size_t write(uint8_t) {
      return 0;
    }
    void flush() {}
  };
  NoSerial noserial_;
};

}  // namespace ble
}  // namespace driver
}  // namespace kaleidoscope
