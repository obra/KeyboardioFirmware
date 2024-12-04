/* Kaleidoscope - Firmware for computer input devices
 * Copyright (C) 2013-2025 Keyboard.io, inc.
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

#include "kaleidoscope/driver/hid/Base.h"
#include "Bluefruit.h"
#include "TinyUSB.h"

namespace kaleidoscope {
namespace driver {
namespace hid {

struct HybridProps {
  typedef TinyUSBProps USBProps;
  typedef TinyUSB<TinyUSBProps> USB;
  typedef BluefruitProps BLEProps;
  typedef Bluefruit<BLEProps> BLE;
};

template<typename _Props>
class Hybrid {
 private:
  typename _Props::USB hidusb;
  typename _Props::BLE hidble;
  uint8_t host_connection_mode_;

 public:
  Hybrid()
    : host_connection_mode_(MODE_USB) {}

  void setup() {
    hidusb.setup();
    hidble.setup();
  }

  void onUSBReset() {
    hidusb.keyboard().onUSBReset();
  }

  base::KeyboardItf &keyboard() {
    if (host_connection_mode_ == MODE_USB) {
      return hidusb.keyboard();
    } else {
      return hidble.keyboard();
    }
  }

  base::MouseItf &mouse() {
    if (host_connection_mode_ == MODE_USB) {
      return hidusb.mouse();
    } else {
      return hidble.mouse();
    }
  }

  base::AbsoluteMouseItf &absoluteMouse() {
    if (host_connection_mode_ == MODE_USB) {
      return hidusb.absoluteMouse();
    } else {
      return hidble.absoluteMouse();
    }
  }

  void setHostConnectionMode(uint8_t mode) {
    host_connection_mode_ = mode;
    LOG_LV2("HYBRID", "host_connection_mode_=%d", host_connection_mode_);
  }
};

}  // namespace hid
}  // namespace driver
}  // namespace kaleidoscope
