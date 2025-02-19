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

#include <stdint.h>  // for uint8_t, int8_t, uint16_t

namespace kaleidoscope {
namespace driver {
namespace hid {
namespace base {

class NoAbsoluteMouse {
 public:
  NoAbsoluteMouse() {}

  void begin() {}
  void move(int8_t x, int8_t y, int8_t wheel) {}
  void moveTo(uint16_t x, uint16_t y, uint8_t wheel) {}

  void click(uint8_t buttons) {}
  void press(uint8_t buttons) {}
  void release(uint8_t buttons) {}
};

struct AbsoluteMouseProps {
  typedef NoAbsoluteMouse AbsoluteMouse;
};

// Abstract interface base class to enable hybrid HID drivers
class AbsoluteMouseItf {
// Vtables are too big for AVR
#ifndef ARDUINO_ARCH_AVR
 public:
  virtual void setup()                                       = 0;
  virtual void move(int8_t x, int8_t y, int8_t wheel)        = 0;
  virtual void moveTo(uint16_t x, uint16_t y, uint8_t wheel) = 0;
  virtual void clickButtons(uint8_t buttons)                 = 0;
  virtual void pressButtons(uint8_t buttons)                 = 0;
  virtual void releaseButtons(uint8_t buttons)               = 0;
#endif
};

template<typename _Props>
class AbsoluteMouse : public AbsoluteMouseItf {
 private:
  typename _Props::AbsoluteMouse absolute_mouse_;

 public:
  AbsoluteMouse() {}

  void setup() {
    absolute_mouse_.begin();
  }

  void move(int8_t x, int8_t y, int8_t wheel) {
    absolute_mouse_.move(x, y, wheel);
  }
  void moveTo(uint16_t x, uint16_t y, uint8_t wheel) {
    absolute_mouse_.moveTo(x, y, wheel);
  }

  void clickButtons(uint8_t buttons) {
    absolute_mouse_.click(buttons);
  }
  void pressButtons(uint8_t buttons) {
    absolute_mouse_.press(buttons);
  }
  void releaseButtons(uint8_t buttons) {
    absolute_mouse_.release(buttons);
  }
};

}  // namespace base
}  // namespace hid
}  // namespace driver
}  // namespace kaleidoscope
