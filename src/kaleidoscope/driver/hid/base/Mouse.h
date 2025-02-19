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

#include <stdint.h>  // for int8_t, uint8_t

namespace kaleidoscope {
namespace driver {
namespace hid {
namespace base {

class NoMouse {
 public:
  NoMouse() {}

  void begin() {}
  void sendReport() {}
  void move(int8_t x, int8_t y, int8_t vWheel, int8_t hWheel) {}
  void stop(bool x, bool y, bool vWheel, bool hWheel) {}
  void releaseAll() {}
  void press(uint8_t buttons) {}
  void release(uint8_t buttons) {}
  void click(uint8_t buttons) {}
};

struct MouseProps {
  typedef NoMouse Mouse;
};

// Abstract interface base class to enable hybrid HID drivers
class MouseItf {
// Vtables are too big for AVR
#ifndef ARDUINO_ARCH_AVR
 public:
  virtual void setup()                                                        = 0;
  virtual void sendReport()                                                   = 0;
  virtual void move(int8_t x, int8_t y, int8_t vWheel = 0, int8_t hWheel = 0) = 0;
  /*
   * Unused in-tree in Kaleidoscope, but defined by some HID drivers
   */
  // virtual void stop(bool x, bool y, bool vWheel = false, bool hWheel = false) = 0;
  virtual void releaseAllButtons()             = 0;
  virtual void pressButtons(uint8_t buttons)   = 0;
  virtual void releaseButtons(uint8_t buttons) = 0;
  virtual void clickButtons(uint8_t buttons)   = 0;
#endif
};

template<typename _Props>
class Mouse : public MouseItf {
 private:
  typename _Props::Mouse mouse_;

 public:
  Mouse() {}

  void setup() {
    mouse_.begin();
  }

  void sendReport() {
    mouse_.sendReport();
  }

  void move(int8_t x, int8_t y, int8_t vWheel = 0, int8_t hWheel = 0) {
    mouse_.move(x, y, vWheel, hWheel);
  }
  void stop(bool x, bool y, bool vWheel = false, bool hWheel = false) {
    mouse_.stop(x, y, vWheel, hWheel);
  }
  void releaseAllButtons() {
    mouse_.releaseAll();
  }
  void pressButtons(uint8_t buttons) {
    mouse_.press(buttons);
  }
  void releaseButtons(uint8_t buttons) {
    mouse_.release(buttons);
  }
  void clickButtons(uint8_t buttons) {
    mouse_.click(buttons);
  }
};

}  // namespace base
}  // namespace hid
}  // namespace driver
}  // namespace kaleidoscope
