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

#include <stdint.h>  // for uint8_t

#ifndef CRGB

struct cRGB {
  uint8_t r, g, b;
};

#define CRGB(r, g, b) \
  (cRGB) {            \
    b, g, r           \
  }

#endif

#include "kaleidoscope/driver/led/Base.h"  // for Base, BaseProps

namespace kaleidoscope {
namespace driver {
namespace led {

/*
 * The purpose of this class is to serve as a default inside the base
 * `kaleidoscope::device::Base` class, with a name more descriptive than
 * `leddriver::Base`. In practice, one shouldn't use it, and should override the
 * driver in the device description.
 */
class None : public kaleidoscope::driver::led::Base<BaseProps> {};

}  // namespace led
}  // namespace driver
}  // namespace kaleidoscope
