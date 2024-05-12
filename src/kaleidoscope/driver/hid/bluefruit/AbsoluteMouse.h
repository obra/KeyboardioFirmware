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

#ifdef ARDUINO_ARCH_NRF52

#include <bluefruit.h>

#include "kaleidoscope/driver/hid/apis/AbsoluteMouseAPI.h"
#include "kaleidoscope/driver/hid/bluefruit/HIDD.h"

namespace kaleidoscope {
namespace driver {
namespace hid {
namespace bluefruit {

class AbsoluteMouse_ : public AbsoluteMouseAPI {
 public:
  void sendReport(void *data, int length) override {
    if (blehid.isBootMode()) {
      return;
    }
    blehid.inputReport(RID_ABS_MOUSE, data, length);
  }
};

struct AbsoluteMouseProps {
  typedef AbsoluteMouse_ AbsoluteMouse;
};

template<typename _Props>
class AbsoluteMouse : public base::AbsoluteMouse<_Props> {};

}  // namespace bluefruit
}  // namespace hid
}  // namespace driver
}  // namespace kaleidoscope
#endif /* ARDUINO_ARCH_NRF52 */
