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

#include "kaleidoscope/driver/hid/Base.h"  // for Base, BaseProps
#include "kaleidoscope/driver/hid/bluefruit/AbsoluteMouse.h"
#include "kaleidoscope/driver/hid/bluefruit/Keyboard.h"
#include "kaleidoscope/driver/hid/bluefruit/Mouse.h"

#ifdef ARDUINO_ARCH_NRF52

namespace kaleidoscope {
namespace driver {
namespace hid {

struct BluefruitProps : public BaseProps {
  typedef bluefruit::KeyboardProps KeyboardProps;
  typedef bluefruit::Keyboard<KeyboardProps> Keyboard;
  typedef bluefruit::MouseProps MouseProps;
  typedef bluefruit::Mouse<MouseProps> Mouse;
  typedef bluefruit::AbsoluteMouseProps AbsoluteMouseProps;
  typedef bluefruit::AbsoluteMouse<AbsoluteMouseProps> AbsoluteMouse;
};

template<typename _Props>
class Bluefruit : public Base<_Props> {};

}  // namespace hid
}  // namespace driver
}  // namespace kaleidoscope

#endif /* ARDUINO_ARCH_NRF52 */
