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

#include "kaleidoscope/driver/battery_charger/Base.h"

namespace kaleidoscope {
namespace driver {
namespace battery_charger {
/** @brief Properties for the None (null) battery charger driver
 *
 * This struct defines properties for devices that do not have battery charging
 * capabilities. It sets has_battery_charger to false to indicate no battery
 * charger is available.
 */
struct NoneProps : public BaseProps {
  static constexpr bool has_battery_charger = false;
};

/** @brief Null battery charger driver
 *
 * This class implements a null battery charger driver for devices that do not
 * have battery charging capabilities. All methods return false indicating no
 * battery is being charged.
 *
 * This driver is used as the default for devices without battery charging,
 * ensuring a consistent API even when battery charging is not available.
 */
class None : public Base<NoneProps> {
 public:
  /**
   * @brief Constructor
   */
  None() {}


};

}  // namespace battery_charger
}  // namespace driver
}  // namespace kaleidoscope