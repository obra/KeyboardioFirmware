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

#include "kaleidoscope/driver/battery_gauge/Base.h"

namespace kaleidoscope {
namespace driver {
namespace battery_gauge {

/** @brief Properties for the None (null) battery gauge driver
 *
 * This struct defines properties for devices that do not have battery monitoring
 * capabilities. It sets has_battery_gauge to false to indicate no battery gauge
 * is available.
 */
struct NoneProps : public BaseProps {
  static constexpr bool has_battery_gauge = false;
};

/** @brief Null battery gauge driver
 *
 * This class implements a null battery gauge driver for devices that do not
 * have battery monitoring capabilities. All methods return default values
 * indicating no battery is present.
 *
 * This driver is used as the default for devices without battery monitoring,
 * ensuring a consistent API even when battery monitoring is not available.
 */
class None : public Base<NoneProps> {
 public:
  /**
   * @brief Constructor
   */
  None() {}

  /**
   * @brief Initialize the null battery gauge
   *
   * This is a no-op for the null driver.
   */
  void setup() override {}

  /**
   * @brief Begin battery monitoring
   *
   * This is a no-op for the null driver.
   *
   * @return false Always returns false to indicate no battery monitoring is available
   */
  bool begin() override { return false; }

  /**
   * @brief Get the current battery voltage
   *
   * @return uint16_t Always returns 0 to indicate no battery is present
   */
  uint16_t getVoltage() const override { return 0; }

  /**
   * @brief Get the current battery level
   *
   * @return uint8_t Always returns 0 to indicate no battery is present
   */
  uint8_t getBatteryLevel() const override { return 0; }
};

}  // namespace battery_gauge
}  // namespace driver
}  // namespace kaleidoscope