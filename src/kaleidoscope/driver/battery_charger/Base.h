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

#include <stdint.h>

namespace kaleidoscope {
namespace driver {
namespace battery_charger {

/** @defgroup kaleidoscope_driver_battery_charger Battery Charger driver APIs
 * @{
 */

/** @brief Base properties for battery charger drivers
 *
 * This struct defines the basic properties that all battery charger drivers
 * must implement. Device-specific drivers can extend this with additional
 * properties.
 */
struct BaseProps {
  /**
   * @brief Indicates whether the device has battery charger capabilities
   * 
   * This should be set to true for any device that implements battery charging,
   * and false for the None driver or devices without a battery charger.
   */
  static constexpr bool has_battery_charger = false;
};

/** @brief Base class for battery charger drivers
 *
 * This class defines the minimal interface that all battery charger drivers must implement.
 * It provides basic battery monitoring capabilities including voltage and charge level
 * readings.
 *
 * Device-specific implementations should inherit from this class and implement the
 * pure virtual methods according to their hardware capabilities.
 *
 * @tparam BatteryChargerProps Properties class defining the capabilities of the battery charger
 */
template<typename BatteryChargerProps>
class Base {
 public:
  /** @brief Constructor
   */
  Base() {}

  /** @brief Initialize the battery charger hardware
   *
   * This method should be called once during device setup to initialize the
   * battery charger hardware and prepare it for operation.
   */
  virtual void setup() {}

  /** @brief Begin battery charging
   *
   * This method should be called after setup() to start the actual battery
   * charging operations. It may perform additional initialization that
   * requires the device to be fully powered and ready.
   *
   * @return true if charging started successfully, false otherwise
   */
  virtual bool begin() {
    return true;
  }


 protected:
  typedef BatteryChargerProps Props_;
};

/** @} */

}  // namespace battery_charger
}  // namespace driver
}  // namespace kaleidoscope    