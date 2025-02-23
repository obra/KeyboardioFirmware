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
namespace battery_gauge {

/** @defgroup kaleidoscope_driver_battery_gauge Battery Gauge driver APIs
 * @{
 */

/** @brief Base properties for battery gauge drivers
 *
 * This struct defines the basic properties that all battery gauge drivers
 * must implement. Device-specific drivers can extend this with additional
 * properties.
 */
struct BaseProps {
  /**
   * @brief Indicates whether the device has battery gauge capabilities
   * 
   * This should be set to true for any device that implements battery monitoring,
   * and false for the None driver or devices without a battery gauge.
   */
  static constexpr bool has_battery_gauge = false;
};

/** @brief Base class for battery gauge drivers
 *
 * This class defines the minimal interface that all battery gauge drivers must implement.
 * It provides basic battery monitoring capabilities including voltage and charge level
 * readings.
 *
 * Device-specific implementations should inherit from this class and implement the
 * pure virtual methods according to their hardware capabilities.
 *
 * @tparam BatteryGaugeProps Properties class defining the capabilities of the battery gauge
 */
template<typename BatteryGaugeProps>
class Base {
 public:
  /** @brief Constructor
   */
  Base() {}

  /** @brief Initialize the battery gauge hardware
   *
   * This method should be called once during device setup to initialize the
   * battery gauge hardware and prepare it for operation.
   */
  virtual void setup() {}

  /** @brief Begin battery monitoring
   *
   * This method should be called after setup() to start the actual battery
   * monitoring operations. It may perform additional initialization that
   * requires the device to be fully powered and ready.
   *
   * @return true if monitoring started successfully, false otherwise
   */
  virtual bool begin() {
    return true;
  }

  /** @brief Get the current battery voltage
   *
   * @return Battery voltage in millivolts (mV)
   */
  virtual uint16_t getVoltage() const {
    return 0;
  }

  /** @brief Get the current battery level
   *
   * Returns the battery's current charge level as a percentage.
   * The value is based on the battery gauge's internal calculations
   * which may take into account factors like voltage, temperature,
   * and battery chemistry.
   *
   * @return Battery level as a percentage (0-100)
   */
  virtual uint8_t getBatteryLevel() const {
    return 0;
  }

 protected:
  typedef BatteryGaugeProps Props_;
};

/** @} */

}  // namespace battery_gauge
}  // namespace driver
}  // namespace kaleidoscope