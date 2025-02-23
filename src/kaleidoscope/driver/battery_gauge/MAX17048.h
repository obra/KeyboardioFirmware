/* Kaleidoscope - Firmware for computer input devices
 * Copyright (C) 2025-2025 Keyboard.io, inc.
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

#include <Wire.h>
#include "kaleidoscope/driver/battery_gauge/Base.h"
#include "SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h"
#include "kaleidoscope/trace.h"

namespace kaleidoscope {
namespace driver {
namespace battery_gauge {

/** @brief Properties for the MAX17048 battery gauge driver
 *
 * This struct defines properties for the MAX17048 battery gauge IC.
 * It extends BaseProps with additional capabilities specific to the MAX17048.
 */
struct MAX17048Props : public BaseProps {
  static constexpr bool has_battery_gauge  = true;
  static constexpr bool has_voltage_alerts = true;
  static constexpr bool has_charge_rate    = true;
  static constexpr bool has_hibernate      = true;

  // Battery configuration
  static constexpr uint16_t battery_voltage_min    = 3300;  // 3.3V minimum
  static constexpr uint16_t battery_voltage_max    = 4200;  // 4.2V maximum for LiPo
  static constexpr uint8_t battery_alert_threshold = 15;    // Alert at 15% by default
};

/** @brief MAX17048 battery gauge driver
 *
 * This class implements a driver for the MAX17048 battery fuel gauge IC.
 * It provides battery voltage monitoring, state of charge estimation,
 * and various power management features.
 *
 * @tparam _Props Properties class defining the capabilities of the battery gauge
 */
template<typename _Props>
class MAX17048 : public Base<_Props> {
 private:
  mutable SFE_MAX1704X lipo_{MAX1704X_MAX17048};  // SparkFun MAX17048 library instance, mutable to allow use in const methods
  bool initialized_{false};  // Track whether initialization succeeded
  uint8_t last_battery_level_{0};  // Last reported battery level

 public:
  /**
   * @brief Constructor
   */
  MAX17048() {}

  /**
   * @brief Initialize the MAX17048 hardware
   *
   * Sets up the I2C communication and configures the chip for operation.
   */
  void setup() override {
    // Initialize I2C
    DEBUG_TRACE("MAX17048", "Initializing I2C");
    Wire.begin();

    DEBUG_TRACE("MAX17048", "I2C initialized");
    // Initialize and verify communication with the MAX17048
    if (!lipo_.begin(Wire)) {
      DEBUG_TRACE("MAX17048", "Failed to initialize MAX17048");
      initialized_ = false;
      return;  // Failed to initialize
    }
    initialized_ = true;
    DEBUG_TRACE("MAX17048", "MAX17048 initialized");

    // Configure alert pin
    pinMode(_Props::alert_pin, INPUT_PULLUP);

    // Configure alert thresholds and enable alerts
    lipo_.setThreshold(_Props::battery_alert_threshold);
    lipo_.enableSOCAlert();  // Enable 1% change detection
    clearAlert();      // Clear any pending alerts

    // Configure voltage alerts for battery protection
    setVoltageAlertThresholds(_Props::battery_voltage_min, _Props::battery_voltage_max);


    DEBUG_TRACE("MAX17048", "Alerts enabled");
  }

  /**
   * @brief Begin battery monitoring
   *
   * Initializes the MAX17048 and verifies communication.
   * Also performs initial configuration of alert thresholds
   * and power management settings.
   *
   * @return true if the device was initialized successfully, false otherwise
   */
  bool begin() override {
    return initialized_;  // Return initialization state
  }

  /**
   * @brief Get the current battery voltage
   *
   * @return Battery voltage in millivolts (mV), returns 0 if uninitialized
   */
  uint16_t getVoltage() const override {
    if (!initialized_) return 0;
    return static_cast<uint16_t>(lipo_.getVoltage() * 1000);
  }

  /**
   * @brief Get the current battery level
   *
   * Returns the battery's state of charge as calculated by
   * the MAX17048's ModelGauge algorithm.
   *
   * @return Battery level as a percentage (0-100), returns 0 if uninitialized
   */
  uint8_t getBatteryLevel() const override {
    if (!initialized_) return 0;
    return static_cast<uint8_t>(lipo_.getSOC());
  }

  /**
   * @brief Get the current charge rate
   *
   * Returns the rate of change of the battery's state of charge.
   * Positive values indicate charging, negative values indicate discharging.
   *
   * @return Charge rate in 0.208%/hr units, returns 0 if uninitialized
   */
  int16_t getChargeRate() const {
    if (!initialized_) return 0;
    // The CRATE register LSB is 0.208%/hr
    // We want raw units, so no conversion needed
    return static_cast<int16_t>(lipo_.getChangeRate() / 0.208f);  // TODO: Replace with direct register read
  }

  /**
   * @brief Restart the fuel gauge calculations
   *
   * Forces a restart of the ModelGauge algorithm. This can be useful
   * after a full charge cycle or if the battery has been replaced.
   *
   * @return true if restart successful, false if uninitialized or restart failed
   */
  bool quickStart() {
    if (!initialized_) return false;
    return (lipo_.quickStart() == 0);
  }

  /**
   * @brief Put the device into sleep mode
   *
   * Halts all operations except alert monitoring.
   * Power consumption drops to <1μA.
   *
   * @return true if sleep mode entered successfully, false if uninitialized or sleep failed
   */
  bool sleep() {
    if (!initialized_) return false;
    return (lipo_.sleep() == 0);
  }

  /**
   * @brief Wake the device from sleep mode
   *
   * Resumes normal operation after sleep.
   *
   * @return true if wake successful, false if uninitialized or wake failed
   */
  bool wake() {
    if (!initialized_) return false;
    return (lipo_.wake() == 0);
  }

  /**
   * @brief Set the battery level alert threshold
   *
   * Configures the threshold at which a low battery alert will be triggered.
   *
   * @param percentage Alert threshold (1-32%)
   */
  void setBatteryAlertThreshold(uint8_t percentage) {
    // Constrain to valid range (1-32%)
    percentage = (percentage < 1) ? 1 : (percentage > 32) ? 32
                                                          : percentage;
    lipo_.setThreshold(percentage);
  }

  /**
   * @brief Get the current battery alert threshold
   *
   * @return uint8_t Current alert threshold (1-32%)
   */
  uint8_t getBatteryAlertThreshold() {
    return lipo_.getThreshold();
  }

  /**
   * @brief Set voltage alert thresholds
   *
   * Configures the voltage range for alerts. An alert will be
   * triggered if the battery voltage goes outside this range.
   * Has no effect if device is uninitialized.
   *
   * @param min_mv Minimum voltage threshold in millivolts
   * @param max_mv Maximum voltage threshold in millivolts
   */
  void setVoltageAlertThresholds(uint16_t min_mv, uint16_t max_mv) {
    if (!initialized_) return;
    // VALRT register uses 20mV per LSB
    uint8_t min_bits = min_mv / 20;
    uint8_t max_bits = max_mv / 20;
    lipo_.setVALRTMin(min_bits);
    lipo_.setVALRTMax(max_bits);
  }

  /**
   * @brief Get the minimum voltage alert threshold
   *
   * @return uint16_t Minimum voltage threshold in millivolts, returns 0 if uninitialized
   */
  uint16_t getVoltageAlertMin() {
    if (!initialized_) return 0;
    // VALRT register uses 20mV per LSB
    return lipo_.getVALRTMin() * 20;
  }

  /**
   * @brief Get the maximum voltage alert threshold
   *
   * @return uint16_t Maximum voltage threshold in millivolts, returns 0 if uninitialized
   */
  uint16_t getVoltageAlertMax() {
    if (!initialized_) return 0;
    // VALRT register uses 20mV per LSB
    return lipo_.getVALRTMax() * 20;
  }

  /**
   * @brief Check if there is a pending alert from the MAX17048
   * 
   * This checks the physical ALERT pin first, then reads the alert status
   * register if needed. This avoids unnecessary I2C transactions.
   * 
   * @return true if there is a pending alert
   */
  bool hasAlert() const {
    if (!initialized_) return false;

    // Check ALERT pin first - if it's high, there's definitely no alert
    if (digitalRead(_Props::alert_pin) == HIGH)
      return false;

    // ALERT pin is low, we got an alert;

    return true;
  }

  /**
   * @brief Check if battery level is below threshold
   *
   * @return bool true if battery level is below the configured threshold, false if uninitialized
   */
  bool hasLowBatteryAlert() {
    if (!initialized_) return false;
    return lipo_.isLow();
  }

  /**
   * @brief Check if battery level has changed significantly
   *
   * @return bool true if battery level has changed by 1% or more, false if uninitialized
   */
  bool hasChangeAlert() {
    if (!initialized_) return false;
    return lipo_.isChange();
  }

  /**
   * @brief Check if voltage is below minimum threshold
   *
   * @return bool true if battery voltage is below minimum threshold, false if uninitialized
   */
  bool hasLowVoltageAlert() {
    if (!initialized_) return false;
    return lipo_.isVoltageLow();
  }

  /**
   * @brief Check if voltage is above maximum threshold
   *
   * @return bool true if battery voltage is above maximum threshold, false if uninitialized
   */
  bool hasHighVoltageAlert() {
    if (!initialized_) return false;
    return lipo_.isVoltageHigh();
  }

  /**
   * @brief Clear all alert flags
   * Has no effect if device is uninitialized.
   */
  void clearAlert() {
    if (!initialized_) return;
    lipo_.clearAlert();
  }

  /**
   * @brief Enable battery level change alerts
   * Has no effect if device is uninitialized.
   */
  void enableChangeAlert() {
    if (!initialized_) return;
    lipo_.enableSOCAlert();
  }

  /**
   * @brief Disable battery level change alerts
   * Has no effect if device is uninitialized.
   */
  void disableChangeAlert() {
    if (!initialized_) return;
    lipo_.disableSOCAlert();
  }

  /**
   * @brief Check if device is in hibernate mode
   *
   * @return bool true if the device is hibernating, false if uninitialized
   */
  bool isHibernating() {
    if (!initialized_) return false;
    return lipo_.isHibernating();
  }

  /**
   * @brief Enable hibernate mode
   *
   * In hibernate mode, the device will enter a low power state
   * when battery activity is below the hibernate threshold.
   * Has no effect if device is uninitialized.
   */
  void enableHibernate() {
    if (!initialized_) return;
    lipo_.enableHibernate();
  }

  /**
   * @brief Disable hibernate mode
   * Has no effect if device is uninitialized.
   */
  void disableHibernate() {
    if (!initialized_) return;
    lipo_.disableHibernate();
  }

  /**
   * @brief Set hibernate mode thresholds
   *
   * Configures the activity thresholds for entering and exiting
   * hibernate mode. Has no effect if device is uninitialized.
   *
   * @param active_rate Activity threshold to exit hibernate (0.208%/hr units)
   * @param hibernate_rate Inactivity threshold to enter hibernate (0.208%/hr units)
   */
  void setHibernateThresholds(uint8_t active_rate, uint8_t hibernate_rate) {
    if (!initialized_) return;
    lipo_.setHIBRTActThr(active_rate);
    lipo_.setHIBRTHibThr(hibernate_rate);
  }

  /**
   * @brief Set the battery reset voltage threshold
   *
   * Sets the voltage threshold for detecting battery removal/insertion.
   * Has no effect if device is uninitialized.
   *
   * @param mv Reset voltage threshold in millivolts
   */
  void setResetVoltage(uint16_t mv) {
    if (!initialized_) return;
    // Reset voltage register uses 40mV per LSB
    uint8_t bits = mv / 40;
    lipo_.setResetVoltage(bits);  // TODO: Replace with direct register write
  }

  /**
   * @brief Enable the voltage comparator
   *
   * The comparator is used for battery presence detection.
   * Enabling it increases power consumption by 0.5μA.
   * Has no effect if device is uninitialized.
   */
  void enableComparator() {
    if (!initialized_) return;
    lipo_.enableComparator();
  }

  /**
   * @brief Disable the voltage comparator
   *
   * Disabling the comparator saves 0.5μA but prevents
   * battery presence detection.
   * Has no effect if device is uninitialized.
   */
  void disableComparator() {
    if (!initialized_) return;
    lipo_.disableComparator();
  }

  /**
   * @brief Get the device ID
   *
   * @return uint8_t Device ID as programmed at factory, returns 0 if uninitialized
   */
  uint8_t getDeviceID() {
    if (!initialized_) return 0;
    return lipo_.getID();
  }

  /**
   * @brief Get the device version
   *
   * @return uint16_t Device version number, returns 0 if uninitialized
   */
  uint16_t getVersion() {
    if (!initialized_) return 0;
    return lipo_.getVersion();
  }
};

}  // namespace battery_gauge
}  // namespace driver
}  // namespace kaleidoscope
