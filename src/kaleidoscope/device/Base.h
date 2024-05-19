/* -*- mode: c++ -*-
 * Kaleidoscope - Firmware for computer input devices
 * Copyright (C) 2017-2025 Keyboard.io, inc.
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

// IWYU pragma: private, include "kaleidoscope/device/device.h"

/** @file kaleidoscope/device/Base.h
 * Base class for Kaleidoscope device libraries.
 */

#pragma once

#include <stdint.h>  // for uint8_t, int8_t, uint32_t
#include <string.h>  // for size_t, strlen, memcpy

#include "kaleidoscope/driver/bootloader/None.h"  // for None
#include "kaleidoscope/driver/hid/Base.h"         // for Base, BaseProps
#include "kaleidoscope/driver/keyscanner/Base.h"  // for BaseProps
#include "kaleidoscope/driver/keyscanner/None.h"  // for None
#include "kaleidoscope/driver/led/None.h"         // for cRGB, BaseProps, CRGB, None
#include "kaleidoscope/driver/mcu/Base.h"         // for BaseProps
#include "kaleidoscope/driver/mcu/None.h"         // for None
#include "kaleidoscope/driver/storage/Base.h"     // for BaseProps
#include "kaleidoscope/driver/storage/None.h"     // for None
#include "kaleidoscope/driver/ble/None.h"         // for None

// Connection mode for host HID and Serial
enum HostConnectionMode {
  MODE_USB = 1,
  MODE_BLE = 2,
};

#ifndef CRGB
#error cRGB and CRGB *must* be defined before including this header!
#endif

/* All hardware libraries must define the following types and macros:
 * kaleidoscope::Device - a typedef to your device's class.
 * CRGB(r,g,b) - explained below
 * cRGB, a structure with at least three members: r, g, and b -
 * compilation will fail otherwise.
 *
 * Despite its name, the members do not need to be in the order r g b -- most
 * likely they will be in an order that is convenient for the hardware. So
 * initializing a cRGB with a struct literal will give surprising results for any
 * colors where r, g, and b do not have the same value. Each Hardware library
 * defines a CRGB(r,g,b) macro which returns a literal cRGB with the given values.
 */

namespace kaleidoscope {
namespace device {

struct BaseProps {
  typedef kaleidoscope::driver::hid::BaseProps HIDProps;
  typedef kaleidoscope::driver::hid::Base<HIDProps> HID;
  typedef kaleidoscope::driver::keyscanner::BaseProps KeyScannerProps;
  typedef kaleidoscope::driver::keyscanner::None KeyScanner;
  typedef kaleidoscope::driver::led::BaseProps LEDDriverProps;
  typedef kaleidoscope::driver::led::None LEDDriver;
  typedef kaleidoscope::driver::mcu::BaseProps MCUProps;
  typedef kaleidoscope::driver::mcu::None MCU;
  typedef kaleidoscope::driver::bootloader::None Bootloader;
  typedef kaleidoscope::driver::storage::BaseProps StorageProps;
  typedef kaleidoscope::driver::storage::None Storage;
  typedef kaleidoscope::driver::ble::None BLE;
  static constexpr const char *short_name = USB_PRODUCT;
  static constexpr const bool isHybridHostConnection    = false;
};

template<typename _DeviceProps>
class Base {
 private:
  class NoOpSerial {
   public:
    NoOpSerial() {}
    void begin(uint32_t baud) {}
    size_t print(...) {
      return 0;
    }
    size_t println(...) {
      return 0;
    }
    int peek() {
      return 0;
    }
    long parseInt() {  // NOLINT(runtime/int)
      return 0;
    }
    int available() {
      return 0;
    }
    int read() {
      return 0;
    }
  };

  NoOpSerial noop_serial_;

 public:
  Base()
    : host_connection_mode_(MODE_USB), primary_host_connection_mode_(MODE_USB) {}

  typedef _DeviceProps Props;

  typedef typename _DeviceProps::HIDProps HIDProps;
  typedef typename _DeviceProps::HID HID;
  typedef typename _DeviceProps::KeyScanner KeyScanner;
  typedef typename _DeviceProps::KeyScannerProps KeyScannerProps;
  typedef typename _DeviceProps::KeyScannerProps::KeyAddr KeyAddr;
  typedef typename _DeviceProps::LEDDriverProps LEDDriverProps;
  typedef typename _DeviceProps::LEDDriver LEDDriver;
  typedef typename _DeviceProps::MCUProps MCUProps;
  typedef typename _DeviceProps::MCU MCU;
  typedef typename _DeviceProps::Bootloader Bootloader;
  typedef typename _DeviceProps::StorageProps StorageProps;
  typedef typename _DeviceProps::Storage Storage;
  typedef typename _DeviceProps::BLE BLE;

  static constexpr uint8_t matrix_rows    = KeyScannerProps::matrix_rows;
  static constexpr uint8_t matrix_columns = KeyScannerProps::matrix_columns;
  static constexpr uint8_t led_count      = LEDDriverProps::led_count;
  static constexpr auto LEDs() -> decltype(LEDDriver::LEDs()) & {
    return LEDDriver::LEDs();
  }

  /**
   * @returns the number of keys on the keyboard.
   */
  static constexpr int8_t numKeys() {
    return matrix_columns * matrix_rows;
  }

  /**
  * @returns the number of LEDs on the keyboard.
  */
  static constexpr uint8_t numLEDs() {
    return led_count;
  }


  /**
   * Returns the HID driver used by the keyboard.
   */
  HID &hid() {
    return hid_;
  }

  /**
   * Returns the storage driver used by the keyboard.
   */
  Storage &storage() {
    return storage_;
  }

  /**
   * Returns the serial port driver used by the keyboard.
   */
  NoOpSerial &serialPort() {
    return noop_serial_;
  }

  /**
   * Returns the key scanner used by the keyboard.
   */
  KeyScanner &keyScanner() {
    return key_scanner_;
  }

  /**
   * Returns the LED driver
   */
  LEDDriver &ledDriver() {
    return led_driver_;
  }

  /**
   * Returns the BLE driver
   */
  BLE &ble() {
    return ble_;
  }

  /**
   * Returns the short name of the device.
   */
  static const uint8_t getShortName(char *name) {
    memcpy(name, _DeviceProps::short_name, strlen(_DeviceProps::short_name));
    return strlen(_DeviceProps::short_name);
  }

  /**
   * @defgroup kaleidoscope_hardware_leds Kaleidoscope::Hardware/LEDs
   * @{
   */
  /**
   * Sync the LEDs with the underlying hardware. This should make sure that
   * changes made before this call are reflected on the device.
   */
  void syncLeds(void) {
    led_driver_.syncLeds();
  }
  /**
   * Set the color of a per-key LED at a given row and column.
   *
   * Setting the color does not need to take effect immediately, it can be
   * delayed until @ref syncLeds is called.
   *
   * @param key_addr is the matrix address of the LED.
   * @param color is the color to set the LED to.
   */
  void setCrgbAt(KeyAddr key_addr, cRGB color) {
    setCrgbAt(getLedIndex(key_addr), color);
  }
  /**
   * Set the color of a per-key LED at a given LED index.
   *
   * Setting the color does not need to take effect immediately, it can be
   * delayed until @ref syncLeds is called.
   *
   * @param i is the LED index to change the color of.
   * @param color is the color to set it to.
   */
  void setCrgbAt(uint8_t i, cRGB color) {
    led_driver_.setCrgbAt(i, color);
  }
  /**
   * Returns the color of the LED at a given index.
   *
   * @param i is the index of the LED to return the color of.
   *
   * @returns The color at the given position.
   */
  cRGB getCrgbAt(uint8_t i) {
    return led_driver_.getCrgbAt(i);
  }
  /**
   * Returns the color of the LED at a given index.
   *
   * @param key_addr is the key address of the LED to return the color of.
   *
   * @returns The color at the given position.
   */
  cRGB getCrgbAt(KeyAddr key_addr) {
    return getCrgbAt(getLedIndex(key_addr));
  }
  /**
  * Returns the index of the LED at a given row & column.
  *
  * @param key_addr is the matrix address of the LED.
  *
  * @returns The index of the LED at the given position, or -1 if there are no
  * LEDs there.
  */
  int8_t getLedIndex(KeyAddr key_addr) {
    return led_driver_.getLedIndex(key_addr.toInt());
  }
  /** @} */

  /** @defgroup kaleidoscope_hardware_matrix Kaleidoscope::Hardware/Matrix
   * @{
   */
  /**
   * Scan the keyboard matrix, and act on it.
   */
  void scanMatrix(void) {
    key_scanner_.scanMatrix();
  }
  /**
   * Read the state of the keyboard matrix.
   *
   * Do whatever is necessary to read the current keyboard state - but without
   * acting on it.
   *
   * This is primarily used by @ref scanMatrix, but may have other uses too.
   */
  void readMatrix(void) {
    key_scanner_.readMatrix();
  }
  /**
   * Act on the scanned keyboard matrix.
   *
   * Iterate through the scanned state (@see readMatrix), and act on any events.
   */
  void actOnMatrixScan(void) {
    key_scanner_.actOnMatrixScan();
  }
  /** @} */

  /** @defgroup kaleidoscope_hardware_reattach Kaleidoscope::Hardware/Attach & Detach
   *
   * In situations where one wants to re-initialize the devices, perhaps to
   * change settings inbetween, detaching from and then attaching back to the
   * host is a desirable feature to have. Especially if this does not cut power,
   * nor reboot the device.
   *
   * Because different hardware has different ways to accomplish this, the
   * hardware plugin must provide these functions. Kaleidoscope will wrap them,
   * so user code does not have to deal with them directly.
   * @{
   */
  /**
   * Detach the device from the host.
   *
   * Must detach the device, without rebooting or cutting power. Only the end
   * points should get detached, the device must remain powered on.
   */
  void detachFromHost() {
    mcu_.detachFromHost();
  }
  /**
   * Attack the device to the host.
   *
   * Must restore the link detachFromHost severed.
   */
  void attachToHost() {
    mcu_.attachToHost();
  }
  /** @} */

  /**
   * Poll the USB device for a USB reset.
   *
   * @return true if a USB reset has occurred
   * @return false if a USB reset has not occurred
   */
  bool pollUSBReset() {
    return mcu_.pollUSBReset();
  }

  /**
   * Set the hook function to be called on USB reset
   *
   * @param hook function to call on USB reset
   */
  void setUSBResetHook(void (*hook)()) {
    mcu_.setUSBResetHook(hook);
  }

  /**
   * @defgroup kaleidoscope_hardware_keyswitch_state Kaleidoscope::Hardware/Key-switch state
   *
   * These methods offer a way to peek at the key switch states, for those cases
   * where we need to deal with the state closest to the hardware. Some methods
   * offer a way to check if a key is pressed, others return the number of
   * pressed keys.
   *
   * @{
   */
  /**
   * Check if a key is pressed at a given position.
   *
   * @param key_addr is the matrix address of the key.
   *
   * @returns true if the key is pressed, false otherwise.
   */
  bool isKeyswitchPressed(KeyAddr key_addr) {
    return key_scanner_.isKeyswitchPressed(key_addr);
  }
  /**
   * Check if a key is pressed at a given position.
   *
   * @param keyIndex is the key index, as calculated by `keyIndex`.
   *
   * @note Key indexes start at 1, not 0!
   *
   * @returns true if the key is pressed, false otherwise.
   */
  bool isKeyswitchPressed(uint8_t keyIndex) {
    return key_scanner_.isKeyswitchPressed(KeyAddr(--keyIndex));
  }
  /**
   * Check the number of key switches currently pressed.
   *
   * @returns the number of keys pressed.
   */
  uint8_t pressedKeyswitchCount() {
    return key_scanner_.pressedKeyswitchCount();
  }

  /**
   * Check if a key was pressed at a given position on the previous scan
   *
   * @param key_addr is the matrix address of the key.
   *
   * @returns true if the key was pressed, false otherwise.
   */
  bool wasKeyswitchPressed(KeyAddr key_addr) {
    return key_scanner_.wasKeyswitchPressed(key_addr);
  }
  /**
   * Check if a key was pressed at a given position on the previous scan.
   *
   * @param keyIndex is the key index, as calculated by `keyIndex`.
   *
   * @note Key indexes start at 1, not 0!
   *
   * @returns true if the key was pressed, false otherwise.
   */
  bool wasKeyswitchPressed(uint8_t keyIndex) {
    return key_scanner_.wasKeyswitchPressed(KeyAddr(--keyIndex));
  }

  /**
   * Check the number of key switches pressed in the previous scan.
   *
   * @returns the number of keys pressed.
   */
  uint8_t previousPressedKeyswitchCount() {
    return key_scanner_.previousPressedKeyswitchCount();
  }


  /** @} */

  /**
   * @defgroup kaleidoscope_hardware_misc Kaleidoscope::Hardware/Miscellaneous methods
   * @{
   */
  /**
   * Method to do any hardware-specific initialization.
   *
   * Called once when the device boots, this should initialize the device, and
   * bring it up into a useful state.
   */
  void setup() {
    bootloader_.setup();
    mcu_.setup();
    // BLE has to come before HID, because SoftDevice needs initialization
    ble_.setup();
    hid_.setup();
    storage_.setup();
    key_scanner_.setup();
    led_driver_.setup();
  }

  /**
   * Method to configure the device for a hardware test mode
   *
   * Called by the Kaleidoscope Hardware test plugin, this method should
   * do any device-specific initialization needed for factory hardware testing
   *
   */
  void enableHardwareTestMode() {}

  /**
   * Method to put the device into programmable/bootloader mode.
   */
  void rebootBootloader() {
    bootloader_.rebootBootloader();
  }

  /**
   * Return whether the device has a hybrid host connection
   */
  bool isHybridHostConnection() {
    return _DeviceProps::isHybridHostConnection;
  }

  /**
   * Set host connection mode
   */
  void setHostConnectionMode(uint8_t mode) {
    if (!isHybridHostConnection()) {
      return;
    }
    host_connection_mode_ = mode;
    hid_.setHostConnectionMode(mode);
  }

  /**
   * Get current host connection mode
   */
  uint8_t getHostConnectionMode() {
    return host_connection_mode_;
  }

  /**
   * Toggle host connection priority between USB and BLE
   */
  void toggleHostConnectionMode() {
    uint8_t old_mode = primary_host_connection_mode_;
    if (!isHybridHostConnection()) {
      return;
    }
    if (old_mode == MODE_USB) {
      primary_host_connection_mode_ = MODE_BLE;
    } else {
      primary_host_connection_mode_ = MODE_USB;
    }
#if CFG_DEBUG >= 2
    LOG_LV2("DEVICE", "primary_host_connection_mode_=%d", primary_host_connection_mode_);
#endif
  }

  /**
   * Automatically switch modes if hybrid
   *
   * Runtime calls this as part of the main loop.
   */
  void autoHostConnectionMode() {
    uint8_t old_mode = host_connection_mode_;
    if (!isHybridHostConnection()) {
      return;
    }
    if (host_connection_mode_ == MODE_BLE && mcu_.USBConfigured()) {
      if (!ble_.connected() || primary_host_connection_mode_ == MODE_USB) {
        setHostConnectionMode(MODE_USB);
      }
    } else if (host_connection_mode_ == MODE_USB && ble_.connected()) {
      if (!mcu_.USBConfigured() || primary_host_connection_mode_ == MODE_BLE) {
        setHostConnectionMode(MODE_BLE);
      }
    }
#if CFG_DEBUG >= 2
    if (old_mode != host_connection_mode_) {
      LOG_LV2("DEVICE", "autoHostConnectionMode: %d", host_connection_mode_);
    }
#endif
  }

  /**
   * Initialize the serial port
   *
   * The hybrid drivers need to use the only common base class of serial
   * devices, which is Stream. The begin() method to initialize the (emulated)
   * speed of the serial device isn't present in Stream, so each device
   * driver needs to do this in initSerial() instead.
   */
  void initSerial() {}

  /** @} */

 protected:
  HID hid_;
  KeyScanner key_scanner_;
  LEDDriver led_driver_;
  MCU mcu_;
  Bootloader bootloader_;
  Storage storage_;
  BLE ble_;
  uint8_t host_connection_mode_;
  uint8_t primary_host_connection_mode_;
};

}  // namespace device
}  // namespace kaleidoscope

// EXPORT_DEVICE exports a device type from a specific namespace to
// the 'kaleidoscope' namespace as type 'Device'. The corresponding
// properties type is also exported as 'DeviceProps'.
//
// Please note that this macro expects to find two types, one referenced
// by the macro argument DEVICE and another one, the properties class
// with the name of the type that is passed as DEVICE prefixed by 'Props',
// e.g. Model01 as device and Model01Props as properties class.
//
#ifndef KALEIDOSCOPE_VIRTUAL_BUILD
#define EXPORT_DEVICE(DEVICE)        \
  typedef DEVICE##Props DeviceProps; \
  typedef DEVICE Device;
#else  // ifndef KALEIDOSCOPE_VIRTUAL_BUILD
#define EXPORT_DEVICE(DEVICE) \
  typedef DEVICE##Props DeviceProps;
#endif  // ifndef KALEIDOSCOPE_VIRTUAL_BUILD
