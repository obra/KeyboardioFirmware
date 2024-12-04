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

#ifdef ARDUINO_ARCH_NRF52

#include <bluefruit.h>

#include "kaleidoscope/driver/ble/Base.h"  // for Base
#include "kaleidoscope/driver/hid/bluefruit/HIDD.h"
#include <nordic/softdevice/s140_nrf52_6.1.1_API/include/ble_gap.h>  // for ble_gap_addr_t
#include <nordic/softdevice/s140_nrf52_6.1.1_API/include/nrf_error.h>
#include <nordic/softdevice/s140_nrf52_6.1.1_API/include/ble_err.h>

namespace kaleidoscope {
class Hooks;  // Forward declaration
}

// Add these near the top of the file, after the includes
#ifndef DEBUG_BLE
#define DEBUG_BLE 1  // Set to 1 to enable debugging output
#endif

#if DEBUG_BLE
namespace kaleidoscope {
namespace driver {
namespace ble {
namespace debug {

// Helper function to print multiple arguments
template<typename T>
void print_args(T value) {
  Serial.print(value);
}

template<typename T, typename... Args>
void print_args(T first, Args... args) {
  Serial.print(first);
  print_args(args...);
}

}  // namespace debug
}  // namespace ble
}  // namespace driver
}  // namespace kaleidoscope

#define DEBUG_BLE_MSG(...)                                     \
  do {                                                         \
    Serial.print(F("[BLE] "));                                 \
    kaleidoscope::driver::ble::debug::print_args(__VA_ARGS__); \
    Serial.println();                                          \
  } while (0)

#define DEBUG_BLE_PRINT(...)                                   \
  do {                                                         \
    Serial.print(F("[BLE] "));                                 \
    kaleidoscope::driver::ble::debug::print_args(__VA_ARGS__); \
  } while (0)
#else
#define DEBUG_BLE_MSG(...)
#define DEBUG_BLE_PRINT(...)
#endif

namespace kaleidoscope {
namespace driver {
namespace ble {

class BLEUartWrapper : public BLEUart {
 public:
  void flush() {
    /*
     * Override, because BLEUart uses flush() for RX and flushTXD() for TX
     * Don't actually call it, because it causes a reboot (hard fault?),
     * and we're not using its TX FIFO anyway.
     */
    // flushTXD();
  }
};

class BLEBluefruit : public Base {
 public:
  BLEBluefruit() {}

  void setup();

  static void startAdvLtd();
  static void stopAdv();
  bool connected();
  Stream &serialPort();
  void setBatteryLevel(uint8_t level);
  static void selectDevice(uint8_t device_id);
  static void stopAdvertising();

 private:
  static constexpr uint8_t SECURITY_MODE_ENCRYPTED = 1;
  static constexpr uint8_t SECURITY_LEVEL_MIN      = 1;

  static bool getConnectionSecurity(uint16_t conn_handle, ble_gap_conn_sec_t &sec);

  static BLEDis bledis;
  static BLEBas blebas;
  static BLEUartWrapper bleuart;

  static void stopAdv_cb();
  static void connect_cb(uint16_t conn_handle);
  static void secured_cb(uint16_t conn_handle);
  static void pairing_complete_cb(uint16_t conn_handle, uint8_t auth_status);
  static void disconnect_cb(uint16_t conn_handle, uint8_t reason);

  static void startDirectedAdv();
  static void printBLEAddress(const char *prefix, const uint8_t *addr);
  static void advCommon();
  static ble_gap_addr_t base_addr;
  static uint8_t current_device_id;
  static void setSlotSpecificAddress(uint8_t slot_id);
  static void disconnectAndUnbond();
};

}  // namespace ble
}  // namespace driver
}  // namespace kaleidoscope
#endif /* ARDUINO_ARCH_NRF52 */
