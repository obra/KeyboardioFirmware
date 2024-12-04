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

  void setup() {


    Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
    if (!Bluefruit.begin()) {
      DEBUG_BLE_MSG("Failed to initialize Bluefruit");
      return;
    }

    Bluefruit.setTxPower(4);

    Bluefruit.Periph.setConnectCallback(BLEBluefruit::connect_cb);
    Bluefruit.Periph.setDisconnectCallback(BLEBluefruit::disconnect_cb);
    Bluefruit.Security.setSecuredCallback(BLEBluefruit::secured_cb);
    Bluefruit.Security.setPairCompleteCallback(BLEBluefruit::pairing_complete_cb);

    Bluefruit.setAppearance(BLE_APPEARANCE_HID_KEYBOARD);
    Bluefruit.setName(USB_PRODUCT);

    // Configure Device Information Service
    bledis.setPermission(SECMODE_ENC_NO_MITM, SECMODE_NO_ACCESS);
    bledis.setManufacturer(USB_MANUFACTURER);
    bledis.setModel(USB_PRODUCT);
    if (!bledis.begin()) {
      DEBUG_BLE_MSG("Failed to start Device Information Service");
    }

    // Configure Battery Service
    blebas.setPermission(SECMODE_ENC_NO_MITM, SECMODE_NO_ACCESS);
    if (!blebas.begin()) {
      DEBUG_BLE_MSG("Failed to start Battery Service");
    }
    setBatteryLevel(100);

    // Configure UART Service
    bleuart.setPermission(SECMODE_ENC_NO_MITM, SECMODE_ENC_NO_MITM);
    if (!bleuart.begin()) {
      DEBUG_BLE_MSG("Failed to start UART Service");
    }

    Bluefruit.Advertising.setStopCallback(stopAdv_cb);

    // Cache the device's base address during setup
    sd_ble_gap_addr_get(&base_addr);
  }

  // Limited discoverable mode: default for HID seeking new host connections
  static void startAdvLtd() {
    DEBUG_BLE_MSG("Starting limited discoverable advertising");
    if (current_device_id == 0) {
      DEBUG_BLE_MSG("ERROR: No device slot selected");
      return;
    }

    DEBUG_BLE_MSG("Checking connection state...");
    if (Bluefruit.Periph.connected()) {
      DEBUG_BLE_MSG("Already connected.");
      if (Bluefruit.connected()) {
        disconnectAndUnbond();
      }
    }

    // Stop any existing advertising first
    if (Bluefruit.Advertising.isRunning()) {
      DEBUG_BLE_MSG("Stopping current advertising");
      Bluefruit.Advertising.stop();
      delay(100);  // Give the softdevice time to clean up
    }

    // Reset advertising state
    Bluefruit.Advertising.setType(BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED);
    Bluefruit.Advertising.clearData();
    Bluefruit.ScanResponse.clearData();

    DEBUG_BLE_MSG("Setting up new advertising data...");
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    advCommon();

    // Start advertising
    if (!Bluefruit.Advertising.start(0)) {

      DEBUG_BLE_MSG("Failed to start advertising");
    }
  }


  static void stopAdv() {
    Bluefruit.Advertising.stop();
  }

  bool connected() {
    return Bluefruit.Periph.connected();
  }

  Stream &serialPort() {
    return bleuart;
  }

  /**
   * @brief Set the battery level for BLE reporting.
   * 
   * @param level Battery level as a percentage (0-100).
   */
  void setBatteryLevel(uint8_t level) {
    if (level > 100) level = 100;
    blebas.write(level);
  }

  static void selectDevice(uint8_t device_id) {

    if (device_id == current_device_id) {
      DEBUG_BLE_MSG("Already on requested device - no action needed");
      // TODO return;
    }

    // Stop any current advertising
    Bluefruit.Advertising.stop();

    // Check current connection state and disconnect if needed
    if (Bluefruit.connected()) {
      uint16_t conn_handle = Bluefruit.connHandle();
      DEBUG_BLE_MSG("Connected to handle: ", conn_handle, " - Initiating disconnect...");

      if (Bluefruit.disconnect(conn_handle)) {
      } else {
        DEBUG_BLE_MSG("WARNING: Disconnect request failed!");
      }
    } else {
      DEBUG_BLE_MSG("No active connection");
    }

    DEBUG_BLE_MSG("Updating current_device_id: ", current_device_id, " -> ", device_id);
    current_device_id = device_id;


    startDirectedAdv();

    DEBUG_BLE_MSG(" ====== Device Selection Complete ======\n");
  }


  static void stopAdvertising() {
    if (Bluefruit.Advertising.isRunning()) {
      Bluefruit.Advertising.stop();
    }
  }

 private:
  static constexpr uint8_t SECURITY_MODE_ENCRYPTED = 1;
  static constexpr uint8_t SECURITY_LEVEL_MIN      = 1;

  // Get security status for a connection
  static bool getConnectionSecurity(uint16_t conn_handle, ble_gap_conn_sec_t &sec) {
    if (sd_ble_gap_conn_sec_get(conn_handle, &sec) != NRF_SUCCESS) {
      DEBUG_BLE_MSG("Failed to get security status");
      return false;
    }
    return true;
  }


  static BLEDis bledis;
  static BLEBas blebas;
  static BLEUartWrapper bleuart;

  static void stopAdv_cb() {
    DEBUG_BLE_MSG("stopAdv_cb: Advertising stopped");
  }

  static void connect_cb(uint16_t conn_handle) {
    DEBUG_BLE_MSG("Connected");
    printBLEAddress("Current address: ", Bluefruit.getAddr().addr);

    BLEConnection *conn = Bluefruit.Connection(conn_handle);
    if (!conn) {
      DEBUG_BLE_MSG("ERROR: Could not get connection object");
      return;
    }

    if (conn->bonded()) {
      DEBUG_BLE_MSG("Connection reports as already bonded");
    } else {
      DEBUG_BLE_MSG("Initiating new pairing request");
      Bluefruit.Security.setMITM(true);

      conn->requestPairing();
    }
  }

  static void secured_cb(uint16_t conn_handle) {

    BLEConnection *conn = Bluefruit.Connection(conn_handle);
    if (!conn) {
      DEBUG_BLE_MSG("ERROR: Could not get connection object");
      return;
    }

    // Are we bonded?
    if (conn->bonded()) {
      DEBUG_BLE_MSG("Connection reports as already bonded");
    } else {
      DEBUG_BLE_MSG("Initiating new pairing request");
      Bluefruit.Security.setMITM(true);

      conn->requestPairing();
    }
  }

  // Add a new callback for when pairing is complete
  static void pairing_complete_cb(uint16_t conn_handle, uint8_t auth_status) {
    // TODO - remove this callback?
    DEBUG_BLE_MSG("Pairing complete, auth_status = ", auth_status);

    BLEConnection *conn = Bluefruit.Connection(conn_handle);
    if (!conn) {
      DEBUG_BLE_MSG("ERROR: Could not get connection object");
      return;
    }
  }

  static void disconnect_cb(uint16_t conn_handle, uint8_t reason) {
    DEBUG_BLE_MSG("Disconnected, reason = 0x", reason, HEX);

    // Reset advertising state immediately on disconnect
    if (Bluefruit.Advertising.isRunning()) {
      Bluefruit.Advertising.stop();
    }


    switch (reason) {
    case BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION:
      DEBUG_BLE_MSG("Remote device terminated connection");
      startAdvLtd();  // Go back to discoverable mode
      break;

    case BLE_HCI_LOCAL_HOST_TERMINATED_CONNECTION:
      DEBUG_BLE_MSG("Local device terminated connection");
      startAdvLtd();  // Go back to discoverable mode
      break;

    case BLE_HCI_REMOTE_DEV_TERMINATION_DUE_TO_POWER_OFF:
    case BLE_HCI_CONNECTION_TIMEOUT:
    default:
      DEBUG_BLE_MSG("Attempting to reconnect to last device (reason=0x", reason, HEX, ")");

      // Try to reconnect to the last device
      if (current_device_id > 0) {
        startDirectedAdv();

      } else {
        startAdvLtd();
      }
      break;
    }
  }


  static void startDirectedAdv() {

    // Stop any existing advertising first
    if (Bluefruit.Advertising.isRunning()) {
      Bluefruit.Advertising.stop();
      delay(100);
    }

    Bluefruit.Advertising.clearData();
    Bluefruit.ScanResponse.clearData();

    // Use standard connectable advertising
    Bluefruit.Advertising.setType(BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED);

    // Add standard advertising data
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    advCommon();  // Add our standard advertising data

    // Set faster advertising interval for reconnection
    Bluefruit.Advertising.setInterval(32, 64);  // Fast advertising
    Bluefruit.Advertising.setFastTimeout(30);   // 30 seconds fast mode

    if (!Bluefruit.Advertising.start(0)) {
      DEBUG_BLE_MSG("Failed to start advertising");
    }
  }

  // Helper function to format and print MAC addresses
  static void printBLEAddress(const char *prefix, const uint8_t *addr) {
    Serial.print(prefix);

    for (int i = 5; i >= 0; i--) {
      Serial.print(addr[i], HEX);
      if (i > 0) Serial.print(":");
    }
    Serial.println("");
  }


  // Add shared advertising data
  static void advCommon() {
    setSlotSpecificAddress(current_device_id);
    Bluefruit.Advertising.addTxPower();
    Bluefruit.Advertising.addAppearance(BLE_APPEARANCE_HID_KEYBOARD);
    Bluefruit.Advertising.addService(hid::bluefruit::blehid);
    Bluefruit.Advertising.addName();
    Bluefruit.ScanResponse.addService(bleuart);

    /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   * 
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html   
   */
    Bluefruit.Advertising.restartOnDisconnect(true);
    Bluefruit.Advertising.setInterval(32, 244);  // in unit of 0.625 ms
    Bluefruit.Advertising.setFastTimeout(30);    // number of seconds in fast mode
  }

  static ble_gap_addr_t base_addr;   // Declare the static member
  static uint8_t current_device_id;  // Already exists


  static void setSlotSpecificAddress(uint8_t slot_id) {
    // Create slot-specific address
    ble_gap_addr_t slot_addr = base_addr;
    // Increment the last byte by the slot number
    slot_addr.addr[0] += slot_id;

    // Set the slot-specific address
    Bluefruit.setAddr(&slot_addr);
  }

  static void disconnectAndUnbond() {
    uint16_t conn_handle = Bluefruit.connHandle();
    BLEConnection *conn  = Bluefruit.Connection(conn_handle);
    DEBUG_BLE_MSG("Disconnecting current connection and removing bond to peer address");
    ble_gap_addr_t peer_addr = conn->getPeerAddr();
    printBLEAddress("Removing bond to: ", peer_addr.addr);
    bond_remove_key(BLE_GAP_ROLE_PERIPH, &peer_addr);
    Bluefruit.disconnect(conn_handle);
  }
};
}  // namespace ble
}  // namespace driver
}  // namespace kaleidoscope
#endif /* ARDUINO_ARCH_NRF52 */
