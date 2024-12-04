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

#ifdef ARDUINO_ARCH_NRF52

#include "kaleidoscope/driver/ble/Bluefruit.h"  // for Base
#include "kaleidoscope/Runtime.h"
#include "kaleidoscope/hooks.h"
#include "kaleidoscope/host_connection_status.h"

namespace kaleidoscope {
namespace driver {
namespace ble {

uint8_t BLEBluefruit::current_device_id = 0;
ble_gap_addr_t BLEBluefruit::base_addr;

BLEDis BLEBluefruit::bledis;
BLEBas BLEBluefruit::blebas;
BLEUartWrapper BLEBluefruit::bleuart;


void BLEBluefruit::setup() {

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

void BLEBluefruit::startAdvLtd() {
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
  Hooks::onHostConnectionStatusChanged(current_device_id, kaleidoscope::HostConnectionStatus::Advertising);

  // Start advertising
  if (!Bluefruit.Advertising.start(0)) {

    DEBUG_BLE_MSG("Failed to start advertising");
  }
}

void BLEBluefruit::stopAdv() {
  Bluefruit.Advertising.stop();
}

bool BLEBluefruit::connected() {
  return Bluefruit.Periph.connected();
}

Stream &BLEBluefruit::serialPort() {
  return bleuart;
}

void BLEBluefruit::setBatteryLevel(uint8_t level) {
  if (level > 100) level = 100;
  blebas.write(level);
}

void BLEBluefruit::selectDevice(uint8_t device_id) {

  if (device_id == current_device_id) {
    DEBUG_BLE_MSG("Already on requested device - no action needed");
    // TODO(jesse): Implement this
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
  Hooks::onHostConnectionStatusChanged(current_device_id, kaleidoscope::HostConnectionStatus::DeviceSelected);

  startDirectedAdv();

  DEBUG_BLE_MSG(" ====== Device Selection Complete ======\n");
}

void BLEBluefruit::stopAdvertising() {
  if (Bluefruit.Advertising.isRunning()) {
    Bluefruit.Advertising.stop();
  }
}

bool BLEBluefruit::getConnectionSecurity(uint16_t conn_handle, ble_gap_conn_sec_t &sec) {
  if (sd_ble_gap_conn_sec_get(conn_handle, &sec) != NRF_SUCCESS) {
    DEBUG_BLE_MSG("Failed to get security status");
    return false;
  }
  return true;
}


void BLEBluefruit::stopAdv_cb() {
  DEBUG_BLE_MSG("stopAdv_cb: Advertising stopped");
}

void BLEBluefruit::connect_cb(uint16_t conn_handle) {
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
  Hooks::onHostConnectionStatusChanged(current_device_id, kaleidoscope::HostConnectionStatus::Connecting);
}

void BLEBluefruit::secured_cb(uint16_t conn_handle) {
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
  Hooks::onHostConnectionStatusChanged(current_device_id, kaleidoscope::HostConnectionStatus::Connected);
}

void BLEBluefruit::pairing_complete_cb(uint16_t conn_handle, uint8_t auth_status) {
  DEBUG_BLE_MSG("Pairing complete, auth_status = ", auth_status);

  BLEConnection *conn  = Bluefruit.Connection(conn_handle);
  if (!conn) {
    DEBUG_BLE_MSG("ERROR: Could not get connection object");
    return;
  }
  Hooks::onHostConnectionStatusChanged(current_device_id, auth_status == 0 ? kaleidoscope::HostConnectionStatus::PairingSuccess : kaleidoscope::HostConnectionStatus::PairingFailed);
}

void BLEBluefruit::disconnect_cb(uint16_t conn_handle, uint8_t reason) {
  DEBUG_BLE_MSG("Disconnected, reason = 0x", reason, HEX);

  if (Bluefruit.Advertising.isRunning()) {
    Bluefruit.Advertising.stop();
  }

  switch (reason) {
  case BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION:
    DEBUG_BLE_MSG("Remote device terminated connection");
    startAdvLtd();
    break;

  case BLE_HCI_LOCAL_HOST_TERMINATED_CONNECTION:
    DEBUG_BLE_MSG("Local device terminated connection");
    startAdvLtd();
    break;

  case BLE_HCI_REMOTE_DEV_TERMINATION_DUE_TO_POWER_OFF:
  case BLE_HCI_CONNECTION_TIMEOUT:
  default:
    DEBUG_BLE_MSG("Attempting to reconnect to last device (reason=0x", reason, HEX, ")");

    if (current_device_id > 0) {
      startDirectedAdv();
    } else {
      startAdvLtd();
    }
    break;
  }
  Hooks::onHostConnectionStatusChanged(current_device_id, kaleidoscope::HostConnectionStatus::Disconnected);
}

void BLEBluefruit::startDirectedAdv() {

  if (Bluefruit.Advertising.isRunning()) {
    Bluefruit.Advertising.stop();
    delay(100);
  }

  Bluefruit.Advertising.clearData();
  Bluefruit.ScanResponse.clearData();

  Bluefruit.Advertising.setType(BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED);

  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  advCommon();

  Bluefruit.Advertising.setInterval(32, 64);
  Bluefruit.Advertising.setFastTimeout(30);

  if (!Bluefruit.Advertising.start(0)) {
    DEBUG_BLE_MSG("Failed to start advertising");
  }
}

void BLEBluefruit::printBLEAddress(const char *prefix, const uint8_t *addr) {
  Serial.print(prefix);

  for (int i = 5; i >= 0; i--) {
    Serial.print(addr[i], HEX);
    if (i > 0) Serial.print(":");
  }
  Serial.println("");
}

void BLEBluefruit::advCommon() {
  setSlotSpecificAddress(current_device_id);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addAppearance(BLE_APPEARANCE_HID_KEYBOARD);
  Bluefruit.Advertising.addService(hid::bluefruit::blehid);
  Bluefruit.Advertising.addName();
  Bluefruit.ScanResponse.addService(bleuart);

  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.setFastTimeout(30);
}

void BLEBluefruit::setSlotSpecificAddress(uint8_t slot_id) {
  ble_gap_addr_t slot_addr = base_addr;
  slot_addr.addr[0] += slot_id;

  Bluefruit.setAddr(&slot_addr);
}

void BLEBluefruit::disconnectAndUnbond() {
  uint16_t conn_handle = Bluefruit.connHandle();
  BLEConnection *conn  = Bluefruit.Connection(conn_handle);
  DEBUG_BLE_MSG("Disconnecting current connection and removing bond to peer address");
  ble_gap_addr_t peer_addr = conn->getPeerAddr();
  printBLEAddress("Removing bond to: ", peer_addr.addr);
  bond_remove_key(BLE_GAP_ROLE_PERIPH, &peer_addr);
  Bluefruit.disconnect(conn_handle);
}

}  // namespace ble
}  // namespace driver
}  // namespace kaleidoscope

#endif
