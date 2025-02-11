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
/*
  // Configure MTU and queue sizes. This should happen before 'begin'
  Bluefruit.configPrphConn(MTU_SIZE,              // MTU size
                          EVENT_LENGTH,           // Event length
                          HVN_QUEUE_SIZE,        // HVN queue size
                          WRITE_CMD_QUEUE_SIZE); // Write CMD queue size
*/
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

  if (!Bluefruit.begin()) {
    DEBUG_BLE_MSG("Failed to initialize Bluefruit");
    return;
  }

  // Configure connection parameters for low latency using constants from header
  Bluefruit.Periph.setConnInterval(CONN_INTERVAL_MIN_MS, CONN_INTERVAL_MAX_MS);
  Bluefruit.Periph.setConnSlaveLatency(SLAVE_LATENCY);
  Bluefruit.Periph.setConnSupervisionTimeout(SUPERVISION_TIMEOUT_MS);

  // Enable DC/DC regulator for better power efficiency
  sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);

  // Configure low power mode - keeping CONSTLAT for now
  sd_power_mode_set(NRF_POWER_MODE_CONSTLAT);

  // Set lower TX power - still good for 10m range
  Bluefruit.setTxPower(CONN_TX_POWER);

  // Disable Bluefruit's built in LED support since it's designed for a single-color LED, not our RGB LEDs.
  // TODO(anybody): make this configurable down the line.
  Bluefruit.autoConnLed(false);
  Bluefruit.setConnLedInterval(0);

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

  Bluefruit.Advertising.setStopCallback(stop_advertising_cb);

  // Cache the device's base address during setup
  sd_ble_gap_addr_get(&base_addr);
}

void BLEBluefruit::startDiscoverableAdvertising() {
  DEBUG_BLE_MSG("Starting limited discoverable advertising");
  if (current_device_id == 0) {
    DEBUG_BLE_MSG("ERROR: No device slot selected");
    return;
  }

  DEBUG_BLE_MSG("Checking connection state...");
  if (Bluefruit.Periph.connected()) {
    DEBUG_BLE_MSG("Already connected.");
    if (Bluefruit.connected()) {
      DEBUG_BLE_MSG("Disconnecting current connection and removing bond to peer address so we can connect a new device to this slot");
      disconnectAndUnbond();
    }
  }

  stopAdvertising();
  configureAdvertising();

  DEBUG_BLE_MSG("Setting up new advertising data...");
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Hooks::onHostConnectionStatusChanged(current_device_id, kaleidoscope::HostConnectionStatus::Advertising);

  // Start advertising
  if (!Bluefruit.Advertising.start(0)) {

    DEBUG_BLE_MSG("Failed to start advertising");
  }
  DEBUG_BLE_MSG("Started connectable advertising");
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
  if (device_id == 0) {
    DEBUG_BLE_MSG("ERROR: No device slot selected");
    return;
  }
  if (device_id != current_device_id) {
    if (Bluefruit.Periph.connected()) {
      DEBUG_BLE_MSG("Disconnecting current connection");
      disconnect();
      // Wait for disconnect to complete before changing device
      delay(100);
    } else {
      DEBUG_BLE_MSG("Device not connected, skipping disconnect");
    }
    stopAdvertising();
    Hooks::onHostConnectionStatusChanged(current_device_id, kaleidoscope::HostConnectionStatus::DeviceUnselected);

    DEBUG_BLE_MSG("Updating current_device_id: ", current_device_id, " -> ", device_id);

    char name_buffer[48];
    snprintf(name_buffer, sizeof(name_buffer), "%s (#%d)", USB_PRODUCT, device_id);
    Bluefruit.setName(name_buffer);
    Hooks::onHostConnectionStatusChanged(device_id, kaleidoscope::HostConnectionStatus::DeviceSelected);
    current_device_id = device_id;
  }
  startConnectableAdvertising();

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


void BLEBluefruit::stop_advertising_cb() {
  DEBUG_BLE_MSG("stop_advertising_cb: Advertising stopped");
}

void BLEBluefruit::connect_cb(uint16_t conn_handle) {
  BLEConnection* conn = Bluefruit.Connection(conn_handle);
  if (!conn) {
    DEBUG_BLE_MSG("ERROR: Could not get connection object");
    return;
  }

  Bluefruit.Security.setMITM(true);


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
    conn->requestPairing();
  }

  // Get actual connection parameters
  uint16_t actual_interval = conn->getConnectionInterval();
  uint16_t actual_latency = conn->getSlaveLatency();
  uint16_t actual_timeout = conn->getSupervisionTimeout();
  
  DEBUG_BLE_MSG("Connection parameters:");
  char msg[64];
  snprintf(msg, sizeof(msg), "Interval: %dms", (int)(actual_interval * 1.25f)); // Convert from units to ms
  DEBUG_BLE_MSG(msg);
  snprintf(msg, sizeof(msg), "Latency: %d", actual_latency);
  DEBUG_BLE_MSG(msg);
  snprintf(msg, sizeof(msg), "Timeout: %dms", actual_timeout * 10); // Convert to ms
  DEBUG_BLE_MSG(msg);

  // Check if parameters match our requested values
  if ((actual_interval * 1.25f) > CONN_INTERVAL_MAX_MS) {
    snprintf(msg, sizeof(msg), "Warning: Connection interval %dms higher than requested %dms",
             (int)(actual_interval * 1.25f), CONN_INTERVAL_MAX_MS);
    DEBUG_BLE_MSG(msg);
  }

  if (actual_latency > SLAVE_LATENCY) {
    snprintf(msg, sizeof(msg), "Warning: Slave latency %d higher than requested %d",
             actual_latency, SLAVE_LATENCY);
    DEBUG_BLE_MSG(msg);
  }

  // Get and log connection status and errors
  int8_t rssi = conn->getRssi();
  snprintf(msg, sizeof(msg), "RSSI: %d dBm", rssi);
  DEBUG_BLE_MSG(msg);

  uint16_t mtu = conn->getMtu();
  snprintf(msg, sizeof(msg), "MTU: %d bytes", mtu);
  DEBUG_BLE_MSG(msg);

  uint16_t data_length = conn->getDataLength();
  snprintf(msg, sizeof(msg), "Data Length: %d bytes", data_length);
  DEBUG_BLE_MSG(msg);

  snprintf(msg, sizeof(msg), "Connected. Handle: %d", conn_handle);
  DEBUG_BLE_MSG(msg);


  // Supervision timeout: 1000ms (100 * 10ms)
  // TODO move this to "before connection" code
  Bluefruit.Periph.setConnSupervisionTimeout(100);

  //requestConnectionParameter is used to try to force a negotiation after connection is established 

  conn->requestConnectionParameter(CONN_INTERVAL_MIN_MS, SLAVE_LATENCY, SUPERVISION_TIMEOUT_MS);

  DEBUG_BLE_MSG("Connection status: ", conn->connected() ? "connected" : "disconnected");
  DEBUG_BLE_MSG("After we requestConnectionParameter to try to force a lower latency and longer timeout");

  // Get actual connection parameters
  actual_interval = conn->getConnectionInterval();
  actual_latency = conn->getSlaveLatency();
  actual_timeout = conn->getSupervisionTimeout();
  
  DEBUG_BLE_MSG("Connection parameters:");
  snprintf(msg, sizeof(msg), "Interval: %dms", (int)(actual_interval * 1.25f)); // Convert from units to ms
  DEBUG_BLE_MSG(msg);
  snprintf(msg, sizeof(msg), "Latency: %d", actual_latency);
  DEBUG_BLE_MSG(msg);
  snprintf(msg, sizeof(msg), "Timeout: %dms", actual_timeout * 10); // Convert to ms
  DEBUG_BLE_MSG(msg);

  // Check if parameters match our requested values
  if ((actual_interval * 1.25f) > CONN_INTERVAL_MAX_MS) {
    snprintf(msg, sizeof(msg), "Warning: Connection interval %dms higher than requested %dms",
             (int)(actual_interval * 1.25f), CONN_INTERVAL_MAX_MS);
    DEBUG_BLE_MSG(msg);
  }

  if (actual_latency > SLAVE_LATENCY) {
    snprintf(msg, sizeof(msg), "Warning: Slave latency %d higher than requested %d",
             actual_latency, SLAVE_LATENCY);
    DEBUG_BLE_MSG(msg);
  }

    // Send a BLE-specific null keyboard report to prevent the host from getting stuck keys
  kaleidoscope::Runtime.device().setHostConnectionMode(MODE_BLE);
  kaleidoscope::Runtime.device().hid().keyboard().releaseAllKeys();
  Hooks::onHostConnectionStatusChanged(current_device_id, kaleidoscope::HostConnectionStatus::Connected);
}

void BLEBluefruit::pairing_complete_cb(uint16_t conn_handle, uint8_t auth_status) {
  DEBUG_BLE_MSG("Pairing complete, auth_status = ", auth_status);

  // Send pairing status before trying to get connection object
  Hooks::onHostConnectionStatusChanged(current_device_id, 
    auth_status == 0 ? kaleidoscope::HostConnectionStatus::PairingSuccess 
                     : kaleidoscope::HostConnectionStatus::PairingFailed);

  if (auth_status != 0) {
    DEBUG_BLE_MSG("Pairing failed");
    return;
  }

  // Only try to get connection object if we're still connected
  if (Bluefruit.Periph.connected()) {
    BLEConnection *conn = Bluefruit.Connection(conn_handle);
    if (!conn) {
      DEBUG_BLE_MSG("ERROR: Could not get connection object");
      return;
    }

    // Ensure connection parameters are set correctly after pairing
    conn->requestConnectionParameter(CONN_INTERVAL_MIN_MS, SLAVE_LATENCY, SUPERVISION_TIMEOUT_MS);
  } else {
    DEBUG_BLE_MSG("Connection already closed when pairing completed");
  }
}

void BLEBluefruit::disconnect_cb(uint16_t conn_handle, uint8_t reason) {
  DEBUG_BLE_MSG("Disconnected, reason = 0x", reason, HEX);

  // Error 0x516 indicates advertising failed - this can happen during rapid state changes
  // We track consecutive failures to prevent getting stuck in a retry loop
  static uint8_t failure_count = 0;
  static uint32_t last_failure_time = 0;
  
  if (reason == 0x516) {
    DEBUG_BLE_MSG("Nordic error 0x516: Advertising procedure failed");
    
    // Reset failure count if more than 5 seconds have passed
    uint32_t now = millis();
    if ((now - last_failure_time) > 5000) {
      failure_count = 0;
    }
    last_failure_time = now;
    
    if (++failure_count >= 3) {
      DEBUG_BLE_MSG("Multiple advertising failures detected, stopping retry loop");
      failure_count = 0;
      Bluefruit.Advertising.stop();
      delay(500);
      return;
    }
  }

  switch (reason) {
  case BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION:
    DEBUG_BLE_MSG("Remote device terminated connection");
    startDiscoverableAdvertising();
    break;

  case BLE_HCI_LOCAL_HOST_TERMINATED_CONNECTION:
    DEBUG_BLE_MSG("Local device terminated connection. Don't assume we should do something.");
    break;

  case BLE_HCI_REMOTE_DEV_TERMINATION_DUE_TO_POWER_OFF:
  case BLE_HCI_CONNECTION_TIMEOUT:
  default:
    DEBUG_BLE_MSG("Attempting to reconnect to last device");

    if (current_device_id > 0) {
      delay(250);  // Give time for cleanup
      startConnectableAdvertising();
    }
    break;
  }

  Hooks::onHostConnectionStatusChanged(current_device_id, kaleidoscope::HostConnectionStatus::Disconnected);
}

void BLEBluefruit::startConnectableAdvertising() {
  stopAdvertising();
  configureAdvertising();
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE);

  DEBUG_BLE_MSG("Attempting to start advertising...");
  if (!Bluefruit.Advertising.start(0)) {
    DEBUG_BLE_MSG("Failed to start advertising");
  }
  DEBUG_BLE_MSG("Started connectable advertising");
  Hooks::onHostConnectionStatusChanged(current_device_id, kaleidoscope::HostConnectionStatus::Advertising);
}

void BLEBluefruit::printBLEAddress(const char *prefix, const uint8_t *addr) {
  Serial.print(prefix);

  for (int i = 5; i >= 0; i--) {
    Serial.print(addr[i], HEX);
    if (i > 0) Serial.print(":");
  }
  Serial.println("");
}

void BLEBluefruit::configureAdvertising() {
  // Reset advertising state
  Bluefruit.Advertising.setType(BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED);
  Bluefruit.Advertising.clearData();
  Bluefruit.ScanResponse.clearData();

  setSlotSpecificAddress(current_device_id);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addAppearance(BLE_APPEARANCE_HID_KEYBOARD);
  Bluefruit.Advertising.addService(hid::bluefruit::blehid);
  Bluefruit.Advertising.addName();
  Bluefruit.ScanResponse.addService(bleuart);

  // Configure advertising timing parameters for optimal discovery and power usage
  Bluefruit.Advertising.restartOnDisconnect(true);
  // Interval between 20ms (32*0.625) and 152.5ms (244*0.625)
  Bluefruit.Advertising.setInterval(32, 244);    
  // Time in fast advertising mode before switching to slow mode
  Bluefruit.Advertising.setFastTimeout(30);      

  // Log current advertising configuration for debugging connection issues
  DEBUG_BLE_MSG("Advertising configured for device slot: ", current_device_id);
  printBLEAddress("Using BLE address: ", Bluefruit.getAddr().addr);
}

void BLEBluefruit::setSlotSpecificAddress(uint8_t slot_id) {
  ble_gap_addr_t slot_addr = base_addr;
  slot_addr.addr[0] += slot_id;

  Bluefruit.setAddr(&slot_addr);
}

void BLEBluefruit::disconnect() {
  // Disable auto-restart of advertising
  Bluefruit.Advertising.restartOnDisconnect(false);

  // Check current connection state and disconnect if needed
  if (Bluefruit.connected()) {
    uint16_t conn_handle = Bluefruit.connHandle();
    DEBUG_BLE_MSG("Connected to handle: ", conn_handle, " - Initiating disconnect...");

  // Send a BLE-specific null keyboard report before disconnecting to prevent stuck keys
  kaleidoscope::Runtime.device().setHostConnectionMode(MODE_BLE);
  kaleidoscope::Runtime.device().hid().keyboard().releaseAllKeys();

    if (Bluefruit.disconnect(conn_handle)) {
    } else {
      DEBUG_BLE_MSG("WARNING: Disconnect request failed!");
    }
  } else {
    DEBUG_BLE_MSG("No active connection");
  }
}


void BLEBluefruit::disconnectAndUnbond() {
  unbond();
  disconnect();
}

void BLEBluefruit::unbond() {
  uint16_t conn_handle = Bluefruit.connHandle();
  BLEConnection *conn  = Bluefruit.Connection(conn_handle);
  DEBUG_BLE_MSG("Disconnecting current connection and removing bond to peer address");
  ble_gap_addr_t peer_addr = conn->getPeerAddr();
  printBLEAddress("Removing bond to: ", peer_addr.addr);
  bond_remove_key(BLE_GAP_ROLE_PERIPH, &peer_addr);
}

}  // namespace ble
}  // namespace driver
}  // namespace kaleidoscope

#endif
