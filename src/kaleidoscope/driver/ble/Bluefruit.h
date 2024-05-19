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
    // Allow maximum ATT_MTU for better throughput.
    // This needs to be called before Bluefruit.begin()
    Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
    Bluefruit.begin();
    Bluefruit.setTxPower(4);

    Bluefruit.Periph.setConnectCallback(BLEBluefruit::connect_cb);
    Bluefruit.Periph.setDisconnectCallback(BLEBluefruit::disconnect_cb);

    Bluefruit.Security.setSecuredCallback(BLEBluefruit::secured_cb);

    Bluefruit.setAppearance(BLE_APPEARANCE_HID_KEYBOARD);
    Bluefruit.setName(USB_PRODUCT);

    // HOGP spec requires Device Information and Battery Services
    // to have the same Security Level as HID
    bledis.setPermission(SECMODE_ENC_NO_MITM, SECMODE_NO_ACCESS);
    bledis.setManufacturer(USB_MANUFACTURER);
    bledis.setModel(USB_PRODUCT);
    bledis.begin();

    blebas.setPermission(SECMODE_ENC_NO_MITM, SECMODE_NO_ACCESS);
    blebas.begin();
    blebas.write(100);

    // Require encryption for UART
    bleuart.setPermission(SECMODE_ENC_NO_MITM, SECMODE_ENC_NO_MITM);
    bleuart.begin();

    Bluefruit.Advertising.setStopCallback(stopAdv_cb);
  }

  // Limited discoverable mode: default for HID seeking new host connections
  static void startAdvLtd() {
    if (Bluefruit.Periph.connected()) {
      return;
    }
    if (BLEBluefruit::adv_started) {
      return;
    }
    Bluefruit.Advertising.clearData();
    Bluefruit.ScanResponse.clearData();
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE);
    advCommon();
    // Don't be discoverable unless explicitly requested
    Bluefruit.Advertising.restartOnDisconnect(false);
    Bluefruit.Advertising.setInterval(32, 244);
    Bluefruit.Advertising.setFastTimeout(30);
    // Can go up to 180s, per spec, but we might want shorter here
    Bluefruit.Advertising.start(60);
    BLEBluefruit::adv_started = true;
  }

  // Connectable, non-discoverable advertising.
  // Needs to filter for bonded hosts, but that requires an upstream API change
  static void startAdvConn() {
    if (Bluefruit.Periph.connected()) {
      return;
    }
    if (BLEBluefruit::adv_started) {
      return;
    }
    Bluefruit.Advertising.clearData();
    Bluefruit.ScanResponse.clearData();
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED);
    BLEBluefruit::advCommon();
    Bluefruit.Advertising.restartOnDisconnect(false);
    Bluefruit.Advertising.setInterval(32, 244);
    Bluefruit.Advertising.setFastTimeout(30);
    Bluefruit.Advertising.start(0);
    BLEBluefruit::adv_started = true;
  }

  static void stopAdv() {
    Bluefruit.Advertising.stop();
    BLEBluefruit::adv_started = false;
  }

  bool connected() {
    return Bluefruit.Periph.connected();
  }

  Stream &serialPort() {
    return bleuart;
  }

 private:
  static BLEDis bledis;
  static BLEBas blebas;
  static BLEUartWrapper bleuart;

  static bool adv_started;
  static bool bonded;

  // Add shared advertising data
  static void advCommon() {
    Bluefruit.Advertising.addTxPower();
    Bluefruit.Advertising.addAppearance(BLE_APPEARANCE_HID_KEYBOARD);
    Bluefruit.Advertising.addService(hid::bluefruit::blehid);
    Bluefruit.Advertising.addName();
    Bluefruit.ScanResponse.addService(bleuart);
  }

  static void stopAdv_cb() {
    BLEBluefruit::adv_started = false;
  }

  static void connect_cb(uint16_t conn_handle) {
    (void)conn_handle;
    BLEBluefruit::adv_started = false;
  }

  static void secured_cb(uint16_t conn_handle) {
    (void)conn_handle;
    BLEBluefruit::bonded = true;
  }

  static void disconnect_cb(uint16_t conn_handle, uint8_t reason) {
    (void)conn_handle;
    (void)reason;
    // If we're bonded, advertise (non-discoverable) when disconnected.
    // Needs to eventually search the bond database instead.
    if (BLEBluefruit::bonded) {
      BLEBluefruit::startAdvConn();
    }
  }
};

}  // namespace ble
}  // namespace driver
}  // namespace kaleidoscope
#endif /* ARDUINO_ARCH_NRF52 */
