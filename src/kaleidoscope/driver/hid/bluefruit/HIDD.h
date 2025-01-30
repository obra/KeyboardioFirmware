/* Kaleidoscope - Firmware for computer input devices
 * Copyright (C) 2013-2025 Keyboard.io, inc.
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

namespace kaleidoscope {
namespace driver {
namespace hid {
namespace bluefruit {

#define BLE_HID_BCD                       0x110
#define BLE_HID_INFO_REMOTE_WAKE          0x01
#define BLE_HID_INFO_NORMALLY_CONNECTABLE 0x02

enum {
  RID_KEYBOARD = 1,
  RID_MOUSE,
  RID_CONSUMER_CONTROL,
  RID_SYSTEM_CONTROL,
  RID_ABS_MOUSE,
};

enum class ReportType {
  BootKeyboard,
  BootMouse,
  Input
};

class HIDD : public BLEHidGeneric {
 public:
  HIDD();
  err_t begin();
  void setLEDcb(BLECharacteristic::write_cb_t fp);

  /**
   * Send a boot keyboard report with retries
   * @param data Report data
   * @param length Length of report
   * @return true if send was successful
   */
  bool sendBootKeyboardReport(const void* data, uint8_t length);

  /**
   * Send a boot mouse report with retries
   * @param data Report data
   * @param length Length of report
   * @return true if send was successful
   */
  bool sendBootMouseReport(const void* data, uint8_t length);

  /**
   * Send an input report with retries
   * @param report_id Report ID
   * @param data Report data
   * @param length Length of report
   * @return true if send was successful
   */
  bool sendInputReport(uint8_t report_id, const void* data, uint8_t length);

 private:
  bool send_with_retries(ReportType type, uint8_t report_id, const void* data, uint8_t length);
};

extern HIDD blehid;

}  // namespace bluefruit
}  // namespace hid
}  // namespace driver
}  // namespace kaleidoscope

#endif /* ARDUINO_ARCH_NRF52 */
