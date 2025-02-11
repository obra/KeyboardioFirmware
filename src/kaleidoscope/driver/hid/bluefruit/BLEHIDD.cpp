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

#ifdef ARDUINO_ARCH_NRF52

#include "kaleidoscope/driver/hid/apis/AbsoluteMouseAPI.h"
#include "kaleidoscope/driver/hid/apis/BootKeyboardAPI.h"
#include "kaleidoscope/driver/hid/apis/ConsumerControlAPI.h"
#include "kaleidoscope/driver/hid/apis/MouseAPI.h"
#include "kaleidoscope/driver/hid/apis/SystemControlAPI.h"
#include "kaleidoscope/driver/hid/bluefruit/HIDD.h"

namespace kaleidoscope {
namespace driver {
namespace hid {
namespace bluefruit {

static const uint8_t report_desc[] = {
  DESCRIPTOR_BOOT_KEYBOARD(HID_REPORT_ID(RID_KEYBOARD)),
  DESCRIPTOR_MOUSE(HID_REPORT_ID(RID_MOUSE)),
  DESCRIPTOR_CONSUMER_CONTROL(HID_REPORT_ID(RID_CONSUMER_CONTROL)),
  DESCRIPTOR_SYSTEM_CONTROL(HID_REPORT_ID(RID_SYSTEM_CONTROL)),
  DESCRIPTOR_ABSOLUTE_MOUSE(HID_REPORT_ID(RID_ABS_MOUSE)),
};

HIDD::HIDD()
  : BLEHidGeneric(5, 1, 0) {}

err_t HIDD::begin() {
  uint16_t in_lens[] = {
    BOOT_REPORT_LEN,
    sizeof(HID_MouseReport_Data_t),
    sizeof(HID_ConsumerControlReport_Data_t),
    sizeof(HID_SystemControlReport_Data_t),
    sizeof(HID_MouseAbsoluteReport_Data_t),
  };
  uint16_t out_lens[] = {1};

  setHidInfo(BLE_HID_BCD, 0, BLE_HID_INFO_NORMALLY_CONNECTABLE | BLE_HID_INFO_NORMALLY_CONNECTABLE);
  enableKeyboard(true);
  enableMouse(true);
  setReportLen(in_lens, out_lens, NULL);
  setReportMap(report_desc, sizeof(report_desc));

  VERIFY_STATUS(BLEHidGeneric::begin());

  return ERROR_NONE;
}

void HIDD::setLEDcb(BLECharacteristic::write_cb_t fp) {
  setOutputReportCallback(RID_KEYBOARD, fp);
  _chr_boot_keyboard_output->setWriteCallback(fp);
}

// Retry configuration
static constexpr uint8_t MAX_RETRIES = 6;
static constexpr uint32_t RETRY_DELAY_MS = 10;

// Track consecutive failures across all senders
static uint16_t consecutive_failures = 0;

bool HIDD::send_with_retries(ReportType type, uint8_t report_id, const void* data, uint8_t length) {
  uint8_t retry_count = 0;
  
  do {
    BLEConnection* conn = Bluefruit.Connection(0);
    if (!conn || !conn->connected()) {
          tone(PIN_SPEAKER, 2000, 50);
          delay(50);
          tone(PIN_SPEAKER, 2000, 50);

      return false;
    }

    bool success;
    switch (type) {
      case ReportType::BootKeyboard:
        success = BLEHidGeneric::bootKeyboardReport(data, length);
        break;
      case ReportType::BootMouse:
        success = BLEHidGeneric::bootMouseReport(data, length);
        break;
      case ReportType::Input:
        success = BLEHidGeneric::inputReport(report_id, data, length);
        break;
    }

    if (success) {
      consecutive_failures = 0;
      return true;
    }
    tone(PIN_SPEAKER, 1000, 100);

    consecutive_failures++;
    retry_count++;
    
    if (retry_count < MAX_RETRIES) {
      delay(RETRY_DELAY_MS);
    }
    
  } while (retry_count < MAX_RETRIES);
  
  return false;
}

bool HIDD::sendBootKeyboardReport(const void* data, uint8_t length) {
  return send_with_retries(ReportType::BootKeyboard, 0, data, length);
}

bool HIDD::sendBootMouseReport(const void* data, uint8_t length) {
  return send_with_retries(ReportType::BootMouse, 0, data, length);
}

bool HIDD::sendInputReport(uint8_t report_id, const void* data, uint8_t length) {
  return send_with_retries(ReportType::Input, report_id, data, length);
}


HIDD blehid;

}  // namespace bluefruit
}  // namespace hid
}  // namespace driver
}  // namespace kaleidoscope

#endif /* ARDUINO_ARCH_NRF52 */
