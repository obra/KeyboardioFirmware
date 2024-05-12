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

HIDD blehid;

}  // namespace bluefruit
}  // namespace hid
}  // namespace driver
}  // namespace kaleidoscope

#endif /* ARDUINO_ARCH_NRF52 */
