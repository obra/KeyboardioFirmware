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

// From Kaleidoscope:
#include "kaleidoscope/driver/hid/base/Keyboard.h"  // for Keyboard, KeyboardProps

#include "kaleidoscope/driver/hid/apis/BootKeyboardAPI.h"
#include "kaleidoscope/driver/hid/apis/ConsumerControlAPI.h"
#include "kaleidoscope/driver/hid/apis/SystemControlAPI.h"
#include "kaleidoscope/driver/hid/bluefruit/HIDD.h"

namespace kaleidoscope {
namespace driver {
namespace hid {
namespace bluefruit {

class BootKeyboard_ : public BootKeyboardAPI {
 public:
  BootKeyboard_()
    : BootKeyboardAPI(1) {}

  void begin() {
    blehid.begin();
    blehid.setLEDcb(LED_cb);
  }

  uint8_t getProtocol() override {
    return !blehid.isBootMode();
  }

  void onUSBReset() override {}

  uint8_t getLeds() override {
    return LEDs;
  }

 protected:
  static uint8_t LEDs;

  void setReportDescriptor(uint8_t bootkb_only) override {}

  int SendHIDReport(const void *data, int len) override {
    bool ret;

    if (blehid.isBootMode()) {
      ret = blehid.bootKeyboardReport(data, len);
    } else {
      ret = blehid.inputReport(RID_KEYBOARD, data, len);
    }
    return ret ? len : -1;
  }

  static void LED_cb(uint16_t hdl, BLECharacteristic *chr, uint8_t *data, uint16_t len) {
    LEDs = data[0];
    LOG_LV2("HID", "LED report: %02x", LEDs);
  }
};

class ConsumerControl_ : public ConsumerControlAPI {
 public:
  void sendReportUnchecked() override {
    if (blehid.isBootMode()) {
      return;
    }
    blehid.inputReport(RID_CONSUMER_CONTROL, &report_, sizeof(report_));
  }
};

class SystemControl_ : public SystemControlAPI {
  void sendReport(void *data, int length) override {
    if (blehid.isBootMode()) {
      return;
    }
    blehid.inputReport(RID_SYSTEM_CONTROL, data, length);
  }
  bool wakeupHost(uint8_t s) override {
    return false;
  }
};

struct KeyboardProps : public base::KeyboardProps {
  typedef BootKeyboard_ BootKeyboard;
  typedef ConsumerControl_ ConsumerControl;
  typedef SystemControl_ SystemControl;
};

template<typename _Props>
class Keyboard : public base::Keyboard<_Props> {};

}  // namespace bluefruit
}  // namespace hid
}  // namespace driver
}  // namespace kaleidoscope

#endif /* ARDUINO_ARCH_NRF52 */
