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

// Include debug macros first
#include "kaleidoscope/driver/ble/Bluefruit.h"  // for DEBUG_BLE_MSG
#include "kaleidoscope/trace.h"

#include "nrf_sdm.h"
#include "nrf_error.h"
#include "ble_err.h"

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
    : BootKeyboardAPI(1) {
    report_count_ = 0;
    failed_reports_ = 0;
  }

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
  uint32_t report_count_;
  uint32_t failed_reports_;

  void setReportDescriptor(uint8_t bootkb_only) override {}

  int SendHIDReport(const void *data, int len) override {
    bool ret;
    char msg[128];
    const uint8_t* report = static_cast<const uint8_t*>(data);

    // Track consecutive successes/failures
    static uint16_t consecutive_failures = 0;

    // Retry configuration
    static constexpr uint8_t MAX_RETRIES = 3;
    static constexpr uint32_t RETRY_DELAY_MS = 5;
    uint8_t retry_count = 0;

    do {
      
  
      // Attempt to send report
      if (blehid.isBootMode()) {
        ret = blehid.bootKeyboardReport(data, len);
      } else {
        ret = blehid.inputReport(RID_KEYBOARD, data, len);
      }



      if (!ret) {
              BLEConnection* conn = Bluefruit.Connection(0);
      bool has_connection = (conn && conn->connected());
      bool notifications_enabled = false;
  
      
      // Get detailed error information
      if (!has_connection) {
          DEBUG_TRACE("HID", "Send failed - No active connection");
      } else {
     
      
          // Check if we have HVN credits (TX buffers)
          bool has_hvn = conn->getHvnPacket();
          if (!has_hvn) {
              DEBUG_TRACE("HID", "Send failed - No HVN credits available (TX buffers full). Status: 0x%04lX", 
                      (unsigned long)sd_ble_gatts_hvx(Bluefruit.connHandle(), nullptr));
          }
          
 

        }

        // Check if we should retry
        if (retry_count < MAX_RETRIES) {
          retry_count++;
          DEBUG_TRACE("HID", "Attempt %d failed, retrying...", 
                   retry_count);
          delay(RETRY_DELAY_MS);
          continue;
        }
      }
      break;
    } while (true);

    if (!ret) {
      failed_reports_++;
      consecutive_failures++;
      
      DEBUG_TRACE("HID", "HID Report failed after %d retries (total: %d, consecutive: %d) ", 
               retry_count, failed_reports_, consecutive_failures);
      
      return -1;
    } else {
      if (consecutive_failures > 0) {
        DEBUG_TRACE("HID", "Recovered after %d consecutive failures", consecutive_failures);
      }
        consecutive_failures = 0;
      
      if (retry_count > 0) {
        DEBUG_TRACE("HID", "HID Report succeeded after %d retries. ", 
                 retry_count);
        DEBUG_BLE_MSG(msg);
      }
    }
    
    report_count_++;
    return len;
  }

  static void LED_cb(uint16_t hdl, BLECharacteristic *chr, uint8_t *data, uint16_t len) {
    LEDs = data[0];
    DEBUG_TRACE("HID", "LED report: %02x", LEDs);
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
