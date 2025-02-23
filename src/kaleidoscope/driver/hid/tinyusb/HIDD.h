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

#ifdef USE_TINYUSB

#include <stdint.h>  // for uint8_t, int8_t, uint16_t

#include "Adafruit_TinyUSB.h"
#include "kaleidoscope/driver/hid/Base.h"
#include "kaleidoscope/driver/hid/base/Keyboard.h"
#include "kaleidoscope/driver/hid/apis/ConsumerControlAPI.h"
#include "kaleidoscope/driver/hid/apis/MouseAPI.h"
#include "kaleidoscope/driver/hid/apis/SystemControlAPI.h"

// IWYU pragma: no_include "DeviceAPIs/AbsoluteMouseAPI.hpp"

namespace kaleidoscope {
namespace driver {
namespace hid {
namespace tinyusb {

class HIDD : public Adafruit_USBD_HID {
 public:
  HIDD(uint8_t const *desc_report,
       uint16_t len,
       uint8_t protocol    = HID_ITF_PROTOCOL_NONE,
       uint8_t interval_ms = 4)
    : Adafruit_USBD_HID(desc_report, len, protocol, interval_ms) {}

 protected:
  bool sendReport(uint8_t report_id, void const *report, uint8_t len) {
    if (TinyUSBDevice.suspended()) {
      TinyUSBDevice.remoteWakeup();
      /*
       * The spec says we should wait for 30ms for a resume.
       * (USB 2.0 §7.1.7.7)
       *
       * 20ms of host echoing the resume downstream, and 10ms of recovery time
       * once the bus has resumed.
       *  
       * Instead of doing that, we roll that into our blocking delay below, so that
       * if we're ready earlier, we don't block the runloop and potentially miss
       * other events
       */
    }
    /*
     * Block for up to 250ms for endpoint to become ready. If the host is polling,
     * this should take about 1ms (the requested polling interval, but the
     * host is allowed to poll less frequently). On resume, especially remote
     * wakeup on macOS, not only can the resume take longer much than the
     * minimum (over 2 seconds, instead of 20ms), the recovery period can be
     * over 100ms (instead of 10ms).
     *
     * This will mitigate, but not necessarily eliminate, stuck key situations
     * during resume. One scenario is a key down report being queued while in
     * the recovery period, where the bus has resumed, but the host isn't
     * polling yet. The endpoint will be ready, but the transmission won't
     * happen until the host resumes polling. The transmission of the key up
     * event might time out, leading to a stuck key situation.
     *
     * Unfortunately, we usually can't abort a packet transmission once queued.
     */
    for (int timeout = 250; timeout-- && !ready();) {
      delay(1);
    }
    if (!ready()) {
      return false;
    }
    return Adafruit_USBD_HID::sendReport(report_id, report, len);
  }
};

}  // namespace tinyusb
}  // namespace hid
}  // namespace driver
}  // namespace kaleidoscope

#endif /* USE_TINYUSB */
