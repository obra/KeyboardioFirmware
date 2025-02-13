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

// Initialize static members
TaskHandle_t HIDD::report_task_handle_ = nullptr;
SemaphoreHandle_t HIDD::report_semaphore_ = nullptr;
volatile bool HIDD::task_running_ = false;

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
  // Initialize BLE HID service first
  err_t status = BLEHidGeneric::begin();
  if (status != ERROR_NONE) {
    return status;
  }

  // Create semaphore for signaling new reports
  report_semaphore_ = xSemaphoreCreateBinary();
  if (!report_semaphore_) {
    end();  // Clean up if semaphore creation fails
    return NRF_ERROR_INTERNAL;
  }

  // Create task for processing reports
  task_running_ = true;
  BaseType_t task_created = xTaskCreate(
    processReportQueue_,
    "HID_Reports",
    configMINIMAL_STACK_SIZE * 2,
    this,
    2,  // Priority - higher than main loop, lower than BLE stack
    &report_task_handle_
  );

  if (task_created != pdPASS) {
    end();  // Clean up if task creation fails
    return NRF_ERROR_INTERNAL;
  }

  return ERROR_NONE;
}

void HIDD::setLEDcb(BLECharacteristic::write_cb_t fp) {
  setOutputReportCallback(RID_KEYBOARD, fp);
  _chr_boot_keyboard_output->setWriteCallback(fp);
}

void HIDD::end() {
  task_running_ = false;
  
  // Signal task to exit if it's waiting
  if (report_semaphore_) {
    xSemaphoreGive(report_semaphore_);
  }
  
  // Wait for task to finish (with timeout)
  if (report_task_handle_) {
    for (int i = 0; i < 10 && eTaskGetState(report_task_handle_) != eDeleted; i++) {
      delay(10);
    }
    report_task_handle_ = nullptr;
  }

  if (report_semaphore_) {
    vSemaphoreDelete(report_semaphore_);
    report_semaphore_ = nullptr;
  }

  clearReportQueue();
}

void HIDD::startReportProcessing() {
  if (!task_running_) {
    task_running_ = true;
    xSemaphoreGive(report_semaphore_);  // Wake up task if it's waiting
  }
}

void HIDD::stopReportProcessing() {
  task_running_ = false;
}

void HIDD::clearReportQueue() {
  while (!report_queue_.isEmpty()) {
    report_queue_.pop();
  }
}

bool HIDD::processNextReport_() {
  if (report_queue_.isEmpty()) {
    return true;
  }

  // Get a copy of the report
  QueuedReport report = report_queue_.peek();
  
  // Check connection before attempting send
  BLEConnection* conn = Bluefruit.Connection(0);
  if (!conn || !conn->connected()) {
    return false;
  }

  bool success = false;
  switch (report.type) {
    case ReportType::BootKeyboard:
      success = BLEHidGeneric::bootKeyboardReport(report.data, report.length);
      break;
    case ReportType::BootMouse:
      success = BLEHidGeneric::bootMouseReport(report.data, report.length);
      break;
    case ReportType::Input:
      success = BLEHidGeneric::inputReport(report.report_id, report.data, report.length);
      break;
  }

  if (success) {
    report_queue_.pop();
    return true;
  }

  // Update the retries count in the queue
  if (report.retries_left > 0) {
    report.retries_left--;
    // If we're out of retries, remove the report
    if (report.retries_left == 0) {
      report_queue_.pop();
    }
  }

  return false;
}

void HIDD::processReportQueue_(void* pvParameters) {
  HIDD* hidd = static_cast<HIDD*>(pvParameters);
  uint8_t consecutive_failures = 0;
  
  while (true) {  // Task runs forever, controlled by task_running_ flag
    if (!hidd->task_running_) {
      vTaskDelay(pdMS_TO_TICKS(100));  // Sleep when not active
      continue;
    }

    if (xSemaphoreTake(hidd->report_semaphore_, pdMS_TO_TICKS(10)) == pdTRUE) {
      while (!hidd->report_queue_.isEmpty()) {
        if (!hidd->processNextReport_()) {
          // Failed to send, apply backoff
          consecutive_failures++;
          uint32_t delay = calculateBackoffDelay_(consecutive_failures);
          vTaskDelay(pdMS_TO_TICKS(delay));
          break;
        } else {
          consecutive_failures = 0;
        }
      }
    }
  }

  vTaskDelete(nullptr);  // Clean exit path if we ever break out of the loop
}

bool HIDD::queueReport_(ReportType type, uint8_t report_id, const void* data, uint8_t length) {
// If we decide we don't want to queue reports when we're not yet processing them,
// we can just return false here. 
// if (!task_running_) return false;
  
  if (report_queue_.isFull()) {
    // Make space by removing oldest report
    report_queue_.pop();
  }

  QueuedReport report;
  report.type = type;
  report.report_id = report_id;
  memcpy(report.data, data, length);
  report.length = length;
  report.retries_left = MAX_RETRIES;

  report_queue_.push(report);
  xSemaphoreGive(report_semaphore_);
  
  return true;
}

bool HIDD::sendBootKeyboardReport(const void* data, uint8_t length) {
  return queueReport_(ReportType::BootKeyboard, 0, data, length);
}

bool HIDD::sendBootMouseReport(const void* data, uint8_t length) {
  return queueReport_(ReportType::BootMouse, 0, data, length);
}

bool HIDD::sendInputReport(uint8_t report_id, const void* data, uint8_t length) {
  return queueReport_(ReportType::Input, report_id, data, length);
}


HIDD blehid;

}  // namespace bluefruit
}  // namespace hid
}  // namespace driver
}  // namespace kaleidoscope

#endif /* ARDUINO_ARCH_NRF52 */
