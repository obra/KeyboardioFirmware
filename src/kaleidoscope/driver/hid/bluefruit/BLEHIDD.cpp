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
#include "kaleidoscope/driver/ble/Bluefruit.h"

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
  : BLEHidGeneric(5, 1, 0), queue_handle_(nullptr) {}

err_t HIDD::begin() {
  uint16_t in_lens[] = {
    BOOT_REPORT_LEN,
    sizeof(HID_MouseReport_Data_t),
    sizeof(HID_ConsumerControlReport_Data_t),
    sizeof(HID_SystemControlReport_Data_t),
    sizeof(HID_MouseAbsoluteReport_Data_t),
  };
  uint16_t out_lens[] = {1};

  setHidInfo(BLE_HID_BCD, 0, BLE_HID_INFO_REMOTE_WAKE | BLE_HID_INFO_NORMALLY_CONNECTABLE);
  enableKeyboard(true);
  enableMouse(true);
  setReportLen(in_lens, out_lens, NULL);
  setReportMap(report_desc, sizeof(report_desc));
  
  // Initialize BLE HID service first
  err_t status = BLEHidGeneric::begin();
  if (status != ERROR_NONE) {
    return status;
  }

  // Create queue for reports
  queue_handle_ = xQueueCreateStatic(QUEUE_SIZE,
                                   sizeof(QueuedReport),
                                   queue_storage_,
                                   &queue_buffer_);
  if (!queue_handle_) {
    return NRF_ERROR_INTERNAL;
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

  queue_handle_ = nullptr;  // Static queue doesn't need deletion
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
  if (queue_handle_) {
    xQueueReset(queue_handle_);
  }
}

void HIDD::processReportQueue_(void* pvParameters) {
  HIDD* hidd = static_cast<HIDD*>(pvParameters);
  
  while (true) {  // Task runs forever, controlled by task_running_ flag
    if (!hidd->task_running_) {
      vTaskDelay(pdMS_TO_TICKS(100));  // Sleep when not active
      continue;
    }

    // When we are waiting for new reports to send, wait as long as we can 
    // before the next iteration of the loop to use as little power as possible
    if (xSemaphoreTake(hidd->report_semaphore_, portMAX_DELAY) == pdTRUE) {

      while (uxQueueMessagesWaiting(hidd->queue_handle_) > 0) {
        if (!hidd->processNextReport_()) {
          // Failed to send - wait a bit before next retry
          vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
        }
      }
    }
  }

  vTaskDelete(nullptr);  // Clean exit path if we ever break out of the loop
}

bool HIDD::processNextReport_() {
  // Check connection before attempting send
  BLEConnection* conn = Bluefruit.Connection(0);
  if (!conn || !conn->connected()) {
    return true;
  }

  QueuedReport report;
  // Peek at the next report without removing it
  if (xQueuePeek(queue_handle_, &report, 0) != pdTRUE) {
    return true;  // Queue is empty
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
    // Only remove the item if send was successful
    xQueueReceive(queue_handle_, &report, 0);
    return true;
  } else if (report.retries_left > 0) {
    // Update retry count in place without removing the item
    report.retries_left--;
    // Use xQueueOverwrite to safely update the item at the front of the queue
    // This is safe because we know the item exists (we just peeked at it)
    xQueueOverwrite(queue_handle_, &report);
    DEBUG_BLE_MSG("Retrying report, %d retries left", report.retries_left);
    return false;  // Signal failure so we'll wait before next retry
  } else {
    // Out of retries, remove the failed item
    DEBUG_BLE_MSG("Failed to send report, removing from queue");
    xQueueReceive(queue_handle_, &report, 0);
    return true;
  }
}

bool HIDD::queueReport_(ReportType type, uint8_t report_id, const void* data, uint8_t length) {
  if (!queue_handle_) return false;

  // Try to queue the report for 5 seconds before considering eviction
  const TickType_t QUEUE_RETRY_TIMEOUT = pdMS_TO_TICKS(5000); // 5 seconds in ticks
  TickType_t start_time = xTaskGetTickCount();
  
  // Keep trying to queue while there's no space and we haven't exceeded our timeout
  while (uxQueueSpacesAvailable(queue_handle_) == 0) {
    // Calculate remaining time in our retry window
    TickType_t now = xTaskGetTickCount();
    TickType_t elapsed = now - start_time;
    
    // If we've exceeded our 5-second retry window, break out to try eviction
    if (elapsed >= QUEUE_RETRY_TIMEOUT) {
      DEBUG_BLE_MSG("Failed to queue report, exceeded retry window");
      break;
    }
    
    // Wait a short time before checking again
    // Use shorter delays early in the retry window, longer ones later
    TickType_t delay_time = (elapsed < pdMS_TO_TICKS(1000)) ? 
                           pdMS_TO_TICKS(1) :  // 1ms delays in first second
                           pdMS_TO_TICKS(10);  // 10ms delays after that
    vTaskDelay(delay_time);
  }

  // If queue is still full after retrying, we need to evict the oldest report
  if (uxQueueSpacesAvailable(queue_handle_) == 0) {
    // Log or track this eviction if needed
    QueuedReport dummy;
    if (xQueueReceive(queue_handle_, &dummy, 0) != pdTRUE) {
      // If we somehow failed to remove an item, we can't queue the new one
      return false;
    }
  }

  // Prepare the report structure for queuing
  QueuedReport report;
  report.type = type;
  report.report_id = report_id;
  memcpy(report.data, data, length);
  report.length = length;
  report.retries_left = MAX_RETRIES;
  
  // Queue the new report
  BaseType_t success = xQueueSend(queue_handle_, &report, 0);
  
  if (success == pdTRUE) {
    // Signal the processing task that new data is available
    xSemaphoreGive(report_semaphore_);
  }
  
  return success == pdTRUE;
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
