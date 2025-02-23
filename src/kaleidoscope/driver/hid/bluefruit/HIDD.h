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
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"

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

struct QueuedReport {
  ReportType type;
  uint8_t report_id;
  uint8_t data[32];  // Max HID report size
  uint8_t length;
  uint8_t retries_left;
};

class HIDD : public BLEHidGeneric {
 public:
  HIDD();
  virtual err_t begin() override;
  void end();
  void setLEDcb(BLECharacteristic::write_cb_t fp);

  /**
   * Send a boot keyboard report with retries
   * @param data Report data
   * @param length Length of report
   * @return true if report was successfully queued
   */
  bool sendBootKeyboardReport(const void* data, uint8_t length);

  /**
   * Send a boot mouse report with retries
   * @param data Report data
   * @param length Length of report
   * @return true if report was successfully queued
   */
  bool sendBootMouseReport(const void* data, uint8_t length);

  /**
   * Send an input report with retries
   * @param report_id Report ID
   * @param data Report data
   * @param length Length of report
   * @return true if report was successfully queued
   */
  bool sendInputReport(uint8_t report_id, const void* data, uint8_t length);

  /**
   * Clear all pending reports from the queue
   * Called when disconnecting or changing connection state
   */
  void clearReportQueue();

  /**
   * Start processing reports from the queue
   * Called when a BLE connection is established
   */
  void startReportProcessing();

  /**
   * Stop processing reports from the queue
   * Called when BLE connection is lost or terminated
   */
  void stopReportProcessing();

 private:
  static constexpr size_t QUEUE_SIZE = 512; // Absurdly large queue size
  static constexpr uint8_t MAX_RETRIES = 200; 
  static constexpr uint8_t RETRY_DELAY_MS = 15;  // Time between retries
  static constexpr uint8_t KEYSTROKE_INTERVAL_MS = 5;
  ;  // Min time between keystrokes
  // Queue storage and handle
  StaticQueue_t queue_buffer_;
  uint8_t queue_storage_[QUEUE_SIZE * sizeof(QueuedReport)];
  QueueHandle_t queue_handle_;

  // Task management
  static TaskHandle_t report_task_handle_;
  static SemaphoreHandle_t report_semaphore_;
  static volatile bool task_running_;

  /**
   * FreeRTOS task function that processes queued reports
   * Runs continuously but sleeps when report processing is stopped
   * @param pvParameters Pointer to the HIDD instance
   */
  static void processReportQueue_(void* pvParameters);

  /**
   * Process the next report in the queue
   * @return true if report was sent successfully
   */
  bool processNextReport_();

  /**
   * Queue a report for sending with retry logic
   * @param type Type of report (Boot Keyboard, Boot Mouse, or Input)
   * @param report_id Report ID (used for Input reports)
   * @param data Report data
   * @param length Length of report
   * @return true if report was successfully queued
   */
  bool queueReport_(ReportType type, uint8_t report_id, const void* data, uint8_t length);
};

extern HIDD blehid;

}  // namespace bluefruit
}  // namespace hid
}  // namespace driver
}  // namespace kaleidoscope

#endif /* ARDUINO_ARCH_NRF52 */
