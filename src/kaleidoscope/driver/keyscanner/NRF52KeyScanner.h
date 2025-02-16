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
#include "kaleidoscope/driver/keyscanner/Base.h"
#include "kaleidoscope/keyswitch_state.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "Arduino.h"
#include "nrf_timer.h"

namespace kaleidoscope {
namespace driver {
namespace keyscanner {

/// @brief Base properties for NRF52-based key scanners
struct NRF52KeyScannerProps : kaleidoscope::driver::keyscanner::BaseProps {
  /// @brief Interval between key matrix scans in microseconds
  static constexpr uint32_t keyscan_interval_micros = 1500;

  /// @brief Type used to store the state of a matrix row
  typedef uint16_t RowState;

  /*
   * The following two arrays must be shadowed by the descendant keyscanner
   * description class to define the actual matrix pins.
   */
  static constexpr uint8_t matrix_row_pins[] = {};
  static constexpr uint8_t matrix_col_pins[] = {};

  // Define the KeyScanner type that will use these props
  template<typename _Props>
  class NRF52KeyScanner;

  typedef NRF52KeyScanner<NRF52KeyScannerProps> KeyScanner;
};

// Base interface for timer handler
class TimerHandlerInterface {
 public:
  virtual void handleTimer() = 0;
  virtual void postReadMatrix() = 0;
  virtual ~TimerHandlerInterface() = default;
};

// Global pointer to active scanner instance
extern TimerHandlerInterface* active_scanner_;

template<typename _Props>
class NRF52KeyScanner : public Base<_Props>, public TimerHandlerInterface {
 private:
  typedef NRF52KeyScanner<_Props> ThisType;

  // Packed event structure to minimize queue memory usage
  struct Event {
    uint8_t row : 4;
    uint8_t col : 4;
    bool pressed : 1;
  };

  // Static queue storage and control structures
  static constexpr size_t EVENT_QUEUE_SIZE = 64;
  static StaticQueue_t event_queue_buffer_;
  static uint8_t event_queue_storage_[EVENT_QUEUE_SIZE * sizeof(Event)];
  static QueueHandle_t event_queue_handle_;

 protected:
  struct row_state_t {
    typename _Props::RowState previous;
    typename _Props::RowState current;
  };

  static row_state_t matrix_state_[_Props::matrix_rows];
  static uint32_t next_scan_at_;

  // Protected methods for subclasses to modify matrix state
  /// @brief Queue a key event for processing
  /// This is used by external code (like encoders) that handles its own debouncing
  /// @return true if event was queued, false if queue was full
  bool queueKeyEvent(uint8_t row, uint8_t col, bool state) {
    Event event = {row, col, state};
    BaseType_t higher_priority_task_woken = pdFALSE;
    
    // Use ISR version since this might be called from timer interrupt
    BaseType_t success = xQueueSendFromISR(
      event_queue_handle_,
      &event,
      &higher_priority_task_woken
    );
    
    // Handle potential task switch if needed
    portYIELD_FROM_ISR(higher_priority_task_woken);
    
    return success == pdTRUE;
  }

  /// @brief Directly set the matrix state for a key
  /// This is used by external code (like encoders) that needs immediate state changes
  void setMatrixState(uint8_t row, uint8_t col, bool state) {
    if (state) {
      matrix_state_[row].current |= (1UL << col);
    } else {
      matrix_state_[row].current &= ~(1UL << col);
    }
  }

  /// @brief Update the matrix state from a queued event (internal use only)
  void applyQueuedEvent(const Event& event) {
    setMatrixState(event.row, event.col, event.pressed);
  }

  bool getMatrixState(uint8_t row, uint8_t col) const {
    return bitRead(matrix_state_[row].current, col);
  }

  void updateMatrixScanKeyState() {
    // Update previous state with current state
    for (uint8_t row = 0; row < _Props::matrix_rows; row++) {
      matrix_state_[row].previous = matrix_state_[row].current;
    }
  }

 public:
  typedef typename _Props::KeyAddr KeyAddr;

  /// @brief Suspend the timer interrupt used for matrix scanning
  void suspendTimer() {
    // Stop the timer
    NRF_TIMER1->TASKS_STOP = 1;
    // Disable timer interrupt
    NVIC_DisableIRQ(TIMER1_IRQn);
    // Clear any pending interrupts
    NVIC_ClearPendingIRQ(TIMER1_IRQn);
  }

  /// @brief Resume the timer interrupt used for matrix scanning
  void resumeTimer() {
    // Clear any pending interrupts
    NVIC_ClearPendingIRQ(TIMER1_IRQn);
    // Re-enable timer interrupt
    NVIC_EnableIRQ(TIMER1_IRQn);
    // Start the timer
    NRF_TIMER1->TASKS_START = 1;
  }

  void setup() {
    // Store this instance as the active scanner
    active_scanner_ = this;

    // Create the event queue
    event_queue_handle_ = xQueueCreateStatic(
      EVENT_QUEUE_SIZE,
      sizeof(Event),
      event_queue_storage_,
      &event_queue_buffer_
    );

    // Configure GPIO
    for (uint8_t i = 0; i < _Props::matrix_rows; i++) {
      pinMode(_Props::matrix_row_pins[i], OUTPUT);
      digitalWrite(_Props::matrix_row_pins[i], HIGH);
    }
    for (uint8_t i = 0; i < _Props::matrix_columns; i++) {
      pinMode(_Props::matrix_col_pins[i], INPUT_PULLUP);
    }

    // Configure hardware timer for scanning
    NRF_TIMER1->MODE = TIMER_MODE_MODE_Timer;  // Set timer mode
    NRF_TIMER1->BITMODE = TIMER_BITMODE_BITMODE_32Bit;  // 32-bit timer
    NRF_TIMER1->PRESCALER = 5;  // 1 MHz, to get our interval in microseconds
    NRF_TIMER1->CC[0] = _Props::keyscan_interval_micros;  // Set compare value
    NRF_TIMER1->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Msk;  // Auto clear on compare match
    NRF_TIMER1->INTENSET = TIMER_INTENSET_COMPARE0_Msk;  // Enable compare interrupt

    // Enable timer interrupt
    NVIC_SetPriority(TIMER1_IRQn, 3);  // Set priority (0-3, lower is higher)
    NVIC_ClearPendingIRQ(TIMER1_IRQn);
    NVIC_EnableIRQ(TIMER1_IRQn);

    // Start timer
    NRF_TIMER1->TASKS_START = 1;
  }

  void readMatrix() {
    // Matrix is read in the timer handler
  }

  virtual void postReadMatrix() {
    // Override this to handle post-read matrix operations like encoder handling
  }

  /// @brief Process any buffered events and update matrix state
  void scanMatrix() {
    Event event;
    
    // Process any buffered events
    while (xQueueReceive(event_queue_handle_, &event, 0) == pdTRUE) {
      // Update matrix state with latest event
      applyQueuedEvent(event);
    }
    
    // Process any state changes
    actOnMatrixScan();
    
    // Update previous state for next scan
    updateMatrixScanKeyState();
  }

  uint8_t pressedKeyswitchCount() {
    uint8_t count = 0;
    for (uint8_t r = 0; r < _Props::matrix_rows; r++) {
      count += __builtin_popcount(matrix_state_[r].current);
    }
    return count;
  }

  bool isKeyswitchPressed(KeyAddr key_addr) {
    return bitRead(matrix_state_[key_addr.row()].current, key_addr.col());
  }

  /// @brief Check if there are any events queued in the buffer
  /// @return true if there are events waiting to be processed
  bool hasQueuedEvents() const {
    return uxQueueMessagesWaiting(event_queue_handle_) > 0;
  }

  uint8_t previousPressedKeyswitchCount() {
    uint8_t count = 0;
    for (uint8_t r = 0; r < _Props::matrix_rows; r++) {
      count += __builtin_popcount(matrix_state_[r].previous);
    }
    return count;
  }

  bool wasKeyswitchPressed(KeyAddr key_addr) {
    return bitRead(matrix_state_[key_addr.row()].previous, key_addr.col());
  }

  /// @brief Process any changes in the matrix state and generate key events
  void actOnMatrixScan() {
    // Process any state changes
    for (uint8_t row = 0; row < _Props::matrix_rows; row++) {
      typename _Props::RowState changes = matrix_state_[row].previous ^ matrix_state_[row].current;
      if (changes) {
        for (uint8_t col = 0; col < _Props::matrix_columns; col++) {
          if (bitRead(changes, col)) {
            // Get previous and current state to form keyState
            uint8_t keyState = (bitRead(matrix_state_[row].previous, col) << 0) | 
                              (bitRead(matrix_state_[row].current, col) << 1);
            ThisType::handleKeyswitchEvent(Key_NoKey, KeyAddr(row, col), keyState);
          }
        }
      }
    }
  }

  // Timer handler interface implementation
  void handleTimer() override {
    for (uint8_t row = 0; row < _Props::matrix_rows; row++) {
      digitalWrite(_Props::matrix_row_pins[row], LOW);
      delayMicroseconds(10);
      for (uint8_t col = 0; col < _Props::matrix_columns; col++) {
        bool current_state = !digitalRead(_Props::matrix_col_pins[col]);
        
        if (current_state != getMatrixState(row, col)) {
          if (++debounce_counters_[row][col] >= DEBOUNCE_THRESHOLD) {
            if (!queueKeyEvent(row, col, current_state)) {
              // Queue is full, we'll try again next scan
              continue;
            }
            debounce_counters_[row][col] = 0;
          }
        } else {
          debounce_counters_[row][col] = 0;
        }
      }
      digitalWrite(_Props::matrix_row_pins[row], HIGH);
    }
  }

 private:  
  static constexpr uint8_t DEBOUNCE_THRESHOLD = 3;
  static uint8_t debounce_counters_[_Props::matrix_rows][_Props::matrix_columns];

  static void gpio_handler(uint32_t pin) {
    // Wake-on-key handler
  }
};

// Static member initialization
template<typename _Props>
uint8_t NRF52KeyScanner<_Props>::debounce_counters_[_Props::matrix_rows][_Props::matrix_columns] = {0};

template<typename _Props>
StaticQueue_t NRF52KeyScanner<_Props>::event_queue_buffer_;

template<typename _Props>
uint8_t NRF52KeyScanner<_Props>::event_queue_storage_[EVENT_QUEUE_SIZE * sizeof(typename NRF52KeyScanner<_Props>::Event)];

template<typename _Props>
QueueHandle_t NRF52KeyScanner<_Props>::event_queue_handle_ = nullptr;

template<typename _Props>
typename NRF52KeyScanner<_Props>::row_state_t NRF52KeyScanner<_Props>::matrix_state_[_Props::matrix_rows];

template<typename _Props>
uint32_t NRF52KeyScanner<_Props>::next_scan_at_ = 0;

} // namespace keyscanner
} // namespace driver
} // namespace kaleidoscope

#endif // ARDUINO_ARCH_NRF52
