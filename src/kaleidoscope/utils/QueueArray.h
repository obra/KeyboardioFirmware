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
#include <FreeRTOS.h>
#include <task.h>
#endif

namespace kaleidoscope {

// Platform-specific critical section macros
#ifdef ARDUINO_ARCH_NRF52
  #define CRITICAL_SECTION_START()  taskENTER_CRITICAL()
  #define CRITICAL_SECTION_END()    taskEXIT_CRITICAL()
#else
  #define CRITICAL_SECTION_START()  noInterrupts()
  #define CRITICAL_SECTION_END()    interrupts()
#endif

template<typename T, size_t SIZE = 32>
class QueueArray {
 public:
  QueueArray() : head_(0), tail_(0) {}

  bool push(const T& item) {
    CRITICAL_SECTION_START();

    size_t next_tail = (tail_ + 1) % SIZE;
    if (next_tail == head_) {  // Full check
      CRITICAL_SECTION_END();
      return false;
    }

    buffer_[tail_] = item;
    tail_ = next_tail;

    CRITICAL_SECTION_END();
    return true;
  }

  T pop() {
    CRITICAL_SECTION_START();

    if (head_ == tail_) {  // Empty check
      CRITICAL_SECTION_END();
      return T();
    }

    T item = buffer_[head_];
    head_ = (head_ + 1) % SIZE;

    CRITICAL_SECTION_END();
    return item;
  }
  T peek() const {
    CRITICAL_SECTION_START();
    
    if (head_ == tail_) {  // Empty check
      CRITICAL_SECTION_END();
      return T();
    }

    T item = buffer_[head_];
    
    CRITICAL_SECTION_END();
    return item;
  }
  
  bool isEmpty() const {
    CRITICAL_SECTION_START();
    bool empty = (head_ == tail_);
    CRITICAL_SECTION_END();
    return empty;
  }

  bool isFull() const {
    CRITICAL_SECTION_START();
    bool full = ((tail_ + 1) % SIZE) == head_;
    CRITICAL_SECTION_END();
    return full;
  }

  size_t size() const {
    CRITICAL_SECTION_START();
    size_t t = tail_;
    size_t h = head_;
    size_t count = t >= h ? t - h : SIZE - (h - t);
    CRITICAL_SECTION_END();
    return count;
  }

 private:
  T buffer_[SIZE];
  volatile size_t head_;
  volatile size_t tail_;
};

#undef CRITICAL_SECTION_START
#undef CRITICAL_SECTION_END

} // namespace kaleidoscope
