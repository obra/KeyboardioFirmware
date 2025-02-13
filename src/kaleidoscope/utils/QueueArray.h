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

namespace kaleidoscope {

template<typename T, size_t SIZE = 32>
class QueueArray {
 public:
  QueueArray() : head_(0), tail_(0), count_(0) {}

  bool push(const T& item) {
    noInterrupts();
    if (count_ >= SIZE) {
      interrupts();
      return false;
    }
    buffer_[tail_] = item;
    tail_ = (tail_ + 1) % SIZE;
    count_++;
    interrupts();
    return true;
  }

  T pop() {
    noInterrupts();
    if (count_ == 0) {
      interrupts();
      return T();
    }
    T item = buffer_[head_];
    head_ = (head_ + 1) % SIZE;
    count_--;
    interrupts();
    return item;
  }

  T peek() const {
    noInterrupts();
    T item = count_ == 0 ? T() : buffer_[head_];
    interrupts();
    return item;
  }

  bool isEmpty() const {
    noInterrupts();
    bool empty = (count_ == 0);
    interrupts();
    return empty;
  }

  bool isFull() const {
    noInterrupts();
    bool full = (count_ >= SIZE);
    interrupts();
    return full;
  }

  size_t size() const {
    noInterrupts();
    size_t count = count_;
    interrupts();
    return count;
  }

 private:
  T buffer_[SIZE];
  volatile size_t head_;
  volatile size_t tail_;
  volatile size_t count_;
};

} // namespace kaleidoscope
