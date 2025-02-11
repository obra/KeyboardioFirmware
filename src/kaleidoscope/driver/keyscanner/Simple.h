/* Kaleidoscope - Firmware for computer input devices
 * Copyright (C) 2018-2025 Keyboard.io, inc.
 *
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

#include <stdint.h>                               // for uint16_t, uint8_t, uint32_t
#include "kaleidoscope/driver/keyscanner/Base.h"  // for BaseProps
#include "kaleidoscope/driver/keyscanner/None.h"  // for None


namespace kaleidoscope {
namespace driver {
namespace keyscanner {


struct SimpleProps : kaleidoscope::driver::keyscanner::BaseProps {
  static const uint32_t keyscan_interval_micros = 1500;
  typedef uint16_t RowState;

  /*
   * The following two lines declare an empty array. Both of these must be
   * shadowed by the descendant keyscanner description class.
   */
  static constexpr uint8_t matrix_row_pins[] = {};
  static constexpr uint8_t matrix_col_pins[] = {};
};


#ifndef KALEIDOSCOPE_VIRTUAL_BUILD
template<typename _KeyScannerProps>
class Simple : public kaleidoscope::driver::keyscanner::Base<_KeyScannerProps> {
 protected:
  /*
    each of these variables are storing the state for a row of keys

    so for key 0, the counter is represented by db0[0] and db1[0]
    and the state in debounced_state[0].
  */
  struct debounce_t {
    typename _KeyScannerProps::RowState db0;              // counter bit 0
    typename _KeyScannerProps::RowState db1;              // counter bit 1
    typename _KeyScannerProps::RowState debounced_state;  // debounced state
  };

  struct row_state_t {
    typename _KeyScannerProps::RowState previous;
    typename _KeyScannerProps::RowState current;
    debounce_t debouncer;
  };

 private:
  typedef Simple<_KeyScannerProps> ThisType;
  typedef _KeyScannerProps KeyScannerProps_;
  static row_state_t matrix_state_[_KeyScannerProps::matrix_rows];
  static uint32_t next_scan_at_;

 protected:
  // Protected methods for subclasses to modify matrix state
  void setKeyState(uint8_t row, uint8_t col, bool state) {
    if (state) {
      matrix_state_[row].current |= (1UL << col);
    } else {
      matrix_state_[row].current &= ~(1UL << col);
    }
  }

  bool getKeyState(uint8_t row, uint8_t col) const {
    return (matrix_state_[row].current & (1UL << col)) != 0;
  }

 public:
  void setup() {
    static_assert(
      sizeof(_KeyScannerProps::matrix_row_pins) > 0,
      "The key scanner description has an empty array of matrix row pins.");
    static_assert(
      sizeof(_KeyScannerProps::matrix_col_pins) > 0,
      "The key scanner description has an empty array of matrix column pins.");


    for (uint8_t i = 0; i < _KeyScannerProps::matrix_columns; i++) {
      pinMode(_KeyScannerProps::matrix_col_pins[i], INPUT_PULLUP);
    }

    for (uint8_t i = 0; i < _KeyScannerProps::matrix_rows; i++) {
      pinMode(_KeyScannerProps::matrix_row_pins[i], OUTPUT);
      digitalWrite(_KeyScannerProps::matrix_row_pins[i], HIGH);
    }
  }


  __attribute__((optimize(3))) void readMatrix(void) {
    typename _KeyScannerProps::RowState any_debounced_changes = 0;

    for (uint8_t current_row = 0; current_row < _KeyScannerProps::matrix_rows; current_row++) {
      digitalWrite(_KeyScannerProps::matrix_row_pins[current_row], LOW);
      typename _KeyScannerProps::RowState hot_pins = readCols();
      digitalWrite(_KeyScannerProps::matrix_row_pins[current_row], HIGH);


      any_debounced_changes |= debounce(hot_pins, &matrix_state_[current_row].debouncer);

      if (any_debounced_changes) {
        for (uint8_t current_row = 0; current_row < _KeyScannerProps::matrix_rows; current_row++) {
          matrix_state_[current_row].current = matrix_state_[current_row].debouncer.debounced_state;
        }
      }
    }
    postReadMatrix();
  }

  virtual void postReadMatrix() {}

  void scanMatrix() {
    uint32_t current_micros_ = micros();
    if (current_micros_ >= next_scan_at_) {
      next_scan_at_ = current_micros_ + _KeyScannerProps::keyscan_interval_micros;
      readMatrix();
    }
    actOnMatrixScan();
    updateMatrixScanKeyState();
  }

  void __attribute__((optimize(3))) actOnMatrixScan() {
    for (uint8_t row = 0; row < _KeyScannerProps::matrix_rows; row++) {
      for (uint8_t col = 0; col < _KeyScannerProps::matrix_columns; col++) {
        uint8_t keyState = (bitRead(matrix_state_[row].previous, col) << 0) | (bitRead(matrix_state_[row].current, col) << 1);
        if (keyState) {
          ThisType::handleKeyswitchEvent(Key_NoKey, typename _KeyScannerProps::KeyAddr(row, col), keyState);
        }
      }
    }
  }

  void __attribute__((optimize(3))) updateMatrixScanKeyState (){ 
    for (uint8_t row = 0; row < _KeyScannerProps::matrix_rows; row++) {
    matrix_state_[row].previous = matrix_state_[row].current;
    }

  }

  uint8_t pressedKeyswitchCount() {
    uint8_t count = 0;

    for (int8_t r = 0; r < _KeyScannerProps::matrix_rows; r++) {
      count += __builtin_popcount(matrix_state_[r].current);
    }
    return count;
  }
  bool isKeyswitchPressed(typename _KeyScannerProps::KeyAddr key_addr) {
    return (bitRead(matrix_state_[key_addr.row()].current, key_addr.col()) != 0);
  }

  uint8_t previousPressedKeyswitchCount() {
    uint8_t count = 0;

    for (int8_t r = 0; r < _KeyScannerProps::matrix_rows; r++) {
      count += __builtin_popcount(matrix_state_[r].previous);
    }
    return count;
  }
  bool wasKeyswitchPressed(typename _KeyScannerProps::KeyAddr key_addr) {
    return (bitRead(matrix_state_[key_addr.row()].previous,
                    key_addr.col()) != 0);
  }


 private:
  /*
   * This function has loop unrolling disabled on purpose: we want to give the
   * hardware enough time to produce stable PIN reads for us. If we unroll the
   * loop, we will not have that, because even with the NOP, the codepath is too
   * fast. If we don't have stable reads, then entire rows or columns will behave
   * erratically.
   *
   * For this reason, we ask the compiler to not unroll our loop, which in turn,
   * gives hardware enough time to produce stable reads, at the cost of a little
   * bit of speed.
   *
   * Do not remove the attribute!
   * TODO : see if this is true on ARM. Jesse thinks it was an AVRism
   */
  // __attribute__((optimize("no-unroll-loops")))
  typename _KeyScannerProps::RowState
  readCols() {
    typename _KeyScannerProps::RowState hot_pins = 0;
    for (uint8_t i = 0; i < _KeyScannerProps::matrix_columns; i++) {
      //     asm("NOP");  // We need to pause a beat before reading or we may read before the pin is hot
      hot_pins |= (!digitalRead(_KeyScannerProps::matrix_col_pins[i]) << i);
    }

    return hot_pins;
  }

  static inline typename _KeyScannerProps::RowState debounce(
    typename _KeyScannerProps::RowState sample, debounce_t *debouncer) {
    typename _KeyScannerProps::RowState delta, changes;

    // Use xor to detect changes from last stable state:
    // if a key has changed, it's bit will be 1, otherwise 0
    delta = sample ^ debouncer->debounced_state;

    // Increment counters and reset any unchanged bits:
    // increment bit 1 for all changed keys
    debouncer->db1 = ((debouncer->db1) ^ (debouncer->db0)) & delta;
    // increment bit 0 for all changed keys
    debouncer->db0 = ~(debouncer->db0) & delta;

    // Calculate returned change set: if delta is still true
    // and the counter has wrapped back to 0, the key is changed.

    changes = ~(~delta | (debouncer->db0) | (debouncer->db1));
    // Update state: in this case use xor to flip any bit that is true in changes.
    debouncer->debounced_state ^= changes;

    return changes;
  }
};
#else   // ifndef KALEIDOSCOPE_VIRTUAL_BUILD
template<typename _KeyScannerProps>
class Simple : public keyscanner::None {};
#endif  // ifndef KALEIDOSCOPE_VIRTUAL_BUILD

}  // namespace keyscanner
}  // namespace driver
}  // namespace kaleidoscope
