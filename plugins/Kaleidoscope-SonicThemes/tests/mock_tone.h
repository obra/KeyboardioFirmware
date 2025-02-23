/* Kaleidoscope-SonicThemes -- Audio feedback themes for Kaleidoscope
 * Copyright 2013-2025 Keyboard.io, inc.
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
namespace testing {

class MockTone {
 public:
  struct ToneEvent {
    uint16_t frequency;
    uint16_t duration;
  };

  static void clear() {
    last_tone_  = ToneEvent{0, 0};
    tone_count_ = 0;
  }

  static void playTone(uint16_t freq, uint16_t duration) {
    last_tone_ = ToneEvent{freq, duration};
    tone_count_++;
  }

  static ToneEvent lastTone() { return last_tone_; }
  static uint16_t toneCount() { return tone_count_; }

 private:
  static ToneEvent last_tone_;
  static uint16_t tone_count_;
};

}  // namespace testing
}  // namespace kaleidoscope