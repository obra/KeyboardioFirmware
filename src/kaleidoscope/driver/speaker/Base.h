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
#include <Arduino.h>

namespace kaleidoscope {
namespace driver {
namespace speaker {

struct BaseProps {
  static constexpr uint8_t kMaxTuneLength = 16;
};

template<typename _SpeakerProps>
class Base {
 public:
  Base() {}

  void setup() {}
  void playTone(unsigned int frequency, uint32_t duration) {}
  void playTone(unsigned int frequency) {
    playTone(frequency, 0);  // 0 duration means play until stopped
  }
  void stopTone() {}
  bool isPlaying() { return false; }

  void playTune(const uint16_t *notes, const uint16_t *durations, uint8_t length);
  void stopTune();
  void update();

 protected:
  uint16_t current_notes_[_SpeakerProps::kMaxTuneLength];
  uint16_t current_durations_[_SpeakerProps::kMaxTuneLength];
  uint8_t current_length_;
  uint8_t current_index_;
  uint32_t next_note_time_;
};

}  // namespace speaker
}  // namespace driver
}  // namespace kaleidoscope
