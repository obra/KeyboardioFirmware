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

#include "kaleidoscope/driver/speaker/Base.h"


namespace kaleidoscope {
namespace driver {
namespace speaker {

template<typename _SpeakerProps>
void Base<_SpeakerProps>::playTune(const uint16_t *notes, const uint16_t *durations, uint8_t length) {
  if (length > _SpeakerProps::kMaxTuneLength)
    length = _SpeakerProps::kMaxTuneLength;

  memcpy(current_notes_, notes, length * sizeof(uint16_t));
  memcpy(current_durations_, durations, length * sizeof(uint16_t));
  current_length_ = length;
  current_index_  = 0;
  next_note_time_ = 0;
}

template<typename _SpeakerProps>
void Base<_SpeakerProps>::stopTune() {
  current_length_ = 0;
  current_index_  = 0;
  next_note_time_ = 0;
  stopTone();
}

template<typename _SpeakerProps>
void Base<_SpeakerProps>::update() {
  if (current_length_ == 0 || current_index_ >= current_length_)
    return;

  uint32_t now = millis();
  if (now < next_note_time_)
    return;

  if (next_note_time_ == 0) {
    // Start playing the first note
    playTone(current_notes_[current_index_], current_durations_[current_index_]);
    next_note_time_ = now + current_durations_[current_index_];
  } else {
    // Move to next note
    current_index_++;
    if (current_index_ < current_length_) {
      playTone(current_notes_[current_index_], current_durations_[current_index_]);
      next_note_time_ = now + current_durations_[current_index_];
    } else {
      stopTune();
    }
  }
}

template class Base<BaseProps>;

}  // namespace speaker
}  // namespace driver
}  // namespace kaleidoscope
