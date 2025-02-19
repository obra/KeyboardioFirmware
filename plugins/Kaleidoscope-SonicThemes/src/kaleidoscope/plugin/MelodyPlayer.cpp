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



#include "kaleidoscope/plugin/MelodyPlayer.h"

namespace kaleidoscope {
namespace plugin {

void MelodyPlayer::playMelody(const Note *melody, uint8_t length) {
  current_melody_ = melody;
  melody_length_ = length;
  current_note_ = 0;
  is_playing_ = true;
  next_note_time_ = 0;
  
  // Start playing the first note immediately
  playNextNote();
}

void MelodyPlayer::stop() {
  if (is_playing_) {
    Runtime.device().speaker().stopTone();
    is_playing_ = false;
  }
}

void MelodyPlayer::update() {
  if (!is_playing_ || current_note_ >= melody_length_) return;

  uint32_t current_time = millis();
  if (current_time >= next_note_time_) {
    playNextNote();
  }
}

void MelodyPlayer::playNextNote() {
  if (current_note_ < melody_length_) {
    const Note &note = current_melody_[current_note_];
    Runtime.device().speaker().playTone(note.frequency, note.duration);
    next_note_time_ = millis() + note.duration;
    current_note_++;
  } else {
    is_playing_ = false;
  }
}

}  // namespace plugin
}  // namespace kaleidoscope