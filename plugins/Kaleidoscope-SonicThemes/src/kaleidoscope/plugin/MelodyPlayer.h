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

#include <Arduino.h>
#include <stdint.h>
#include "kaleidoscope/Runtime.h"

namespace kaleidoscope {
namespace plugin {

struct Note {
  uint16_t frequency;  // Hz (20-25000)
  uint16_t duration;   // ms
};

class MelodyPlayer {
 public:
  MelodyPlayer()
    : current_melody_(nullptr), melody_length_(0), current_note_(0), is_playing_(false), next_note_time_(0) {}

  void playMelody(const Note *melody, uint8_t length);
  void stop();
  void update();

 private:
  void playNextNote();

  const Note *current_melody_;
  uint8_t melody_length_;
  uint8_t current_note_;
  bool is_playing_;
  uint32_t next_note_time_;
};

}  // namespace plugin
}  // namespace kaleidoscope