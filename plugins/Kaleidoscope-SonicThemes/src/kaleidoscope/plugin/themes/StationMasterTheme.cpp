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



#include "kaleidoscope/plugin/themes/StationMasterTheme.h"

namespace kaleidoscope {
namespace plugin {
namespace themes {

// Layer change melody - ascending notes for layer up
const Note StationMasterTheme::layer_change_melody_[] PROGMEM = {
  {440, 50},  // A4
  {523, 50},  // C5
};

// Boot melody - train station jingle
const Note StationMasterTheme::boot_melody_[] PROGMEM = {
  {440, 100},  // A4
  {523, 100},  // C5
  {659, 100},  // E5
  {880, 100},  // A5
  {1047, 200}, // C6
};

// Error melody - descending minor third
const Note StationMasterTheme::error_melody_[] PROGMEM = {
  {440, 100},  // A4
  {392, 100},  // G4
  {349, 200},  // F4
};

// Key press melody - short click
const Note StationMasterTheme::key_press_melody_[] PROGMEM = {
  {1047, 10},  // C6 - very short click
};

// Modifier press melody - slightly lower click
const Note StationMasterTheme::modifier_press_melody_[] PROGMEM = {
  {880, 20},   // A5 - slightly longer click
};

// Leader sequence melody - ascending arpeggio
const Note StationMasterTheme::leader_sequence_melody_[] PROGMEM = {
  {440, 50},   // A4
  {523, 50},   // C5
  {659, 50},   // E5
  {880, 100},  // A5
};

// Macro play melody - playful trill
const Note StationMasterTheme::macro_play_melody_[] PROGMEM = {
  {880, 50},   // A5
  {1047, 50},  // C6
  {880, 50},   // A5
  {1047, 100}, // C6
};

const Note* StationMasterTheme::getMelody(SoundEvent event, uint8_t &length) const {
  switch (event) {
    case SoundEvent::LayerChange:
      length = sizeof(layer_change_melody_) / sizeof(Note);
      return layer_change_melody_;
    case SoundEvent::Boot:
      length = sizeof(boot_melody_) / sizeof(Note);
      return boot_melody_;
    case SoundEvent::Error:
      length = sizeof(error_melody_) / sizeof(Note);
      return error_melody_;
    case SoundEvent::KeyPress:
      length = sizeof(key_press_melody_) / sizeof(Note);
      return key_press_melody_;
    case SoundEvent::ModifierPress:
      length = sizeof(modifier_press_melody_) / sizeof(Note);
      return modifier_press_melody_;
    case SoundEvent::LeaderSequence:
      length = sizeof(leader_sequence_melody_) / sizeof(Note);
      return leader_sequence_melody_;
    case SoundEvent::MacroPlay:
      length = sizeof(macro_play_melody_) / sizeof(Note);
      return macro_play_melody_;
    default:
      length = 0;
      return nullptr;
  }
}

const StationMasterTheme station_master_theme;

}  // namespace themes
}  // namespace plugin
}  // namespace kaleidoscope