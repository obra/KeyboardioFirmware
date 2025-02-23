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

#include "../Theme.h"
#include "../Phonemes.h"

namespace kaleidoscope {
namespace plugin {

// Formant frequencies for basic vowels
struct Formant {
  uint16_t f1;  // First formant
  uint16_t f2;  // Second formant
  uint16_t duration;
};

// Basic vowel formants (simplified for piezo)
static const Formant PROGMEM vowel_ee = {270, 2300, 150};  // "ee" as in "beep"
static const Formant PROGMEM vowel_ah = {730, 1100, 150};  // "ah" as in "father"
static const Formant PROGMEM vowel_oo = {300, 870, 150};   // "oo" as in "boot"
static const Formant PROGMEM vowel_ay = {660, 1700, 150};  // "ay" as in "layer"

// Number pronunciation patterns
static const Note PROGMEM number_one[] = {
  {440, 100},  // w
  {300, 150},  // u
  {440, 150},  // n
};

static const Note PROGMEM number_two[] = {
  {440, 100},  // t
  {300, 200},  // oo
};

static const Note PROGMEM number_three[] = {
  {440, 100},  // th
  {270, 100},  // r
  {270, 150},  // ee
};

static const Note PROGMEM number_four[] = {
  {440, 100},  // f
  {730, 150},  // o
  {440, 150},  // r
};

static const Note PROGMEM number_five[] = {
  {440, 100},  // f
  {270, 100},  // ah
  {440, 100},  // y
  {440, 150},  // v
};

static const Note PROGMEM number_six[] = {
  {440, 100},  // s
  {270, 100},  // i
  {440, 100},  // k
  {440, 100},  // s
};

static const Note PROGMEM number_seven[] = {
  {440, 100},  // s
  {270, 100},  // e
  {440, 100},  // v
  {270, 100},  // e
  {440, 100},  // n
};

static const Note PROGMEM number_eight[] = {
  {270, 100},  // ey
  {440, 150},  // t
};

// Layer change melodies for each layer
static Note layer_0_notes[8];  // Allocate enough space for "Layer Zero"
static Note layer_1_notes[8];  // Allocate enough space for "Layer One"

// Initialize in constructor or setup
phonemes::create_melody(layer_0_notes,
                        phonemes::LAYER,
                        phonemes::ZERO);

phonemes::create_melody(layer_1_notes,
                        phonemes::LAYER,
                        phonemes::ONE);

// Connect: "Online"
static const Note PROGMEM connect_notes[] = {
  {730, 150},   // o
  {1100, 100},  // n
  {270, 150},   // line
  {2300, 200},
};

// Disconnect: "Offline"
static const Note PROGMEM disconnect_notes[] = {
  {730, 150},   // o
  {1100, 100},  // f
  {270, 150},   // line
  {2300, 200},
  {440, 200},  // falling tone
};

// Charging start: "Power"
static const Note PROGMEM charging_start_notes[] = {
  {730, 150},  // p
  {730, 150},  // ow
  {270, 200},  // er
};

// Charging complete: "Full"
static const Note PROGMEM charging_stop_notes[] = {
  {300, 150},  // f
  {300, 150},  // u
  {660, 200},  // ll
};

// Low battery: "Low"
static const Note PROGMEM low_battery_notes[] = {
  {660, 150},  // l
  {730, 200},  // o
  {440, 250},  // w (with falling tone)
};

// Boot: "Ready"
static const Note PROGMEM boot_notes[] = {
  {440, 100},   // r
  {270, 150},   // ee
  {660, 150},   // d
  {1700, 200},  // y
};

// Error: "Error"
static const Note PROGMEM error_notes[] = {
  {270, 150},  // e
  {440, 100},  // r
  {730, 150},  // o
  {440, 200},  // r
  {330, 300},  // (falling tone)
};

// Theme definition with layer-specific melodies
class SpeechTheme {
 public:
  static const Note *getLayerChangeMelody(uint8_t layer, uint8_t &length) {
    switch (layer) {
    case 0:
      length = sizeof(layer_0_notes) / sizeof(Note);
      return layer_0_notes;
    case 1:
      length = sizeof(layer_1_notes) / sizeof(Note);
      return layer_1_notes;
    // ... cases for other layers
    default:
      length = sizeof(layer_change_notes) / sizeof(Note);
      return layer_change_notes;
    }
  }
};

// Theme definition
static const Theme PROGMEM speech_theme = {
  "Speech Synthesis",
  {
    melody_from_array(layer_change_notes),
    melody_from_array(connect_notes),
    melody_from_array(disconnect_notes),
    melody_from_array(charging_start_notes),
    melody_from_array(charging_stop_notes),
    melody_from_array(low_battery_notes),
    melody_from_array(boot_notes),
    melody_from_array(error_notes),
  }};

}  // namespace plugin
}  // namespace kaleidoscope