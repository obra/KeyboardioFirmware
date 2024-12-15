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

#include <stdint.h>
#include "MelodyPlayer.h"

namespace kaleidoscope {
namespace plugin {
namespace phonemes {

// Basic sound unit with frequency transitions
struct Phoneme {
  uint16_t f1;        // Starting/main frequency
  uint16_t f2;        // Ending/secondary frequency (0 if no transition)
  uint16_t duration;  // Duration in ms
  uint8_t amplitude;  // Relative amplitude (0-255)
};

// Vowels (monophthongs)
static const Phoneme PROGMEM EE = {2300, 0, 150, 255};  // as in "beet"
static const Phoneme PROGMEM I  = {1800, 0, 130, 255};  // as in "bit"
static const Phoneme PROGMEM EH = {1600, 0, 130, 255};  // as in "bet"
static const Phoneme PROGMEM AE = {1700, 0, 150, 255};  // as in "bat"
static const Phoneme PROGMEM AH = {1100, 0, 150, 255};  // as in "father"
static const Phoneme PROGMEM AW = {870, 0, 150, 255};   // as in "bought"
static const Phoneme PROGMEM UH = {850, 0, 130, 255};   // as in "but"
static const Phoneme PROGMEM OO = {800, 0, 150, 255};   // as in "boot"
static const Phoneme PROGMEM U  = {900, 0, 130, 255};   // as in "put"

// Diphthongs (vowel transitions)
static const Phoneme PROGMEM AY = {1700, 2300, 200, 255};  // as in "bite"
static const Phoneme PROGMEM OY = {870, 2300, 200, 255};   // as in "boy"
static const Phoneme PROGMEM OW = {870, 800, 200, 255};    // as in "boat"
static const Phoneme PROGMEM AW = {1700, 800, 200, 255};   // as in "bout"

// Stops
static const Phoneme PROGMEM P = {1000, 0, 40, 200};  // "p"
static const Phoneme PROGMEM B = {500, 0, 40, 200};   // "b"
static const Phoneme PROGMEM T = {1500, 0, 40, 200};  // "t"
static const Phoneme PROGMEM D = {700, 0, 40, 200};   // "d"
static const Phoneme PROGMEM K = {2000, 0, 40, 200};  // "k"
static const Phoneme PROGMEM G = {1000, 0, 40, 200};  // "g"

// Fricatives
static const Phoneme PROGMEM F  = {2000, 0, 80, 150};  // "f"
static const Phoneme PROGMEM V  = {1000, 0, 80, 150};  // "v"
static const Phoneme PROGMEM TH = {1500, 0, 80, 150};  // "th" (thin)
static const Phoneme PROGMEM DH = {750, 0, 80, 150};   // "th" (this)
static const Phoneme PROGMEM S  = {4000, 0, 80, 150};  // "s"
static const Phoneme PROGMEM Z  = {2000, 0, 80, 150};  // "z"
static const Phoneme PROGMEM SH = {3000, 0, 80, 150};  // "sh"
static const Phoneme PROGMEM ZH = {1500, 0, 80, 150};  // "zh" (measure)

// Nasals
static const Phoneme PROGMEM M  = {400, 0, 100, 200};  // "m"
static const Phoneme PROGMEM N  = {500, 0, 100, 200};  // "n"
static const Phoneme PROGMEM NG = {450, 0, 100, 200};  // "ng"

// Approximants
static const Phoneme PROGMEM L = {600, 0, 100, 200};  // "l"
static const Phoneme PROGMEM R = {800, 0, 100, 200};  // "r"
static const Phoneme PROGMEM W = {600, 0, 80, 200};   // "w"
static const Phoneme PROGMEM Y = {2300, 0, 80, 200};  // "y"

// Helper functions for building words
template<size_t N>
constexpr uint8_t phoneme_count(const Phoneme (&)[N]) {
  return N;
}

// Convert phoneme sequence to note sequence
inline void phonemes_to_notes(const Phoneme *phonemes, size_t count, Note *notes) {
  for (size_t i = 0; i < count; i++) {
    notes[i].frequency = phonemes[i].f1;
    notes[i].duration  = phonemes[i].duration;

    // If there's a frequency transition, add it as a separate note
    if (phonemes[i].f2 != 0) {
      notes[i + 1].frequency = phonemes[i].f2;
      notes[i + 1].duration  = phonemes[i].duration / 2;
      i++;  // Skip the next slot as we used it
    }
  }
}

// Common word patterns
struct Word {
  const Phoneme *phonemes;
  uint8_t length;
};

// Example word definitions
static const Phoneme PROGMEM LAYER_PHONEMES[] = {
  L, AY, Y, ER};

static const Phoneme PROGMEM READY_PHONEMES[] = {
  R, EH, D, EE};

static const Word PROGMEM LAYER = {LAYER_PHONEMES, phoneme_count(LAYER_PHONEMES)};
static const Word PROGMEM READY = {READY_PHONEMES, phoneme_count(READY_PHONEMES)};

// Number patterns
static const Phoneme PROGMEM ZERO_PHONEMES[]  = {Z, EE, R, OW};
static const Phoneme PROGMEM ONE_PHONEMES[]   = {W, UH, N};
static const Phoneme PROGMEM TWO_PHONEMES[]   = {T, OO};
static const Phoneme PROGMEM THREE_PHONEMES[] = {TH, R, EE};
static const Phoneme PROGMEM FOUR_PHONEMES[]  = {F, AW, R};
static const Phoneme PROGMEM FIVE_PHONEMES[]  = {F, AY, V};
static const Phoneme PROGMEM SIX_PHONEMES[]   = {S, I, K, S};
static const Phoneme PROGMEM SEVEN_PHONEMES[] = {S, EH, V, EH, N};
static const Phoneme PROGMEM EIGHT_PHONEMES[] = {AY, T};

// Helper to create melody from word
template<typename... Words>
void create_melody(Note *notes, const Words &...words) {
  size_t offset = 0;
  (void)std::initializer_list<int>{
    ([&offset, notes](const Word &word) {
      phonemes_to_notes(word.phonemes, word.length, notes + offset);
      offset += word.length;
      return 0;
    }(words),
     0)...};
}

}  // namespace phonemes
}  // namespace plugin
}  // namespace kaleidoscope