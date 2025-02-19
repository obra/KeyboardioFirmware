/* Kaleidoscope-SonicThemes -- Audio feedback themes for Kaleidoscope
 * Copyright 2023-2025 Keyboard.io, inc.
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

#include <Kaleidoscope.h>
#include <Kaleidoscope-SonicThemes.h>
#include <Kaleidoscope-EEPROM-Settings.h>
#include <Kaleidoscope-FocusSerial.h>

// Optional: Define your layers for testing layer change sounds
enum { QWERTY,
       FUNCTION };  // Layer IDs

// First, we set up the keymap
KEYMAPS(
  [QWERTY] = KEYMAP_STACKED(
    Key_Q, Key_W, Key_E, Key_R, Key_T, Key_A, Key_S, Key_D, Key_F, Key_G, Key_Z, Key_X, Key_C, Key_V, Key_B, Key_Esc, Key_Tab, Key_Space, Key_Enter, Key_Backspace,

    Key_Y,
    Key_U,
    Key_I,
    Key_O,
    Key_P,
    Key_H,
    Key_J,
    Key_K,
    Key_L,
    Key_Semicolon,
    Key_N,
    Key_M,
    Key_Comma,
    Key_Period,
    Key_Slash,
    ShiftToLayer(FUNCTION),
    Key_Minus,
    Key_Quote,
    Key_Slash,
    Key_Equals),

  [FUNCTION] = KEYMAP_STACKED(
    ___, ___, ___, ___, ___, ___, ___, ___, ___, ___, ___, ___, ___, ___, ___, ___, ___, ___, ___, ___,

    ___,
    ___,
    ___,
    ___,
    ___,
    ___,
    ___,
    ___,
    ___,
    ___,
    ___,
    ___,
    ___,
    ___,
    ___,
    ___,
    ___,
    ___,
    ___,
    ___))

KALEIDOSCOPE_INIT_PLUGINS(
  EEPROMSettings,
  Focus,
  SonicThemes);

void setup() {
  Kaleidoscope.setup();

  // Optional: Start with Speech theme
  SonicThemes.nextTheme();  // Cycles from Station Master to Speech theme
}

void loop() {
  Kaleidoscope.loop();
}