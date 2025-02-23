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

namespace kaleidoscope {
namespace plugin {

// Simple, short beeps
static const Note PROGMEM minimal_layer_notes[] = {
  {880, 30},  // A5 - short high beep
};

static const Note PROGMEM minimal_connect_notes[] = {
  {440, 30},  // A4
  {880, 30},  // A5 - ascending pair
};

static const Note PROGMEM minimal_disconnect_notes[] = {
  {880, 30},  // A5
  {440, 30},  // A4 - descending pair
};

static const Note PROGMEM minimal_charging_start_notes[] = {
  {660, 30},  // E5 - medium beep
};

static const Note PROGMEM minimal_charging_stop_notes[] = {
  {880, 30},  // A5 - high beep
};

static const Note PROGMEM minimal_low_battery_notes[] = {
  {220, 50},  // A3 - two low beeps
  {220, 50},
};

static const Note PROGMEM minimal_boot_notes[] = {
  {440, 30},  // A4 - simple startup
};

static const Note PROGMEM minimal_error_notes[] = {
  {220, 100},  // A3 - long low beep
};

// Theme definition
static const Theme PROGMEM minimal_theme = {
  "Minimal",
  {
    melody_from_array(minimal_layer_notes),
    melody_from_array(minimal_connect_notes),
    melody_from_array(minimal_disconnect_notes),
    melody_from_array(minimal_charging_start_notes),
    melody_from_array(minimal_charging_stop_notes),
    melody_from_array(minimal_low_battery_notes),
    melody_from_array(minimal_boot_notes),
    melody_from_array(minimal_error_notes),
  }};

}  // namespace plugin
}  // namespace kaleidoscope