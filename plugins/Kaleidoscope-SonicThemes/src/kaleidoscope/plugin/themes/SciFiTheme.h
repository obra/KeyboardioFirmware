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

// Futuristic sound effects
static const Note PROGMEM scifi_layer_notes[] = {
  {1760, 20},  // A6 - quick sweep
  {2093, 20},  // C7
  {2637, 20},  // E7
};

static const Note PROGMEM scifi_connect_notes[] = {
  {2093, 30},  // C7 - "teleport in"
  {2349, 30},  // D7
  {2637, 30},  // E7
  {3136, 80},  // G7
};

static const Note PROGMEM scifi_disconnect_notes[] = {
  {3136, 30},  // G7 - "teleport out"
  {2637, 30},  // E7
  {2349, 30},  // D7
  {2093, 80},  // C7
};

static const Note PROGMEM scifi_charging_start_notes[] = {
  {2093, 20},  // C7 - "energy up"
  {2349, 20},  // D7
  {2637, 40},  // E7
};

static const Note PROGMEM scifi_charging_stop_notes[] = {
  {2637, 20},  // E7 - "energy full"
  {3136, 40},  // G7
};

static const Note PROGMEM scifi_low_battery_notes[] = {
  {311, 100},  // Eb4 - "warning" klaxon
  {277, 100},  // C#4
  {311, 200},  // Eb4
};

static const Note PROGMEM scifi_boot_notes[] = {
  {1760, 40},   // A6 - "computer startup"
  {2093, 40},   // C7
  {2349, 40},   // D7
  {2637, 40},   // E7
  {3136, 100},  // G7
};

static const Note PROGMEM scifi_error_notes[] = {
  {311, 50},   // Eb4 - "system error"
  {277, 50},   // C#4
  {247, 100},  // B3
};

// Theme definition
static const Theme PROGMEM scifi_theme = {
  "Sci-Fi",
  {
    melody_from_array(scifi_layer_notes),
    melody_from_array(scifi_connect_notes),
    melody_from_array(scifi_disconnect_notes),
    melody_from_array(scifi_charging_start_notes),
    melody_from_array(scifi_charging_stop_notes),
    melody_from_array(scifi_low_battery_notes),
    melody_from_array(scifi_boot_notes),
    melody_from_array(scifi_error_notes),
  }};

}  // namespace plugin
}  // namespace kaleidoscope