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

// 8-bit style sound effects
static const Note PROGMEM retro_layer_notes[] = {
  {1319, 50},  // E6 - "coin" sound
  {1760, 50},  // A6
};

static const Note PROGMEM retro_connect_notes[] = {
  {784, 50},    // G5 - "power up" sequence
  {988, 50},    // B5
  {1319, 50},   // E6
  {1760, 100},  // A6
};

static const Note PROGMEM retro_disconnect_notes[] = {
  {1760, 50},  // A6 - "power down" sequence
  {1319, 50},  // E6
  {988, 50},   // B5
  {784, 100},  // G5
};

static const Note PROGMEM retro_charging_start_notes[] = {
  {1319, 30},  // E6 - "item get"
  {1760, 80},  // A6
};

static const Note PROGMEM retro_charging_stop_notes[] = {
  {1760, 30},  // A6 - "item complete"
  {2093, 80},  // C7
};

static const Note PROGMEM retro_low_battery_notes[] = {
  {220, 100},  // A3 - "danger" warning
  {220, 100},  // A3
  {220, 200},  // A3
};

static const Note PROGMEM retro_boot_notes[] = {
  {784, 80},    // G5 - "game start" fanfare
  {988, 80},    // B5
  {1319, 80},   // E6
  {1760, 80},   // A6
  {2093, 200},  // C7
};

static const Note PROGMEM retro_error_notes[] = {
  {220, 100},  // A3 - "game over" sound
  {196, 100},  // G3
  {175, 200},  // F3
};

// Theme definition
static const Theme PROGMEM retro_gaming_theme = {
  "Retro Gaming",
  {
    melody_from_array(retro_layer_notes),
    melody_from_array(retro_connect_notes),
    melody_from_array(retro_disconnect_notes),
    melody_from_array(retro_charging_start_notes),
    melody_from_array(retro_charging_stop_notes),
    melody_from_array(retro_low_battery_notes),
    melody_from_array(retro_boot_notes),
    melody_from_array(retro_error_notes),
  }};

}  // namespace plugin
}  // namespace kaleidoscope