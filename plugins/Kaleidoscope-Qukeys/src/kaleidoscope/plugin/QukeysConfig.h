/* -*- mode: c++ -*-
 * Kaleidoscope - Firmware for computer input devices
 * Copyright (C) 2023  Keyboard.io, Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdint.h>  // for uint16_t

#include "kaleidoscope/event_handler_result.h"  // for EventHandlerResult
#include "kaleidoscope/plugin.h"                // for Plugin

namespace kaleidoscope {
namespace plugin {

// =============================================================================
// Plugin for configuration of Qukeys via Focus and persistent storage of
// settins in EEPROM (i.e. Chrysalis).
class QukeysConfig : public Plugin {
 public:
  EventHandlerResult onNameQuery();
  EventHandlerResult onSetup();
  EventHandlerResult onFocusEvent(const char *command);

 private:
  void loadSettings();
  void updateSettings();

  // The base address in persistent storage for configuration data:
  uint16_t settings_addr_;

  // struct used to temporarily cache settings
  struct Settings_ {
    uint16_t hold_timeout_;
    uint8_t overlap_threshold_;
    uint8_t minimum_hold_time_;
    uint8_t minimum_prior_interval_;
    uint8_t tap_repeat_interval_;
  };
};

}  // namespace plugin
}  // namespace kaleidoscope

extern kaleidoscope::plugin::QukeysConfig QukeysConfig;
