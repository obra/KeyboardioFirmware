/* -*- mode: c++ -*-
 * Kaleidoscope-Idle-LEDs -- Turn off the LEDs when the keyboard's idle
 * Copyright (C) 2018, 2019, 2021, 2022  Keyboard.io, Inc
 * Copyright (C) 2019  Dygma, Inc
 *
 * This program is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, version 3.
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

#include <stdint.h>  // for uint32_t, uint16_t

#include "kaleidoscope/KeyEvent.h"              // for KeyEvent
#include "kaleidoscope/event_handler_result.h"  // for EventHandlerResult
#include "kaleidoscope/plugin.h"                // for Plugin

namespace kaleidoscope {
namespace plugin {

class IdleLEDs : public kaleidoscope::Plugin {
 public:
  static uint32_t idle_time_limit;

  static uint32_t idleTimeoutSeconds();
  static void setIdleTimeoutSeconds(uint32_t new_limit);
  static void suspend();
  static void resume();

  EventHandlerResult beforeEachCycle();
  EventHandlerResult onKeyEvent(KeyEvent &event);

 private:
  static bool idle_;
  static uint32_t start_time_;
};

class PersistentIdleLEDs : public IdleLEDs {
 public:
  EventHandlerResult onSetup();
  EventHandlerResult onNameQuery();
  EventHandlerResult onFocusEvent(const char *input);

  static void setIdleTimeoutSeconds(uint32_t new_limit);

 private:
  static uint16_t settings_base_;
};

}  // namespace plugin
}  // namespace kaleidoscope

extern kaleidoscope::plugin::IdleLEDs IdleLEDs;
extern kaleidoscope::plugin::PersistentIdleLEDs PersistentIdleLEDs;
