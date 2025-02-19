/* -*- mode: c++ -*-
 * Kaleidoscope-Idle-LEDs -- Turn off the LEDs when the keyboard's idle
 * Copyright (C) 2018, 2019, 2020, 2021  Keyboard.io, Inc
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

#include "kaleidoscope/plugin/IdleLEDs.h"

#include <Arduino.h>                       // for F, PSTR, __FlashStringHelper
#include <Kaleidoscope-EEPROM-Settings.h>  // for EEPROMSettings
#include <Kaleidoscope-FocusSerial.h>      // for Focus, FocusSerial
#include <stdint.h>                        // for uint32_t, uint16_t

#include "kaleidoscope/KeyEvent.h"              // for KeyEvent
#include "kaleidoscope/Runtime.h"               // for Runtime, Runtime_
#include "kaleidoscope/device/device.h"         // for VirtualProps::Storage, Base<>::Storage
#include "kaleidoscope/event_handler_result.h"  // for EventHandlerResult, EventHandlerResult::OK
#include "kaleidoscope/plugin/LEDControl.h"     // for LEDControl

namespace kaleidoscope {
namespace plugin {

uint32_t IdleLEDs::idle_time_limit = 600000;  // 10 minutes
uint32_t IdleLEDs::start_time_     = 0;
bool IdleLEDs::idle_;

uint32_t IdleLEDs::idleTimeoutSeconds() {
  return idle_time_limit / 1000;
}

void IdleLEDs::setIdleTimeoutSeconds(uint32_t new_limit) {
  idle_time_limit = new_limit * 1000;
}

EventHandlerResult IdleLEDs::beforeEachCycle() {
  if (idle_time_limit == 0)
    return EventHandlerResult::OK;

  if (::LEDControl.isEnabled() &&
      Runtime.hasTimeExpired(start_time_, idle_time_limit)) {
    ::LEDControl.disable();
    idle_ = true;
  }

  return EventHandlerResult::OK;
}

EventHandlerResult IdleLEDs::onKeyEvent(KeyEvent &event) {
  if (idle_) {
    ::LEDControl.enable();
    idle_ = false;
  }

  start_time_ = Runtime.millisAtCycleStart();

  return EventHandlerResult::OK;
}

uint16_t PersistentIdleLEDs::settings_base_;

EventHandlerResult PersistentIdleLEDs::onNameQuery() {
  return ::Focus.sendName(F("PersistentIdleLEDs"));
}

EventHandlerResult PersistentIdleLEDs::onSetup() {
  uint16_t idle_time;

  bool success = ::EEPROMSettings.requestSliceAndLoadData(&settings_base_, &idle_time);

  // If idleTime is max, assume that EEPROM is uninitialized, and store the
  // defaults.
  if (idle_time == 0xffff || !success) {
    idle_time = idle_time_limit / 1000;
    setIdleTimeoutSeconds(idle_time);
  } else {
  IdleLEDs::setIdleTimeoutSeconds(idle_time);
}
  return EventHandlerResult::OK;
}

void PersistentIdleLEDs::setIdleTimeoutSeconds(uint32_t new_limit) {
  IdleLEDs::setIdleTimeoutSeconds(new_limit);
  uint16_t stored_limit = (uint16_t)new_limit;
  Runtime.storage().put(settings_base_, stored_limit);
  Runtime.storage().commit();
}

EventHandlerResult PersistentIdleLEDs::onFocusEvent(const char *input) {
  const char *cmd = PSTR("idleleds.time_limit");

  if (::Focus.inputMatchesHelp(input))
    return ::Focus.printHelp(cmd);

  if (!::Focus.inputMatchesCommand(input, cmd))
    return EventHandlerResult::OK;

  if (::Focus.isEOL()) {
    ::Focus.send(idleTimeoutSeconds());
  } else {
    uint16_t idle_time;
    ::Focus.read(idle_time);
    setIdleTimeoutSeconds(idle_time);
  }
  return EventHandlerResult::EVENT_CONSUMED;
}

}  // namespace plugin
}  // namespace kaleidoscope

kaleidoscope::plugin::IdleLEDs IdleLEDs;
kaleidoscope::plugin::PersistentIdleLEDs PersistentIdleLEDs;
