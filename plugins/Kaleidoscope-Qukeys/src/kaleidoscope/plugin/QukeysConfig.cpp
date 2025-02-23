/* -*- mode: c++ -*-
 * Kaleidoscope - Firmware for computer input devices
 * Copyright (C) 2023  Keyboard.io, Inc.
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

#include "kaleidoscope/plugin/Qukeys.h"
#include "kaleidoscope/plugin/QukeysConfig.h"

#include <Arduino.h>                       // for F
#include <Kaleidoscope-EEPROM-Settings.h>  // for EEPROMSettings
#include <Kaleidoscope-FocusSerial.h>      // for Focus, FocusSerial
#include <stdint.h>                        // for uint16_t, uint32_t, uint8_t

#include "kaleidoscope/Runtime.h"               // for Runtime, Runtime_
#include "kaleidoscope/device/device.h"         // for VirtualProps::Storage, Base<>::Storage
#include "kaleidoscope/event_handler_result.h"  // for EventHandlerResult, EventHandlerResult::OK

namespace kaleidoscope {
namespace plugin {

void QukeysConfig::loadSettings() {
  Settings_ settings;

  Runtime.storage().get(settings_addr_, settings);
  ::Qukeys.setHoldTimeout(settings.hold_timeout_);
  ::Qukeys.setOverlapThreshold(settings.overlap_threshold_);
  ::Qukeys.setMinimumHoldTime(settings.minimum_hold_time_);
  ::Qukeys.setMinimumPriorInterval(settings.minimum_prior_interval_);
  ::Qukeys.setMaxIntervalForTapRepeat(settings.tap_repeat_interval_);
}

void QukeysConfig::updateSettings() {
  Settings_ settings;

  settings.hold_timeout_           = ::Qukeys.getHoldTimeout();
  settings.overlap_threshold_      = ::Qukeys.getOverlapThreshold();
  settings.minimum_hold_time_      = ::Qukeys.getMinimumHoldTime();
  settings.minimum_prior_interval_ = ::Qukeys.getMinimumPriorInterval();
  settings.tap_repeat_interval_    = ::Qukeys.getMaxIntervalForTapRepeat();

  Runtime.storage().put(settings_addr_, settings);
  Runtime.storage().commit();
}

EventHandlerResult QukeysConfig::onSetup() {
  settings_addr_ = ::EEPROMSettings.requestSlice(sizeof(Settings_));

  // If the EEPROM is empty, store the default settings.
  if (Runtime.storage().isSliceUninitialized(settings_addr_, sizeof(Settings_))) {
    updateSettings();
  }

  loadSettings();
  return EventHandlerResult::OK;
}

EventHandlerResult QukeysConfig::onNameQuery() {
  return ::Focus.sendName(F("QukeysConfig"));
}

EventHandlerResult QukeysConfig::onFocusEvent(const char *input) {
  enum Command : uint8_t {
    HOLD_TIMEOUT,
    MAX_INTERVAL_FOR_TAP_REPEAT,
    OVERLAP_THRESHOLD,
    MINIMUM_HOLD_TIME,
    MINIMUM_PRIOR_INTERVAL,
  } cmd;
  const char *cmd_hold_timeout            = PSTR("qukeys.hold_timeout");
  const char *cmd_max_tap_repeat_interval = PSTR("qukeys.max_tap_repeat_interval");
  const char *cmd_overlap_threshold       = PSTR("qukeys.overlap_threshold");
  const char *cmd_minimum_hold_time       = PSTR("qukeys.minimum_hold_time");
  const char *cmd_minimum_prior_interval  = PSTR("qukeys.minimum_prior_interval");

  if (::Focus.inputMatchesHelp(input))
    return ::Focus.printHelp(
      cmd_hold_timeout,
      cmd_max_tap_repeat_interval,
      cmd_overlap_threshold,
      cmd_minimum_hold_time,
      cmd_minimum_prior_interval);

  if (::Focus.inputMatchesCommand(input, cmd_hold_timeout))
    cmd = Command::HOLD_TIMEOUT;
  else if (::Focus.inputMatchesCommand(input, cmd_max_tap_repeat_interval))
    cmd = Command::MAX_INTERVAL_FOR_TAP_REPEAT;
  else if (::Focus.inputMatchesCommand(input, cmd_overlap_threshold))
    cmd = Command::OVERLAP_THRESHOLD;
  else if (::Focus.inputMatchesCommand(input, cmd_minimum_hold_time))
    cmd = Command::MINIMUM_HOLD_TIME;
  else if (::Focus.inputMatchesCommand(input, cmd_minimum_prior_interval))
    cmd = Command::MINIMUM_PRIOR_INTERVAL;
  else
    return EventHandlerResult::OK;

  if (::Focus.isEOL()) {
    // If there are no arguments, send back the current configuration values.
    switch (cmd) {
    case Command::HOLD_TIMEOUT:
      ::Focus.send(::Qukeys.getHoldTimeout());
      break;
    case Command::MAX_INTERVAL_FOR_TAP_REPEAT:
      ::Focus.send(::Qukeys.getMaxIntervalForTapRepeat());
      break;
    case Command::OVERLAP_THRESHOLD:
      ::Focus.send(::Qukeys.getOverlapThreshold());
      break;
    case Command::MINIMUM_HOLD_TIME:
      ::Focus.send(::Qukeys.getMinimumHoldTime());
      break;
    case Command::MINIMUM_PRIOR_INTERVAL:
      ::Focus.send(::Qukeys.getMinimumPriorInterval());
      break;
    default:
      // if a valid command is issued but there is no 0-arg handler for it,
      // we stop processing the event.
      return EventHandlerResult::ABORT;
    }
  } else {
    switch (cmd) {
      uint16_t v;
      uint8_t b;

    case Command::HOLD_TIMEOUT:
      ::Focus.read(v);
      ::Qukeys.setHoldTimeout(v);
      break;
    case Command::MAX_INTERVAL_FOR_TAP_REPEAT:
      ::Focus.read(b);
      ::Qukeys.setMaxIntervalForTapRepeat(b);
      break;
    case Command::OVERLAP_THRESHOLD:
      ::Focus.read(b);
      ::Qukeys.setOverlapThreshold(b);
      break;
    case Command::MINIMUM_HOLD_TIME:
      ::Focus.read(b);
      ::Qukeys.setMinimumHoldTime(b);
      break;
    case Command::MINIMUM_PRIOR_INTERVAL:
      ::Focus.read(b);
      ::Qukeys.setMinimumPriorInterval(b);
      break;
    default:
      return EventHandlerResult::ABORT;
    }
    updateSettings();
  }

  return EventHandlerResult::EVENT_CONSUMED;
}

}  // namespace plugin
}  // namespace kaleidoscope

kaleidoscope::plugin::QukeysConfig QukeysConfig;
