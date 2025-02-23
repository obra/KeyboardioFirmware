/* Kaleidoscope-OneShot -- One-shot modifiers and layers
 * Copyright 2022-2025 Keyboard.io, inc.
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

#include "kaleidoscope/plugin/OneShot.h"

#include <Arduino.h>                       // for F
#include <Kaleidoscope-EEPROM-Settings.h>  // for EEPROMSettings
#include <Kaleidoscope-FocusSerial.h>      // for Focus, FocusSerial
#include <stdint.h>                        // for uint16_t, uint32_t, uint8_t

#include "kaleidoscope/Runtime.h"               // for Runtime, Runtime_
#include "kaleidoscope/device/device.h"         // for VirtualProps::Storage, Base<>::Storage
#include "kaleidoscope/event_handler_result.h"  // for EventHandlerResult, EventHandlerResult::OK

namespace kaleidoscope {
namespace plugin {

EventHandlerResult OneShotConfig::onSetup() {
  bool success = ::EEPROMSettings.requestSliceAndLoadData(&settings_base_, &::OneShot.settings_);

  if (!success) {
    Runtime.storage().put(settings_base_, ::OneShot.settings_);
    Runtime.storage().commit();
  }

  return EventHandlerResult::OK;
}

EventHandlerResult OneShotConfig::onNameQuery() {
  return ::Focus.sendName(F("OneShotConfig"));
}

EventHandlerResult OneShotConfig::onFocusEvent(const char *input) {
  enum Command : uint8_t {
    TIMEOUT,
    HOLD_TIMEOUT,
    DOUBLE_TAP_TIMEOUT,
	STICKY_TIMEOUT,
    STICKABLE_KEYS,
    AUTO_MODS,
    AUTO_LAYERS,
  } cmd;
  const char *cmd_timeout            = PSTR("oneshot.timeout");
  const char *cmd_hold_timeout       = PSTR("oneshot.hold_timeout");
  const char *cmd_double_tap_timeout = PSTR("oneshot.double_tap_timeout");
  const char *cmd_sticky_temeout     = PSTR("oneshot.sticky_timeout");
  const char *cmd_stickable_keys     = PSTR("oneshot.stickable_keys");
  const char *cmd_auto_mods          = PSTR("oneshot.auto_mods");
  const char *cmd_auto_layers        = PSTR("oneshot.auto_layers");

  if (::Focus.inputMatchesHelp(input))
    return ::Focus.printHelp(
      cmd_timeout,
      cmd_hold_timeout,
      cmd_double_tap_timeout,
      cmd_stickable_keys,
      cmd_auto_mods,
      cmd_auto_layers);

  if (::Focus.inputMatchesCommand(input, cmd_timeout))
    cmd = Command::TIMEOUT;
  else if (::Focus.inputMatchesCommand(input, cmd_hold_timeout))
    cmd = Command::HOLD_TIMEOUT;
  else if (::Focus.inputMatchesCommand(input, cmd_double_tap_timeout))
    cmd = Command::DOUBLE_TAP_TIMEOUT;
  else if (::Focus.inputMatchesCommand(input, cmd_sticky_timeout))
	cmd = Command::STICKY_TIMEOUT;
  else if (::Focus.inputMatchesCommand(input, cmd_stickable_keys))
    cmd = Command::STICKABLE_KEYS;
  else if (::Focus.inputMatchesCommand(input, cmd_auto_mods))
    cmd = Command::AUTO_MODS;
  else if (::Focus.inputMatchesCommand(input, cmd_auto_layers))
    cmd = Command::AUTO_LAYERS;
  else
    return EventHandlerResult::OK;

  if (::Focus.isEOL()) {
    // If there are no arguments, send back the current configuration values.
    switch (cmd) {
    case Command::TIMEOUT:
      ::Focus.send(::OneShot.getTimeout());
      break;
    case Command::HOLD_TIMEOUT:
      ::Focus.send(::OneShot.getHoldTimeout());
      break;
    case Command::DOUBLE_TAP_TIMEOUT:
      ::Focus.send(::OneShot.getDoubleTapTimeout());
      break;
	case Command::STICKY_TIMEOUT:
	  ::Focus.send(::OneShot.getStickyTimeout());
	  break;
    case Command::STICKABLE_KEYS:
      ::Focus.send(::OneShot.settings_.stickable_keys);
      break;
    case Command::AUTO_MODS:
      ::Focus.send(::OneShot.settings_.auto_modifiers ? 1 : 0);
      break;
    case Command::AUTO_LAYERS:
      ::Focus.send(::OneShot.settings_.auto_layers ? 1 : 0);
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

    case Command::TIMEOUT:
      ::Focus.read(v);
      ::OneShot.setTimeout(v);
      break;
    case Command::HOLD_TIMEOUT:
      ::Focus.read(v);
      ::OneShot.setHoldTimeout(v);
      break;
    case Command::DOUBLE_TAP_TIMEOUT:
      ::Focus.read(v);
      ::OneShot.setDoubleTapTimeout(static_cast<int16_t>(v));
      break;
	case Command::STICKY_TIMEOUT:
	  ::Focus.read(v);
	  ::OneShot.setStickyTimeout(static_cast<uint32_t>(v));
	  break;
    case Command::STICKABLE_KEYS:
      ::Focus.read(v);
      ::OneShot.settings_.stickable_keys = v;
      break;
    case Command::AUTO_MODS:
      ::Focus.read(b);
      ::OneShot.settings_.auto_modifiers = b;
      break;
    case Command::AUTO_LAYERS:
      ::Focus.read(b);
      ::OneShot.settings_.auto_layers = b;
      break;
    default:
      return EventHandlerResult::ABORT;
    }
    Runtime.storage().put(settings_base_, ::OneShot.settings_);
    Runtime.storage().commit();
  }

  return EventHandlerResult::EVENT_CONSUMED;
}

}  // namespace plugin
}  // namespace kaleidoscope

kaleidoscope::plugin::OneShotConfig OneShotConfig;
