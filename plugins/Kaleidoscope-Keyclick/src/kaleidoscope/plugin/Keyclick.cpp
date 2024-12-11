/* Kaleidoscope-Keyclick -- Keyclick sound feedback for Kaleidoscope.
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



#include "kaleidoscope/plugin/Keyclick.h"

#include <Kaleidoscope-EEPROM-Settings.h>  // for EEPROMSettings
#include <Kaleidoscope-FocusSerial.h>      // for Focus, FocusSerial

namespace kaleidoscope {
namespace plugin {

EventHandlerResult Keyclick::onSetup() {
  // Request storage space and load settings
  bool success = ::EEPROMSettings.requestSliceAndLoadData(&settings_base_, &settings_);

  // If settings weren't loaded successfully, use defaults
  if (!success) {
    settings_.enabled = true;
    settings_.version = 0x01;
    updateSettings();
  }

  return EventHandlerResult::OK;
}

EventHandlerResult Keyclick::onKeyswitchEvent(KeyEvent &event) {
  // Handle the toggle key
  if (event.key == Key_ToggleKeyclick) {
    if (event.state) {  // Only toggle on press, not release
      toggle();
    }
    return EventHandlerResult::EVENT_CONSUMED;
  }

  // If enabled and it's a keypress (not release), play the click
  if (settings_.enabled && event.state) {
    // Use our speaker driver to play the keyclick sound
    Runtime.device().speaker().playTone(DEFAULT_FREQUENCY, DEFAULT_DURATION);
  }

  return EventHandlerResult::OK;
}

EventHandlerResult Keyclick::onFocusEvent(const char *command) {
  if (::Focus.inputMatchesCommand(command, "keyclick.enabled")) {
    if (::Focus.isEOL()) {
      ::Focus.send(settings_.enabled);
    } else {
      uint8_t enabled;
      ::Focus.read(enabled);
      if (enabled)
        enable();
      else
        disable();
    }
  } else {
    return EventHandlerResult::OK;
  }

  return EventHandlerResult::EVENT_CONSUMED;
}

EventHandlerResult Keyclick::onNameQuery() {
  return ::Focus.sendName(F("Keyclick"));
}

}  // namespace plugin
}  // namespace kaleidoscope

kaleidoscope::plugin::Keyclick Keyclick;