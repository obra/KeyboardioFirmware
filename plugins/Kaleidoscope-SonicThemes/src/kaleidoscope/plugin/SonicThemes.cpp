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



#include "kaleidoscope/plugin/SonicThemes.h" // for SonicThemes
#include "kaleidoscope/plugin/themes/StationMasterTheme.h" // for StationMasterTheme
#include "kaleidoscope/keyswitch_state.h" // for INJECTED
namespace kaleidoscope {
namespace plugin {

// Update THEME_COUNT and themes_ array
static constexpr uint8_t THEME_COUNT = 1;  // Only StationMasterTheme for now

// Theme registry
const Theme *const SonicThemes::themes_[THEME_COUNT] = {
  &themes::station_master_theme
};

EventHandlerResult SonicThemes::onSetup() {
  playEvent(SoundEvent::Boot);
  return EventHandlerResult::OK;
}

EventHandlerResult SonicThemes::beforeReportingState(const KeyEvent &event) {
  melody_player_.update();

  if (!enabled_) return EventHandlerResult::OK;

  // Handle key events
  if (event.state & INJECTED) return EventHandlerResult::OK;

  if (event.state) {  // Key pressed
    if (event.key.isKeyboardModifier()) {
      playEvent(SoundEvent::ModifierPress);
    } else {
      playEvent(SoundEvent::KeyPress);
    }
  }

  return EventHandlerResult::OK;
}

EventHandlerResult SonicThemes::afterEachCycle() {
  melody_player_.update();
  return EventHandlerResult::OK;
}

EventHandlerResult SonicThemes::onLayerChange() {
  if (!enabled_) return EventHandlerResult::OK;

  uint8_t length;
  const Note *melody = themes_[current_theme_index_]->getMelody(SoundEvent::LayerChange, length);
  if (melody) {
    melody_player_.playMelody(melody, length);
  }

  return EventHandlerResult::OK;
}

void SonicThemes::playEvent(SoundEvent event) {
  if (!enabled_) return;

  uint8_t length;
  const Note *melody = themes_[current_theme_index_]->getMelody(event, length);
  if (melody) {
    melody_player_.playMelody(melody, length);
  }
}

// Focus API handlers
EventHandlerResult SonicThemes::onNameQuery() {
  return ::Focus.sendName(F("SonicThemes"));
}

EventHandlerResult SonicThemes::onFocusEvent(const char *command) {
  if (::Focus.inputMatchesCommand(command, "sonicthemes.enabled")) {
    if (::Focus.isEOL()) {
      ::Focus.send(enabled_);
    } else {
      uint8_t enabled;
      ::Focus.read(enabled);
      enabled_ = enabled;
    }
    return EventHandlerResult::EVENT_CONSUMED;
  }

  if (::Focus.inputMatchesHelp(command)) {
    ::Focus.send(F("sonicthemes.enabled"));
    return EventHandlerResult::EVENT_CONSUMED;
  }

  return EventHandlerResult::OK;
}

}  // namespace plugin
}  // namespace kaleidoscope

kaleidoscope::plugin::SonicThemes SonicThemes;