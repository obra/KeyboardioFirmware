#pragma once

#include <stdint.h>                             // for uint8_t, uint16_t
#include "kaleidoscope/KeyEvent.h"              // for KeyEvent
#include "kaleidoscope/event_handler_result.h"  // for EventHandlerResult
#include "kaleidoscope/key_defs.h"              // for Key
#include "kaleidoscope/plugin.h"                // for Plugin
#include "kaleidoscope/Runtime.h"               // for Runtime
#include "Kaleidoscope-Ranges.h"                // for KEYCLICK_TOGGLE

namespace kaleidoscope {
namespace plugin {

// Default settings for the keyclick sound
static constexpr uint16_t DEFAULT_FREQUENCY = 5000;  // Hz - higher frequency for a sharper click
static constexpr uint8_t DEFAULT_DURATION   = 5;     // ms - very short for a click rather than a beep

class Keyclick : public kaleidoscope::Plugin {
 public:
  EventHandlerResult onSetup();
  EventHandlerResult onKeyswitchEvent(KeyEvent &event);
  EventHandlerResult onFocusEvent(const char *command);
  EventHandlerResult onNameQuery();

  // Toggle the keyclick feature
  void toggle() {
    settings_.enabled = !settings_.enabled;
    updateSettings();
  }

  // Getters and setters
  bool isEnabled() const { return settings_.enabled; }
  void enable() {
    settings_.enabled = true;
    updateSettings();
  }
  void disable() {
    settings_.enabled = false;
    updateSettings();
  }

 private:
  // Settings stored in EEPROM
  struct Settings {
    bool enabled    = true;
    uint8_t version = 0x01;  // For future compatibility
  };

  uint16_t settings_base_;  // EEPROM storage location
  Settings settings_;

  void updateSettings() {
    Runtime.storage().put(settings_base_, settings_);
    Runtime.storage().commit();
  }
};

}  // namespace plugin
}  // namespace kaleidoscope

// Key definition for use in keymaps
constexpr Key Key_ToggleKeyclick(kaleidoscope::ranges::KEYCLICK_TOGGLE);

extern kaleidoscope::plugin::Keyclick Keyclick;