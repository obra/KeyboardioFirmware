/* Kaleidoscope - Firmware for computer input devices
 * Copyright (C) 2013-2025 Keyboard.io, inc.
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

#include "kaleidoscope/plugin/LEDControl.h"

#include <Arduino.h>                   // for PSTR, strncmp_P
#include <Kaleidoscope-FocusSerial.h>  // for Focus, FocusSerial

#include "kaleidoscope/KeyAddrMap.h"               // for KeyAddrMap<>::Iterator, KeyAddrMap
#include "kaleidoscope/KeyEvent.h"                 // for KeyEvent
#include "kaleidoscope/KeyMap.h"                   // for KeyMap
#include "kaleidoscope/LiveKeys.h"                 // for LiveKeys, live_keys
#include "kaleidoscope/hooks.h"                    // for Hooks
#include "kaleidoscope/keyswitch_state.h"          // for keyToggledOn
#include "kaleidoscope_internal/LEDModeManager.h"  // for LEDModeManager, LEDModeFactory

using namespace kaleidoscope::internal;  // NOLINT(build/namespaces)

namespace kaleidoscope {
namespace plugin {

static constexpr uint8_t uninitialized_mode_id = 255;

uint8_t LEDControl::mode_id_       = uninitialized_mode_id;
uint8_t LEDControl::num_led_modes_ = LEDModeManager::numLEDModes();
LEDMode *LEDControl::cur_led_mode_ = nullptr;
bool LEDControl::enabled_          = true;

LEDControl::LEDControl(void) {
}
uint8_t LEDControl::sync_interval_   = 32;
uint16_t LEDControl::last_sync_time_ = 0;

void LEDControl::next_mode() {
  ++mode_id_;

  if (mode_id_ >= num_led_modes_) {
    return set_mode(0);
  }

  return set_mode(mode_id_);
}

void LEDControl::prev_mode() {
  if (mode_id_ == 0) {
    // wrap around
    mode_id_ = num_led_modes_ - 1;
  } else {
    mode_id_--;
  }

  return set_mode(mode_id_);
}

void LEDControl::set_mode(uint8_t mode_) {
  if (mode_ >= num_led_modes_)
    return;

  mode_id_ = mode_;

  // Cache the LED mode
  //
  cur_led_mode_ = LEDModeManager::getLEDMode(mode_id_);

  refreshAll();

  Hooks::onLEDModeChange();
}

void LEDControl::activate(LEDModeInterface *plugin) {
  for (uint8_t i = 0; i < num_led_modes_; i++) {

    led_mode_management::LEDModeFactory fac;

    LEDModeManager::retreiveLEDModeFactoryFromPROGMEM(i, fac);

    if (fac.isAssociatedWithPlugin(plugin)) {
      set_mode(i);
      return;
    }
  }
}

void LEDControl::set_all_leds_to(uint8_t r, uint8_t g, uint8_t b) {
  if (!Runtime.has_leds)
    return;

  cRGB color;
  color.r = r;
  color.g = g;
  color.b = b;
  set_all_leds_to(color);
}

void LEDControl::set_all_leds_to(cRGB color) {
  if (!Runtime.has_leds)
    return;

  bool will_be_on = (color.r != 0 || color.g != 0 || color.b != 0);
  bool was_off = !Runtime.device().ledDriver().areAnyLEDsOn();

  for (auto led_index : Runtime.device().LEDs().all()) {
    Runtime.device().ledDriver().setCrgbAt(led_index.offset(), color);
  }

  Runtime.device().ledDriver().updateAllLEDState(will_be_on, was_off);
}

void LEDControl::setCrgbAt(uint8_t led_index, cRGB crgb) {
  if (!Runtime.has_leds)
    return;

  // Check LED state change
  cRGB current = Runtime.device().ledDriver().getCrgbAt(led_index);
  bool was_off = (current.r == 0 && current.g == 0 && current.b == 0);
  bool will_be_on = (crgb.r != 0 || crgb.g != 0 || crgb.b != 0);

  Runtime.device().ledDriver().setCrgbAt(led_index, crgb);
  Runtime.device().ledDriver().updateLEDState(will_be_on, was_off);
}

void LEDControl::setCrgbAt(KeyAddr key_addr, cRGB color) {
  setCrgbAt(Runtime.device().getLedIndex(key_addr), color);
}

cRGB LEDControl::getCrgbAt(uint8_t led_index) {
  return Runtime.device().getCrgbAt(led_index);
}
cRGB LEDControl::getCrgbAt(KeyAddr key_addr) {
  return Runtime.device().getCrgbAt(Runtime.device().getLedIndex(key_addr));
}

void LEDControl::syncLeds(void) {
  if (!enabled_)
    return;

  // This would be a good spot to introduce a new hook function so that a plugin
  // that needs to override the color of an LED used by an LED mode can do so
  // efficiently.
  Hooks::beforeSyncingLeds();

  Runtime.device().syncLeds();
}

EventHandlerResult LEDControl::onSetup() {
  set_all_leds_to({0, 0, 0});

  LEDModeManager::setupPersistentLEDModes();

  if (mode_id_ == uninitialized_mode_id) {
    set_mode(0);
  }

  return EventHandlerResult::OK;
}

void LEDControl::disable() {
  set_all_leds_to(CRGB(0, 0, 0));
  Runtime.device().syncLeds();
  enabled_ = false;
}

void LEDControl::enable() {
  enabled_ = true;
  refreshAll();
  Runtime.device().syncLeds();
}

EventHandlerResult LEDControl::onKeyEvent(KeyEvent &event) {
  if (event.key.getFlags() != (SYNTHETIC | IS_INTERNAL | LED_TOGGLE))
    return EventHandlerResult::OK;

  if (keyToggledOn(event.state)) {
    if (event.key == Key_LEDEffectNext || event.key == Key_LEDEffectPrevious) {
      // First, check for an active shift key.
      bool shift_active = false;
      // This change should be back-ported to #904
      for (Key active_key : live_keys.all()) {
        if (active_key.isKeyboardShift()) {
          shift_active = true;
          break;
        }
      }
      // Next, record which key (next or previous) was pressed as a boolean.
      bool key_is_next = (event.key == Key_LEDEffectNext);
      // This is basically an XOR with two booleans. If the "next" key was
      // pressed and no shift key is active, or if the "previous" key was
      // pressed and a shift key is active, we activate the next LED
      // Mode. Otherwise, we activate the previous mode.
      if (key_is_next != shift_active) {
        next_mode();
      } else {
        prev_mode();
      }
    } else if (event.key == Key_LEDToggle) {
      if (enabled_)
        disable();
      else
        enable();
    }
  }

  return EventHandlerResult::EVENT_CONSUMED;
}

EventHandlerResult LEDControl::afterEachCycle() {
  if (!enabled_)
    return EventHandlerResult::OK;

  if (Runtime.hasTimeExpired(last_sync_time_, sync_interval_)) {
    syncLeds();
    last_sync_time_ += sync_interval_;
    update();
  }

  return EventHandlerResult::OK;
}


}  // namespace plugin
}  // namespace kaleidoscope

kaleidoscope::plugin::LEDControl LEDControl;
