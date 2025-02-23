/* -*- mode: c++ -*-
 * Kaleidoscope-Turbo
 * Copyright (C) 2018 ash lea
 * Copyright (C) 2022 Keyboard.io, Inc.
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

#include "kaleidoscope/plugin/Turbo.h"

#include <Arduino.h>                   // for F, __FlashStringHelper
#include <Kaleidoscope-FocusSerial.h>  // for Focus, FocusSerial
#include <stdint.h>                    // for uint16_t, uint32_t

#include "kaleidoscope/KeyAddr.h"                         // for MatrixAddr, MatrixAddr<>::Range
#include "kaleidoscope/KeyAddrMap.h"                      // for KeyAddrMap<>::Iterator, KeyAddrMap
#include "kaleidoscope/KeyEvent.h"                        // for KeyEvent
#include "kaleidoscope/KeyMap.h"                          // for KeyMap
#include "kaleidoscope/LiveKeys.h"                        // for LiveKeys, live_keys
#include "kaleidoscope/Runtime.h"                         // for Runtime, Runtime_
#include "kaleidoscope/driver/hid/keyboardio/Keyboard.h"  // for Keyboard
#include "kaleidoscope/event_handler_result.h"            // for EventHandlerResult, EventHandle...
#include "kaleidoscope/key_defs.h"                        // for Key
#include "kaleidoscope/keyswitch_state.h"                 // for keyToggledOff, keyToggledOn
#include "kaleidoscope/plugin/LEDControl.h"               // for LEDControl

namespace kaleidoscope {
namespace plugin {

uint16_t Turbo::interval() {
  return interval_;
}
void Turbo::interval(uint16_t newVal) {
  interval_ = newVal;
}

uint16_t Turbo::flashInterval() {
  return flash_interval_;
}
void Turbo::flashInterval(uint16_t newVal) {
  flash_interval_ = newVal;
}

bool Turbo::sticky() {
  return sticky_;
}
void Turbo::sticky(bool newVal) {
  sticky_ = newVal;
}

bool Turbo::flash() {
  return flash_;
}
void Turbo::flash(bool newVal) {
  flash_ = newVal;
}

cRGB Turbo::activeColor() {
  return active_color_;
}
void Turbo::activeColor(cRGB newVal) {
  active_color_ = newVal;
}

EventHandlerResult Turbo::onKeyEvent(KeyEvent &event) {
  // If any key toggles off, reset its LED to normal.
  if (active_ && flash_ && keyToggledOff(event.state)) {
    if (event.key.isKeyboardKey())
      LEDControl::refreshAt(event.addr);
  }

  // Ignore any non-Turbo key events.
  if (event.key != Key_Turbo)
    return EventHandlerResult::OK;

  // Remember the address so we can pin its LED on each cycle when necessary
  turbo_key_addr_ = event.addr;


  if (active_) {
    // If Turbo is active, and in "sticky" mode, we abort the event when a Turbo
    // key toggles off, leaving it in the active keys array.  This means that a
    // layer change won't hide an active Turbo key.
    if (sticky_ && keyToggledOff(event.state)) {
      return EventHandlerResult::ABORT;
    }
    // If not in "sticky" mode and a Turbo key toggles off, or if in "sticky"
    // mode and a Turbo key toggles on, we deactivate Turbo.
    active_ = false;
    LEDControl::setCrgbAt(event.addr, CRGB(0, 0, 0));
    if (flash_)
      LEDControl::refreshAll();

  } else if (keyToggledOn(event.state)) {
    // If Turbo is inactive, turn it on when a Turbo key is pressed.
    active_     = true;
    start_time_ = Runtime.millisAtCycleStart() - interval_;
    LEDControl::setCrgbAt(event.addr, active_color_);
  }

  // We assume that other plugins don't need to know about Turbo key events.
  return EventHandlerResult::EVENT_CONSUMED;
}

EventHandlerResult Turbo::afterEachCycle() {
  if (active_) {
    if (Runtime.hasTimeExpired(start_time_, interval_)) {
      // Reset the timer.
      start_time_ = Runtime.millisAtCycleStart();

      // Clear the existing Keyboard HID report. It might be nice to keep the
      // modifiers active, but I'll save that for another time.
      Runtime.hid().keyboard().releaseAllKeys();
      // Send the empty report to register the release of all the held keys.
      Runtime.hid().keyboard().sendReport();

      // Go through the `live_keys[]` array and add any Keyboard HID keys to the
      // new report.
      for (Key key : live_keys.all()) {
        if (key.isKeyboardKey()) {
          Runtime.addToReport(key);
        }
      }

      // Send the re-populated keyboard report.
      Runtime.hid().keyboard().sendReport();
    } else {
      LEDControl::setCrgbAt(turbo_key_addr_, active_color_);
    }
  }
  return EventHandlerResult::OK;
}

EventHandlerResult Turbo::beforeSyncingLeds() {
  if (flash_ && active_) {
    static bool leds_on = false;
    cRGB color          = CRGB(0, 0, 0);
    if (leds_on) {
      color = active_color_;
    }
    if (Runtime.hasTimeExpired(flash_start_time_, flash_interval_)) {
      flash_start_time_ = Runtime.millisAtCycleStart();
      leds_on           = !leds_on;
    }
    for (KeyAddr key_addr : KeyAddr::all()) {
      Key key = live_keys[key_addr];
      if (key.isKeyboardKey()) {
        LEDControl::setCrgbAt(key_addr, color);
      }
    }
  }
  return EventHandlerResult::OK;
}

EventHandlerResult Turbo::onNameQuery() {
  return ::Focus.sendName(F("Turbo"));
}

}  // namespace plugin
}  // namespace kaleidoscope

kaleidoscope::plugin::Turbo Turbo;
