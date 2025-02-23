/* Kaleidoscope-Qukeys -- Assign two keycodes to a single key
 * Copyright (C) 2017-2020  Michael Richters
 *
 * This program is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, version 3 or (at your option) any later version.
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
#pragma once

#include <Arduino.h>              // for PROGMEM
#include <Kaleidoscope-Ranges.h>  // for DUL_FIRST, DUM_FIRST
#include <stdint.h>               // for uint8_t, uint16_t, int8_t

#include "kaleidoscope/KeyAddr.h"               // for KeyAddr
#include "kaleidoscope/KeyAddrEventQueue.h"     // for KeyAddrEventQueue
#include "kaleidoscope/KeyEvent.h"              // for KeyEvent
#include "kaleidoscope/KeyEventTracker.h"       // for KeyEventTracker
#include "kaleidoscope/event_handler_result.h"  // for EventHandlerResult
#include "kaleidoscope/key_defs.h"              // for Key, Key_Transparent
#include "kaleidoscope/plugin.h"                // for Plugin

// IWYU pragma: no_include "HIDAliases.h"

// DualUse Key definitions for Qukeys in the keymap
#define MT(mod, key)   kaleidoscope::plugin::ModTapKey(Key_##mod, Key_##key)

#define SFT_T(key)     MT(LeftShift, key)
#define CTL_T(key)     MT(LeftControl, key)
#define ALT_T(key)     MT(LeftAlt, key)
#define GUI_T(key)     MT(LeftGui, key)

#define LT(layer, key) kaleidoscope::plugin::LayerTapKey(layer, Key_##key)

namespace kaleidoscope {
namespace plugin {

constexpr Key ModTapKey(Key mod_key, Key tap_key) {
  return Key(kaleidoscope::ranges::DUM_FIRST +
             ((mod_key.getKeyCode() - HID_KEYBOARD_FIRST_MODIFIER) << 8) + tap_key.getKeyCode());
}

constexpr Key LayerTapKey(uint8_t layer, Key tap_key) {
  return Key(kaleidoscope::ranges::DUL_FIRST +
             (layer << 8) + tap_key.getKeyCode());
}

// Data structure for an individual qukey
struct Qukey {
  // The layer this qukey is mapped on.
  int8_t layer;
  // The keyswitch address of the qukey.
  KeyAddr addr;
  // The alternake Key value this qukey should use (when held).
  Key alternate_key;

  // This is the constructor that should be used when creating a Qukey object in
  // the PROGMEM array that will be used by Qukeys (i.e. in the `QUKEYS()`
  // macro).
  constexpr Qukey(int8_t layer, KeyAddr k, Key alternate_key)
    : layer(layer), addr(k), alternate_key(alternate_key) {}
  // This constructor is here so that we can create an empty Qukey object in RAM
  // into which we can copy the values from a PROGMEM Qukey object.
  Qukey() = default;
};


class Qukeys : public kaleidoscope::Plugin {

 public:
  // Methods for turning the plugin on and off.
  void activate() {
    active_ = true;
  }
  void deactivate() {
    active_ = false;
  }
  void toggle() {
    active_ = !active_;
  }

  // Set the timeout (in milliseconds) for a held qukey. If a qukey is held at
  // least this long, it will take on its alternate value (unless its primary
  // value is a modifier -- i.e. a SpaceCadet type key).
  void setHoldTimeout(uint16_t hold_timeout) {
    hold_timeout_ = hold_timeout;
  }

  // Get the timeout (in milliseconds) for a held qukey.
  uint16_t getHoldTimeout() {
    return hold_timeout_;
  }

  // Set the timeout (in milliseconds) for the tap-repeat feature. If a qukey is
  // tapped twice in a row in less time than this amount, it will allow the user
  // to hold the key with its primary value (unless it's a SpaceCadet type key,
  // in which case it will repeat the alternate value instead). This requires a
  // quick tap immediately followed by a press and hold, and will result in a
  // single press-and-hold on the host system. If a double tap is done instead,
  // it will result in two independent taps.
  void setMaxIntervalForTapRepeat(uint8_t ttl) {
    tap_repeat_.timeout = ttl;
  }

  // Get the timeout (in milliseconds) for the tap-repeat feature.
  uint8_t getMaxIntervalForTapRepeat() {
    return tap_repeat_.timeout;
  }

  // Set the percentage of the duration of a subsequent key's press that must
  // overlap with the qukey preceding it above which the qukey will take on its
  // alternate key value. In other words, if the user presses qukey `Q`, then
  // key `K`, and the overlap of the two keys (`O`) is less than this
  // percentage, the qukey will take on its primary value, but if it's greater
  // (i.e. `K` spends a very small fraction of its time pressed after the `Q` is
  // released), the qukey will take on its alternate value.
  void setOverlapThreshold(uint8_t percentage) {
    // Only percentages less than 100% are meaningful. 0% means to turn off the
    // rollover grace period and rely solely on release order to determine the
    // qukey's state.
    if (percentage < 100) {
      overlap_threshold_ = percentage;
    } else {
      overlap_threshold_ = 0;
    }
  }

  // Get the overlap threshold percentage. See above for the description of the
  // setting itself.
  uint8_t getOverlapThreshold() {
    return overlap_threshold_;
  }

  // Set the minimum length of time a qukey must be held before it can resolve
  // to its alternate key value. If a qukey is pressed and released in less than
  // this number of milliseconds, it will always produce its primary key value.
  void setMinimumHoldTime(uint8_t min_hold_time) {
    minimum_hold_time_ = min_hold_time;
  }

  // Get the minimum hold time. See the setter above for a more detailed
  // description.
  uint8_t getMinimumHoldTime() {
    return minimum_hold_time_;
  }

  // Set the minimum interval between the previous keypress and the qukey press
  // to make the qukey eligible to become its alternate keycode.
  void setMinimumPriorInterval(uint8_t min_interval) {
    minimum_prior_interval_ = min_interval;
  }

  // Getter for the above setter.
  uint8_t getMinimumPriorInterval() {
    return minimum_prior_interval_;
  }

  // Function for defining the array of qukeys data (in PROGMEM). It's a
  // template function that takes as its sole argument an array reference of
  // size `_qukeys_count`, so there's no need to use `sizeof` to calculate the
  // correct size, and pass it as a separate parameter.
  template<uint8_t _qukeys_count>
  void configureQukeys(Qukey const (&qukeys)[_qukeys_count]) {
    qukeys_       = qukeys;
    qukeys_count_ = _qukeys_count;
  }


  // A wildcard value for a qukey that exists on every layer.
  static constexpr int8_t layer_wildcard{-1};

  // Kaleidoscope hook functions.
  EventHandlerResult onNameQuery();
  EventHandlerResult onKeyswitchEvent(KeyEvent &event);
  EventHandlerResult afterEachCycle();

 private:
  // An array of Qukey objects in PROGMEM.
  Qukey const *qukeys_{nullptr};
  uint8_t qukeys_count_{0};

  // The maximum number of events in the queue at a time.
  static constexpr uint8_t queue_capacity_{8};

  // The event queue stores a series of press and release events.
  KeyAddrEventQueue<queue_capacity_> event_queue_;

  // This determines whether the plugin is on or off.
  bool active_{true};

  // This variable stores the percentage number between 0 and 99 that determines
  // how forgiving the plugin is of rollover from a qukey to a modified key.
  uint8_t overlap_threshold_{80};

  // The number of milliseconds until a qukey held on its own will take on its
  // alternate state (or primary state, in the case of a SpaceCadet-type qukey).
  uint16_t hold_timeout_{250};

  // The minimum number of milliseconds a qukey must be held before it is
  // allowed to take on its alternate key value (to limit unintended modifiers
  // for very fast typists).
  uint8_t minimum_hold_time_{50};

  // The minimum interval in milliseconds between the previous keypress and the
  // press of a qukey required to make the qukey eligible to take on its
  // alternate value.
  uint8_t minimum_prior_interval_{75};

  // Timestamp of the keypress event immediately prior to the queue head event.
  // The initial value is 256 to ensure that it won't trigger an error if a
  // qukey is pressed before `minimum_prior_interval_` milliseconds after the
  // keyboard powers on, and that value can only be as high as 255.
  uint16_t prior_keypress_timestamp_{256};

  // This is a guard against re-processing events when qukeys flushes them from
  // its event queue. We can't just use an "injected" key state flag, because
  // that would cause other plugins to also ignore the event.
  KeyEventTracker event_tracker_;

  // A cache of the current qukey's primary and alternate key values, so we
  // don't have to keep looking them up from PROGMEM.
  struct {
    Key primary_key{Key_Transparent};
    Key alternate_key{Key_Transparent};
  } queue_head_;

  // Internal helper methods.
  bool processQueue();
  void flushEvent(Key event_key);
  bool isQukey(KeyAddr k);
  bool isDualUseKey(Key key);
  bool releaseDelayed(uint16_t overlap_start, uint16_t overlap_end) const;
  bool isKeyAddrInQueueBeforeIndex(KeyAddr k, uint8_t index) const;

  // Tap-repeat feature support.
  struct {
    KeyAddr addr{KeyAddr::invalid_state};
    uint16_t start_time;
    uint8_t timeout{200};
  } tap_repeat_;
  bool shouldWaitForTapRepeat();
};

// This function returns true for any key that we expect to be used chorded with
// a "modified" key. This includes actual keyboard modifiers, but also layer
// shift keys. Used for determining if a qukey is a SpaceCadet-type key.
bool isModifierKey(Key key);

}  // namespace plugin
}  // namespace kaleidoscope

extern kaleidoscope::plugin::Qukeys Qukeys;

// Macro for use in sketch file to simplify definition of the qukeys array and
// guarantee that the count is set correctly. This is considerably less
// important than it used to be, with the `configureQukeys()` function taking
// care of guaranteeing the correct count setting.
#define QUKEYS(qukey_defs...)                                       \
  {                                                                 \
    static kaleidoscope::plugin::Qukey const qk_table[] PROGMEM = { \
      qukey_defs};                                                  \
    Qukeys.configureQukeys(qk_table);                               \
  }
