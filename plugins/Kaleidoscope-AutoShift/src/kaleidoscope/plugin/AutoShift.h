/* Kaleidoscope-AutoShift -- Automatic shift on long press
 * Copyright 2021-2025 Keyboard.io, inc.
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

#pragma once

#include <stdint.h>  // for uint8_t, uint16_t

#include "kaleidoscope/KeyAddrEventQueue.h"     // for KeyAddrEventQueue
#include "kaleidoscope/KeyEvent.h"              // for KeyEvent
#include "kaleidoscope/KeyEventTracker.h"       // for KeyEventTracker
#include "kaleidoscope/event_handler_result.h"  // for EventHandlerResult
#include "kaleidoscope/key_defs.h"              // for Key
#include "kaleidoscope/plugin.h"                // for Plugin

namespace kaleidoscope {
namespace plugin {

struct LongPress {
  // The key that should result in a different value on long press.
  Key key;
  // The alternate Key value that should be produced on long press.
  Key alternate_key;

  // This is the constructor that should be used when creating a LongPress object in
  // the PROGMEM array that will be used by explicit mappings (i.e. in the `AUTOSHIFT()`
  // macro).
  constexpr LongPress(Key key, Key alternate_key)
    : key(key), alternate_key(alternate_key) {}
  // This constructor is here so that we can create an empty LongPress object in RAM
  // into which we can copy the values from a PROGMEM LongPress object.
  LongPress() = default;
};

// =============================================================================
/// Kaleidoscope plugin for long-press auto-shift keys
///
/// This plugin allows the user to "long-press" keys to automatically apply the
/// `shift` modifier to the keypress.  By enabling AutoShift, it's possible to
/// produce capital letters (for example) without holding a separate modifier
/// key first.
class AutoShift : public Plugin {

 public:
  // ---------------------------------------------------------------------------
  // Inner class for `Key` categories that can be configured to be auto-shifted
  // by long-pressing.  Most of this class is purely internal, but user code
  // that enables or disables these auto-shift categories might use the
  // following as constants:
  //
  // - `AutoShift::Categories::letterKeys()`
  // - `AutoShift::Categories::numberKeys()`
  // - `AutoShift::Categories::symbolKeys()`
  // - `AutoShift::Categories::arrowKeys()`
  // - `AutoShift::Categories::functionKeys()`
  // - `AutoShift::Categories::printableKeys()`
  // - `AutoShift::Categories::allKeys()`
  //
  // The first two ("letter keys" and "number keys") are self-explanatory.  The
  // third ("symbol keys") includes keys that produce symbols other than letters
  // and numbers, but not whitespace characters, modifiers, et cetera.  We could
  // perhaps add another category for function keys.
  class Categories {
   private:
    // raw bitfield data
    uint8_t raw_bits_{0};

    // constants for bits in the `raw_bits_` bitfield
    static constexpr uint8_t LETTERS   = 1 << 0;
    static constexpr uint8_t NUMBERS   = 1 << 1;
    static constexpr uint8_t SYMBOLS   = 1 << 2;
    static constexpr uint8_t ARROWS    = 1 << 3;
    static constexpr uint8_t FUNCTIONS = 1 << 4;
    static constexpr uint8_t ALL       = 1 << 7;

   public:
    // Basic un-checked constructor
    explicit constexpr Categories(uint8_t raw_bits)
      : raw_bits_(raw_bits) {}

    static constexpr Categories letterKeys() {
      return Categories(LETTERS);
    }
    static constexpr Categories numberKeys() {
      return Categories(NUMBERS);
    }
    static constexpr Categories symbolKeys() {
      return Categories(SYMBOLS);
    }
    static constexpr Categories arrowKeys() {
      return Categories(ARROWS);
    }
    static constexpr Categories functionKeys() {
      return Categories(FUNCTIONS);
    }
    static constexpr Categories printableKeys() {
      return Categories(LETTERS | NUMBERS | SYMBOLS);
    }
    static constexpr Categories allKeys() {
      return Categories(ALL);
    }

    constexpr void set(uint8_t raw_bits) {
      raw_bits_ = raw_bits;
    }
    constexpr void add(Categories categories) {
      this->raw_bits_ |= categories.raw_bits_;
    }
    constexpr void remove(Categories categories) {
      this->raw_bits_ &= ~(categories.raw_bits_);
    }
    constexpr bool contains(Categories categories) const {
      return (this->raw_bits_ & categories.raw_bits_) != 0;
      // More correct test:
      // return (~(this->raw_bits_) & categories.raw_bits_) == 0;
    }

    // Shorthand for combining categories:
    // e.g. `Categories::letterKeys() | Categories::numberKeys()`
    constexpr Categories operator|(Categories other) const {
      return Categories(this->raw_bits_ | other.raw_bits_);
    }

    // A conversion to integer is provided for the sake of interactions with the
    // Focus plugin.
    explicit constexpr operator uint8_t() {
      return raw_bits_;
    }
  };

  // ---------------------------------------------------------------------------
  // This lets the AutoShiftConfig plugin access the internal config variables
  // directly. Mainly useful for calls to `Runtime.storage.get()`.
  friend class AutoShiftConfig;

  // ---------------------------------------------------------------------------
  // Configuration functions

  /// Returns `true` if AutoShift is active, `false` otherwise
  bool enabled() {
    return settings_.enabled;
  }
  /// Activates the AutoShift plugin (held keys will trigger auto-shift)
  void enable() {
    settings_.enabled = true;
  }
  /// Deactivates the AutoShift plugin (held keys will not trigger auto-shift)
  void disable();
  /// Turns AutoShift on if it's off, and vice versa
  void toggle() {
    if (settings_.enabled) {
      disable();
    } else {
      enable();
    }
  }

  /// Returns the hold time required to trigger auto-shift (in ms)
  uint16_t timeout() {
    return settings_.timeout;
  }
  /// Sets the hold time required to trigger auto-shift (in ms)
  void setTimeout(uint16_t new_timeout) {
    settings_.timeout = new_timeout;
  }

  /// Returns the set of categories currently eligible for auto-shift
  Categories enabledCategories() {
    return settings_.enabled_categories;
  }
  /// Adds `category` to the set eligible for auto-shift
  ///
  /// Possible values for `category` are:
  ///  - `AutoShift::Categories::letterKeys()`
  ///  - `AutoShift::Categories::numberKeys()`
  ///  - `AutoShift::Categories::symbolKeys()`
  ///  - `AutoShift::Categories::arrowKeys()`
  ///  - `AutoShift::Categories::functionKeys()`
  ///  - `AutoShift::Categories::printableKeys()`
  ///  - `AutoShift::Categories::allKeys()`
  void enable(Categories category) {
    settings_.enabled_categories.add(category);
  }
  /// Removes a `Key` category from the set eligible for auto-shift
  void disable(Categories category) {
    settings_.enabled_categories.remove(category);
  }
  /// Replaces the list of `Key` categories eligible for auto-shift
  void setEnabled(Categories categories) {
    settings_.enabled_categories = categories;
  }
  /// Returns `true` if the given category is eligible for auto-shift
  bool isEnabled(Categories category) {
    return settings_.enabled_categories.contains(category);
  }

  /// The category representing letter keys
  static constexpr Categories letterKeys() {
    return Categories::letterKeys();
  }
  /// The category representing number keys (on the number row)
  static constexpr Categories numberKeys() {
    return Categories::numberKeys();
  }
  /// The category representing other printable symbol keys
  static constexpr Categories symbolKeys() {
    return Categories::symbolKeys();
  }
  /// The category representing arrow keys
  static constexpr Categories arrowKeys() {
    return Categories::arrowKeys();
  }
  /// The category representing function keys
  static constexpr Categories functionKeys() {
    return Categories::functionKeys();
  }
  /// Letters, numbers, and other symbols
  static constexpr Categories printableKeys() {
    return Categories::printableKeys();
  }
  /// All non-modifier keyboard keys
  static constexpr Categories allKeys() {
    return Categories::allKeys();
  }

  // ---------------------------------------------------------------------------
  /// Determines which keys will trigger auto-shift if held long enough
  ///
  /// This function can be overridden by the user sketch to configure which keys
  /// can trigger auto-shift.
  bool isAutoShiftable(Key key);

  // ---------------------------------------------------------------------------
  // Event handlers
  EventHandlerResult onKeyswitchEvent(KeyEvent &event);
  EventHandlerResult afterEachCycle();

  template<uint8_t _explicitmappings_count>
  void configureLongPresses(LongPress const (&explicitmappings)[_explicitmappings_count]) {
    explicitmappings_       = explicitmappings;
    explicitmappings_count_ = _explicitmappings_count;
  }

 private:
  // ---------------------------------------------------------------------------
  /// A container for AutoShift configuration settings
  struct Settings {
    /// The overall state of the plugin (on/off)
    bool enabled = true;
    /// The length of time (ms) a key must be held to trigger auto-shift
    uint16_t timeout = 175;
    /// The set of `Key` categories eligible to be auto-shifted
    Categories enabled_categories = AutoShift::Categories::printableKeys();
  };
  Settings settings_;

  // ---------------------------------------------------------------------------
  // Key event queue state variables

  // A device for processing only new events
  KeyEventTracker event_tracker_;

  // The maximum number of events in the queue at a time.
  static constexpr uint8_t queue_capacity_{4};

  // The event queue stores a series of press and release events.
  KeyAddrEventQueue<queue_capacity_> queue_;

  // If there's a delayed keypress from AutoShift, this stored event will
  // contain a valid `KeyAddr`.  The default constructor produces an event addr
  // of `KeyAddr::none()`, so the plugin will start in an inactive state.
  KeyEvent pending_event_;

  void flushQueue();
  void flushEvent(bool is_long_press = false);
  bool checkForRelease() const;

  /// The default function for `isAutoShiftable()`
  bool enabledForKey(Key key);

  bool isExplicitlyMapped(Key key);

  // An array of LongPress objects in PROGMEM.
  LongPress const *explicitmappings_{nullptr};
  uint8_t explicitmappings_count_{0};

  // A cache of the current explicit config key values, so we
  // don't have to keep looking them up from PROGMEM.
  LongPress mapped_key_ = {.key = Key_Transparent, .alternate_key = Key_Transparent};
};

// =============================================================================
/// Configuration plugin for persistent storage of settings
class AutoShiftConfig : public Plugin {
 public:
  EventHandlerResult onSetup();
  EventHandlerResult onFocusEvent(const char *input);
  void disableAutoShiftIfUnconfigured();

 private:
  // The base address in persistent storage for configuration data
  uint16_t settings_base_;
};

}  // namespace plugin
}  // namespace kaleidoscope

extern kaleidoscope::plugin::AutoShift AutoShift;
extern kaleidoscope::plugin::AutoShiftConfig AutoShiftConfig;

#define AUTOSHIFT(longpress_defs...)                                    \
  {                                                                     \
    static kaleidoscope::plugin::LongPress const qk_table[] PROGMEM = { \
      longpress_defs};                                                  \
    AutoShift.configureLongPresses(qk_table);                           \
  }
