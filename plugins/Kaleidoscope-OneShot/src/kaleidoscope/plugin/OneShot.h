/* Kaleidoscope-OneShot -- One-shot modifiers and layers
 * Copyright 2016-2025 Keyboard.io, inc.
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

#include <Kaleidoscope-Ranges.h>  // for OSL_FIRST, OSM_FIRST, OS_FIRST, OS_LAST
#include <stdint.h>               // for uint16_t, uint8_t, int16_t

#include "kaleidoscope/KeyAddr.h"               // for KeyAddr
#include "kaleidoscope/KeyAddrBitfield.h"       // for KeyAddrBitfield
#include "kaleidoscope/KeyEvent.h"              // for KeyEvent
#include "kaleidoscope/Runtime.h"               // for Runtime, Runtime_
#include "kaleidoscope/event_handler_result.h"  // for EventHandlerResult
#include "kaleidoscope/key_defs.h"              // for Key
#include "kaleidoscope/plugin.h"                // for Plugin

// IWYU pragma: no_include "HIDAliases.h"

// ----------------------------------------------------------------------------
// Keymap macros

#define OSM(k) ::kaleidoscope::plugin::OneShotModifierKey(Key_##k)
#define OSL(n) ::kaleidoscope::plugin::OneShotLayerKey(n)

namespace kaleidoscope {
namespace plugin {

constexpr Key OneShotModifierKey(Key mod_key) {
  return Key(kaleidoscope::ranges::OSM_FIRST +
             mod_key.getKeyCode() - HID_KEYBOARD_FIRST_MODIFIER);
}

constexpr Key OneShotLayerKey(uint8_t layer) {
  return Key(kaleidoscope::ranges::OSL_FIRST + layer);
}

class OneShot : public kaleidoscope::Plugin {
 public:
  // --------------------------------------------------------------------------
  // Configuration functions

  void enableStickablity() {}
  void enableStickability(Key key);
  template<typename... Keys>
  void enableStickability(Key key, Keys &&...keys) {
    enableStickability(key);
    enableStickability(keys...);
  }
  void enableStickabilityForModifiers();
  void enableStickabilityForLayers();

  void disableStickability() {}
  void disableStickability(Key key);
  template<typename... Keys>
  void disableStickability(Key key, Keys &&...keys) {
    disableStickability(key);
    disableStickability(keys...);
  }
  void disableStickabilityForModifiers();
  void disableStickabilityForLayers();

  void enableAutoModifiers() {
    settings_.auto_modifiers = true;
  }
  void enableAutoLayers() {
    settings_.auto_layers = true;
  }
  void enableAutoOneShot() {
    enableAutoModifiers();
    enableAutoLayers();
  }

  void disableAutoModifiers() {
    settings_.auto_modifiers = false;
  }
  void disableAutoLayers() {
    settings_.auto_layers = false;
  }
  void disableAutoOneShot() {
    disableAutoModifiers();
    disableAutoLayers();
  }

  void toggleAutoModifiers() {
    settings_.auto_modifiers = !settings_.auto_modifiers;
  }
  void toggleAutoLayers() {
    settings_.auto_layers = !settings_.auto_layers;
  }
  void toggleAutoOneShot() {
    if (settings_.auto_modifiers || settings_.auto_layers) {
      disableAutoOneShot();
    } else {
      enableAutoOneShot();
    }
  }

  // --------------------------------------------------------------------------
  // Global test functions

  bool isActive() const;
  bool isSticky() const;

  // --------------------------------------------------------------------------
  // Single-key test functions
  bool isOneShotKey(Key key) const {
    return (key.getRaw() >= kaleidoscope::ranges::OS_FIRST &&
            key.getRaw() <= kaleidoscope::ranges::OS_LAST);
  }

  /// Determine if the given `key` is allowed to become sticky.
  bool isStickable(Key key) const;

  bool isStickableDefault(Key key) const;

  bool isTemporary(KeyAddr key_addr) const;  // inline?
  bool isPending(KeyAddr key_addr) const;
  bool isSticky(KeyAddr key_addr) const;  // inline?
  bool isActive(KeyAddr key_addr) const;  // inline?

  // --------------------------------------------------------------------------
  // Public OneShot state control

  /// Put a key in the "pending" OneShot state.
  ///
  /// This function puts the key at `key_addr` in the "pending" OneShot state.
  /// This is appropriate to use when a key toggles on and you want it to behave
  /// like a OneShot key starting with the current event, and lasting until the
  /// key becomes inactive (cancelled by a subsequent keypress).
  void setPending(KeyAddr key_addr);

  /// Put a key directly in the "one-shot" state.
  ///
  /// This function puts the key at `key_addr` in the "one-shot" state.  This is
  /// usually the state of a OneShot key after it is released, but before it is
  /// cancelled by a subsequent keypress.  In most cases, you probably want to
  /// use `setPending()` instead, rather than calling this function explicitly,
  /// as OneShot will automatically cause any key in the "pending" state to
  /// progress to this state when it is (physically) released.
  void setOneShot(KeyAddr key_addr);

  /// Put a key in the "sticky" OneShot state.
  ///
  /// This function puts the key at `key_addr` in the "sticky" OneShot state.
  /// It will remain active until it is pressed again.
  void setSticky(KeyAddr key_addr);

  /// Clear any OneShot state for a key.
  ///
  /// This function clears any OneShot state of the key at `key_addr`.  It does
  /// not, however, release the key if it is held.
  void clear(KeyAddr key_addr);

  // --------------------------------------------------------------------------
  // Utility function for other plugins to cancel OneShot keys
  void cancel(bool with_stickies = false);

  // --------------------------------------------------------------------------
  // Timeout onfiguration functions
  void setTimeout(uint16_t ttl) {
    settings_.timeout = ttl;
  }
  uint16_t getTimeout() {
    return settings_.timeout;
  }
  void setHoldTimeout(uint16_t ttl) {
    settings_.hold_timeout = ttl;
  }
  uint16_t getHoldTimeout() {
    return settings_.hold_timeout;
  }
  void setDoubleTapTimeout(int16_t ttl) {
    settings_.double_tap_timeout = ttl;
  }
  
  int16_t getDoubleTapTimeout() {
    return settings_.double_tap_timeout;
  }

  void setStickyTimeout(uint32_t ttl) {
	settings_.sticky_time_out_ = ttl;
  }

  void setStickyTimeOutSeconds(uint32_t ttl) {
	settings.sticky_time_out_ = ttl * 1000;
  }

  uint32_t getStickyTimeOut() {
    return settings_.sticky_time_out_;
  }
  // --------------------------------------------------------------------------
  // Plugin hook functions

  EventHandlerResult onNameQuery();
  EventHandlerResult onKeyEvent(KeyEvent &event);
  EventHandlerResult afterReportingState(const KeyEvent &event);
  EventHandlerResult afterEachCycle();

  friend class OneShotConfig;

 private:
  // --------------------------------------------------------------------------
  // Constants
  static constexpr uint8_t oneshot_key_count         = 16;
  static constexpr uint8_t oneshot_mod_count         = 8;
  static constexpr uint8_t oneshot_layer_count       = oneshot_key_count - oneshot_mod_count;
  static constexpr uint16_t stickable_modifiers_mask = uint16_t(uint16_t(-1) >> oneshot_layer_count);
  static constexpr uint16_t stickable_layers_mask    = uint16_t(uint16_t(-1) << oneshot_mod_count);
  static constexpr KeyAddr invalid_key_addr          = KeyAddr(KeyAddr::invalid_state);

  // --------------------------------------------------------------------------
  // Configuration variables
  struct Settings {
    uint16_t timeout           = 2500;
    uint16_t hold_timeout      = 250;
    int16_t double_tap_timeout = -1;
    uint32_t sticky_time_out_   = -1;

    uint16_t stickable_keys = -1;
    bool auto_modifiers     = false;
    bool auto_layers        = false;
  };
  Settings settings_;

  // --------------------------------------------------------------------------
  // State variables
  KeyAddrBitfield temp_addrs_;
  KeyAddrBitfield glue_addrs_;

  uint32_t start_time_   = 0;
  KeyAddr prev_key_addr_ = invalid_key_addr;

  // --------------------------------------------------------------------------
  // Internal utility functions
  bool hasTimedOut(uint16_t ttl) const {
    return Runtime.hasTimeExpired(start_time_, ttl);
  }
  bool hasTimedOut32(uint32_t ttl) const {
	return Runtime.hasTimeExpired(start_time_, ttl);
  }
  bool hasDoubleTapTimedOut() const {
    // Derive the true double-tap timeout value if we're using the default.
    uint16_t dtto = (settings_.double_tap_timeout < 0)
                      ? settings_.timeout
                      : settings_.double_tap_timeout;
    return hasTimedOut(dtto);
  }
  bool hasStickyTimedOut() const {
	// uint32_t dtto = (sticky_time_out_ < 0) ? timeout_ : sticky_time_out_;
	return hasTimedOut32(sticky_time_out_);
  }
  uint8_t getOneShotKeyIndex(Key oneshot_key) const;
  uint8_t getKeyIndex(Key key) const;
  Key decodeOneShotKey(Key oneshot_key) const;

  void pressKey(KeyAddr key_addr, Key oneshot_key);
  void holdKey(KeyAddr key_addr) const;
  void releaseKey(KeyAddr key_addr);
};

// =============================================================================
// Plugin for configuration of OneShot via Focus and persistent storage of
// settins in EEPROM (i.e. Chrysalis).
class OneShotConfig : public Plugin {
 public:
  EventHandlerResult onNameQuery();
  EventHandlerResult onSetup();
  EventHandlerResult onFocusEvent(const char *command);

 private:
  // The base address in persistent storage for configuration data:
  uint16_t settings_base_;
};

}  // namespace plugin
}  // namespace kaleidoscope

extern kaleidoscope::plugin::OneShot OneShot;
extern kaleidoscope::plugin::OneShotConfig OneShotConfig;
