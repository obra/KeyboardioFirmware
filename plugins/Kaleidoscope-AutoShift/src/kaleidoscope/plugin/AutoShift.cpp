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

#include "kaleidoscope/plugin/AutoShift.h"

#include "kaleidoscope/KeyAddr.h"          // for KeyAddr, MatrixAddr
#include "kaleidoscope/KeyEvent.h"         // for KeyEvent
#include "kaleidoscope/KeyEventTracker.h"  // for KeyEventTracker
#include "kaleidoscope/Runtime.h"          // for Runtime, Runtime_
#include "kaleidoscope/key_defs.h"         // for Key, Key_0, Key_1, Key_A, Key_F1, Key_F12, Key...
#include "kaleidoscope/keyswitch_state.h"  // for keyToggledOn, keyIsInjected
#include "kaleidoscope/progmem_helpers.h"  // for cloneFromProgmem

// IWYU pragma: no_include "HIDAliases.h"

namespace kaleidoscope {
namespace plugin {


// =============================================================================
// AutoShift functions

void AutoShift::disable() {
  settings_.enabled = false;
  if (pending_event_.addr.isValid()) {
    Runtime.handleKeyswitchEvent(pending_event_);
  }
}

// -----------------------------------------------------------------------------
// Test for whether or not to apply AutoShift to a given `Key`.  This function
// can be overridden from the user sketch.
__attribute__((weak)) bool AutoShift::isAutoShiftable(Key key) {
  return enabledForKey(key);
}

// The default method that determines whether a particular key is an auto-shift
// candidate.  Used by `isAutoShiftable()`, separate to allow re-use when the
// caller is overridden.
bool AutoShift::enabledForKey(Key key) {
  // We only support auto-shifting keyboard keys.  We could also explicitly
  // ignore modifier keys, but there's no need to do so, as none of the code
  // below matches modifiers.
  if (!key.isKeyboardKey())
    return false;

  // Check whether we have an explicit mapping for that key
  if (isExplicitlyMapped(key)) {
    return true;
  }

  // We compare only the keycode, and disregard any modifier flags applied to
  // the key.  This simplifies the comparison, and also allows AutoShift to
  // apply to keys like `RALT(Key_E)`.
  uint8_t keycode = key.getKeyCode();

  if (isEnabled(Categories::allKeys())) {
    if (keycode < HID_KEYBOARD_FIRST_MODIFIER)
      return true;
  }
  if (isEnabled(Categories::letterKeys())) {
    if (keycode >= Key_A.getKeyCode() && keycode <= Key_Z.getKeyCode())
      return true;
  }
  if (isEnabled(Categories::numberKeys())) {
    if (keycode >= Key_1.getKeyCode() && keycode <= Key_0.getKeyCode())
      return true;
  }
  if (isEnabled(Categories::symbolKeys())) {
    if ((keycode >= Key_Minus.getKeyCode() && keycode <= Key_Slash.getKeyCode()) ||
        (keycode == Key_NonUsBackslashAndPipe.getKeyCode()))
      return true;
  }
  if (isEnabled(Categories::arrowKeys())) {
    if (keycode >= Key_RightArrow.getKeyCode() &&
        keycode <= Key_LeftArrow.getKeyCode())
      return true;
  }
  if (isEnabled(Categories::functionKeys())) {
    if ((keycode >= Key_F1.getKeyCode() && keycode <= Key_F12.getKeyCode()) ||
        (keycode >= Key_F13.getKeyCode() && keycode <= Key_F24.getKeyCode()))
      return true;
  }

  return false;
}

bool AutoShift::isExplicitlyMapped(Key key) {
  // Check whether the given key has an explicit mapping to a different one
  for (uint8_t i{0}; i < explicitmappings_count_; ++i) {
    LongPress mappedKey = cloneFromProgmem(explicitmappings_[i]);
    if (mappedKey.key == key) {
      // cache the mapped key to not have to search it again
      mapped_key_.key           = mappedKey.key;
      mapped_key_.alternate_key = mappedKey.alternate_key;
      return true;
    }
  }

  // If no matches were found, clear mapped_key_ and return false
  mapped_key_.key           = Key_Transparent;
  mapped_key_.alternate_key = Key_Transparent;
  return false;
}

// =============================================================================
// Event handler hook functions

// -----------------------------------------------------------------------------
EventHandlerResult AutoShift::onKeyswitchEvent(KeyEvent &event) {
  // If AutoShift has already processed and released this event, ignore it.
  // There's no need to update the event tracker in this one case.
  if (event_tracker_.shouldIgnore(event)) {
    // We should never get an event that's in our queue here, but just in case
    // some other plugin sends one, abort.
    if (queue_.shouldAbort(event))
      return EventHandlerResult::ABORT;
    return EventHandlerResult::OK;
  }

  // If event.addr is not a physical key, ignore it; some other plugin injected
  // it.  This check should be unnecessary.
  if (!event.addr.isValid() || keyIsInjected(event.state)) {
    return EventHandlerResult::OK;
  }

  // Do nothing if disabled.
  if (!settings_.enabled)
    return EventHandlerResult::OK;

  if (!queue_.isEmpty()) {
    // There's an unresolved AutoShift key press.
    if (keyToggledOn(event.state) ||
        event.addr == queue_.addr(0) ||
        queue_.isFull()) {
      // If a new key toggled on, the unresolved key toggled off (it was a
      // "tap"), or if the queue is full, we clear the queue, and the key event
      // does not get modified.
      flushEvent(false);
      flushQueue();
    } else {
      // Otherwise, add the release event to the queue.  We do this so that
      // rollover from a modifier to an auto-shifted key will result in the
      // modifier being applied to the key.
      queue_.append(event);
      return EventHandlerResult::ABORT;
    }
  }

  if (keyToggledOn(event.state) && isAutoShiftable(event.key)) {
    // The key is eligible to be auto-shifted, so we add it to the queue and
    // defer processing of the event.
    queue_.append(event);
    return EventHandlerResult::ABORT;
  }

  return EventHandlerResult::OK;
}

// -----------------------------------------------------------------------------
EventHandlerResult AutoShift::afterEachCycle() {
  // If there's a pending AutoShift event, and it has timed out, we need to
  // release the event with the `shift` flag applied.
  if (!queue_.isEmpty() &&
      Runtime.hasTimeExpired(queue_.timestamp(0), settings_.timeout)) {
    // Toggle the state of the `SHIFT_HELD` bit in the modifier flags for the
    // key for the pending event.
    flushEvent(true);
    flushQueue();
  }
  return EventHandlerResult::OK;
}

void AutoShift::flushQueue() {
  while (!queue_.isEmpty()) {
    if (queue_.isRelease(0) || checkForRelease()) {
      flushEvent(false);
    } else {
      return;
    }
  }
}

bool AutoShift::checkForRelease() const {
  KeyAddr queue_head_addr = queue_.addr(0);
  for (uint8_t i = 1; i < queue_.length(); ++i) {
    if (queue_.addr(i) == queue_head_addr) {
      // This key's release is also in the queue
      return true;
    }
  }
  return false;
}

void AutoShift::flushEvent(bool is_long_press) {
  if (queue_.isEmpty())
    return;
  KeyEvent event = queue_.event(0);
  if (is_long_press) {
    event.key = Runtime.lookupKey(event.addr);

    // If we have an explicit mapping for that key, apply that.
    if (mapped_key_.key != Key_Transparent) {
      event.key = mapped_key_.alternate_key;
    } else {
      // If there was no explicit mapping, just add the shift modifier
      // event.key = longpresses[event.key]
      uint8_t flags = event.key.getFlags();
      flags ^= SHIFT_HELD;
      event.key.setFlags(flags);
    }
  }
  queue_.shift();
  Runtime.handleKeyswitchEvent(event);
}

}  // namespace plugin
}  // namespace kaleidoscope

kaleidoscope::plugin::AutoShift AutoShift;
