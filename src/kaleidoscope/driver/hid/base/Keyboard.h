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

#pragma once

#include <stdint.h>  // for uint8_t

#include "kaleidoscope/key_defs.h"  // for Key, Key_LeftAlt, Key_LeftControl, Key_LeftGui, Key_L...

#ifndef HID_BOOT_PROTOCOL
#define HID_BOOT_PROTOCOL 0
#endif

namespace kaleidoscope {
namespace driver {
namespace hid {
namespace base {

class NoBootKeyboard {
 public:
  NoBootKeyboard() {}
  void begin() {}

  uint8_t getProtocol() {
    return 1;
  }

  uint8_t getBootOnly() {
    return 0;
  }
  void setBootOnly(uint8_t bootonly) {}

  void sendReport() {}

  void press(uint8_t code) {}
  void release(uint8_t code) {}
  void releaseAll() {}

  bool isModifierActive(uint8_t code) {
    return false;
  }
  bool wasModifierActive(uint8_t code) {
    return false;
  }
  bool isAnyModifierActive() {
    return false;
  }
  bool wasAnyModifierActive() {
    return false;
  }
  bool isKeyPressed(uint8_t code) {
    return false;
  }

  uint8_t getLeds() {
    return 0;
  }

  void onUSBReset() {}
};

class NoConsumerControl {
 public:
  NoConsumerControl() {}
  void begin() {}

  void sendReport() {}
  void releaseAll() {}

  void press(uint8_t code) {}
  void release(uint8_t code) {}
};

class NoSystemControl {
 public:
  NoSystemControl() {}
  void begin() {}

  void press(uint8_t code) {}
  void release() {}
};

struct KeyboardProps {
  typedef NoBootKeyboard BootKeyboard;
  typedef NoConsumerControl ConsumerControl;
  typedef NoSystemControl SystemControl;
};

// Abstract interface base class to enable hybrid HID drivers
class KeyboardItf {
// Vtables are too big for AVR
#ifndef ARDUINO_ARCH_AVR
 public:
  virtual void setup() = 0;

  virtual void sendReport()                           = 0;
  virtual void releaseAllKeys()                       = 0;
  virtual void pressConsumerControl(Key mapped_key)   = 0;
  virtual void releaseConsumerControl(Key mapped_key) = 0;
  virtual void pressSystemControl(Key mapped_key)     = 0;
  virtual void releaseSystemControl(Key mapped_key)   = 0;
  virtual void pressKey(Key pressed_key)              = 0;
  virtual void pressModifiers(Key pressed_key)        = 0;
  virtual void releaseModifiers(Key released_key)     = 0;
  virtual void clearModifiers()                       = 0;
  virtual void pressRawKey(Key pressed_key)           = 0;
  virtual void releaseRawKey(Key released_key)        = 0;
  virtual void releaseKey(Key released_key)           = 0;

  virtual bool isKeyPressed(Key key)                  = 0;
  virtual bool isModifierKeyActive(Key modifier_key)  = 0;
  virtual bool wasModifierKeyActive(Key modifier_key) = 0;
  virtual bool isAnyModifierKeyActive()               = 0;
  virtual bool wasAnyModifierKeyActive()              = 0;
  virtual uint8_t getKeyboardLEDs()                   = 0;
  virtual uint8_t getProtocol()                       = 0;
  virtual uint8_t getBootOnly()                       = 0;

  virtual void setBootOnly(uint8_t bootonly) = 0;
  virtual void onUSBReset()                  = 0;
#endif
};

template<typename _Props>
class Keyboard : public KeyboardItf {
 private:
  typename _Props::BootKeyboard boot_keyboard_;
  typename _Props::ConsumerControl consumer_control_;
  typename _Props::SystemControl system_control_;

 public:
  Keyboard() {}

  void setup() __attribute__((noinline)) {
    boot_keyboard_.begin();
    consumer_control_.begin();
    system_control_.begin();
  }

  void sendReport() __attribute__((noinline)) {
    boot_keyboard_.sendReport();
    if (boot_keyboard_.getProtocol() != HID_BOOT_PROTOCOL) {
      consumer_control_.sendReport();
    }
  }
  void releaseAllKeys() __attribute__((noinline)) {
    boot_keyboard_.releaseAll();
    if (boot_keyboard_.getProtocol() != HID_BOOT_PROTOCOL) {
      consumer_control_.releaseAll();
    }
  }
  void pressConsumerControl(Key mapped_key) {
    consumer_control_.press(CONSUMER(mapped_key));
  }
  void releaseConsumerControl(Key mapped_key) {
    consumer_control_.release(CONSUMER(mapped_key));
  }
  void pressSystemControl(Key mapped_key) {
    uint8_t keycode = mapped_key.getKeyCode();
    system_control_.press(keycode);
    last_system_control_keycode_ = keycode;
  }
  void releaseSystemControl(Key mapped_key) {
    uint8_t keycode = mapped_key.getKeyCode();
    if (keycode == last_system_control_keycode_) {
      system_control_.release();
    }
  }

  void pressKey(Key pressed_key) __attribute__((noinline)) {
    pressModifiers(pressed_key);
    pressRawKey(pressed_key);
  }

  void pressModifiers(Key pressed_key) {
    pressModifiers(pressed_key.getFlags());
  }
  void releaseModifiers(Key released_key) {
    releaseModifiers(released_key.getFlags());
  }
  void clearModifiers() {
    releaseRawKey(Key_LeftShift);
    releaseRawKey(Key_LeftControl);
    releaseRawKey(Key_LeftAlt);
    releaseRawKey(Key_RightAlt);
    releaseRawKey(Key_LeftGui);
  }

  // pressRawKey takes a Key object and calles KeyboardioHID's ".press" method
  // with its keycode. It does no processing of any flags or modifiers on the key
  void pressRawKey(Key pressed_key) {
    boot_keyboard_.press(pressed_key.getKeyCode());
  }

  void releaseRawKey(Key released_key) {
    boot_keyboard_.release(released_key.getKeyCode());
  }

  void releaseKey(Key released_key) {
    releaseModifiers(released_key.getFlags());
    releaseRawKey(released_key);
  }

  bool isKeyPressed(Key key) {
    return boot_keyboard_.isKeyPressed(key.getKeyCode());
  }

  bool isModifierKeyActive(Key modifier_key) {
    return boot_keyboard_.isModifierActive(modifier_key.getKeyCode());
  }

  bool wasModifierKeyActive(Key modifier_key) {
    return boot_keyboard_.wasModifierActive(modifier_key.getKeyCode());
  }

  bool isAnyModifierKeyActive() {
    return boot_keyboard_.isAnyModifierActive();
  }

  bool wasAnyModifierKeyActive() {
    return boot_keyboard_.wasAnyModifierActive();
  }

  uint8_t getKeyboardLEDs() {
    return boot_keyboard_.getLeds();
  }

  uint8_t getProtocol() {
    return boot_keyboard_.getProtocol();
  }

  uint8_t getBootOnly() {
    return boot_keyboard_.getBootOnly();
  }
  void setBootOnly(uint8_t bootonly) {
    boot_keyboard_.setBootOnly(bootonly);
  }

  void onUSBReset() {
    boot_keyboard_.onUSBReset();
  }

 private:
  // To prevent premature release of a System Control key when rolling
  // over from one to another, we record the last System Control
  // keycode that was pressed. It's initialized to zero, which should
  // not be a valid System Control keycode.
  uint8_t last_system_control_keycode_ = 0;

  // pressModifiers takes a bitmap of modifier keys that must be included in
  // the upcoming USB HID report and passes them through to KeyboardioHID
  // immediately

  void pressModifiers(uint8_t flags) {
    if (flags & SHIFT_HELD) {
      pressRawKey(Key_LeftShift);
    }
    if (flags & CTRL_HELD) {
      pressRawKey(Key_LeftControl);
    }
    if (flags & LALT_HELD) {
      pressRawKey(Key_LeftAlt);
    }
    if (flags & RALT_HELD) {
      pressRawKey(Key_RightAlt);
    }
    if (flags & GUI_HELD) {
      pressRawKey(Key_LeftGui);
    }
  }

  // releaseModifiers takes a bitmap of modifier keys that must not be included in
  // the upcoming USB HID report and passes them through to KeyboardioHID
  // immediately

  void releaseModifiers(uint8_t flags) {
    if (flags & SHIFT_HELD) {
      releaseRawKey(Key_LeftShift);
    }
    if (flags & CTRL_HELD) {
      releaseRawKey(Key_LeftControl);
    }
    if (flags & LALT_HELD) {
      releaseRawKey(Key_LeftAlt);
    }
    if (flags & RALT_HELD) {
      releaseRawKey(Key_RightAlt);
    }
    if (flags & GUI_HELD) {
      releaseRawKey(Key_LeftGui);
    }
  }
};

}  // namespace base
}  // namespace hid
}  // namespace driver
}  // namespace kaleidoscope
