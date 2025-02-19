/* Kaleidoscope-Hardware-Keyboardio-Preonic -- Keyboardio Preonic hardware support for Kaleidoscope
 * Copyright 2018-2025 Keyboard.io, inc.
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
#ifndef BUILD_INFORMATION
#define BUILD_INFORMATION "locally built on " __DATE__ " at " __TIME__
#endif

#include "Kaleidoscope.h"
#include "Kaleidoscope-EEPROM-Settings.h"
#include "Kaleidoscope-EEPROM-Keymap.h"
#include "Kaleidoscope-Escape-OneShot.h"
#include "Kaleidoscope-FirmwareVersion.h"
#include "Kaleidoscope-FocusSerial.h"
#include "Kaleidoscope-Macros.h"
#include "Kaleidoscope-MouseKeys.h"
#include "Kaleidoscope-OneShot.h"
#include "Kaleidoscope-Qukeys.h"
#include "Kaleidoscope-SpaceCadet.h"
#include "Kaleidoscope-DynamicMacros.h"
#include "Kaleidoscope-LayerNames.h"
#include "Kaleidoscope-HostPowerManagement.h"
#include "Kaleidoscope-LEDControl.h"
#include "Kaleidoscope-Keyclick.h"
#include "Kaleidoscope-MagicCombo.h"

#include "Kaleidoscope-LEDEffect-Rainbow.h"
#include "Kaleidoscope-LEDEffect-BootGreeting.h"
// Support for shared palettes for other plugins, like Colormap below
#include "Kaleidoscope-LED-Palette-Theme.h"

// Support for an LED mode that lets one configure per-layer color maps
#include "Kaleidoscope-PreonicColormap.h"


// Support for setting and saving the default LED mode
#include "Kaleidoscope-DefaultLEDModeConfig.h"

// Support for changing the brightness of the LEDs
#include "Kaleidoscope-LEDBrightnessConfig.h"

// Support for showing device status with the LED Indicators
#include "Kaleidoscope-LEDIndicators.h"


/** This 'enum' is a list of all the macros used by the Model 100's firmware
  * The names aren't particularly important. What is important is that each
  * is unique.
  *
  * These are the names of your macros. They'll be used in two places.
  * The first is in your keymap definitions. There, you'll use the syntax
  * `M(MACRO_NAME)` to mark a specific keymap position as triggering `MACRO_NAME`
  *
  * The second usage is in the 'switch' statement in the `macroAction` function.
  * That switch statement actually runs the code associated with a macro when
  * a macro key is pressed.
  */

enum {
  MACRO_VERSION_INFO,
  MACRO_ANY,
  MACRO_TOGGLE_MODE,
  MACRO_BT_SELECT_1,  // Select slot 1
  MACRO_BT_SELECT_2,  // Select slot 2
  MACRO_BT_SELECT_3,  // Select slot 3
  MACRO_BT_SELECT_4,  // Select slot 4
  MACRO_BT_PAIR,      // Start pairing for selected slot
  MACRO_BT_OFF
};

// Define our magic combo
enum {
  COMBO_ENTER_TEST_MODE
};

// Configure the magic combo
USE_MAGIC_COMBOS(
  [COMBO_ENTER_TEST_MODE] = {
    .action = [] (uint8_t combo_index) {
    //  PreonicHardwareTestMode.enterTestMode();
    },
    .keys = {
      R1C0,  // `
      R5C0,  // Hyper
      R5C11  // Right arrow
    }
  }
);

#define Key_Star LSHIFT(Key_8)
#define Key_Plus LSHIFT(Key_Equals)

enum {
  QWERTY,
  LOWER,
  RAISE,
  FUN
};

// clang-format off
KEYMAPS(
  [QWERTY] = KEYMAP
  (
    Consumer_VolumeDecrement, Consumer_VolumeIncrement, M(MACRO_ANY), ShiftToLayer(FUN),                 Consumer_PlaySlashPause,
    Key_Backtick,   Key_1,           Key_2,           Key_3,                   Key_4,           Key_5,           Key_6,           Key_7,           Key_8,                      Key_9,           Key_0,           Key_Minus,
    Key_Tab,        Key_Q,           Key_W,           Key_E,                   Key_R,           Key_T,           Key_Y,           Key_U,           Key_I,                      Key_O,           Key_P,           Key_Backspace,
    Key_Escape,     Key_A,           Key_S,           Key_D,                   Key_F,           Key_G,           Key_H,           Key_J,           Key_K,                      Key_L,           Key_Semicolon,   Key_Quote,
    Key_LeftShift,  Key_Z,           Key_X,           Key_C,                   Key_V,           Key_B,           Key_N,           Key_M,           Key_Comma,                  Key_Period,      Key_Slash,       Key_Enter,
    Key_Hyper,      Key_LeftControl, Key_LeftAlt,     Key_LeftGui,            ShiftToLayer(LOWER),       Key_Backspace,   Key_Space,       ShiftToLayer(RAISE),       Key_LeftArrow,             Key_DownArrow,   Key_UpArrow,     Key_RightArrow
  ),

  [LOWER] = KEYMAP
  (
   ___, ___,  ___,            ___,             ___,
    ___,            ___,             ___,             ___,             ___,             ___,             ___,             ___,             Key_Slash,      Key_Star,        Key_Minus,       Key_Equals,
    ___,            ___,             ___,             ___,             ___,             ___,             ___,             Key_7,           Key_8,          Key_9,           Key_Minus,       ___,
    ___,            ___,             ___,             ___,             ___,             ___,             ___,             Key_4,           Key_5,          Key_6,           Key_Plus,        ___,
    ___,            ___,             ___,             ___,             ___,             ___,             ___,             Key_1,           Key_2,          Key_3,           Key_Plus,        ___,
    ___,            ___,             ___,             ___,             ___,             ___,             Key_Backspace,   Key_0,           Key_Period,     ___,             Key_Enter,       ___
  ),

  [RAISE] = KEYMAP
  (
      ___, ___,  ___,            ___,             ___,
    Key_Escape,     Key_F1,          Key_F2,          Key_F3,                 Key_F4,          Key_F5,          Key_F6,          Key_F7,          Key_F8,                     Key_F9,          Key_F10,         Key_Backspace,
    Key_Backtick,   ___,             Key_mouseUp,     ___,                    Key_mouseWarpNW, Key_mouseWarpNE, Key_Backslash,   LSHIFT(Key_LeftBracket), LSHIFT(Key_RightBracket), Key_LeftBracket, Key_RightBracket, Key_Delete,
    Key_CapsLock,   Key_mouseL,      Key_mouseDn,     Key_mouseR,             Key_mouseWarpSW, Key_mouseWarpSE, Key_LeftArrow,   Key_DownArrow,   Key_UpArrow,                Key_RightArrow,  ___,             ___,
    ___,            Key_mouseBtnL,   Key_mouseBtnM,   Key_mouseBtnR,          ___,             Key_mouseWarpEnd,___,             ___,             ___,                        ___,             ___,             ___,
    ___,            ___,             Key_RightAlt,    ___,                    ___,             ___,             ___,             ___,             ___,                        ___,             ___,             ___
  ),

  [FUN] = KEYMAP
  (
       ___, ___, ___,            ___,             ___,
    M(MACRO_BT_OFF),M(MACRO_BT_SELECT_1),M(MACRO_BT_SELECT_2),M(MACRO_BT_SELECT_3),M(MACRO_BT_SELECT_4), M(MACRO_BT_PAIR),             ___,             ___,             ___,                        ___,            ___, M(MACRO_VERSION_INFO),
    Key_LEDEffectNext,___,           Consumer_VolumeIncrement,___,            ___,             ___,             ___,             ___,             ___,                        ___,             ___,             ___,
    ___,            Consumer_ScanPreviousTrack,Consumer_VolumeDecrement,Consumer_ScanNextTrack,___,             ___,             ___,             ___,             ___,                        ___,             ___,             ___,
    Key_ToggleKeyclick,___,             Consumer_Mute,   ___,                    ___,             ___,             ___,             ___,             ___,                        ___,             ___,             ___,
    ___,            ___,             ___,             ___,                    ___,             ___,             ___,             ___,             ___,                        ___,             ___,              ___
  )
);


// Colors names of the EGA palette, for convenient use in colormaps. Should
// match the palette definition below. Optional, one can just use the indexes
// directly, too.
enum {
  BLACK,
  BLUE,
  GREEN,
  CYAN,
  RED,
  MAGENTA,
  BROWN,
  LIGHT_GRAY,
  DARK_GRAY,
  BRIGHT_BLUE,
  BRIGHT_GREEN,
  BRIGHT_CYAN,
  BRIGHT_RED,
  BRIGHT_MAGENTA,
  YELLOW,
  WHITE
};

// Define an EGA palette. Conveniently, that's exactly 16 colors, just like the
// limit of LEDPaletteTheme.
PALETTE(
    CRGB(0x00, 0x00, 0x00),  // [0x0] black
    CRGB(0x00, 0x00, 0xaa),  // [0x1] blue
    CRGB(0x00, 0xaa, 0x00),  // [0x2] green
    CRGB(0x00, 0xaa, 0xaa),  // [0x3] cyan
    CRGB(0xaa, 0x00, 0x00),  // [0x4] red
    CRGB(0xaa, 0x00, 0xaa),  // [0x5] magenta
    CRGB(0xaa, 0x55, 0x00),  // [0x6] brown
    CRGB(0xaa, 0xaa, 0xaa),  // [0x7] light gray
    CRGB(0x55, 0x55, 0x55),  // [0x8] dark gray
    CRGB(0x55, 0x55, 0xff),  // [0x9] bright blue
    CRGB(0x55, 0xff, 0x55),  // [0xa] bright green
    CRGB(0x55, 0xff, 0xff),  // [0xb] bright cyan
    CRGB(0xff, 0x55, 0x55),  // [0xc] bright red
    CRGB(0xff, 0x55, 0xff),  // [0xd] bright magenta
    CRGB(0xff, 0xff, 0x55),  // [0xe] yellow
    CRGB(0xff, 0xff, 0xff)   // [0xf] white
)

COLORMAPS(
    [0] = COLORMAP (BLACK,  BLACK, BLACK, BLACK),
    [1] = COLORMAP (BLACK,  BLACK, BLACK, BLACK),
    [2] = COLORMAP (BLACK,  BLACK, BLACK, BLACK),
    [3] = COLORMAP (BLACK,  BLACK, BLACK, BLACK),
    [4] = COLORMAP (BLACK,  BLACK, BLACK, BLACK),
    [5] = COLORMAP (BLACK,  BLACK, BLACK, BLACK),
    [6] = COLORMAP (BLACK,  BLACK, BLACK, BLACK),
    [7] = COLORMAP (BLACK,  BLACK, BLACK, BLACK),
    [8] = COLORMAP (BLACK,  BLACK, BLACK, BLACK)  
)


// clang-format on

KALEIDOSCOPE_INIT_PLUGINS(
  // ----------------------------------------------------------------------
  // Chrysalis plugins

  // The EEPROMSettings & EEPROMKeymap plugins make it possible to have an
  // editable keymap in EEPROM.
  EEPROMSettings,
  EEPROMKeymap,

  // Focus allows bi-directional communication with the host, and is the
  // interface through which the keymap in EEPROM can be edited.
  Focus,

  // FocusSettingsCommand adds a few Focus commands, intended to aid in
  // changing some settings of the keyboard, such as the default layer (via the
  // `settings.defaultLayer` command)
  FocusSettingsCommand,

  // FocusEEPROMCommand adds a set of Focus commands, which are very helpful in
  // both debugging, and in backing up one's EEPROM contents.
  FocusEEPROMCommand,

  // The FirmwareVersion plugin lets Chrysalis query the version of the firmware
  // programmatically.
  FirmwareVersion,

  // The LayerNames plugin allows Chrysalis to display - and edit - custom layer
  // names, to be shown instead of the default indexes.
  LayerNames,

  // ----------------------------------------------------------------------
  // Keystroke-handling plugins

  // The Qukeys plugin enables the "Secondary action" functionality in
  // Chrysalis. Keys with secondary actions will have their primary action
  // performed when tapped, but the secondary action when held.
  Qukeys,

  // SpaceCadet can turn your shifts into parens on tap, while keeping them as
  // Shifts when held. SpaceCadetConfig lets Chrysalis configure some aspects of
  // the plugin.
  SpaceCadet,
  SpaceCadetConfig,

  // Enables the "Sticky" behavior for modifiers, and the "Layer shift when
  // held" functionality for layer keys.
  OneShot,
  OneShotConfig,
  EscapeOneShot,
  EscapeOneShotConfig,

  // The macros plugin adds support for macros
  Macros,

  // Enables dynamic, Chrysalis-editable macros.
  DynamicMacros,

  // The MouseKeys plugin lets you add keys to your keymap which move the mouse.
  MouseKeys,
  MouseKeysConfig,

  // The MagicCombo plugin lets you use key combinations to trigger custom
  // actions - a bit like Macros, but triggered by pressing multiple keys at the same time.
  MagicCombo,

  // LEDControl provides support for other LED modes
  LEDControl,

  // Enables setting, saving (via Chrysalis), and restoring (on boot) the
  // default LED mode.
  DefaultLEDModeConfig,


  // We start with the LED effect that turns off all the LEDs.
  LEDOff,

  LEDPaletteTheme,

  // The Colormap effect makes it possible to set up per-layer colormaps
  PreonicColormapEffect,
  DefaultPreonicColormap,

  // The HostPowerManagement plugin allows us to turn LEDs off when then host
  // goes to sleep, and resume them when it wakes up.
 // HostPowerManagement,

  LEDBrightnessConfig,
  LEDRainbowEffect,

  // The Keyclick plugin for audible feedback
  Keyclick,
  // LEDIndicators shows device status indicators using the LEDs
  // It should be listed after all other LED plugins
  LEDIndicators
);


void configureIndicators() {
  // Configure LED mapping to use our four LEDs
  static const KeyAddr indicator_leds[] = {
    KeyAddr(0, 0),
    KeyAddr(0, 1),
    KeyAddr(0, 2),
    KeyAddr(0, 3)};
  LEDIndicators.setSlots(4, indicator_leds);
}


static void versionInfoMacro(uint8_t key_state) {
  if (keyToggledOn(key_state)) {
    Macros.type(PSTR("Keyboardio Preonic - Firmware version "));
    Macros.type(PSTR(BUILD_INFORMATION));
  }
}

/** anyKeyMacro is used to provide the functionality of the 'Any' key.
 *
 * When the 'any key' macro is toggled on, a random alphanumeric key is
 * selected. While the key is held, the function generates a synthetic
 * keypress event repeating that randomly selected key.
 *
 */

static void anyKeyMacro(KeyEvent &event) {
  if (keyToggledOn(event.state)) {
    event.key.setKeyCode(Key_A.getKeyCode() + (uint8_t)(millis() % 36));
    event.key.setFlags(0);
  }
}

/** macroAction dispatches keymap events that are tied to a macro
    to that macro. It takes two uint8_t parameters.

    The first is the macro being called (the entry in the 'enum' earlier in this file).
    The second is the state of the keyswitch. You can use the keyswitch state to figure out
    if the key has just been toggled on, is currently pressed or if it's just been released.

    The 'switch' statement should have a 'case' for each entry of the macro enum.
    Each 'case' statement should call out to a function to handle the macro in question.

 */

const macro_t *macroAction(uint8_t macro_id, KeyEvent &event) {
  switch (macro_id) {
  case MACRO_VERSION_INFO:
    versionInfoMacro(event.state);
    break;
  case MACRO_ANY:
    anyKeyMacro(event);
    break;
  case MACRO_TOGGLE_MODE:
    if (keyToggledOn(event.state)) {
      kaleidoscope::Runtime.device().toggleHostConnectionMode();
    }
    break;
  case MACRO_BT_SELECT_1:
    if (keyToggledOn(event.state)) {
      kaleidoscope::Runtime.device().ble().selectDevice(1);
      kaleidoscope::Runtime.device().setHostConnectionMode(MODE_BLE);
    }
    break;
  case MACRO_BT_SELECT_2:
    if (keyToggledOn(event.state)) {
      kaleidoscope::Runtime.device().ble().selectDevice(2);
      kaleidoscope::Runtime.device().setHostConnectionMode(MODE_BLE);
    }
    break;
  case MACRO_BT_SELECT_3:
    if (keyToggledOn(event.state)) {
      kaleidoscope::Runtime.device().ble().selectDevice(3);
      kaleidoscope::Runtime.device().setHostConnectionMode(MODE_BLE);
    }
    break;
  case MACRO_BT_SELECT_4:
    if (keyToggledOn(event.state)) {
      kaleidoscope::Runtime.device().ble().selectDevice(4);
      kaleidoscope::Runtime.device().setHostConnectionMode(MODE_BLE);
    }
    break;
  case MACRO_BT_PAIR:
    if (keyToggledOn(event.state)) {
      kaleidoscope::Runtime.device().ble().startDiscoverableAdvertising();
    }
    break;
  case MACRO_BT_OFF:
    if (keyToggledOn(event.state)) {
      kaleidoscope::Runtime.device().ble().stopAdvertising();
      kaleidoscope::Runtime.device().ble().disconnect();
      kaleidoscope::Runtime.device().setHostConnectionMode(MODE_USB);
    }
    break;
  }
  return MACRO_NONE;
}

void setup() {
  Kaleidoscope.setup();
  configureIndicators();

 EEPROMKeymap.setup(9);
  PreonicColormapEffect.max_layers(9);
  LEDRainbowEffect.brightness(25);

  DynamicMacros.reserve_storage(512);

  LayerNames.reserve_storage(128);

  // Disable Keyclick by default
  Keyclick.disable();

  Layer.move(EEPROMSettings.default_layer());

  // To avoid any surprises, SpaceCadet is turned off by default. However, it
  // can be permanently enabled via Chrysalis, so we should only disable it if
  // no configuration exists.
  SpaceCadetConfig.disableSpaceCadetIfUnconfigured();

//  DefaultLEDModeConfig.activateLEDModeIfUnconfigured(&LEDRainbowEffect);

  DefaultLEDModeConfig.activateLEDModeIfUnconfigured(&LEDOff);

  //kaleidoscope::Runtime.device().ble().selectDevice(1);
  //kaleidoscope::Runtime.device().setHostConnectionMode(MODE_BLE);

}

void loop() {
  Kaleidoscope.loop();
}
