// -*- mode: c++ -*-

#include <Kaleidoscope.h>

#include <Kaleidoscope-AutoShift.h>
#include <Kaleidoscope-EEPROM-Settings.h>
#include <Kaleidoscope-EEPROM-Keymap.h>
#include <Kaleidoscope-FocusSerial.h>
#include <Kaleidoscope-Macros.h>

enum {
  TOGGLE_AUTOSHIFT,
};

// clang-format off
KEYMAPS(
  [0] = KEYMAP_STACKED
  (
      Key_NoKey,    Key_1, Key_2, Key_3, Key_4, Key_5, Key_NoKey,
      Key_Backtick, Key_Q, Key_W, Key_E, Key_R, Key_T, Key_Tab,
      Key_PageUp,   Key_A, Key_S, Key_D, Key_F, Key_G,
      Key_PageDown, Key_Z, Key_X, Key_C, Key_V, Key_B, Key_Escape,

      Key_LeftControl, Key_Backspace, Key_LeftGui, Key_LeftShift,
      XXX,

      M(TOGGLE_AUTOSHIFT), Key_6, Key_7, Key_8,     Key_9,      Key_0,         Key_skip,
      Key_Enter,           Key_Y, Key_U, Key_I,     Key_O,      Key_P,         Key_Equals,
                           Key_H, Key_J, Key_K,     Key_L,      Key_Semicolon, Key_Quote,
      Key_skip,            Key_N, Key_M, Key_Comma, Key_Period, Key_Slash,     Key_Minus,

      Key_RightShift, Key_RightAlt, Key_Spacebar, Key_RightControl,
      XXX
   ),
)
// clang-format on

// Defining a macro (on the "any" key: see above) to turn AutoShift on and off
const macro_t *macroAction(uint8_t macro_id, KeyEvent &event) {
  switch (macro_id) {
  case TOGGLE_AUTOSHIFT:
    if (keyToggledOn(event.state))
      AutoShift.toggle();
    break;
  }
  return MACRO_NONE;
}

// This sketch uses the AutoShiftConfig plugin, which enables run-time
// configuration of AutoShift configuration settings.  All of the plugins marked
// "for AutoShiftConfig" are optional; AutoShift itself will work without them.
KALEIDOSCOPE_INIT_PLUGINS(
  EEPROMSettings,        // for AutoShiftConfig
  EEPROMKeymap,          // for AutoShiftConfig
  Focus,                 // for AutoShiftConfig
  FocusEEPROMCommand,    // for AutoShiftConfig
  FocusSettingsCommand,  // for AutoShiftConfig
  AutoShift,
  AutoShiftConfig,  // for AutoShiftConfig
  Macros            // for toggle AutoShift Macro
);

void setup() {
  // Enable AutoShift for letter keys and number keys only:
  AutoShift.setEnabled(AutoShift.letterKeys() | AutoShift.numberKeys());
  // Add symbol keys to the enabled categories:
  AutoShift.enable(AutoShift.symbolKeys());
  // instead of shifting, produce a backslash on long pressing slash
  AUTOSHIFT(
    kaleidoscope::plugin::LongPress(Key_Slash, Key_Backslash), )
  // Set the AutoShift long-press time to 150ms:
  AutoShift.setTimeout(150);
  // Start with AutoShift turned off:
  AutoShift.disable();

  Kaleidoscope.setup();
}

void loop() {
  Kaleidoscope.loop();
}
