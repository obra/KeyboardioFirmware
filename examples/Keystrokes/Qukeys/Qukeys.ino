// -*- mode: c++ -*-

#include <Kaleidoscope.h>
#include <Kaleidoscope-Qukeys.h>
#include <Kaleidoscope-Macros.h>

enum { MACRO_TOGGLE_QUKEYS };

// To define DualUse Qukeys in the keymap itself, we can use `SFT_T(key)`,
// `CTL_T(key)`, `ALT_T(key)`, `GUI_T(key)`, and `LT(layer, key)`, as defined
// for the home row on the right side of the keymap (and one of the palm keys)
// below.

// clang-format off
KEYMAPS(
  [0] = KEYMAP_STACKED
  (
      Key_NoKey,    Key_1, Key_2, Key_3, Key_4, Key_5, Key_NoKey,
      Key_Backtick, Key_Q, Key_W, Key_E, Key_R, Key_T, Key_Tab,
      Key_PageUp,   Key_A, Key_S, Key_D, Key_F, Key_G,
      Key_PageDown, Key_Z, Key_X, Key_C, Key_V, Key_B, Key_Escape,

      Key_LeftControl, Key_Backspace, Key_LeftGui, Key_LeftShift,
      Key_Q,

      M(MACRO_TOGGLE_QUKEYS), Key_6, Key_7,    Key_8,     Key_9,      Key_0,            Key_skip,
      Key_Enter,              Key_Y, Key_U,    Key_I,     Key_O,      Key_P,            Key_Equals,
                              Key_H, SFT_T(J), CTL_T(K),  ALT_T(L),   GUI_T(Semicolon), Key_Quote,
      Key_skip,               Key_N, Key_M,    Key_Comma, Key_Period, Key_Slash,        Key_Minus,

      Key_RightShift, Key_RightAlt, Key_Spacebar, Key_RightControl,
      LT(1,E)
   ),
  [1] = KEYMAP_STACKED
  (
      ___,   Key_B, Key_C, Key_D, Key_E, Key_F, Key_G,
      Key_A, Key_B, Key_C, Key_D, Key_E, Key_F, Key_G,
      Key_A, Key_B, Key_C, Key_D, Key_E, Key_F,
      Key_A, Key_B, Key_C, Key_D, Key_E, Key_F, Key_G,

      Key_1, Key_2, Key_3, Key_4,
      ___,


      ___,   Key_B, Key_C, Key_D, Key_E, Key_F, Key_G,
      Key_A, Key_B, Key_C, Key_D, Key_E, Key_F, Key_G,
      Key_A, Key_B, Key_C, Key_D, Key_E, Key_F,
      Key_A, Key_B, Key_C, Key_D, Key_E, Key_F, Key_G,

      Key_1, Key_2, Key_3, Key_4,
      ___
   ),
)
// clang-format on

// Defining a macro (on the "any" key: see above) to toggle Qukeys on and off
const macro_t *macroAction(uint8_t macro_id, KeyEvent &event) {
  switch (macro_id) {
  case MACRO_TOGGLE_QUKEYS:
    if (keyToggledOn(event.state))
      Qukeys.toggle();
    break;
  }
  return MACRO_NONE;
}

// Use Qukeys
KALEIDOSCOPE_INIT_PLUGINS(Qukeys, QukeysConfig, Macros);

void setup() {
  // The following Qukey definitions are for the left side of the home row (and
  // the left palm key) of the Keyboardio Model01 keyboard.  For other
  // keyboards, the `KeyAddr(row, col)` coordinates will need adjustment.
  QUKEYS(
    kaleidoscope::plugin::Qukey(0, KeyAddr(2, 1), Key_LeftGui),      // A/cmd
    kaleidoscope::plugin::Qukey(0, KeyAddr(2, 2), Key_LeftAlt),      // S/alt
    kaleidoscope::plugin::Qukey(0, KeyAddr(2, 3), Key_LeftControl),  // D/ctrl
    kaleidoscope::plugin::Qukey(0, KeyAddr(2, 4), Key_LeftShift),    // F/shift
    kaleidoscope::plugin::Qukey(0, KeyAddr(3, 6), ShiftToLayer(1))   // Q/layer-shift (on `fn`)
  )
  Qukeys.setHoldTimeout(1000);
  Qukeys.setOverlapThreshold(50);
  Qukeys.setMinimumHoldTime(100);
  Qukeys.setMinimumPriorInterval(80);
  Qukeys.setMaxIntervalForTapRepeat(150);

  Kaleidoscope.setup();
}

void loop() {
  Kaleidoscope.loop();
}
