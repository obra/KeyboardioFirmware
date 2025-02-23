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

#include <Arduino.h>

//end of add your includes here
#ifdef __cplusplus
extern "C" {
#endif
void loop();
void setup();
#ifdef __cplusplus
}  // extern "C"
#endif

//add your function definitions for the project KeyboardIO here

#define TS(X)  //Serial.print(micros() );Serial.print("\t");Serial.println(X);

#include <stdio.h>
#include <math.h>
#include <stdint.h>

#include "kaleidoscope/version.h"
#include "kaleidoscope/device/device.h"
#include "kaleidoscope/device/key_indexes.h"
#include "kaleidoscope_internal/device.h"
#include "kaleidoscope_internal/deprecations.h"


// Note: The CONVERT_TO_KEY macro can be redefined to use different
//       host_keymap-keymaps on different layers (see key_defs.h for its
//       default definition.

#define CONVERT_AND_CHECK_KEY(KEY)                                         \
  (                                                                        \
    (                                                                      \
      struct {                                                             \
        static_assert(CONVERT_TO_KEY(KEY) != kaleidoscope::bad_keymap_key, \
                      "Bad key definition: \'" #KEY "\'");                 \
      }){},                                                                \
    CONVERT_TO_KEY(KEY))

#ifdef PER_KEY_DATA_STACKED
#define KEYMAP_STACKED(...)                   \
  {                                           \
    MAP_LIST(                                 \
      CONVERT_AND_CHECK_KEY,                  \
      PER_KEY_DATA_STACKED(XXX, __VA_ARGS__)) \
  }

#endif

#ifdef PER_KEY_DATA
#define KEYMAP(...)                   \
  {                                   \
    MAP_LIST(                         \
      CONVERT_AND_CHECK_KEY,          \
      PER_KEY_DATA(XXX, __VA_ARGS__)) \
  }
#endif

#include "kaleidoscope/KeyAddr.h"
#include "kaleidoscope/KeyEvent.h"
#include "kaleidoscope/key_defs.h"
#include "kaleidoscope/keyswitch_state.h"
#include "kaleidoscope/layers.h"
#include "kaleidoscope_internal/sketch_exploration/sketch_exploration.h"
#include "kaleidoscope/macro_map.h"
#include "kaleidoscope/hooks.h"
#include "kaleidoscope_internal/LEDModeManager.h"
#include "kaleidoscope/macro_helpers.h"
#include "kaleidoscope/plugin.h"
#include "kaleidoscope/Runtime.h"

#define HOOK_MAX 64

#ifndef VERSION
#define VERSION "locally-built"
#endif

/** Kaleidoscope API (major) version.
 *
 * The API is guaranteed to be backwards compatible for the entire duration of a
 * major version. However, breaking changes may come, and result in a major
 * version bump. To help migration, the `KALEIDOSCOPE_API_VERSION` macro can be
 * used to check the major version provided by the Kaleidoscope we are compiling
 * against. This can be used to error out with a helpful message, or change how
 * the API is used - it is entirely up to the plugin or sketch author. The point
 * of this macro is to let them easily check the version.
 */
#define KALEIDOSCOPE_API_VERSION 2

/** Required Kaleidoscope major version.
 *
 * For the sake of convenience, defining `KALEIDOSCOPE_REQUIRED_API_VERSION`
 * before including `Kaleidoscope.h` itself will result in comparing its value
 * to `KALEIDOSCOPE_API_VERSION`. If they differ, a helpful error message is
 * printed.
 *
 * Done so that a new API version would result in a helpful error message,
 * instead of cryptic compile errors.
 */
#if defined(KALEIDOSCOPE_REQUIRED_API_VERSION) && (KALEIDOSCOPE_REQUIRED_API_VERSION != KALEIDOSCOPE_API_VERSION)
#define xstr(a) str(a)
#define str(a)  #a
static_assert(KALEIDOSCOPE_REQUIRED_API_VERSION == KALEIDOSCOPE_API_VERSION,
              "Kaleidoscope API version mismatch! We have version " xstr(KALEIDOSCOPE_API_VERSION) " available, but version " xstr(KALEIDOSCOPE_REQUIRED_API_VERSION) " is required.");
#endif

// Use this function macro to register plugins with Kaleidoscope's
// hooking system. The macro accepts a list of plugin instances that
// must have been instantiated at global scope.
//
#define KALEIDOSCOPE_INIT_PLUGINS(...) _KALEIDOSCOPE_INIT_PLUGINS(__VA_ARGS__)

// For compatibility/usability reasons we enable the global variable
// Kaleidoscope in global namespace.
//
extern kaleidoscope::Runtime_ &Kaleidoscope;

// Have a list of plugins that are always added.
// Those plugins might be configured by macros defined at the top of the sketch.
// Plugins can thus be turned into complete no-ops.

// For the sketch compilation unit, we pretend that there is a NoOpPlugin
// symbol. This is intented to be passed to the KALEIDOSCOPE_INIT_PLUGINS(...)
// function macro. As no existing non-inline functions of this macro are called
// for this plugin instance, it does not hurt that the instance is not
// actually instanciated in no other compilation unit.
//
#ifdef KALEIDOSCOPE_SKETCH
namespace kaleidoscope {
extern Plugin NoOpPlugin; // This is on purpose not instanciated in any
// compilation unit.
} // namespace kaleidoscope
#endif

// Plugins leading the list passed to KALEIDOSCOPE_INIT_PLUGINS
//
// Note: As long as we do not actually pass a plugin to
//       KALEIDOSCOPE_STANDARD_PLUGINS_START, we need to pass at least
//       a no-op plugin to satisfy the interface. The NoOpPlugin
//       can be replaced as soon as other plugins are added.
//
#define KALEIDOSCOPE_STANDARD_PLUGINS_START kaleidoscope::NoOpPlugin

// Plugins trailing the list passed to KALEIDOSCOPE_INIT_PLUGINS
//
// Note: As long as we do not actually pass a plugin to
//       KALEIDOSCOPE_STANDARD_PLUGINS_END, we need to pass at least
//       a no-op plugin to satisfy the interface. The NoOpPlugin
//       can be replaced as soon as other plugins are added.
//
#define KALEIDOSCOPE_STANDARD_PLUGINS_END kaleidoscope::NoOpPlugin
