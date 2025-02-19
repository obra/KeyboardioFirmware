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

#include "kaleidoscope/KeyEvent.h"                 // IWYU pragma: keep
#include "kaleidoscope/event_handler_result.h"     // for EventHandlerResult
#include "kaleidoscope/event_handlers.h"           // for _FOR_EACH_EVENT_HANDLER
#include "kaleidoscope/macro_helpers.h"            // for __NL__, MAKE_TEMPLATE_SIGNATURE, UNWRAP
#include "kaleidoscope_internal/event_dispatch.h"  // IWYU pragma: keep

namespace kaleidoscope {
// Forward declarations to enable friend declarations.
class Layer_;    // IWYU pragma: keep
class Runtime_;  // IWYU pragma: keep

namespace plugin {
// Forward declarations to enable friend declarations.
class FocusSerial;
class LEDControl;
}  // namespace plugin

namespace driver {
namespace ble {
class BLEBluefruit;  // Forward declaration
}  // namespace ble
}  // namespace driver

namespace sketch_exploration {
void pluginsExploreSketch();
}  // namespace sketch_exploration

// The reason why the hook routing entry point functions live within
// class Hooks and not directly within a namespace is, that we want
// to restrict who is allowed to trigger hooks, mainly to prevent
// user code from calling hook methods.
//
// A note to maintainers: When you add new hooks that are supposed to
// be called from other places than the friend classes and functions listed
// below, just add a suitable friend declaration.

class Hooks {

  // The following friend declarations restrict access to
  // the hook routing system.

  // Runtime_ calls Hooks::onSetup, Hooks::beforeReportingState
  // and Hooks::afterEachCycle.
  friend class Layer_;
  friend class Runtime_;
  friend class plugin::FocusSerial;
  friend class plugin::LEDControl;
  friend class driver::ble::BLEBluefruit;  // Allow BLEBluefruit to trigger hooks
  friend void sketch_exploration::pluginsExploreSketch();

 private:
  // The following private functions are just to be called by classes
  // and functions that are declared as friends above.

  // clang-format off
#define DEFINE_WEAK_HOOK_FUNCTION(                                             \
    HOOK_NAME, HOOK_VERSION, DEPRECATION_TAG,                                  \
    SHOULD_EXIT_IF_RESULT_NOT_OK,                                              \
    TMPL_PARAM_TYPE_LIST, TMPL_PARAM_LIST, TMPL_DUMMY_ARGS_LIST,               \
    SIGNATURE, ARGS_LIST)                                               __NL__ \
                                                                        __NL__ \
   MAKE_TEMPLATE_SIGNATURE(UNWRAP TMPL_PARAM_TYPE_LIST)                 __NL__ \
   static EventHandlerResult HOOK_NAME SIGNATURE;

  _FOR_EACH_EVENT_HANDLER(DEFINE_WEAK_HOOK_FUNCTION)

#undef DEFINE_WEAK_HOOK_FUNCTION
  // clang-format on
};

}  // namespace kaleidoscope
