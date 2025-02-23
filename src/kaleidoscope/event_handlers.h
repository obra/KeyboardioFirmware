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

// clang-format off

#include "kaleidoscope/host_connection_status.h"
#pragma once

// This file defines the names and argument signatures for all event handlers
// in the Kaleidoscope core.
//
// When adding a new hook or event handler to Kaleidoscope, it needs only
// to be added here and to the place in the codebase where it should be called.
//
// _FOR_EACH_EVENT_HANDLER is what's called an "X Macro". Its first argument,
// OPERATION, is the name of another macro. When called, it will execute
// OPERATION once for each event handler defined herein with the following
// parameters:
//
// HOOK_NAME, SHOULD_EXIT_IF_RESULT_NOT_OK, SIGNATURE, ARGS_LIST, ...
//
// Any additional parameters that are added to an invokation
// of _FOR_EACH_EVENT_HANDLER are passed through to OP.

#define _ABORTABLE true
#define _NOT_ABORTABLE false

// Define the callback for the LED mode hook handler
typedef void (*LedModeCallback)(const char*);

#define _CURRENT_IMPLEMENTATION

//******************************************************************************
//******************************************************************************
// Important
//******************************************************************************
//******************************************************************************
// When adding new handlers or handler versions, don't forget
// to add them both to the _FOR_EACH_EVENT_HANDLER and
// the _PROCESS_EVENT_HANDLER_VERSIONS function macros.
//******************************************************************************
//******************************************************************************

// The following properties are passed to each invocation of the OPERATION
// macro of the below definition of macro function _FOR_EACH_EVENT_HANDLER:
//
// handler name:
//    This is the name that must be used to implement a handler in a plugin.
//
// handler version:
//    Sometimes changes to the firmware's implementation require handler
//    call signatures (the list of parameters) to be adapted. To allow
//    for a deprecation periode to avoid breaking user code, it is
//    desirable to have multiple overloads of a handler with the same
//    name at the same time. Every overload must be assigned a version
//    number and every individual version must appear in the below definition
//    of the macros _FOR_EACH_EVENT_HANDLER and _PROCESS_EVENT_HANDLER_VERSIONS.
//
// deprecation flag:
//    If a specific version is meant to be deprecated, pass the DEPRECATED(...)
//    macro with a meaningful deprecation message. Pass _CURRENT_IMPLEMENTATION
//    otherwise.
//
// abortable flag:
//    Plugins' event handlers are called in the same order as the plugins
//    are passed to the KALEIDOSCOPE_INIT_PLUGINS(...) macro within the
//    sketch. For some handlers it is desirable to not call subsequent
//    plugins' handlers once a plugin's handler returned a value other
//    than kaleidoscope::EventHandlerResult::OK. To enable this,
//    pass the abortable flag value _ABORTABLE, _NOT_ABORTABLE otherwise.
//
// template parameter type list:
//    The hook's template type list in parenthesis, with a trailing comma.
//      e.g. (, typename _T1, typename _T2)
//    Pass empty parenthesis if the hook is non templatized.
//
// template parameters:
//    The hook's template parameters in parenthesis, with a trailing comma.
//    The template parameter names must match the template type list.
//      e.g. (, _T1, _T2)
//    Pass empty parenthesis if the hook is non templatized.
//
// dummy template arguments:
//    Supply a list of already defined dummy types that could realistically
//    be used to instantiate the template hook. Those types are only used
//    during hook method signature checks.
//    Please add parenthesis and a trailing comma.
//      e.g. (, int, int)
//    Pass empty parenthesis if the hook is non templatized.
//
// call signature:
//    The type of arguments passed to the handlers as a comma separated
//    list in brackets. Every parameter must be named.
//
// call parameters:
//    The list of parameters as they would be passed to a call to the handler.
//    Parameter names must match the names assigned to the call arguments.

namespace kaleidoscope {

// This dummy class can be used as dummy template argument to
// be passed in the definition of template hooks.
//
class SignatureCheckDummy {};
} // namespace kaleidoscope

#define _FOR_EACH_EVENT_HANDLER(OPERATION, ...)                           __NL__ \
                                                                          __NL__ \
   /* Called by Focus, when handling the `plugins` command. */            __NL__ \
   /* Should send the plugin name if that makes sense, */                 __NL__ \
   /* but can be no-op. */                                                __NL__ \
   OPERATION(onNameQuery,                                                 __NL__ \
             1,                                                           __NL__ \
             _CURRENT_IMPLEMENTATION,                                     __NL__ \
             _NOT_ABORTABLE,                                              __NL__ \
             (), (), (),                                                  __NL__ \
             (), (), ##__VA_ARGS__)                                       __NL__ \
                                                                          __NL__ \
   /* Called by Focus, when handling the `led.modes` command. */          __NL__ \
   /* Is implemented in LEDModeInterface.h, but depends on */             __NL__ \
   /* the LED plugin to set `led_mode_name_`. */                          __NL__ \
   OPERATION(onLedEffectQuery,                                            __NL__ \
             1,                                                           __NL__ \
             _CURRENT_IMPLEMENTATION,                                     __NL__ \
             _NOT_ABORTABLE,                                              __NL__ \
             (), (), (), /* non template */                               __NL__ \
             (LedModeCallback callback),                                  __NL__ \
             (callback), ##__VA_ARGS__)                                   __NL__ \
                                                                          __NL__ \
   OPERATION(onSetup,                                                     __NL__ \
             1,                                                           __NL__ \
             _CURRENT_IMPLEMENTATION,                                     __NL__ \
               _NOT_ABORTABLE,                                            __NL__ \
               (),(),(), /* non template */                               __NL__ \
               (),(), ##__VA_ARGS__)                                      __NL__ \
                                                                          __NL__ \
   /* Called at the very start of each cycle, before gathering         */ __NL__ \
   /* events, before doing anything else.                              */ __NL__ \
   OPERATION(beforeEachCycle,                                             __NL__ \
             1,                                                           __NL__ \
             _CURRENT_IMPLEMENTATION,                                     __NL__ \
               _NOT_ABORTABLE,                                            __NL__ \
               (),(),(), /* non template */                               __NL__ \
               (), (), ##__VA_ARGS__)                                     __NL__ \
                                                                          __NL__ \
   /* Function called for every physical keyswitch event (toggle on or */ __NL__ \
   /* off). The `event` parameter is passed by reference so its key    */ __NL__ \
   /* value can be modified. If it returns EventHandlerResult::OK, the */ __NL__ \
   /* next handler will be passed the event; otherwise Kaleidoscope    */ __NL__ \
   /* will stop processing the event. Plugins that implement this      */ __NL__ \
   /* handler must not process the same event id twice in order to     */ __NL__ \
   /* prevent handler loops. Events may be aborted or queued for later */ __NL__ \
   /* release (by calling `Runtime.handleKeyswitchEvent()`), but any   */ __NL__ \
   /* plugin that does so must release events in ascending order,      */ __NL__ \
   /* counting by ones.                                                */ __NL__ \
   OPERATION(onKeyswitchEvent,                                            __NL__ \
             1,                                                           __NL__ \
             _CURRENT_IMPLEMENTATION,                                     __NL__ \
             _ABORTABLE,                                                  __NL__ \
             (),(),(), /* non template */                                 __NL__ \
             (KeyEvent &event),                                           __NL__ \
             (event), ##__VA_ARGS__)                                      __NL__ \
                                                                          __NL__ \
   /* Function called for every logical key event, including ones that */ __NL__ \
   /* originate from a physical keyswitch and ones that are injected   */ __NL__ \
   /* by plugins. The `event` parameter is passed by reference so its  */ __NL__ \
   /* key value can be modified. If it returns EventHandlerResult::OK, */ __NL__ \
   /* the next handler will be passed the event; otherwise             */ __NL__ \
   /* Kaleidoscope will stop processing the event.                     */ __NL__ \
   OPERATION(onKeyEvent,                                                  __NL__ \
             1,                                                           __NL__ \
             _CURRENT_IMPLEMENTATION,                                     __NL__ \
             _ABORTABLE,                                                  __NL__ \
             (),(),(), /* non template */                                 __NL__ \
             (KeyEvent &event),                                           __NL__ \
             (event), ##__VA_ARGS__)                                      __NL__ \
                                                                          __NL__ \
   /* Called when a new set of HID reports (Keyboard, Consumer         */ __NL__ \
   /* Control, and System Control) is being constructed in response to */ __NL__ \
   /* a key event. This is mainly useful for plugins that need to add  */ __NL__ \
   /* values to HID reports based on special `Key` values other than   */ __NL__ \
   /* the builtin ones.                                                */ __NL__ \
   OPERATION(onAddToReport,                                               __NL__ \
             1,                                                           __NL__ \
             _CURRENT_IMPLEMENTATION,                                     __NL__ \
             _ABORTABLE,                                                  __NL__ \
             (),(),(), /* non template */                                 __NL__ \
             (Key key),                                                   __NL__ \
             (key), ##__VA_ARGS__)                                        __NL__ \
                                                                          __NL__ \
                                                                          __NL__ \
   /* Called by an external plugin (such as Kaleidoscope-FocusSerial)  */ __NL__ \
   /* via Kaleidoscope::onFocusEvent. This is where Focus events can   */ __NL__ \
   /* be handled. The function can return EventHandlerResult::OK, and  */ __NL__ \
   /* allow other plugins to handle the same command (with the caveat  */ __NL__ \
   /* that arguments can only be parsed once), or                      */ __NL__ \
   /* EventHandlerResult::EVENT_CONSUMED, in which case no other       */ __NL__ \
   /* plugin will have a chance to react to the event.                 */ __NL__ \
   OPERATION(onFocusEvent,                                                __NL__ \
             1,                                                           __NL__ \
             _CURRENT_IMPLEMENTATION,                                     __NL__ \
                _ABORTABLE,                                               __NL__ \
                (),(),(), /* non template */                              __NL__ \
                (const char *input),                                      __NL__ \
                (input), ##__VA_ARGS__)                                   __NL__ \
                                                                          __NL__ \
   /* Called when the layer state changes. Which layes changed are    */  __NL__ \
   /* not passed as arguments. If one needs that info, they should    */  __NL__ \
   /* track Layer.getState() themselves.                              */  __NL__ \
   OPERATION(onLayerChange,                                               __NL__ \
             1,                                                           __NL__ \
             _CURRENT_IMPLEMENTATION,                                     __NL__ \
                _NOT_ABORTABLE,                                           __NL__ \
                (),(),(), /* non template */                              __NL__ \
                (), (), ##__VA_ARGS__)                                    __NL__ \
   /* Called when the LED mode changes. If one needs to know what      */ __NL__ \
   /* from and what to the mode changed, they should track that        */ __NL__ \
   /* themselves.                                                      */ __NL__ \
   OPERATION(onLEDModeChange,                                             __NL__ \
             1,                                                           __NL__ \
             _CURRENT_IMPLEMENTATION,                                     __NL__ \
                _NOT_ABORTABLE,                                           __NL__ \
                (),(),(), /* non template */                              __NL__ \
                (), (), ##__VA_ARGS__)                                    __NL__ \
   /* Called immediately before the LEDs get updated. This is for */      __NL__ \
   /* plugins that override the current LED mode. */                      __NL__ \
   OPERATION(beforeSyncingLeds,                                           __NL__ \
             1,                                                           __NL__ \
             _CURRENT_IMPLEMENTATION,                                     __NL__ \
             _NOT_ABORTABLE,                                              __NL__ \
             (),(),(), /* non template */                                 __NL__ \
             (), (), ##__VA_ARGS__)                                       __NL__ \
                                                                          __NL__ \
   /* Called before reporting our state to the host. This is the       */ __NL__ \
   /* last point in a cycle where a plugin can alter what gets         */ __NL__ \
   /* reported to the host.                                            */ __NL__ \
   OPERATION(beforeReportingState,                                        __NL__ \
             1,                                                           __NL__ \
             _CURRENT_IMPLEMENTATION,                                     __NL__ \
             _ABORTABLE,                                                  __NL__ \
             (),(),(), /* non template */                                 __NL__ \
             (const KeyEvent &event),                                     __NL__ \
             (event), ##__VA_ARGS__)                                      __NL__ \
                                                                          __NL__ \
   /* Called after reporting our state to the host. This is the last   */ __NL__ \
   /* point at which a plugin can do something in response to an event */ __NL__ \
   /* before the next event is processed, if multiple events occur in  */ __NL__ \
   /* and are processed in a single cycle (usually due to delayed      */ __NL__ \
   /* events or generated events).                                     */ __NL__ \
   OPERATION(afterReportingState,                                         __NL__ \
             1,                                                           __NL__ \
             _CURRENT_IMPLEMENTATION,                                     __NL__ \
             _NOT_ABORTABLE,                                              __NL__ \
             (),(),(), /* non template */                                 __NL__ \
             (const KeyEvent &event),                                     __NL__ \
             (event), ##__VA_ARGS__)                                      __NL__ \
                                                                          __NL__ \
   /* Called at the very end of a cycle, after everything's            */ __NL__ \
   /* said and done.                                                   */ __NL__ \
   OPERATION(afterEachCycle,                                              __NL__ \
             1,                                                           __NL__ \
             _CURRENT_IMPLEMENTATION,                                     __NL__ \
               _NOT_ABORTABLE,                                            __NL__ \
               (),(),(), /* non template */                               __NL__ \
               (),(),##__VA_ARGS__)                                       __NL__ \
                                                                          __NL__ \
   /* Called before setup to enable plugins at compile time            */ __NL__ \
   /* to explore the sketch.                                           */ __NL__ \
   OPERATION(exploreSketch ,                                              __NL__ \
             1,                                                           __NL__ \
             _CURRENT_IMPLEMENTATION,                                     __NL__ \
               _NOT_ABORTABLE,                                            __NL__ \
               (,typename _Sketch),                                       __NL__ \
               (,_Sketch),                                                __NL__ \
               (,kaleidoscope::SignatureCheckDummy),                      __NL__ \
               (),                                                        __NL__ \
               (),##__VA_ARGS__)                                          __NL__ \
                                                                          __NL__ \
   /* Called when a host's connection status changes */                   __NL__ \
   OPERATION(onHostConnectionStatusChanged,                              __NL__ \
             1,                                                          __NL__ \
             _CURRENT_IMPLEMENTATION,                                    __NL__ \
             _NOT_ABORTABLE,                                            __NL__ \
             (),(),(), /* non template */                               __NL__ \
             (uint8_t device_id, kaleidoscope::HostConnectionStatus status),          __NL__ \
             (device_id, status), ##__VA_ARGS__)

// The following function macro lists event handler/hook method names and
// available versions. It is used to auto-generate code that is
// related to event handlers.
//
// The operators START and END are invoked with a list of all
// available hook versions while the operator OP is invoked for every
// version individually.
//
// If necessary, add handlers as provided in the following example.
//
// Example: Lets assume that a hook fancyHook has three versions,
//          then the operator invokation would be.
//
// START(fancyHook, 1, 2, 3)
//    OP(fancyHook, 1)
//    OP(fancyHook, 2)
//    OP(fancyHook, 3)
// END(fancyHook, 1, 2, 3)
//
#define _PROCESS_EVENT_HANDLER_VERSIONS(START, OP, END, ...)            __NL__ \
   START(onSetup, 1)                                                    __NL__ \
      OP(onSetup, 1)                                                    __NL__ \
   END(onSetup, 1)                                                      __NL__ \
                                                                        __NL__ \
   START(onLedEffectQuery, 1)                                           __NL__ \
      OP(onLedEffectQuery, 1)                                           __NL__ \
   END(onLedEffectQuery, 1)                                             __NL__ \
                                                                        __NL__ \
   START(onNameQuery, 1)                                                __NL__ \
      OP(onNameQuery, 1)                                                __NL__ \
   END(onNameQuery, 1)                                                  __NL__ \
                                                                        __NL__ \
   START(beforeEachCycle, 1)                                            __NL__ \
      OP(beforeEachCycle, 1)                                            __NL__ \
   END(beforeEachCycle, 1)                                              __NL__ \
                                                                        __NL__ \
   START(onKeyswitchEvent, 1)                                           __NL__ \
      OP(onKeyswitchEvent, 1)                                           __NL__ \
   END(onKeyswitchEvent, 1)                                             __NL__ \
                                                                        __NL__ \
   START(onKeyEvent, 1)                                                 __NL__ \
      OP(onKeyEvent, 1)                                                 __NL__ \
   END(onKeyEvent, 1)                                                   __NL__ \
                                                                        __NL__ \
   START(onAddToReport, 1)                                              __NL__ \
      OP(onAddToReport, 1)                                              __NL__ \
   END(onAddToReport, 1)                                                __NL__ \
                                                                        __NL__ \
   START(onFocusEvent, 1)                                               __NL__ \
      OP(onFocusEvent, 1)                                               __NL__ \
   END(onFocusEvent, 1)                                                 __NL__ \
                                                                        __NL__ \
   START(onLayerChange, 1)                                              __NL__ \
      OP(onLayerChange, 1)                                              __NL__ \
   END(onLayerChange, 1)                                                __NL__ \
                                                                        __NL__ \
   START(onLEDModeChange, 1)                                            __NL__ \
      OP(onLEDModeChange, 1)                                            __NL__ \
   END(onLEDModeChange, 1)                                              __NL__ \
                                                                        __NL__ \
   START(beforeSyncingLeds, 1)                                          __NL__ \
      OP(beforeSyncingLeds, 1)                                          __NL__ \
   END(beforeSyncingLeds, 1)                                            __NL__ \
                                                                        __NL__ \
   START(beforeReportingState, 1)                                       __NL__ \
      OP(beforeReportingState, 1)                                       __NL__ \
   END(beforeReportingState, 1)                                         __NL__ \
                                                                        __NL__ \
   START(afterReportingState, 1)                                        __NL__ \
      OP(afterReportingState, 1)                                        __NL__ \
   END(afterReportingState, 1)                                          __NL__ \
                                                                        __NL__ \
   START(afterEachCycle, 1)                                             __NL__ \
      OP(afterEachCycle, 1)                                             __NL__ \
   END(afterEachCycle, 1)                                               __NL__ \
                                                                        __NL__ \
   START(exploreSketch, 1)                                              __NL__ \
      OP(exploreSketch, 1)                                              __NL__ \
   END(exploreSketch, 1)                                                __NL__ \
                                                                        __NL__ \
   START(onHostConnectionStatusChanged, 1)                               __NL__ \
      OP(onHostConnectionStatusChanged, 1)                               __NL__ \
   END(onHostConnectionStatusChanged, 1)                                 
