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

/*
 * This file contains pre-processor macros, which are not namespaced, the
 * vast majority of code these macros generate will live under the
 * kaleidoscope_internal namespace. That is why this header is here.
 *
 * The only exception to this is the _KALEIDOSCOPE_INIT_PLUGINS macro, which
 * also places a few symbols under the kaleidoscope namespace. But that code is
 * tightly coupled with the other parts, which are in kaleidoscope_internal.
 *
 * The reason we use an entirely separate namespace is that
 * Some plugins' classes are in the kaleidoscope namespace, and the instances
 * of those classes have the same names as the classes, but in the global
 * namespace. In these cases, the macros herein would resolve to the classes,
 * when we want them to resolve to instances. To avoid this, we put the
 * generated code in an entirely different namespace.
 */

// clang-format off

#pragma once

#include "kaleidoscope/event_handlers.h"                                  // for _FOR_EACH_EVENT...
#include "kaleidoscope/macro_helpers.h"                                   // for __NL__, UNWRAP
#include "kaleidoscope/plugin.h"  // IWYU pragma: keep
#include "kaleidoscope_internal/eventhandler_signature_check.h"           // for _PREPARE_EVENT_...
#include "kaleidoscope_internal/sketch_exploration/plugin_exploration.h"  // for _INIT_PLUGIN_EX...
#include "kaleidoscope_internal/sketch_exploration/sketch_exploration.h"  // IWYU pragma: keep

// Some words about the design of hook routing:
//
// The EventDispatcher class implements a compile-time loop over all plugins, which
// calls an event handler on each plugin.
//
// Each hook called by the firmware core compiles down to only a single
// static function call (in class Hooks), rather than one for each plugin.
//
// This approach is better than using virtual event handlers in classes
// derived from kaleidoscope::Plugin because it results in significantly fewer
// virtual function calls and virtual function tables (vtables).
//
// The call to event handlers through kaleidoscope::Hooks and
// kaleidoscope_internal::EventDispatcher is templated to allow for compile time
// static polymorphisms (plugins' EventHandlers aren't virtual).
//
// Technically, plugins don't need to be derived from kaleidoscope::Plugin, but
// only need to implement a set of event handlers whose call signature match the
// API definition. This approach is similar to duck typing in scripting languages:
// The compiler only cares that a method's signature matches, not that the plugin
// is derived from kaleidoscope::Plugin.
//
// Static hook functions inside the Hooks class each use the EventDispatcher
// helper class to cast an associated EventHandler on each plugin instance.


// This defines an auxiliary class 'EventHandler_Foo' for each hook 'Foo'.
// Kaleidoscope::Hooks calls the EventDispatcher class, which in turn invokes
// the event handler method 'Foo' of each registered plugin with a given
// set of arguments.

// The following macros help to generate type or instance names
// by concatenating macro parameter tokens.
//
#define _NAME3(A, B, C)       A##B##C
#define _NAME4(A, B, C, D)    A##B##C##D
#define _NAME5(A, B, C, D, E) A##B##C##D##E

#define _REGISTER_EVENT_HANDLER(                                                 \
    HOOK_NAME, HOOK_VERSION, DEPRECATION_TAG,                                    \
    SHOULD_EXIT_IF_RESULT_NOT_OK,                                                \
    TMPL_PARAM_TYPE_LIST, TMPL_PARAM_LIST, TMPL_DUMMY_ARGS_LIST,                 \
    SIGNATURE, ARGS_LIST)                                                 __NL__ \
                                                                          __NL__ \
  namespace kaleidoscope_internal {                                       __NL__ \
                                                                          __NL__ \
   template<bool hook_is_implemented__,                                   __NL__ \
            typename Plugin__                                             __NL__ \
            UNWRAP TMPL_PARAM_TYPE_LIST /* may evaluate empty */          __NL__ \
            , typename... Args__>                                         __NL__ \
   struct _NAME5(EventHandler_, HOOK_NAME, _v, HOOK_VERSION, _caller) {   __NL__ \
      static DEPRECATION_TAG kaleidoscope::EventHandlerResult             __NL__ \
        call(Plugin__ &plugin, Args__&&... hook_args) {                   __NL__ \
         return plugin.TEMPLATE_KEYWORD(UNWRAP TMPL_PARAM_LIST) HOOK_NAME __NL__ \
            ADD_TEMPLATE_BRACES(UNWRAP TMPL_PARAM_LIST)                   __NL__ \
            (hook_args...);                                               __NL__ \
      }                                                                   __NL__ \
   };                                                                     __NL__ \
                                                                          __NL__ \
   /* This specialization is used for those hooks that a plugin does */   __NL__ \
   /* not implement. */                                                   __NL__ \
                                                                          __NL__ \
   template<typename Plugin__                                             __NL__ \
            UNWRAP TMPL_PARAM_TYPE_LIST /* may evaluate empty */          __NL__ \
            , typename... Args__>                                         __NL__ \
   struct _NAME5(EventHandler_, HOOK_NAME, _v, HOOK_VERSION, _caller)     __NL__ \
             <false, Plugin__ UNWRAP TMPL_PARAM_LIST, Args__...> {        __NL__ \
      static kaleidoscope::EventHandlerResult                             __NL__ \
        call(Plugin__ &/*plugin*/, Args__&&... /*hook_args*/) {           __NL__ \
         return kaleidoscope::EventHandlerResult::OK;                     __NL__ \
      }                                                                   __NL__ \
   };                                                                     __NL__ \
                                                                          __NL__ \
   MAKE_TEMPLATE_SIGNATURE(UNWRAP TMPL_PARAM_TYPE_LIST)                   __NL__ \
   struct _NAME4(EventHandler_, HOOK_NAME, _v, HOOK_VERSION) {            __NL__ \
                                                                          __NL__ \
      static bool shouldExitIfResultNotOk() {                             __NL__ \
        return SHOULD_EXIT_IF_RESULT_NOT_OK;                              __NL__ \
      }                                                                   __NL__ \
                                                                          __NL__ \
      template<typename Plugin__,                                         __NL__ \
               typename... Args__>                                        __NL__ \
      static kaleidoscope::EventHandlerResult                             __NL__ \
        call(Plugin__ &plugin, Args__&&... hook_args) {                   __NL__ \
                                                                          __NL__ \
         _VALIDATE_EVENT_HANDLER_SIGNATURE(HOOK_NAME, Plugin__)           __NL__ \
                                                                          __NL__ \
         static constexpr bool derived_implements_hook                    __NL__ \
            = HookVersionImplemented_##HOOK_NAME<                         __NL__ \
                 Plugin__, HOOK_VERSION>::value;                          __NL__ \
                                                                          __NL__ \
         /* The caller type adds another level of indirection that */     __NL__ \
         /* is required to enable some hooks not to be implemented */     __NL__ \
         /* by plugins.                                            */     __NL__ \
                                                                          __NL__ \
         typedef _NAME5(EventHandler_, HOOK_NAME, _v, HOOK_VERSION, _caller) __NL__ \
            <                                                             __NL__ \
               derived_implements_hook,                                   __NL__ \
               Plugin__                                                   __NL__ \
               UNWRAP TMPL_PARAM_LIST                                     __NL__ \
               , Args__...                                                __NL__ \
            > Caller;                                                     __NL__ \
                                                                          __NL__ \
         return Caller::call(plugin, hook_args...);                       __NL__ \
      }                                                                   __NL__ \
    };                                                                    __NL__ \
                                                                          __NL__ \
   } 	                                                                  __NL__ \
                                                                          __NL__ \
   namespace kaleidoscope {                                               __NL__ \
                                                                          __NL__ \
     MAKE_TEMPLATE_SIGNATURE(UNWRAP TMPL_PARAM_TYPE_LIST)                 __NL__ \
     EventHandlerResult Hooks::HOOK_NAME SIGNATURE {                      __NL__ \
        return kaleidoscope_internal::EventDispatcher::template           __NL__ \
        apply<kaleidoscope_internal                                       __NL__ \
           ::_NAME4(EventHandler_, HOOK_NAME, _v, HOOK_VERSION)           __NL__ \
                ADD_TEMPLATE_BRACES(UNWRAP TMPL_PARAM_LIST)               __NL__ \
           >ARGS_LIST;                                                    __NL__ \
      }                                                                   __NL__ \
                                                                          __NL__ \
   }

#define _INLINE_EVENT_HANDLER_FOR_PLUGIN(PLUGIN)                            \
                                                                     __NL__ \
   result = EventHandler__::call(PLUGIN, hook_args...);              __NL__ \
                                                                     __NL__ \
   if (EventHandler__::shouldExitIfResultNotOk() &&                  __NL__ \
       result != kaleidoscope::EventHandlerResult::OK) {             __NL__ \
      return result;                                                 __NL__ \
   }                                                                 __NL__

// _KALEIDOSCOPE_INIT_PLUGINS builds the loops that execute the plugins'
// implementations of the various event handlers.
//
// Its arguments are a list of references to plugin instances that have been
// instantiated in the global scope.

// EventDispatcher::apply() implements a compile time for-each loop over all
// plugins. The compiler automatically optimizes away calls to any plugin that
// doesn't implement an EventHandler for a given hook.

#define _KALEIDOSCOPE_INIT_PLUGINS(...)                                       __NL__ \
  namespace kaleidoscope_internal {                                           __NL__ \
  struct EventDispatcher {                                                    __NL__ \
                                                                              __NL__ \
    /* Iterate through plugins, calling each one's event handler with      */ __NL__ \
    /* the arguments passed to the hook                                    */ __NL__ \
    template<typename EventHandler__, typename... Args__ >                    __NL__ \
    static kaleidoscope::EventHandlerResult apply(Args__&&... hook_args) {    __NL__ \
                                                                              __NL__ \
      kaleidoscope::EventHandlerResult result;                                __NL__ \
                                                                              __NL__ \
      /* KALEIDOSCOPE_STANDARD_PLUGINS_START is defined in Kaleidoscope.h */  __NL__ \
      /**/                                                                    __NL__ \
      MAP(_INLINE_EVENT_HANDLER_FOR_PLUGIN,                                   __NL__ \
          KALEIDOSCOPE_STANDARD_PLUGINS_START)                                __NL__ \
                                                                              __NL__ \
      MAP(_INLINE_EVENT_HANDLER_FOR_PLUGIN, __VA_ARGS__)                      __NL__ \
                                                                              __NL__ \
      /* KALEIDOSCOPE_STANDARD_PLUGINS_END is defined in Kaleidoscope.h */    __NL__ \
      /**/                                                                    __NL__ \
      MAP(_INLINE_EVENT_HANDLER_FOR_PLUGIN,                                   __NL__ \
          KALEIDOSCOPE_STANDARD_PLUGINS_END)                                  __NL__ \
                                                                              __NL__ \
                                                                              __NL__ \
      return result;                                                          __NL__ \
    }                                                                         __NL__ \
  };                                                                          __NL__ \
                                                                              __NL__ \
  }                                                                           __NL__ \
  /* We register event handlers here - which is not technically related    */ __NL__ \
  /* to initialization, nor is it in the same namespace - to support the   */ __NL__ \
  /* transition from the old APIs. When the user sketch does not use       */ __NL__ \
  /* KALEIDOSCOPE_INIT_PLUGINS(), the event handlers will not exist        */ __NL__ \
  /* either, thus wrapping them would produce a compile error. For this    */ __NL__ \
  /* reason, we do the wrapping here, tied to _KALEIDOSCOPE_INIT_PLUGINS.  */ __NL__ \
  /*                                                                       */ __NL__ \
  /* TODO(anyone): Move this somewhere else, outside of _internal, once    */ __NL__ \
  /*               the V1 API is removed.                                  */ __NL__ \
                                                                              __NL__ \
  _PREPARE_EVENT_HANDLER_SIGNATURE_CHECK                                      __NL__ \
                                                                              __NL__ \
  _FOR_EACH_EVENT_HANDLER(_REGISTER_EVENT_HANDLER)                            __NL__ \
                                                                              __NL__ \
  /* This generates a PROGMEM array-kind-of data structure that contains   */ __NL__ \
  /* LEDModeFactory entries                                                */ __NL__ \
  _INIT_LED_MODE_MANAGER(__VA_ARGS__)                                         __NL__ \
                                                                              __NL__ \
  _INIT_PLUGIN_EXPLORATION(__VA_ARGS__)
