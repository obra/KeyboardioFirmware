#pragma once

#include <Arduino.h>

//end of add your includes here
#ifdef __cplusplus
extern "C" {
#endif
void loop();
void setup();
#ifdef __cplusplus
} // extern "C"
#endif

//add your function definitions for the project KeyboardIO here

#define TS(X) //Serial.print(micros() );Serial.print("\t");Serial.println(X);

#include <stdio.h>
#include <math.h>
#include <stdint.h>

#include KALEIDOSCOPE_HARDWARE_H
#include "key_events.h"
#include "kaleidoscope/hid.h"
#include "layers.h"
#include "preprocessor_macro_map.h"

#define HOOK_MAX 64

extern HARDWARE_IMPLEMENTATION KeyboardHardware;

#ifndef VERSION
#define VERSION "locally-built"
#endif

const uint8_t KEYMAP_SIZE
__attribute__((deprecated("Kaleidoscope.setup() does not require KEYMAP_SIZE anymore."))) = 0;

struct EventKey {
  byte row_;
  byte col_;
  uint8_t keyState_;
};

class Kaleidoscope_;

// A note to maintainers: How to add additional hooks
//
// 1) Add an appropriate non-virtual (important!) method
//       to class KaleidoscopePlugin
// 2) Add a virtual (important!) method with the same name and call signature
//       to the abstract base class PluginHookAdapter__
// 3) Add a hook specification to template class PluginHookAdapter
//       via the HOOK_TASK macro
// 4) Add an implemenation of the hook to template class PluginHookAdapter
//       that references the name given in HOOK_TASK

// Some words about the design of the plugin interface
//
// The HookLoop class implements a compile-time loop over all plugins.
// This loop is used to call a non virtual hook method of each plugin.
//
// The advantage of this approach against a solution with virtual
// hook methods in classes derived from KaleidoscopePlugin is a significant
// reduction of virtual function calls and a reduction of the amount of
// virtual function tables (vtables).
//
// For every hook that is called by the firmware core, only one virtual
// method (of PluginHookAdapter) is invoked instead of
// one for each plugin and hook. As with this approach plugins derived
// from KaleidoscopePlugin are not (dynamically) polymorphic classes
// (not featuring virtual methods). Thus, classes do not need virtual tables
// and instances no virtual pointers, which, as a consequence,
// significantly reduces binary size.
//
// Note: A the time of writing this, the KaleidoscopePlugin is still
//       featuring a virtual begin() method to support legacy implementations
//       of plugins.
//       This means that there is actually still a virtual table generated
//       for each plugin class and a vptr resides in every plugin instance.
//       Those might both vanish when the begin() method would be removed from
//       KaleidoscopePlugin in future versions of the firmware.
//
// The call to hook methods through PluginHookAdapter and HookLoop
// is templated to allow for compile time static polymorphisms
// (hook methods of plugins not beeing virtual).
//
// ***For non template experts:
//       This is somewhat similar to duck typing in scripting languages
//       as only the signature of a method counts and not the fact that the class
//       is actually derived from a base class. Thus, strictly speaking,
//       plugins do not need to be derived from KaleidoscopePlugin, but
//       only need to an implement equivalent set of hooks.
//
// If a plugin implements a method (thereby hiding the equivalent method
// of the plugin base class) the derived plugins's method is called instead
// of the version of the hook provided by the base class KaleidoscopePlugin.
// This is possible as the hook method is not invoked via a base class ptr
// or reference but via a reference to the actual derived plugin class.
//
// HookLoop::apply() implements a compile time for-each loop
// over all plugins. Under the assumption that only some (few) plugins
// implement many hook methods and that there is only a limited number
// of plugins used in a sketch, this approach is quite efficient both in terms
// of code size and run-time. This is due to the compiler
// optimizing away any calls to hooks that have not been
// implemented by plugins because the base class versions of these hooks are
// mostly noops which are easy to detect and eliminate by the compiler.
//
// Calling the plugins hook method is finally carried out via the Hook__ template
// argument which forwards the call to the hook method to the actual plugin.
//
// Some hooks such as e.g. the eventHandlerHook require a decision about
// whether to continue calling other hook methods after them, or,
// if required, to abort the loop. This decision is implemented through a
// ContinuationPredicate__, a helper class
// whose eval-method generates a boolean return value that is based on the
// hooks' return values. If the predicate's result is true, the loop
// continues or is aborted otherwise. The eventHandlerHook method's
// return value, e.g. signals by itself whether it wants to allow other
// eventHandlerHooks to be called after it.
// By means of this approach the decision is for most hooks evaluated at
// compile-time, which e.g. cases the decision to be optimized out
// for all those hook methods that have a void return value.

class KaleidoscopePlugin {

  friend class Kaleidoscope_;

 protected:

  /** Initial plugin setup hook.
   * All plugins are supposed to provide a singleton object, statically
   * initialized at compile-time (with few exceptions). Because of this, the
   * order in which they are instantiated is unspecified, and cannot be relied
   * upon. For this reason, one's expected to explicitly initialize, "use" the
   * plugins one wishes to, by calling `Kaleidoscope.use()` with a list of plugin
   * object pointers.
   *
   * This function will in turn call the `begin` function of each plugin,
   * so that they can perform any initial setup they wish, such as registering
   * event handler or loop hooks. This is the only time this function will be
   * called. It is intentionally protected, and accessible by the `Kaleidoscope`
   * class only.
   */
  virtual void begin(void)
  __attribute__((deprecated("KaleidoscopePlugin.begin() is deprecated. Please implement KaleidoscopePlugin.init() instead."))) {
  };

 public:

  // The following callbacks handle the synchronized communication
  // between the Kaleidoscope core and its plugins.
  //
  // A plugin can, but does not have to implement all provided hook methods.
  // Calls to unimplemented hook methods are entirely optimized away by the compiler.
  // Thus, a non-implemented hook causes neither memory consumption nor
  // run-time overhead.
  //
  // Note: All hook methods in this class are non virtual on purpose as the actual
  //       interface between the Kaleidoscope singleton class and the
  //       plugins is the polymorphic PluginHookAdapter class

  void init() {
    // By letting the new init method call the legacy
    // begin() method, we make sure that the old hooking
    // interface is supported even if KALEIDOSCOPE_INIT_PLUGINS()
    // and KALEIDOSCOPE_CONNECT_PLUGINS are used to register
    // a plugin that relies on the legacy begin() method
    // to initialize itself and register hooks.
    //
    this->begin();
  }

  // This handler is supposed to return false if no other handlers are
  // supposed to be called after it, false otherwise.
  //
  // The handler is allowed to modify the mappedKey that is therefore
  // passed by reference.
  //
  bool eventHandlerHook(Key &mappedKey, const EventKey &eventKey) {
    return true; // Always allow other handlers to continue
  }

  void preReportHook() {}
  void postReportHook() {}
};

// The following invoke macros are meant to be used with
// KALEIDOSCOPE_CONNECT_PLUGINS in conjunction with the MAP macro
// that casts a specific operation on every member of
// a variadic macro argument list (if present).
//
#define INVOKE_HOOK_FOR_PLUGIN(PLUGIN) \
   \
   hook_return_val = Hook__::invoke(PLUGIN, hook_args...); \
   \
   if (!ContinuationPredicate__::eval(hook_return_val)) { \
      return hook_return_val; \
   }

#define INVOKE_EMPTY_ARGS_HOOK_FOR_PLUGIN(PLUGIN) \
   \
   Hook__::invoke(PLUGIN);

// KALEIDOSCOPE_INIT_PLUGINS is meant to be invoked at global scope of
// the firmware sketch.
//
// Arguments: A list of references to plugin instances that have been
//       instanciated at global scope.
//
// A note concerning possible future optimizations:
//    In C++17 the loop over all plugins, and calling a hook method
//    with a given list of arguments can be implemented much more elegant
//    without using preprocessor macros.
//    The alternative solution would consist in a combination
//    of std::forward_as_tuple and std::apply to call a function
//    (the hook method) for every member of a tuple (containing rvalue
//    references to all plugins). Thus, both the list of plugins and
//    and the hook method call arguments (both variadic) could be wrapped
//    in rvalue-reference-tuples and be passed to some kind
//    of tuple-for-each algorithm.
//
#define KALEIDOSCOPE_INIT_PLUGINS(...) \
   \
   struct HookLoop \
   { \
      template<typename Hook__, /* Invokes the hook method on the plugin */ \
               typename ContinuationPredicate__, /* Decides whether to continue with further hooks */\
               typename... Args__ /* The hook method call arguments */ \
      > \
      /* The Hook__ class defines the return value of the hook method as a \
         nested typedef. \
         To determine the actual return type based on Hook__, we have \
         to rely on the trailing-return-type syntax. */ \
      static auto apply(Args__&&... hook_args) -> typename Hook__::ReturnValue { \
         \
         typename Hook__::ReturnValue hook_return_val; \
         \
         MAP(INVOKE_HOOK_FOR_PLUGIN, __VA_ARGS__) \
         \
         return hook_return_val; \
      } \
      \
      /* An overloaded empty arguments version of apply to support \
         hooks with void arguments */ \
      template<typename Hook__, /* Invokes the hook method on the plugin */ \
               typename... Args__ /* The hook method call arguments */ \
      > \
      static auto apply() -> typename Hook__::ReturnValue { \
         MAP(INVOKE_EMPTY_ARGS_HOOK_FOR_PLUGIN, __VA_ARGS__) \
      } \
   }; \
   \
   PluginHookAdapter<HookLoop> hookLoop;

// This macro is supposed to be called at the end of the begin() method
// of the firmware sketch to connect the plugin hooks.
//
// Unfortunately, the c++ standard (until c++17) does not allow for
// local class templates, neither for classes that feature template functions.
// Thus we have to separate initialization and connection of plugins.
// Otherwise, everything could be called from within the begin() method
// of the firmware sketch.
//
#define KALEIDOSCOPE_CONNECT_PLUGINS \
   Kaleidoscope.connectPlugins(&hookLoop);

// This hook adapter base class defines the main interface that
// is used by the Kaleidoscope class and in all other places
// where hooks are called.
//
class PluginHookAdapter__ {
 public:

  virtual void init() {}

  virtual bool eventHandlerHook(Key &mappedKey, const EventKey &eventKey) {
    return true;
  }

  virtual void preReportHook() {}
  virtual void postReportHook() {}
};

// The HOOK_TASK macro defines an auxiliary Hook class (HOOK) that invokes
// a plugin hook method with a provided set of method arguments. The
// HOOK class is meant to be passed to the PluginLoop in the PluginHookAdapter
// to forward the call to the hook methods of plugins.
//
#define HOOK_TASK(HOOK, RET_VAL, HOOK_METHOD) \
   \
   struct HOOK {  \
      \
      typedef RET_VAL ReturnValue; \
      \
      template<typename Plugin__, typename... Args__> \
      static RET_VAL invoke(Plugin__ &plugin, Args__&&... hook_args) \
      { \
         return plugin.HOOK_METHOD(hook_args...); \
      } \
   };

// A predicate class that decides on hook-plugin-loop continuation.
//
struct ContinueIfHookReturnsTrue {
  static bool eval(bool val) {
    return val;
  }
};

// This is the implementation of the PluginHookAdapter__ interface.
// It is templated to allow it to adapt itself to the list of plugins
// that is used in the sketch. This information is known at compile time
// but differs between sketches as part of the firmware configuration.
//
// The PluginHookAdapter operates on a PluginLoop__, an
// interface class that casts Hook classes on all plugins, therby
// forwaring the actual hook calls. Hook overrides may
// specify ContinuationPredicate__ classes that decide
// if further hook methods of other plugins are allowed to be called based
// on the return value of the most recent call to a hook, e.g. a boolean value
// (see the eventHandlerHook for an example).
//
template<typename PluginLoop__>
class PluginHookAdapter : public PluginHookAdapter__ {
 public:

  HOOK_TASK(InitTask, void, init)
  void init() final {
    PluginLoop__::template apply<InitTask>();
  }

  HOOK_TASK(EventHandlerHookTask, bool, eventHandlerHook)
  bool eventHandlerHook(Key &mappedKey, const EventKey &eventKey) final {
    return PluginLoop__::template apply<EventHandlerHookTask, ContinueIfHookReturnsTrue>
    (mappedKey, eventKey);
  }

  HOOK_TASK(preReportHookTask, void, preReportHook)
  void preReportHook() final {
    PluginLoop__::template apply<preReportHookTask>();
  }

  HOOK_TASK(postReportHookTask, void, postReportHook)
  void postReportHook() final {
    PluginLoop__::template apply<postReportHookTask>();
  }
};

// The Int traits classes allow to determine a suitable integer
// type for uses a given number of bytes. This is necessary
// to make the given implementation platform independent (avr, x86, etc.)
//
template<typename Dummy__, int Bytes__>
struct Int {};
template<typename Dummy__>
struct Int<Dummy__, 2> {
  typedef int16_t Type;
};
template<typename Dummy__>
struct Int<Dummy__, 4> {
  typedef int32_t Type;
};
template<typename Dummy__>
struct Int<Dummy__, 8> {
  typedef int64_t Type;
};

class Kaleidoscope_ {
 public:
  Kaleidoscope_(void);

  void setup(const byte keymap_count)
  __attribute__((deprecated("The keymap_count argument (and the KEYMAP_SIZE macro) are unused, and can be safely removed."))) {
    setup();
  }
  void setup(void);
  void loop(void);

  // ---- Kaleidoscope.use() ----

  // First, we have the zero-argument version, which will satisfy the tail case.
  inline void use() {
  }

  // Then, the one-argument version, that gives us type safety for a single
  // plugin.
  inline void use(KaleidoscopePlugin *p) {
    p->begin();
  }

  // We have a no-op with a int argument, as a temporary hack until we remove
  // the last instance of a NULL-terminated Kaleidoscope.use() call.
  inline void use(int) {
  }

  // And the magic is in the last one, a template. The first parameter is
  // matched out by the compiler, and passed to one of the functions above. The
  // rest of the parameter pack (which may be an empty set in a recursive case),
  // are passed back to either ourselves, or the zero-argument version a few
  // lines above.
  template <typename... Plugins>
  void use(KaleidoscopePlugin *first, Plugins&&... plugins) {
    use(first);
    use(plugins...);
  }

  // ---- hooks ----

  /*
   * In most cases, one only wants a single copy of a hook. On the other hand,
   * plugins that depend on other plugins, may want to make it easier for the
   * end-user to use the plugin, and call the setup function of the dependent
   * plugins too. In case the end-user calls the same setup function, we'd end up
   * with hooks registered multiple times.
   *
   * To avoid this, protection against double-registration has been introduced.
   * The `event_handler_hook_use` and `loop_hook_use` functions will only allow
   * one copy of the hook. The `event_handler_hook_append` and `loop_hook_append`
   * functions will, on the other hand, just append the hooks, and not care about
   * protection.
   */
  typedef Key(*eventHandlerHook)(Key mappedKey, byte row, byte col, uint8_t keyState);
  static eventHandlerHook eventHandlers[HOOK_MAX];

  static void replaceEventHandlerHook(eventHandlerHook oldHook, eventHandlerHook newHook)
  __attribute__((deprecated("Kaleidoscope::replaceEventHandlerHook(...) is deprecated. Please implement KaleidoscopePlugin.eventHandlerHook(...) instead.")));
  static void appendEventHandlerHook(eventHandlerHook hook)
  __attribute__((deprecated("Kaleidoscope::appendEventHandlerHook(...) is deprecated. Please implement KaleidoscopePlugin.eventHandlerHook(...) instead.")));
  static void useEventHandlerHook(eventHandlerHook hook)
  __attribute__((deprecated("Kaleidoscope::useEventHandlerHook(...) is deprecated. Please implement KaleidoscopePlugin.eventHandlerHook(...) instead.")));

  typedef void (*loopHook)(bool postClear);
  static loopHook loopHooks[HOOK_MAX];

  static void replaceLoopHook(loopHook oldHook, loopHook newHook)
  __attribute__((deprecated("Kaleidoscope::replaceLoopHook(...) is deprecated. Please implement KaleidoscopePlugin.beginLoopHook(...) or KaleidoscopePlugin.postReportHook(...) instead.")));
  static void apppostReportHook(loopHook hook)
  __attribute__((deprecated("Kaleidoscope::replaceLoopHook(...) is deprecated. Please implement KaleidoscopePlugin.beginLoopHook(...) or KaleidoscopePlugin.postReportHook(...) instead.")));
  static void useLoopHook(loopHook hook)
  __attribute__((deprecated("Kaleidoscope::replaceLoopHook(...) is deprecated. Please implement KaleidoscopePlugin.beginLoopHook(...) or KaleidoscopePlugin.postReportHook(...) instead.")));

  static bool focusHook(const char *command);

  void connectPlugins(PluginHookAdapter__ *hooks) {
    hooks_ = hooks;
    hooks_->init();
  }

 public:

  PluginHookAdapter__ *hooks_;
};

extern Kaleidoscope_ Kaleidoscope;

#define FOCUS_HOOK_KALEIDOSCOPE FOCUS_HOOK(Kaleidoscope.focusHook,  \
                                           "layer.on\n"             \
                                           "layer.off\n"            \
                                           "layer.getState")

/* -- DEPRECATED aliases; remove them when there are no more users. -- */

void event_handler_hook_use(Kaleidoscope_::eventHandlerHook hook)
__attribute__((deprecated("Use Kaleidoscope.useEventHandlerHook instead")));
void loop_hook_use(Kaleidoscope_::loopHook hook)
__attribute__((deprecated("Use Kaleidoscope.useLoopHook instead")));

void __USE_PLUGINS(KaleidoscopePlugin *plugin, ...)
__attribute__((deprecated("Use Kaleidoscope.use(...) instead")));

#define USE_PLUGINS(...) __USE_PLUGINS(__VA_ARGS__, NULL)
