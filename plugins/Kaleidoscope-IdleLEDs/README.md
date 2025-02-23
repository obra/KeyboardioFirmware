# IdleLEDs

Having LED effects on the keyboard can be exceptionally helpful. However, having
the effects - or lights, in general - on all the time, even when the keyboard is
otherwise idle, is perhaps not the best. When one leaves the keyboard, locks the
computer, what use are the LED effects then?

One could turn them off manually, but... that's too easy to forget, and why do
something the firmware could do for us anyway? What if the LEDs turned
themselves off after some configurable idle time? Say, if one did not press any
keys for the past ten minutes, just shut 'em off.

This is exactly what the `IdleLEDs` plugin does.

## Using the plugin

The plugin comes with reasonable defaults (see below), and can be used out of
the box, without any further configuration:

```c++
#include <Kaleidoscope.h>
#include <Kaleidoscope-LEDControl.h>
#include <Kaleidoscope-LEDEffect-Rainbow.h>
#include <Kaleidoscope-IdleLEDs.h>

KALEIDOSCOPE_INIT_PLUGINS(LEDControl, IdleLEDs, LEDEffectRainbowWave);

void setup () {
  Kaleidoscope.setup ();
}
```

Because the plugin needs to know about key events, it is best to make it one of
the first plugins, so it can catch all of them, before any other plugin would
have a chance to consume key events.

It is also possible to enable run-time configuration via he `Focus` plugin, and
persistent storage of such settings. To do that, one has to use the
`PersistentIdleLEDs` object instead, provided by the plugin:

```c++
#include <Kaleidoscope.h>
#include <Kaleidoscope-LEDControl.h>
#include <Kaleidoscope-LEDEffect-Rainbow.h>
#include <Kaleidoscope-EEPROM-Settings.h>
#include <Kaleidoscope-FocusSerial.h>
#include <Kaleidoscope-IdleLEDs.h>

KALEIDOSCOPE_INIT_PLUGINS(
  EEPROMSettings,
  Focus,
  LEDControl,
  PersistentIdleLEDs,
  LEDEffectRainbowWave
);

void setup () {
  Kaleidoscope.setup ();
}
```

## Plugin Properties

The plugin provides two objects, `IdleLEDs`, and `PersistentIdleLEDs`, both with
the following properties and methods.

### `.idle_time_limit`

> Property storing the amount of time that can pass without a single key being
> pressed before the plugin considers the keyboard idle and turns off the LEDs.
> Value is expressed in milliseconds.

> Defaults to 600000 milliseconds (10 minutes).

> Provided for compatibility reasons. It is recommended to use one of the
> methods below instead of setting this property directly. If using
> `PersistentIdleLEDs`, setting this property will not persist the value to
> storage. Use `.setIdleTimeoutSeconds()` if persistence is desired.

### `.idleTimeoutSeconds()`

> Returns the amount of time (in seconds) that can pass without a single key
> being pressed before the plugin considers the keyboard idle and turns off the
> LEDs.

### `.setIdleTimeoutSeconds(uint32_t new_limit)`

> Sets the amount of time (in seconds) that can pass without a single key being
> pressed before the plugin considers the keyboard idle and turns off the LEDs.
>
> Setting the timeout to 0 will disable the plugin until it is set to a higher
> value.

### `.suspend()`, `.resume()`

> Suspends or resumes `IdleLEDs`. When suspended, `IdleLEDs` will immediately
> enter idle state, and turn LEDs off. When resuming, it will turn the LEDs back
> on, and start checking idleness again.
>
> The intended use of these functions is when LEDs are turned on or off by other
> means, such as part of host power management, and syncing that state with
> `IdleLEDs` is desired.

## Focus commands

The plugin provides a single [Focus][FocusSerial] command, but only when using
the `PersistentIdleLEDs` variant:

 [FocusSerial]: Kaleidoscope-FocusSerial.md

### `idleleds.time_limit [seconds]`

> Sets the idle time limit to `seconds`, when called with an argument. Returns
> the current limit (in seconds) when called without any.
>
> Setting the timeout to 0 will disable the plugin until it is set to a higher
> value.

## Dependencies

* [Kaleidoscope-LEDControl](Kaleidoscope-LEDControl.md)

### Optional dependencies

* [Kaleidoscope-EEPROM-Settings](Kaleidoscope-EEPROM-Settings.md)
* [FocusSerial](Kaleidoscope-FocusSerial.md)

## Further reading

Starting from the [example][plugin:example] is the recommended way of getting
started with the plugin.

 [plugin:example]: /examples/LEDs/IdleLEDs/IdleLEDs.ino
