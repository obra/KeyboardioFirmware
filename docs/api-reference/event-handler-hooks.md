# Kaleidoscope's Plugin Event Handlers

Kaleidoscope provides a set of hook functions that plugins can define in order
to do their work. If one or more of the functions listed here are defined as
methods in a plugin class, that plugin can act on the input events that drive
Kaleidoscope.

In response to input events (plus a few other places), Kaleidoscope calls the
event handlers for each plugin that defines them, in sequence.

## Return values

Every Kaleidoscope event handler function returns a value of type
`EventHandlerResult`, an enum with several variants. In some handlers,
Kaleidoscope ignores the return value, but for others, the result is used as a
signal to control Kaleidoscope's behavior. In particular, some event handler
hooks are "abortable". For those hooks, the return value of the plugin handlers
are used to control what Kaleidoscope does after each plugin's event handler
returns.

- `EventHandlerResult::OK` is used to signal that Kaleidoscope should continue
  on to the next handler in the sequence.

- `EventHandlerResult::ABORT` is used to signal that Kaleidoscope should not
  continue to call the other plugin handlers in the sequence, and stop
  processing the event entirely. This is used by some plugins to cancel events
  and/or delay them so that they occur at a later time, possibly with different
  values.

- `EventHandlerResult::EVENT_CONSUMED` is used to signal that the plugin has
  successfully handled the event, and that there is nothing further to be done,
  so there is no point in continuing to call further plugin event handlers for
  the event.

## Non-event "event" handlers

There are three special "event" handlers that are not called in response to
input events, but are instead called at fixed points during Kaleidoscope's run
time.

### `onSetup()`

This handler is called when Kaleidoscope first starts, at the end
of the `setup()` method. If a plugin needs to do some work after
its constructor is called, but before Kaleidoscope enters its main
loop and starts scanning for keyswitch events, it can do it in this
function.

It takes no arguments, and must return
`kaleidoscope::EventHandlerResult::OK`.

### `beforeEachCycle()`

This handler gets called at the beginning of every keyswitch scan cycle, before
the scan. It can be used by plugins to do things that need to be done
repeatedly, regardless of any input from the user. Typically, this involves
things like checking for timeouts.

Takes no arguments, must return `kaleidoscope::EventHandlerResult::OK`.

### `afterEachCycle()`

This is just like `beforeEachCycle()`, but gets called after the keyswitches
have been scanned (and any input events handled).

## Keyswitch input event handlers

This group of event handlers is triggered when keys on the keyboard are pressed
and released. With one exception, they use a `KeyEvent` object as their one
parameter. The `KeyEvent` class encapsulates the essential data about a key
press (or release):

- `event.addr` contains the `KeyAddr` of the key that toggled on or off.

- `event.state` contains information about the current and former state of the
  key in the form of a `uint8_t` bitfield.

- `event.key` contains the `Key` value of the event. For key presses, this is
  generally determined by means of a keymap lookup. For releases, the value is
  taken from the `live_keys` structure. Because the `event` is passed by
  reference, changing this value in a plugin handler will affect which value
  ends up in the `live_keys` array, and thus, the output of the keyboard.

- `event.id` contains a `KeyEventId` value: an integer, usually monotonically
  increasing. This is useful as a tool to allow plugins to avoid re-processing
  the same event, thus avoiding infinite loops without resorting to an
  `INJECTED` key state flag which would cause other plugins to ignore events
  that they might otherwise be interested in.

### `onKeyswitchEvent(KeyEvent &event)`

This handler is called in response to changes detected in the state of
keyswitches, via the `Runtime.handleKeyswitchEvent()` function. After the
keyswitches are scanned in each cycle, Kaleidoscope goes through them all and
compares the state of each one to its previous state. For any of them that have
either toggled on or off, plugins that define this function get called (until
one of them returns either `ABORT` or `EVENT_CONSUMED`).

This handler should be defined by any plugin that is concerned only with
physical keyswitch events, where the user has pressed or released a physical
key. For example, plugins that determine key values based on the timing of these
physical events should define this handler (for example, Qukeys and
TapDance). Plugins that don't explicitly need to use this handler should define
`onKeyEvent()` instead.

Plugins that use this handler should abide by certain rules in order to interact
with each other to avoid infinite loops. A plugin might return `ABORT` to delay
an event (until some other event or a timeout occurs), then later re-start
processing of the same event by calling `Runtime.handleKeyswitchEvent()`. When
it does this, it must take care to use the same `KeyEventId` value as that
event's `id` parameter, and it should also take care to preserve the order of
any such events. This way, plugins implementing `onKeyswitchEvent()` are able
to keep track of event id numbers that they have already processed fully, and
ignore those events when plugins later in the sequence re-start them.

In more specific detail, plugins that implement `onKeyswitchEvent()` must
guarantee that the `event.id` values they emit when returning `OK` are
monotonically increasing, and should only include `id` values that the plugin
has already received as input. Additionally, such plugins must ignore any event
with an `id` value that it has recently received and finished processing. The
class `KeyEventTracker` can help simplify following these rules.

### `onKeyEvent(KeyEvent &event)`

After a physical keyswitch event is processed by all of the plugins with
`onKeyswitchEvent()` handlers (and they all return `OK`), Kaleidoscope passes
that event on to the `Runtime.handleKeyEvent()` function, which calls plugins'
`onKeyEvent()` handlers. This is also the starting point for events which do not
correspond to physical key events, and can have an invalid `event.addr` value.

Plugins that need to respond to keyboard input, but which do not need to be
closely tied to physical key events (and only those events) should use
`onKeyEvent()` to do their work.

After all `onKeyEvent()` handlers have returned `OK` for an event, the
`live_keys` state array gets updated. For a key press event, the final
`event.key` value gets inserted into `live_keys[event.addr]`. From that point
on, the keyboard will behave as though a key with that value is being held until
that entry in `live_keys` is cleared (most likely as a result of a key release
event's `onKeyEvent()` handlers returning `OK`). Thus, if an `onKeyEvent()`
handler returns `ABORT` for a key release event, the keyboard will behave as
though that key is still held after it has been released. This is what enables
plugins like OneShot to function, but it also means that plugin authors need to
take care about returning `ABORT` (but not `EVENT_CONSUMED`) from an
`onKeyEvent()` handler, because it could result in "stuck" keys.

`onKeyEvent()` handlers should not store events and release them later (by
calling `Runtime.handleKeyEvent()`), and must never call
`Runtime.handleKeyswitchEvent()`.

### `onAddToReport(Key key)`

After the `onKeyEvent()` handlers have all returned `OK`, Kaleidoscope moves on
to sending Keyboard HID reports. It clears the current report, and iterates
through the `live_keys` array, looking for non-empty values, and adding them to
the report. For System Control, Consumer Control, and Keyboard HID type `Key`
values, Kaleidoscope handles adding the keycodes to the correct report, but it
also calls this handler, in case a plugin needs to alter that report.

A return value of `OK` allows Kaleidoscope to proceed with adding the
corresponding keycode(s) to the HID report, and `ABORT` causes it to leave and
keycodes from `key` out of the report.

Note that this only applies to the Keyboard and Consumer Control HID reports,
not the System Control report, which has different semantics, and only supports
a single keycode at a time.

### `beforeReportingState(const KeyEvent &event)`

This gets called right before a set of HID reports is sent. At this point,
plugins have access to a (tentative) complete HID report, as well as the full
state of all live keys on the keyboard. This is especially useful for plugins
that might need to do things like remove keycodes (such as keyboard modifiers)
from the forthcoming report just before it gets sent.

This event handler still has access to the event information for the event that
triggered the report, but because it is passed as a `const` reference, it is no
longer possible to change any of its values.

[Note: The older version of `beforeReportingState()` got called once per cycle,
regardless of the pattern of keyswitches toggling on and off, and many plugins
used it as a place to do things like check for timeouts. This new version does
not get called every cycle, so when porting old code to the newer handlers, it's
important to move any code that must be called every cycle to either
`beforeEachCycle()` or `afterEachCycle()`.]

[Also note: Unlike the deprecated `beforeReportingState()`, this one is
abortable. That is, if it returns a result other than `OK` it will stop the
subsequent handlers from getting called, and if it returns `ABORT`, it will also
stop the report from being sent.]

### `afterReportingState(const KeyEvent &event)`

This gets called after the HID report is sent.  This handler allows a plugin to
react to an event, but wait until after that event has been fully processed to
do so.  For example, the OneShot plugin releases keys that are in the "one-shot"
state in response to key press events, but it does so after those triggering
press events take place.

## Other events

### `onHostConnectionStatusChanged(uint8_t device_id, HostConnectionStatus status)`

Called when a host device's connection status changes. The `device_id` parameter identifies which host device changed status, and the `status` parameter indicates the new connection state.

The possible status values are:
- `Disconnected`: The host is not connected
- `Connecting`: In the process of connecting (including pairing)
- `Connected`: Successfully connected and ready for use
- `PairingFailed`: Failed to establish pairing with the host
- `PairingSuccess`: Successfully paired with the host

Note: The pairing states (`PairingFailed` and `PairingSuccess`) are one-time notifications that occur during the connection process. After these states, the status will transition to either `Connected` (on success) or `Disconnected` (on failure).

### `onLayerChange()`

Called whenever one or more keymap layers are activated or deactivated (just
after the change takes place).

### `onLEDModeChange()`

Called by `LEDControl` whenever the active LED mode changes.

### `beforeSyncingLeds()`

Called immediately before Kaleidoscope sends updated color values to the
LEDs. This event handler is particularly useful to plugins that need to override
the active LED mode (e.g. LED-ActiveModColor).

### `onFocusEvent()`

Used to implement [bi-directional communication](#bidirectional-communication-for-plugins). This is called whenever the firmware receives a command from the host. The only argument is the command name. Can return `kaleidoscope::EventHandlerResult::OK` to let other plugins process the event further, or `kaleidoscope::EventHandlerResult::EVENT_CONSUMED` to stop processing.

### `onNameQuery()`

Used by the [Focus](#bidirecional-communication-for-plugins) plugin, when replying to a `plugins` command. Should either send the plugin name, or not be implemented at all, if the host knowing about the plugin isn't important.


### `exploreSketch()`

## Deprecated

Two existing "event" handlers have been deprecated. In the old version of
Kaleidoscope's main loop, the keyboard's state information was stored in the
keyscanner (which physical switches were on in the current and former scans),
and in the HID reports. The Keyboard HID report would be cleared at the start of
every cycle, and re-populated, on key at a time, calling every
`onKeyswitchEvent()` handler for every active key. Then, once the tentative HID
report was complete, the `beforeReportingState()` handlers would be called, and
the complete report would be sent to the host. In most cycles, that report would
be identical to the previous report, and would be suppressed.

The new system stores the keyboard's current state in the `live_keys` array
instead, and only calls event handlers in response to keyswitch state changes
(and artificially generated events), ultimately sending HID reports in response
to events, rather than at the end of every cycle.

### `onKeyswitchEvent(Key &key, KeyAddr key_addr, uint8_t key_state)`

This handler was called in every cycle, for every non-idle key. Its concept of
an "event" included held keys that did not have a state change. These deprecated
handlers are still called, in response to events and also when preparing the HID
reports, but there is no longer a reasonable mechanism to call them in every
cycle, for every active key, so some functionality could be lost.

It is strongly recommended to switch to using one of the two `KeyEvent`
functions instead, depending on the needs of the plugin (either `onKeyEvent()`
if it is fit for the purpose, or `onKeyswitchEvent()` if necessary). The
`onAddToReport()` function might also be useful, particularly if the plugin in
question uses special `Key` values not recognized by Kaleidoscope itself, but
which should result in keycodes being added to HID reports.

### `beforeReportingState()`

The old version of this handler has been deprecated, but it will still be called
both before HID reports are sent and also once per cycle. It is likely that
these handlers will continue to function, but the code therein should be moved
either to the new `KeyEvent` version of `beforeReportingState()` and/or
`afterEachCycle()` (or `beforeEachCycle()`), depending on whether it needs to be
run only in response to input events or if it must execute every cycle,
respectively.
