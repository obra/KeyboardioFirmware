# AutoShift

AutoShift allows you to type shifted characters by long-pressing a key, rather
than chording it with a modifier key.

## Using the plugin

Using the plugin with its defaults is as simple as including the header, and
enabling the plugin:

```c++
#include <Kaleidoscope.h>
#include <Kaleidoscope-AutoShift.h>

KALEIDOSCOPE_INIT_PLUGINS(AutoShift);
```

With AutoShift enabled, when you first press a key that AutoShift acts on, its
output will be delayed. If you hold the key down long enough, you will see the
shifted symbol appear in the output. If you release the key before the timeout,
the output will be unshifted.

## Turning AutoShift on and off

The `AutoShift` object provides three methods for turning itself on and off:

- To turn the plugin on, call `AutoShift.enable()`.
- To turn the plugin off, call `AutoShift.disable()`.
- To toggle the plugin's state, call `AutoShift.toggle()`.

Note: Disabling the AutoShift plugin does not affect which `Key` categories it
will affect when it is re-enabled.

## Setting the AutoShift long-press delay

To set the length of time AutoShift will wait before adding the `shift` modifier
to the key's output, use `AutoShift.setTimeout(t)`, where `t` is a number of
milliseconds.

## Configuring which keys get auto-shifted

AutoShift provides a set of key categories that can be independently added or
removed from the set of keys that will be auto-shifted when long-pressed:

- `AutoShift.letterKeys()`: Letter keys
- `AutoShift.numberKeys()`: Number keys (number row, not numeric keypad)
- `AutoShift.symbolKeys()`: Other printable symbols
- `AutoShift.arrowKeys()`: Navigational arrow keys
- `AutoShift.functionKeys()`: All function keys (F1-F24)
- `AutoShift.printableKeys()`: Letters, numbers, and symbols
- `AutoShift.allKeys()`: All non-modifier USB Keyboard keys

These categories are restricted to USB Keyboard-type keys, and any modifier
flags attached to the key is ignored when determining if it will be
auto-shifted.  Any of the above expressions can be used as the `category` parameter in the functions described below.

- To include a category in the set that will be auto-shifted, call `AutoShift.enable(category)`
- To remove a category from the set that will be auto-shifted, call `AutoShift.disable(category)`
- To set the full list of categories that will be auto-shifted, call `AutoShift.setEnabled(categories)`, where `categories` can be either a single category from the above list, or list of categories combined using the `|` (bitwise-or) operator (e.g. `AutoShift.setEnabled(AutoShift.letterKeys() | AutoShift.numberKeys())`).

## Advanced customization of which keys get auto-shifted

If the above categories are not sufficient for your auto-shifting needs, it is
possible to get even finer-grained control of which keys are affected by
AutoShift, by overriding the `isAutoShiftable()` method in your sketch. For
example, to make AutoShift only act on keys `A` and `Z`, include the following
code in your sketch:

```c++
bool AutoShift::isAutoShiftable(Key key) {
  if (key == Key_A || key == key_Z)
    return true;
  return false;
}
```

As you can see, this method takes a `Key` as its input and returns either `true`
(for keys eligible to be auto-shifted) or `false` (for keys AutoShift will leave
alone).

## Producing other characters than shifted variants of the keys

It is possible to produce other characters than just shifted variants of the
pressed key by providing an explicit mapping between the pressed key and the
key that should be produced instead.

Such a mapping must be defined in the `setup` method in your sketch:

```
AUTOSHIFT(
  kaleidoscope::plugin::LongPress(Key_Slash, Key_Backslash),
  kaleidoscope::plugin::LongPress(Key_Z,     ShiftToLayer(SYMBOL)),
)
```

Such explicit mappings take precedence over shifting the key. That
means if all alphanumeric characters are configured for AutoShift, but
the ‘e’ key has an explicit mapping to produce ‘ë’, a long press on ‘e’
will result in ’ë’, not ‘E’.

As can be seen in the example above the resulting key does not necessarily need
to be a regular key, but can be any Key object, like the layer shift in the
example. Be aware however that key repeats are not suppressed.

## Plugin compatibility

If you're using AutoShift in a sketch that also includes the Qukeys and/or
SpaceCadet plugins, make sure to register AutoShift after those other plugins in
order to prevent auto-shifts from getting clobbered.  The recommended order is
as follows:

```c++
KALEIDOSCOPE_INIT_PLUGINS(Qukeys, SpaceCadet, AutoShift)
```

It's not generally recommended to use AutoShift on the same key(s) handled by
either Qukeys or SpaceCadet, as this can result in confusing behaviour.

## Further reading

Starting from the [example][plugin:example] is the recommended way of getting
started with the plugin.

 [plugin:example]: /examples/Keystrokes/AutoShift/AutoShift.ino
