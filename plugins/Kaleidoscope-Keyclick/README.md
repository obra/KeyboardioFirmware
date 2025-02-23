# Kaleidoscope-Keyclick

This plugin provides audible feedback when keys are pressed, similar to the click sound of clicky mechanical switches. It uses the keyboard's built-in piezo speaker to produce a short, sharp click sound on each keypress.

## Using the plugin

To use the plugin, include the header, and enable it:

```cpp
#include <Kaleidoscope.h>
#include <Kaleidoscope-EEPROM-Settings.h>
#include <Kaleidoscope-FocusSerial.h>
#include <Kaleidoscope-Keyclick.h>

KALEIDOSCOPE_INIT_PLUGINS(
  EEPROMSettings,
  FocusSerial,
  Keyclick
);
```

## Configuration

The plugin can be configured both at runtime and through the Focus protocol.

### Toggle Key

The plugin provides a `Key_ToggleKeyclick` key that can be added to your keymap to toggle the keyclick feature on and off. The state is preserved in EEPROM.

### Focus Commands

The following Focus command is available:

- `keyclick.enabled`: Get/set whether keyclick is enabled

For example:
```
keyclick.enabled 1  # Enable keyclick
```

## Plugin Properties

- Click frequency: 5000 Hz (higher frequency for a sharper click)
- Click duration: 1ms (very short for a click rather than a beep)
- Settings are stored in EEPROM and persist across reboots

## Dependencies

- [Kaleidoscope-EEPROM-Settings](https://github.com/keyboardio/Kaleidoscope-EEPROM-Settings)
- [Kaleidoscope-FocusSerial](https://github.com/keyboardio/Kaleidoscope-FocusSerial)

## Further reading

The [example][plugin:example] is a good starting point for using the plugin.

  [plugin:example]: /examples/Keyclick/Keyclick.ino 
