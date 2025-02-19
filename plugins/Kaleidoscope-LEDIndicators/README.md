# Kaleidoscope-LEDIndicators

This plugin provides visual feedback through LEDs for various keyboard states and events, with a primary focus on connection status. It supports a variety of visual effects including solid colors, blinking, breathing, and transitions.

## Using the plugin

To use this plugin, include the header and activate it:

```cpp
#include "Kaleidoscope-LEDIndicators.h"

KALEIDOSCOPE_INIT_PLUGINS(
  LEDControl,
  LEDIndicators
);
```

The plugin will automatically handle connection status changes and display appropriate LED indicators:

- **Connecting**: Blue blinking LED
- **Connected**: Solid blue LED
- **Disconnected**: LED off
- **Pairing Success**: Brief green flash, then solid blue
- **Pairing Failed**: Brief red flash, then LED off

## LED Mapping

By default, the plugin uses the first 5 LEDs in the top row for indicators. However, you should configure the LED mapping to use positions that make sense for your specific keyboard layout:

```cpp
void configureIndicators() {
  // Configure LED mapping for your keyboard
  static const KeyAddr indicator_leds[] = {
    KeyAddr(3, 0),  // Top row, 4th key
    KeyAddr(3, 1),  // Top row, 5th key
    KeyAddr(3, 2),  // Top row, 6th key
    KeyAddr(3, 3),  // Top row, 7th key
    KeyAddr(3, 4)   // Top row, 8th key
  };

  // Try to configure the slots
  if (!LEDIndicators.setSlots(5, indicator_leds)) {
    // Configuration failed - LEDs invalid for this device
  }
}
```

The plugin supports up to 8 indicator slots (defined by MAX_SLOTS). Each slot needs a valid LED address for your keyboard. The setSlots() function will validate the LED addresses and return false if any are invalid.

## Colors

The default colors are:
- Blue (0, 0, 255) for connection status
- Red (255, 0, 0) for errors
- Green (0, 255, 0) for success
- Off (0, 0, 0)

You can customize these colors:

```cpp
LEDIndicators.setColor(LEDIndicators.color_blue, {0, 0, 128});   // Dimmer blue
LEDIndicators.setColor(LEDIndicators.color_red, {128, 0, 0});    // Dimmer red
LEDIndicators.setColor(LEDIndicators.color_green, {0, 128, 0});  // Dimmer green
```

## Effects

The plugin supports several visual effects:
- **Solid**: Single color, constant brightness
- **Blink**: Alternates between two colors
- **Breathe**: Smoothly transitions between brightnesses
- **Grow**: Starts dim, breathes to full brightness, stays bright
- **Shrink**: Starts bright, breathes to dim, stays dim

Each effect can be configured with:
- Duration (how long to show the effect)
- Number of cycles to repeat
- Priority level (to handle conflicts between multiple indicators)

## Dependencies

* [Kaleidoscope-LEDControl](https://github.com/keyboardio/Kaleidoscope-LEDControl)

## Further reading

The [example](https://github.com/keyboardio/Kaleidoscope/blob/master/plugins/Kaleidoscope-LEDIndicators/examples) directory contains a fully working demonstration.
