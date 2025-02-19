# Kaleidoscope-SonicThemes

Audio feedback themes for the Keyboardio Preonic using its built-in piezo speaker. 

## Overview

SonicThemes provides customizable audio feedback for keyboard events through different sound themes. Each theme provides distinct sounds for:
- Layer changes
- Bluetooth connection/disconnection
- Battery status changes
- Boot sequence
- Error states

### Sound Events
Each theme responds to the following events:
- **Layer Changes**: Plays when switching between layers
- **Connection Events**:
  - Connect: When Bluetooth connection is established
  - Disconnect: When Bluetooth connection is lost
- **Battery Events**:
  - Charging Start: When power is connected
  - Charging Stop: When fully charged
  - Low Battery: When battery level drops below 20%
- **System Events**:
  - Boot: On keyboard startup
  - Error: For various error conditions

## Using the plugin

### Hardware Requirements

Currently supported keyboards:
- Keyboardio Preonic (with built-in piezo speaker)

### Installation

1. Include the plugin in your sketch:
```cpp
#include <Kaleidoscope-SonicThemes.h>
```

2. Activate the plugin:
```cpp
KALEIDOSCOPE_INIT_PLUGINS(
  SonicThemes,
  // ... other plugins ...
);
```

3. Optional: Configure theme in setup():
```cpp
void setup() {
  Kaleidoscope.setup();
  SonicThemes.enable();  // Enable sound (on by default)
}
```

### Available Themes

1. Station Master
   - Theme: Train station-inspired sounds
   - Style: Clear, distinct tones with professional feel
   - Events:
     - Layer change: Two-tone ascending chime
     - Connect: Four-note welcome melody
     - Disconnect: Four-note descending melody
     - Charging: Simple two-tone indicators
     - Battery Low: Three-note warning sequence
     - Boot: Five-note startup melody
     - Error: Three-note alert sequence

2. Speech Synthesis
   - Theme: Verbal feedback using speech synthesis
   - Style: Clear spoken words and phrases
   - Events:
     - Layer change: "Layer [number]"
     - Connect: "Online"
     - Disconnect: "Offline"
     - Charging: "Power" / "Full"
     - Battery Low: "Low"
     - Boot: "Ready"
     - Error: "Error"

3. Retro Gaming
   - Theme: Classic 8-bit video game sounds
   - Style: Chiptune-inspired sound effects
   - Events:
     - Layer change: "Coin collect" sound
     - Connect: "Power up" sequence
     - Disconnect: "Power down" sequence
     - Charging: "Item get" / "Item complete"
     - Battery Low: "Danger" warning
     - Boot: "Game start" fanfare
     - Error: "Game over" sound

4. Minimal
   - Theme: Simple, unobtrusive beeps
   - Style: Short, clear tones
   - Events:
     - Layer change: Single short beep
     - Connect: Ascending pair
     - Disconnect: Descending pair
     - Charging: Single beeps
     - Battery Low: Double low beep
     - Boot: Single startup tone
     - Error: Long low beep

5. Sci-Fi
   - Theme: Futuristic computer sounds
   - Style: High-tech, electronic tones
   - Events:
     - Layer change: Quick frequency sweep
     - Connect: "Teleport in" sequence
     - Disconnect: "Teleport out" sequence
     - Charging: "Energy" effects
     - Battery Low: Warning klaxon
     - Boot: Computer startup sequence
     - Error: System error alert

### Configuration

#### Runtime Configuration
The plugin supports Focus commands for real-time configuration:

```
sonicthemes.enabled 1  # Enable sounds
sonicthemes.enabled 0  # Disable sounds

sonicthemes.theme 0    # Station Master theme
sonicthemes.theme 1    # Speech Synthesis theme

sonicthemes.enabled    # Returns current enabled state
sonicthemes.theme      # Returns current theme index
```

#### Programmatic Control
The plugin provides methods for programmatic control:

```cpp
// Enable/disable sound
SonicThemes.enable();
SonicThemes.disable();

// Check if enabled
if (SonicThemes.isEnabled()) {
  // ...
}

// Switch themes
SonicThemes.nextTheme();  // Cycle to next theme
```

### Example

Basic usage:

```cpp
#include <Kaleidoscope.h>
#include <Kaleidoscope-SonicThemes.h>

KALEIDOSCOPE_INIT_PLUGINS(SonicThemes);

void setup() {
  Kaleidoscope.setup();
  
  // Optional: Start with a specific theme
  SonicThemes.nextTheme();  // Cycles to next theme
}
```

### Example Sketches

#### Preonic Basic (examples/Preonic/Preonic.ino)
A minimal example showing SonicThemes on the Preonic keyboard:
- Basic QWERTY layout
- Audio feedback for:
  - Boot sequence
  - Bluetooth connection/disconnection
  - Battery status changes
  - Error states

#### Speech Synthesis Demo (examples/SpeechSynth/SpeechSynth.ino)
A standalone Arduino sketch demonstrating the speech synthesis capabilities:
- Shows how to use the phoneme system
- Speaks the Declaration of Independence
- Can be adapted for custom speech patterns
- Useful for testing and development

## Plugin Development

### Adding New Themes

Themes are data structures that define melodies for each keyboard event. Each theme consists of:
1. Note sequences for each event
2. Theme metadata (name, etc.)

#### Basic Theme Structure
```cpp
// Define note sequences for each event
static const Note PROGMEM my_layer_change[] = {
  {440, 80},   // First note: 440Hz for 80ms
  {554, 100},  // Second note: 554Hz for 100ms
};

static const Note PROGMEM my_connect[] = {
  {440, 100},  // A4
  {554, 100},  // C#5
  {659, 100},  // E5
  {880, 150},  // A5
};

// Create the theme
static const Theme PROGMEM my_custom_theme = {
  "My Theme Name",
  {
    melody_from_array(my_layer_change),  // LayerChange event
    melody_from_array(my_connect),       // Connect event
    // ... other event melodies ...
  }
};
```

#### Notes and Frequencies
Each note is defined by:
```cpp
struct Note {
  uint16_t frequency;  // Hz (20-25000)
  uint16_t duration;   // ms
};
```

Common frequencies:
- A4: 440 Hz
- C5: 523 Hz
- E5: 659 Hz
- G5: 784 Hz

#### Event Types
Your theme must provide melodies for all events:
```cpp
enum class SoundEvent {
  LayerChange,    // Layer switching
  Connect,        // Bluetooth connection
  Disconnect,     // Bluetooth disconnection
  ChargingStart,  // Power connected
  ChargingStop,   // Fully charged
  LowBattery,     // Battery below 20%
  Boot,           // Keyboard startup
  Error           // Error conditions
};
```

#### Design Guidelines
1. Keep melodies short (2-5 notes typically)
2. Use distinct patterns for different events
3. Consider the context:
   - Layer changes happen frequently - keep them subtle
   - Warnings (low battery, errors) should be noticeable
   - Boot sequence can be more elaborate
4. Test your melodies at different volumes

#### Example Theme
Here's a minimal theme example:
```cpp
namespace kaleidoscope {
namespace plugin {

// Define melodies
static const Note PROGMEM minimal_layer[] = {
  {440, 50},  // Short beep
};

static const Note PROGMEM minimal_connect[] = {
  {440, 50},
  {880, 50},
};

static const Note PROGMEM minimal_disconnect[] = {
  {880, 50},
  {440, 50},
};

// Create theme
static const Theme PROGMEM minimal_theme = {
  "Minimal",
  {
    melody_from_array(minimal_layer),
    melody_from_array(minimal_connect),
    melody_from_array(minimal_disconnect),
    // ... other required melodies ...
  }
};

}  // namespace plugin
}  // namespace kaleidoscope
```

### Events

The plugin responds to:
- Layer changes (`onLayerChange`)
- Connection state changes (`afterEachCycle`)
- Battery level changes (`afterEachCycle`)
- Boot sequence (`onSetup`)

## Dependencies

- Kaleidoscope-Hardware-Keyboardio-Preonic