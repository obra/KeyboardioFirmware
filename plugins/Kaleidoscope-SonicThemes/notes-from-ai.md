# SonicThemes Development Notes

## Initial Architecture Thoughts

### Core Components Needed
1. Sound Generation System
   - Need a lightweight frequency/duration pair system
   - Should avoid dynamic allocation
   - Consider using fixed-size arrays for melodies

2. Theme Management
   - Each theme should be a separate class/struct
   - Use compile-time constants for melodies
   - Consider using PROGMEM for melody storage

3. Hardware Interface
   - Need to abstract piezo control
   - Consider using existing Keyclick plugin as reference
   - Volume control might need PWM

### Memory Considerations
- Melodies should be stored as frequency/duration pairs
- Can use uint16_t for frequencies (0-20kHz sufficient)
- uint8_t for durations (in 10ms units = up to 2.55s)
- Each theme might need ~100-200 bytes for melodies

### Event System Integration
- Can hook into Kaleidoscope's event system
- Layer changes already have hooks
- Need to investigate battery/connection event hooks
- May need to add new hook points

### Questions to Investigate
1. How does Keyclick handle piezo control?
2. What's the available flash/RAM on the nrf52?
3. Are there existing event hooks for all needed events?
4. What's the minimum viable speech synthesis approach?

### Next Steps
1. Review Keyclick plugin implementation
2. Test basic piezo control
3. Create proof-of-concept for one theme

## Revised Keyclick Plugin Analysis

### Key Findings
1. Hardware Abstraction
   - Uses device.speaker().playTone(frequency, duration) interface
   - Already supports frequency and duration control
   - No volume control available in hardware

2. Implementation Structure
   - Clean event handling through onKeyswitchEvent
   - EEPROM settings support for persistent configuration
   - Focus API integration for runtime configuration

3. Advantages for Our Use
   - playTone API already provides what we need
   - Existing settings and Focus integration patterns
   - Simple and efficient implementation

### Hardware Analysis
1. Tone Implementation
   - Uses PWM2 for tone generation
   - Non-blocking implementation
   - Supports frequencies 20Hz-25kHz
   - Duration in milliseconds
   - Hardware handles timing automatically

2. Key Capabilities
   - Can play tones without CPU intervention
   - Clean start/stop control
   - Precise frequency control
   - Accurate timing

### Revised Architecture Plan
1. Sound System
   ```cpp
   struct Note {
     uint16_t frequency;  // Hz (20-25000)
     uint16_t duration;   // ms
   };

   class MelodyPlayer {
    private:
     const Note* current_melody;
     uint8_t melody_length;
     uint8_t current_note;
     bool is_playing;
     uint32_t next_note_time;

    public:
     void playMelody(const Note* melody, uint8_t length);
     void stop();
     void update(); // Called from plugin's onLoop()
   };
   ```

2. Theme Management
   ```cpp
   enum class SoundEvent {
     LayerChange,
     Connect,
     Disconnect,
     ChargingStart,
     ChargingStop,
     LowBattery,
     Boot,
     Error
   };

   class Theme {
    public:
     virtual const Note* getMelody(SoundEvent event, uint8_t& length) const = 0;
   };
   ```

3. Plugin Core
   ```cpp
   class SonicThemes : public kaleidoscope::Plugin {
    private:
     MelodyPlayer player;
     Theme* current_theme;
     uint8_t enabled;

    public:
     void playEvent(SoundEvent event);
     void nextTheme();
     void enable();
     void disable();

     // Kaleidoscope hooks
     EventHandlerResult onLayerChange();
     EventHandlerResult onSetup();
     EventHandlerResult beforeReportingState();
   };
   ```

### Implementation Strategy
1. Core Functionality First
   - Implement MelodyPlayer with tone() API
   - Create basic event system
   - Test with simple single-note feedback

2. Theme System
   - Start with StationMaster theme
   - Implement note sequences
   - Add theme switching

3. Event Integration
   - Layer change events
   - System events (connect, battery, etc)
   - Error states

### Next Steps
1. Create plugin boilerplate and MelodyPlayer
2. Implement basic event system
3. Create StationMaster theme
4. Add layer change detection

### Implementation Questions
1. How to handle timing between notes in a melody?
2. Should we use a timer for sequencing notes?
3. How to handle melody interruption?
4. What's the optimal way to store melodies in PROGMEM?

### Next Immediate Steps
1. Create plugin boilerplate
2. Implement StationMasterTheme as first theme
3. Create basic melody playback system
4. Test with layer change events first

### Implementation Progress
1. Core Structure ✓
   - Basic plugin framework implemented
   - Data-driven theme system using PROGMEM
   - Non-blocking melody playback
   - Focus API integration

2. Event System ✓
   - Layer change detection
   - Battery state monitoring
   - Connection state tracking
   - Boot sequence handling
   - Error state support

3. Theme System
   - StationMasterTheme implemented ✓
   - Template-based melody size calculation ✓
   - Efficient PROGMEM storage
   - Theme switching mechanism

4. Current Status
   - Basic framework complete
   - Need to verify hardware integration
   - Need to implement remaining themes
   - Need to add tests

### Next Steps
1. Testing
   - Create test framework
   - Verify tone generation
   - Test event system
   - Validate theme switching

2. Hardware Integration
   - Test on actual device
   - Verify battery monitoring
   - Test connection state detection
   - Measure memory usage

3. Additional Features
   - Add EEPROM support
   - Create example sketch
   - Add more themes
   - Complete documentation

### Open Questions
1. Do we need to handle melody interruption differently?
2. Should we add a priority system for events?
3. How to handle concurrent events?
4. Best way to test without hardware?
