# SonicThemes Plugin Development Todo

## Phase 1: Core Infrastructure ✓
- [x] Create basic plugin structure in plugins/Kaleidoscope-SonicThemes/
- [x] Define core interfaces and types for sound generation
- [x] Implement basic piezo speaker control
- [x] Create theme switching mechanism
- [ ] Test basic functionality

## Phase 2: Event System
- [x] Define event types and handlers
- [x] Implement layer change detection
- [x] Implement connection state monitoring
- [x] Implement battery state monitoring
- [x] Add boot sequence detection
- [x] Add error state handling
- [ ] Test all event handlers

## Phase 3: Theme Implementation
- [x] Design and implement "Station Master" theme
- [ ] Design and implement "Retro Gaming" theme
- [ ] Design and implement "Speech Synthesis" theme
- [ ] Design and implement "Minimal" theme
- [ ] Design and implement "Sci-Fi" theme

## Phase 4: Testing & Documentation
- [ ] Create unit test framework
- [ ] Write tests for core functionality
- [ ] Write tests for each theme
- [ ] Write user documentation
- [ ] Write developer documentation

## Phase 5: Optimization
- [ ] Optimize memory usage
- [ ] Measure and optimize performance
- [ ] Test on actual hardware
- [ ] Handle edge cases and hardware limitations

## Current Implementation Tasks
- [x] Reorganize files into correct plugin structure
  - [x] Create library.properties
  - [x] Move MelodyPlayer to separate files
  - [x] Move Theme to separate file
  - [x] Create proper include hierarchy
  - [x] Update include paths

- [x] Complete StationMasterTheme
  - [x] Add disconnect melody
  - [x] Add charging melodies
  - [x] Add battery warning melody
  - [x] Add boot melody
  - [x] Add error melody

- [x] Add Focus API Support
  - [x] Add theme switching commands
  - [x] Add enable/disable commands
  - [ ] Add volume control (if possible)

- [ ] Testing
  - [ ] Create basic test framework
  - [ ] Test tone generation
  - [ ] Test melody sequencing
  - [ ] Test theme switching

## New Tasks
- [ ] Verify PROGMEM usage is correct
- [ ] Add error handling for hardware initialization
- [ ] Test battery/connection state detection
- [ ] Create example sketch
- [ ] Add EEPROM support for persistent settings
