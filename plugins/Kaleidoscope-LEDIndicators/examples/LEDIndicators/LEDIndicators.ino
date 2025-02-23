/* Kaleidoscope-LEDIndicators -- LED indicators for connection status and other events
 * Copyright 2024-2025 Keyboard.io, inc.
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

#include "Kaleidoscope.h"
#include "Kaleidoscope-LEDControl.h"
#include "Kaleidoscope-LEDIndicators.h"

// Configure LED indicators before setup
void configureIndicators() {
  // Configure colors (optional - these are the defaults)
  LEDIndicators.setColor(LEDIndicators.color_blue, {0, 0, 255});   // Blue for connecting/connected
  LEDIndicators.setColor(LEDIndicators.color_red, {255, 0, 0});    // Red for errors/disconnected
  LEDIndicators.setColor(LEDIndicators.color_green, {0, 255, 0});  // Green for success
  LEDIndicators.setColor(LEDIndicators.color_off, {0, 0, 0});      // Off state

  // Configure LED mapping - use specific LED positions that make sense for your keyboard
  // These should be LEDs that are visible and make sense as status indicators
  static const KeyAddr indicator_leds[] = {
    KeyAddr(3, 0),  // Top row, 4th key - typically visible
    KeyAddr(3, 1),  // Top row, 5th key
    KeyAddr(3, 2),  // Top row, 6th key
    KeyAddr(3, 3),  // Top row, 7th key
    KeyAddr(3, 4)   // Top row, 8th key
  };

  // Configure the slots
  if (!LEDIndicators.setSlots(5, indicator_leds)) {
    // Configuration failed - LEDs invalid for this device
    // You should adjust the LED addresses above to match your keyboard
  }
}

KALEIDOSCOPE_INIT_PLUGINS(
  LEDControl,
  LEDIndicators);

void setup() {
  Kaleidoscope.setup();
  configureIndicators();
}

void loop() {
  Kaleidoscope.loop();
}
