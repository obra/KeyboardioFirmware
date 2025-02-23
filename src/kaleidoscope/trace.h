/* Kaleidoscope - Firmware for computer input devices
 * Copyright (C) 2013-2025 Keyboard.io, inc.
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

#pragma once

#include "Arduino.h"

namespace kaleidoscope {
namespace util {

// Debug output mode configuration
// NONE,     // No debug output
// SERIAL,   // Output to Arduino Serial
// LOG_LV2   // Use LOG_LV2 macro (requires nRF SDK)

// Set this to control debug output mode
#ifndef DEBUG_TRACE_MODE
#define DEBUG_TRACE_MODE NONE
#endif

// Helper function to check if Serial is ready
inline bool isSerialReady() {
  return Serial && Serial.dtr();
}

// Main debug trace macro
#if DEBUG_TRACE_MODE == NONE
  #define DEBUG_TRACE(tag, fmt, ...)
#elif DEBUG_TRACE_MODE == SERIAL
  #define DEBUG_TRACE(tag, fmt, ...) do { \
    if (kaleidoscope::util::isSerialReady()) { \
      Serial.print("["); \
      Serial.print(tag); \
      Serial.print("] "); \
      Serial.printf(fmt, ##__VA_ARGS__); \
      Serial.println(); \
    } \
  } while (0)
#else // DebugMode::LOG_LV2
  #define DEBUG_TRACE(tag, fmt, ...) LOG_LV2(tag, fmt, ##__VA_ARGS__)
#endif

} // namespace util
} // namespace kaleidoscope
