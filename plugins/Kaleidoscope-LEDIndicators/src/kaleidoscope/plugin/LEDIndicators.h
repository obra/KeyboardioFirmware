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



#pragma once

#include <stdint.h>  // for uint8_t

#include "kaleidoscope/plugin.h"
#include "kaleidoscope/host_connection_status.h"
#include "kaleidoscope/Runtime.h"
#include "kaleidoscope/plugin/LEDControl.h"
#include "kaleidoscope/KeyAddr.h"  // for KeyAddr::none()

namespace kaleidoscope {
namespace plugin {

/**
 * @brief Enum for indicator effects
 */
enum class IndicatorEffect {
  Solid,    ///< Single color, constant brightness
  Blink,    ///< Alternates between two colors
  Breathe,  ///< Smoothly transitions between two brightnesses of the same color
  Grow,     ///< Starts dim, breathes to full brightness, stays bright
  Shrink    ///< Starts bright, breathes to dim, stays dim
};

/**
 * @brief Class for managing LED indicators
 */
class LEDIndicators : public kaleidoscope::Plugin {
 public:
  LEDIndicators() {}

  // Colors can be configured by the user
  static cRGB color_blue;
  static cRGB color_red;
  static cRGB color_off;
  static cRGB color_green;

  /** @brief Configure the number of indicator slots and their LED mapping
   * @param num_slots Number of slots to use (must be <= MAX_SLOTS)
   * @param leds Array of LED addresses to use for the slots
   * @return true if configuration was successful, false if invalid parameters
   * 
   * Each LED address must be valid for the current device.
   * The number of slots must not exceed MAX_SLOTS.
   * Invalid configurations will be rejected and return false.
   */
  static bool setSlots(uint8_t num_slots, const KeyAddr *leds) {
    if (num_slots > MAX_SLOTS || leds == nullptr)
      return false;

    for (uint8_t i = 0; i < num_slots; i++) {
      uint8_t led_index = Runtime.device().getLedIndex(leds[i]);
      if (led_index >= Runtime.device().led_count)
        return false;
    }

    num_indicator_slots = num_slots;
    for (uint8_t i = 0; i < num_slots; i++) {
      slot_leds[i] = leds[i];
    }

    return true;
  }

  /** @brief Configure an indicator color
   * @param target_color Reference to the color to configure
   * @param new_color The new RGB color value
   */
  static void setColor(cRGB &target_color, const cRGB &new_color) {
    target_color = new_color;
  }

  /** @brief Show a temporary indicator with optional duration
   * @param key_addr The LED index to control
   * @param effect The type of visual effect to show
   * @param color1 Primary color for the effect
   * @param color2 Secondary color for effects that use two colors
   * @param duration_ms How long to show the indicator (0 for indefinite)
   * @param effect_cycles Number of times to repeat the effect
   */
  void showIndicator(KeyAddr key_addr,
                     IndicatorEffect effect     = IndicatorEffect::Solid,
                     cRGB color1                = {0, 0, 0},
                     cRGB color2                = {0, 0, 0},
                     uint16_t duration_ms       = 0,
                     uint16_t effect_cycles     = 0);

  /** @brief Clear a specific indicator
   * @param key_addr The LED index to clear
   */
  void clearIndicator(KeyAddr key_addr);

  /** @brief Clear all active indicators */
  void clearAllIndicators();

  /** @brief Get the LED index for a given slot
   * @param slot The slot number
   * @return The LED index, or KeyAddr::none() if slot is invalid
   */
  KeyAddr getLEDForSlot(uint8_t slot) const {
    // Check slot bounds
    if (slot >= num_indicator_slots) {
      return KeyAddr::none();
    }

    // Get the mapped LED address
    KeyAddr led_addr = slot_leds[slot];
   
    // Validate the LED exists on this device
    uint8_t led_index = Runtime.device().getLedIndex(led_addr);
    
    if (led_index >= Runtime.device().led_count) {
      return KeyAddr::none();
    }

    return led_addr;
  }

  // Plugin hooks
  EventHandlerResult onSetup();
  EventHandlerResult beforeSyncingLeds();
  EventHandlerResult onHostConnectionStatusChanged(uint8_t device_id, HostConnectionStatus status);

 private:
  static constexpr uint8_t MAX_SLOTS = 8;  // Maximum number of slots supported
  static uint8_t num_indicator_slots;      // Number of slots configured
  static KeyAddr slot_leds[MAX_SLOTS];     // LED mapping for slots

  struct Indicator {
    bool active = false;
    KeyAddr key_addr;
    cRGB color1;
    cRGB color2;
    IndicatorEffect effect;
    uint32_t start_time;
    uint16_t duration_ms;
    uint16_t effect_cycles;
    uint32_t last_update;
    uint16_t current_cycle;
  };

  void updateIndicator(uint8_t index);
  cRGB computeCurrentColor(const Indicator &indicator);

  static Indicator indicators_[MAX_SLOTS];
};

}  // namespace plugin
}  // namespace kaleidoscope

extern kaleidoscope::plugin::LEDIndicators LEDIndicators;
