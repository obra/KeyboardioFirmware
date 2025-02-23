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

#include "kaleidoscope/plugin/LEDIndicators.h"
#include <Arduino.h>  // for delay()
#include "kaleidoscope/plugin/LEDControl.h"
#include "kaleidoscope/plugin/LEDControl/LEDUtils.h"

namespace kaleidoscope {
namespace plugin {

// Initialize static members

cRGB LEDIndicators::color_green  = {0,255, 0};
cRGB LEDIndicators::color_red   = {255, 0, 0};
cRGB LEDIndicators::color_off   = {0, 0, 0};
cRGB LEDIndicators::color_blue  = {0, 0,255};

uint8_t LEDIndicators::num_indicator_slots  = 5;  // Default to 5 slots
KeyAddr LEDIndicators::slot_leds[MAX_SLOTS] = {
  KeyAddr(0, 0),
  KeyAddr(0, 1),
  KeyAddr(0, 2),
  KeyAddr(0, 3),
  KeyAddr(0, 4),
};  // Default LED mapping

// Initialize static indicator array
kaleidoscope::plugin::LEDIndicators::Indicator LEDIndicators::indicators_[MAX_SLOTS] = {};

// Setup the plugin
EventHandlerResult LEDIndicators::onSetup() {
  if (!Runtime.has_leds)
    return EventHandlerResult::OK;

  for (uint8_t i = 0; i < num_indicator_slots; i++) {
    uint8_t led_index = Runtime.device().getLedIndex(slot_leds[i]);
    if (led_index >= Runtime.device().led_count) {
      // Invalid LED configuration - disable that slot
      slot_leds[i] = KeyAddr::none();
    }
  }

  // Initialize all indicators to inactive
  clearAllIndicators();

  return EventHandlerResult::OK;
}

// Show a temporary indicator
void LEDIndicators::showIndicator(KeyAddr led_addr,
                                  IndicatorEffect effect,
                                  cRGB color1,
                                  cRGB color2,
                                  uint16_t duration_ms,
                                  uint16_t effect_cycles) {
  if (!Runtime.has_leds) {
    return;
  }
  if (!led_addr.isValid()) {  // Valid LED
    return;
  }

  // Find an available indicator slot
  uint8_t slot = 0xFF;
  for (uint8_t i = 0; i < MAX_SLOTS; i++) {
    if (!indicators_[i].active || indicators_[i].key_addr == led_addr) {
      slot = i;
      break;
    }
  }

  if (slot == 0xFF) {
    return;
  }

  // Set up the indicator
  indicators_[slot].active        = true;
  indicators_[slot].key_addr      = led_addr;
  indicators_[slot].effect        = effect;
  indicators_[slot].color1        = color1;
  indicators_[slot].color2        = color2;
  indicators_[slot].start_time    = Runtime.millisAtCycleStart();
  indicators_[slot].duration_ms   = duration_ms;
  indicators_[slot].effect_cycles = effect_cycles;
  indicators_[slot].current_cycle = 0;
  indicators_[slot].last_update   = Runtime.millisAtCycleStart();

  // Immediately update the indicator
  updateIndicator(slot);
}


// Clear a specific indicator
void LEDIndicators::clearIndicator(KeyAddr key_addr) {
  if (!Runtime.has_leds)
    return;

  for (uint8_t i = 0; i < MAX_SLOTS; i++) {
    if (indicators_[i].key_addr == key_addr && indicators_[i].active) {
      indicators_[i].active = false;
      ::LEDControl.refreshAt(key_addr);
    }
  }
}

// Clear all indicators
void LEDIndicators::clearAllIndicators() {
  if (!Runtime.has_leds)
    return;

  for (uint8_t i = 0; i < MAX_SLOTS; i++) {
    indicators_[i].active = false;
  }
}

// Called before LEDs are synchronized
EventHandlerResult LEDIndicators::beforeSyncingLeds() {
  if (!Runtime.has_leds) {
    return EventHandlerResult::OK;
  }

  // Update and apply all active indicators
  for (uint8_t i = 0; i < MAX_SLOTS; i++) {
    if (indicators_[i].active) {
      if (indicators_[i].duration_ms > 0 &&
          (Runtime.millisAtCycleStart() - indicators_[i].start_time >= indicators_[i].duration_ms)) {
        indicators_[i].active = false;
        ::LEDControl.refreshAt(indicators_[i].key_addr);
      } else {
        updateIndicator(i);
      }
    }
  }

  return EventHandlerResult::OK;
}

// Update an indicator's state
void LEDIndicators::updateIndicator(uint8_t index) {
  if (index >= MAX_SLOTS)
    return;

  Indicator &indicator = indicators_[index];
  if (!indicator.active)
    return;

  // Update the LED with the computed color
  cRGB color = computeCurrentColor(indicator);
  ::LEDControl.setCrgbAt(indicator.key_addr, color);
}

// Compute the current color for an indicator based on its effect
cRGB LEDIndicators::computeCurrentColor(const Indicator &indicator) {
  uint32_t elapsed = Runtime.millisAtCycleStart() - indicator.start_time;

  switch (indicator.effect) {
  case IndicatorEffect::Blink: {
    // Simple 500ms on/off
    bool on = (elapsed / 500) % 2 == 0;
    return on ? indicator.color1 : indicator.color2;
  }

  case IndicatorEffect::Breathe: {
    // Use 8-bit counter for brightness to avoid floating point
    uint8_t brightness = ((elapsed / 8) % 256);
    // Make it smooth by mirroring second half
    if (brightness >= 128) {
      brightness = 255 - brightness;
    }
    brightness = brightness * 2;  // Scale to full range

    return {
      static_cast<uint8_t>((indicator.color1.r * brightness) >> 8),
      static_cast<uint8_t>((indicator.color1.g * brightness) >> 8),
      static_cast<uint8_t>((indicator.color1.b * brightness) >> 8)};
  }

  case IndicatorEffect::Grow: {
    // Simple linear growth over 1 second
    uint16_t brightness = (elapsed > 1000) ? 255 : ((elapsed * 255) / 1000);
    return {
      static_cast<uint8_t>((indicator.color1.r * brightness) >> 8),
      static_cast<uint8_t>((indicator.color1.g * brightness) >> 8),
      static_cast<uint8_t>((indicator.color1.b * brightness) >> 8)};
  }

  case IndicatorEffect::Shrink: {
    // Simple linear decay over 1 second
    uint16_t brightness = (elapsed > 1000) ? 0 : (((1000 - elapsed) * 255) / 1000);
    return {
      static_cast<uint8_t>((indicator.color1.r * brightness) >> 8),
      static_cast<uint8_t>((indicator.color1.g * brightness) >> 8),
      static_cast<uint8_t>((indicator.color1.b * brightness) >> 8)};
  }

  default:
    return indicator.color1;
  }
}

// Handle connection status changes
EventHandlerResult LEDIndicators::onHostConnectionStatusChanged(uint8_t device_id, HostConnectionStatus status) {
  // device_id is 1-indexed (1-4) so adjust it to be zero-indexed for the slot
  uint8_t slot = device_id - 1;
  switch (status) {
  case HostConnectionStatus::Connecting:
    showIndicator(getLEDForSlot(slot),
                  IndicatorEffect::Blink,
                  color_blue,
                  color_off,
                  30000,  // 30 second duration
                  10);    // 10 cycles
    break;
  case HostConnectionStatus::Connected:
    showIndicator(getLEDForSlot(slot),
                  IndicatorEffect::Grow,
                  color_blue,
                  color_off,
                  5000,  // 5 second duration
                  1);     // 1 cycle
    break;
  case HostConnectionStatus::PairingFailed:
  case HostConnectionStatus::Disconnected:
    showIndicator(getLEDForSlot(slot),
                  IndicatorEffect::Shrink,
                  color_red,
                  color_off,
                  10000,  // 10 second duration
                  1);     // 1 cycle
    break;
  case HostConnectionStatus::PairingSuccess:
    showIndicator(getLEDForSlot(slot),
                  IndicatorEffect::Grow,
                  color_blue,
                  color_off,
                  10000,  // 10 second duration
                  1);      // 1 cycle
    break;
  case HostConnectionStatus::DeviceSelected:
    showIndicator(getLEDForSlot(slot),
                  IndicatorEffect::Grow,
                  color_blue,
                  color_off,
                  2000,  // 2 second duration
                  1);     // 1 cycle
    break;
  case HostConnectionStatus::DeviceUnselected:
    clearIndicator(getLEDForSlot(slot));
    break;
  case HostConnectionStatus::Advertising:
    showIndicator(getLEDForSlot(slot),
                  IndicatorEffect::Blink,
                  color_blue,
                  color_off,
                  30000,  // 30 second duration
                  10);   // 10 cycles
    break;
  default:
    break;
  }
  return EventHandlerResult::OK;
}


}  // namespace plugin
}  // namespace kaleidoscope

kaleidoscope::plugin::LEDIndicators LEDIndicators;
