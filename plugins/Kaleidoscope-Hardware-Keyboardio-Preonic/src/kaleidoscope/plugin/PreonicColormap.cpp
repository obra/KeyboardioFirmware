/* Kaleidoscope-Hardware-Keyboardio-Preonic -- Keyboardio Preonic hardware support for Kaleidoscope
 * Copyright 2016-2025 Keyboard.io, inc.
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

#include "kaleidoscope/plugin/PreonicColormap.h"

#include <Arduino.h>                         // for F, PSTR, __FlashStringHelper
#include <Kaleidoscope-FocusSerial.h>        // for Focus, FocusSerial
#include <Kaleidoscope-LED-Palette-Theme.h>  // for LEDPaletteTheme
#include <stdint.h>                          // for uint8_t, uint16_t

#include "kaleidoscope/KeyAddr.h"               // for KeyAddr
#include "kaleidoscope/Runtime.h"               // for Runtime, Runtime_
#include "kaleidoscope/event_handler_result.h"  // for EventHandlerResult, EventHandlerResult::OK
#include "kaleidoscope/layers.h"                // for Layer, Layer_
#include "kaleidoscope/plugin/LEDControl.h"     // for LEDControl

namespace kaleidoscope {
namespace plugin {

uint16_t PreonicColormapEffect::map_base_;
uint8_t PreonicColormapEffect::max_layers_;
uint8_t PreonicColormapEffect::top_layer_;

void PreonicColormapEffect::max_layers(uint8_t max_) {
  if (map_base_ != 0)
    return;

  max_layers_ = max_;
  map_base_   = ::LEDPaletteTheme.reserveThemes(max_layers_);
}

EventHandlerResult PreonicColormapEffect::onNameQuery() {
  return ::Focus.sendName(F("PreonicColormapEffect"));
}

bool PreonicColormapEffect::isUninitialized() {
  return ::LEDPaletteTheme.isThemeUninitialized(map_base_, max_layers_);
}

void PreonicColormapEffect::updateColorIndexAtPosition(uint8_t layer, uint16_t position, uint8_t palette_index) {
  if (layer >= max_layers_) return;

  uint16_t index = Runtime.device().led_count * layer + position;
  ::LEDPaletteTheme.updateColorIndexAtPosition(map_base_, index, palette_index);
}

void PreonicColormapEffect::TransientLEDMode::onActivate() {
  if (!Runtime.has_leds)
    return;

  parent_->top_layer_ = Layer.mostRecent();
  if (parent_->top_layer_ <= parent_->max_layers_)
    ::LEDPaletteTheme.updateHandler(parent_->map_base_, parent_->top_layer_);
}

void PreonicColormapEffect::TransientLEDMode::refreshAt(KeyAddr key_addr) {
  if (parent_->top_layer_ <= parent_->max_layers_)
    ::LEDPaletteTheme.refreshAt(parent_->map_base_, parent_->top_layer_, key_addr);
}

EventHandlerResult PreonicColormapEffect::onLayerChange() {
  if (::LEDControl.get_mode_index() == led_mode_id_)
    ::LEDControl.get_mode<TransientLEDMode>()->onActivate();
  return EventHandlerResult::OK;
}

EventHandlerResult PreonicColormapEffect::onFocusEvent(const char *input) {
  return ::LEDPaletteTheme.themeFocusEvent(input, PSTR("colormap.map"), map_base_, max_layers_);
}

}  // namespace plugin
}  // namespace kaleidoscope

kaleidoscope::plugin::PreonicColormapEffect PreonicColormapEffect;
