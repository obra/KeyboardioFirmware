/* Kaleidoscope-Colormap -- Per-layer colormap effect
 * Copyright 2022-2025 Keyboard.io, inc.
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

#include "kaleidoscope/plugin/Colormap.h"  // for Colormap
#include "kaleidoscope/plugin/DefaultColormap.h"

#include <Arduino.h>                   // for PSTR
#include <Kaleidoscope-FocusSerial.h>  // for Focus
#include <Kaleidoscope-LEDControl.h>   // for LEDControl
#include <stdint.h>                    // for uint8_t

#include "kaleidoscope/Runtime.h"  // for Runtime
#include "kaleidoscope/plugin/DefaultPalette.h"

namespace kaleidoscope {
namespace plugin {

namespace defaultcolormap {
__attribute__((weak)) extern const uint8_t colormaps[][kaleidoscope_internal::device.matrix_rows * kaleidoscope_internal::device.matrix_columns] = {};
__attribute__((weak)) uint8_t colormap_layers                                                                                                    = 0;
}  // namespace defaultcolormap

void DefaultColormap::setup() {
  // If the colormap is already initialized, return early.
  if (!::ColormapEffect.isUninitialized())
    return;

  install();
}

void DefaultColormap::install() {
  DefaultPalette::setup();

  if (defaultcolormap::colormap_layers == 0) return;

  for (uint8_t layer = 0; layer < defaultcolormap::colormap_layers; layer++) {
    for (int8_t i = 0; i < Runtime.device().numKeys(); i++) {
      const int8_t post = Runtime.device().ledDriver().getLedIndex(i);
      const uint8_t idx = pgm_read_byte(&(defaultcolormap::colormaps[layer][i]));
      ::ColormapEffect.updateColorIndexAtPosition(layer, post, idx);
    }
  }
  Runtime.storage().commit();

  ::LEDControl.refreshAll();
}

EventHandlerResult DefaultColormap::onFocusEvent(const char *input) {
  if (!Runtime.has_leds)
    return EventHandlerResult::OK;

  const char *cmd = PSTR("colormap.install");

  if (::Focus.inputMatchesHelp(input))
    return ::Focus.printHelp(cmd);

  if (!::Focus.inputMatchesCommand(input, cmd))
    return EventHandlerResult::OK;

  install();
  return EventHandlerResult::EVENT_CONSUMED;
}

}  // namespace plugin
}  // namespace kaleidoscope

kaleidoscope::plugin::DefaultColormap DefaultColormap;
