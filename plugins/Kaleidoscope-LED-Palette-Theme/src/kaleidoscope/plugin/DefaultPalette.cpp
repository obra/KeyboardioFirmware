/* Kaleidoscope-LED-Palette-Theme -- Palette-based LED theme foundation
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

#include "kaleidoscope/plugin/DefaultPalette.h"
#include "kaleidoscope/plugin/LED-Palette-Theme.h"  // for LEDPaletteTheme

#include <stdint.h>  // for uint8_t

namespace kaleidoscope {
namespace plugin {

namespace ledpalette {
__attribute__((weak)) extern const cRGB palette[] = {};
__attribute__((weak)) bool palette_defined        = false;
}  // namespace ledpalette

void DefaultPalette::setup() {
  if (!ledpalette::palette_defined) return;

  for (uint8_t i = 0; i < LEDPaletteTheme::getPaletteSize(); i++) {
    cRGB color;

    color.r = pgm_read_byte(&(ledpalette::palette[i].r));
    color.g = pgm_read_byte(&(ledpalette::palette[i].g));
    color.b = pgm_read_byte(&(ledpalette::palette[i].b));

    ::LEDPaletteTheme.updatePaletteColor(i, color);
  }
}

}  // namespace plugin
}  // namespace kaleidoscope

kaleidoscope::plugin::DefaultPalette DefaultPalette;
