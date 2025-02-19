/* Kaleidoscope-Hardware-Keyboardio-Preonic -- Keyboardio Preonic hardware support for Kaleidoscope
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

#pragma once

#include <Arduino.h>  // for PROGMEM
#include <stdint.h>   // for uint8_t

#include "kaleidoscope_internal/device.h"       // for device
#include "kaleidoscope/event_handler_result.h"  // for EventHandlerResult
#include "kaleidoscope/plugin.h"                // for Plugin

namespace kaleidoscope {
namespace plugin {

// clang-format off

#define COLORMAPS(layers...)                                            \
  namespace kaleidoscope {                                              \
  namespace plugin {                                                    \
  namespace defaultcolormap {                                           \
  const uint8_t colormaps[][kaleidoscope_internal::device.numLEDs()] PROGMEM = { \
      layers                                                            \
  };                                                                    \
  uint8_t colormap_layers =                                             \
      sizeof(colormaps) / sizeof(*colormaps);                           \
  } /* defaultcolormap */                                               \
  } /* plugin */                                                        \
  } /* kaleidoscope */

#define __IDENTITY__(X) X



#ifdef PER_LED_DATA
#define COLORMAP(...)                                    \
  {                                                      \
    MAP_LIST(__IDENTITY__, PER_LED_DATA(0, __VA_ARGS__)) \
  }
#endif

// clang-format on

namespace defaultcolormap {
extern const uint8_t colormaps[][kaleidoscope_internal::device.numLEDs()];
extern uint8_t colormap_layers;
}  // namespace defaultcolormap

class DefaultPreonicColormap : public Plugin {
 public:
  static void setup();
  static void install();

  EventHandlerResult onFocusEvent(const char *input);
};

}  // namespace plugin
}  // namespace kaleidoscope

extern kaleidoscope::plugin::DefaultPreonicColormap DefaultPreonicColormap;
