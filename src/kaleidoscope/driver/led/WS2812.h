/* Kaleidoscope - Firmware for computer input devices
 *
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

#include "kaleidoscope/driver/led/Base.h"
#include "kaleidoscope/driver/led/Color.h"
#include <Adafruit_NeoPixel.h>

namespace kaleidoscope {
namespace driver {
namespace led {

struct WS2812Props : public BaseProps {
  static constexpr uint8_t led_count = 0;  // Should be set by the user
  static constexpr uint8_t pin       = 0;  // Should be set by the user
  // key_led_map should be defined by the user
};


template<typename _LEDDriverProps>
class WS2812 : public Base<_LEDDriverProps> {
 private:
  Adafruit_NeoPixel pixels;
  bool modified_ = false;

 public:
  WS2812()
    : pixels(_LEDDriverProps::led_count, _LEDDriverProps::pin, NEO_GRB + NEO_KHZ800) {
  }

  void setup() {
    pixels.begin();
    pixels.show();             // Initialize all pixels to 'off'
    pixels.setBrightness(50);  // Set initial brightness

    for (int i = 0; i < _LEDDriverProps::led_count; i++) {
      pixels.setPixelColor(i, pixels.Color(0, 150, 150));
    }
    modified_ = true;

    syncLeds();
  }


  void syncLeds() {
    if (modified_) {
      pixels.show();
      modified_ = false;
    }
  }

  void setCrgbAt(uint8_t i, cRGB color) {

    pixels.setPixelColor(i, pixels.Color(color.r, color.g, color.b));
    modified_ = true;
  }

  cRGB getCrgbAt(uint8_t i) {
    uint32_t color = pixels.getPixelColor(i);
    cRGB rgb;
    rgb.r = (color >> 16) & 0xFF;
    rgb.g = (color >> 8) & 0xFF;
    rgb.b = color & 0xFF;
    return rgb;
  }

  void setBrightness(uint8_t brightness) {
    pixels.setBrightness(brightness);
    modified_ = true;
  }

  uint8_t getBrightness() {
    return pixels.getBrightness();
  }
};

}  // namespace led
}  // namespace driver
}  // namespace kaleidoscope
