/* Kaleidoscope - Firmware for computer input devices
 * Copyright (C) 2019-2025 Keyboard.io, inc.
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

#ifndef CRGB
#error cRGB and CRGB *must* be defined before including this header!
#endif

#include <Arduino.h>  // for PROGMEM, pgm_read_byte
#include <stdint.h>   // for uint8_t

namespace kaleidoscope {
namespace driver {
namespace led {

constexpr uint8_t no_led = uint8_t(-1);

struct BaseProps {
  static constexpr uint8_t led_count = 0;

  // C++ does not allow empty constexpr arrays
  //
  static constexpr uint8_t key_led_map[] PROGMEM = {no_led};
};

template<typename _LEDDriverProps>
class Base {
 public:
  Base() : active_leds_(0), last_led_activity_time_(0) {}

  void setup() {}
  void syncLeds(void) {}
  void setCrgbAt(uint8_t i, cRGB color) {}
  cRGB getCrgbAt(uint8_t i) {
    cRGB c = {
      0, 0, 0};
    return c;
  }
  void setBrightness(uint8_t brightness) {}
  uint8_t getBrightness() {
    return 255;
  }

  /**
   * @returns true if any LEDs are currently lit (non-black)
   */
  bool areAnyLEDsOn() const {
    return active_leds_ > 0;
  }

  /**
   * @returns timestamp in milliseconds when LEDs were last turned on or off
   */
  uint32_t LEDsLastOn() const {
    return last_led_activity_time_;
  }

  /**
   * @brief Update the LED activity state for a single LED
   * 
   * This method tracks when individual LEDs turn on or off, maintaining
   * a count of active LEDs and updating the last activity timestamp.
   * 
   * @param will_be_on true if the LED will be turned on
   * @param was_off true if the LED was previously off
   */
  void updateLEDState(bool will_be_on, bool was_off) {
    if (will_be_on && was_off) {
      active_leds_++;
      last_led_activity_time_ = millis();
    } else if (!will_be_on && !was_off) {
      active_leds_--;
      last_led_activity_time_ = millis();
    }
  }

  /**
   * @brief Update the LED activity state for all LEDs
   * 
   * This method is used when setting all LEDs to the same state at once.
   * It updates the active LED count and last activity timestamp accordingly.
   * 
   * @param will_be_on true if the LEDs will be turned on
   * @param was_off true if all LEDs were previously off
   */
  void updateAllLEDState(bool will_be_on, bool was_off) {
    if (will_be_on && was_off) {
      active_leds_ = _LEDDriverProps::led_count;
      last_led_activity_time_ = millis();
    } else if (!will_be_on) {
      active_leds_ = 0;
      last_led_activity_time_ = millis();
    }
  }

  static uint8_t getLedIndex(uint8_t key_offset) {

    // Give the compiler the oportunity to optimize
    // for boards without LEDs.
    //
    if (_LEDDriverProps::led_count == 0) {
      return no_led;
    }

    if (key_offset >= sizeof(_LEDDriverProps::key_led_map)) {
      return no_led;
    }

    return pgm_read_byte(&_LEDDriverProps::key_led_map[key_offset]);
  }

  class LEDRangeIterator {
   private:
    uint8_t offset_;

   public:
    LEDRangeIterator()
      : offset_(0) {}
    explicit LEDRangeIterator(uint8_t offset)
      : offset_(offset) {}

    typedef LEDRangeIterator ThisType;

    constexpr uint8_t offset() const {
      return offset_;
    }

    ThisType operator++() {
      ++offset_;
      return *this;
    }

    ThisType operator++(int) {  // postfix ++
      ThisType copy(*this);
      ++*this;  // call the prefix increment
      return copy;
    }

    ThisType operator--() {
      --offset_;
      return *this;
    }

    ThisType operator--(int) {  // postfix ++
      ThisType copy(*this);
      --*this;  // call the prefix increment
      return copy;
    }

    bool operator==(const ThisType &other) const {
      return offset_ == other.offset_;
    }

    bool operator!=(const ThisType &other) const {
      return offset_ != other.offset_;
    }

    struct Range {
      typedef LEDRangeIterator Iterator;
      static constexpr ThisType begin() {
        return ThisType(uint8_t(0));
      }
      static constexpr ThisType end() {
        return ThisType(_LEDDriverProps::led_count);
      }
    };

    static constexpr Range all() {
      return Range{};
    }
    constexpr const ThisType &operator*() const {
      return *this;
    }
    constexpr bool isValid(uint8_t index) const {
      return (_LEDDriverProps::led_count > 0 && index < _LEDDriverProps::led_count);
    }
  };

  static LEDRangeIterator &LEDs() {
    static LEDRangeIterator leds;
    return leds;
  }

 protected:
  typedef _LEDDriverProps Props_;
    /** Number of LEDs that are currently lit (non-black) */
  uint16_t active_leds_;
  
  /** Timestamp of the last LED state change in milliseconds */
  uint32_t last_led_activity_time_;
};

}  // namespace led
}  // namespace driver
}  // namespace kaleidoscope
