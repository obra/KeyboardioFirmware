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

#include "kaleidoscope/driver/speaker/Base.h"

namespace kaleidoscope {
namespace driver {
namespace speaker {

struct PiezoProps : public BaseProps {
  static constexpr int8_t pin = -1;  // Default to -1, meaning no speaker
};

template<typename _SpeakerProps>
class Piezo : public Base<_SpeakerProps> {
  static_assert(_SpeakerProps::pin >= 0,
                "SpeakerProps must have a pin defined");

 public:
  Piezo()
    : Base<_SpeakerProps>() {}

  void setup() {
    if (_SpeakerProps::pin != -1) {
      pinMode(_SpeakerProps::pin, OUTPUT);
    }
  }

  void playTone(unsigned int frequency, uint32_t duration) {
    if (_SpeakerProps::pin != -1) {
      ::tone(_SpeakerProps::pin, frequency, duration);
    }
  }

  void playTone(unsigned int frequency) {
    if (_SpeakerProps::pin != -1) {
      ::tone(_SpeakerProps::pin, frequency);  // Arduino's tone() without duration plays until noTone()
    }
  }

  void stopTone() {
    if (_SpeakerProps::pin != -1) {
      ::noTone(_SpeakerProps::pin);
    }
  }

  bool isPlaying() {
    // This is a simplification, as Arduino's tone() function is blocking
    // In a real implementation, we'd need to track the playing state
    return false;
  }
};

}  // namespace speaker
}  // namespace driver
}  // namespace kaleidoscope
