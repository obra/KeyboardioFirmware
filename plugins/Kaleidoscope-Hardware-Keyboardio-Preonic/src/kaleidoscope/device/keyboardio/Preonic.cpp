/* Kaleidoscope-Hardware-Keyboardio-Preonic -- Keyboardio Preonic hardware support for Kaleidoscope
 * Copyright 2021-2025 Keyboard.io, inc.
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

#ifdef ARDUINO_PREONIC

#include <Arduino.h>
#include <Wire.h>                                      // For some reason, not including this breaks bluetooth pairing/connecting
#include "kaleidoscope/driver/keyscanner/Base_Impl.h"  // For Base<>
#include "kaleidoscope/driver/speaker/Base_Impl.h"     // For Base<>

#include "kaleidoscope/device/keyboardio/Preonic.h"
#include "kaleidoscope/Runtime.h"


// Here, we set up aliases to the device's KeyScanner and KeyScannerProps
// in the global namespace within the scope of this file. We'll use these
// aliases to simplify some template initialization code below.
using KeyScannerProps = typename kaleidoscope::device::keyboardio::PreonicProps::KeyScannerProps;
using KeyScanner      = typename kaleidoscope::device::keyboardio::PreonicProps::KeyScanner;
using LEDDriverProps  = typename kaleidoscope::device::keyboardio::PreonicProps::LEDDriverProps;
using SpeakerProps    = typename kaleidoscope::device::keyboardio::PreonicProps::SpeakerProps;

// Template instantiation for the speaker driver - must be in global namespace
namespace kaleidoscope {
namespace driver {
namespace speaker {
template class Base<kaleidoscope::device::keyboardio::PreonicSpeakerProps>;
}  // namespace speaker
}  // namespace driver
}  // namespace kaleidoscope

namespace kaleidoscope {
namespace device {
namespace keyboardio {

// `KeyScannerProps` here refers to the alias set up above. We do not need to
// prefix the `matrix_rows` and `matrix_columns` names within the array
// declaration, because those are resolved within the context of the class, so
// the `matrix_rows` in `KeyScannerProps::matrix_row_pins[matrix_rows]` gets
// resolved as `KeyScannerProps::matrix_rows`.
const uint8_t KeyScannerProps::matrix_rows;
const uint8_t KeyScannerProps::matrix_columns;
constexpr uint8_t KeyScannerProps::matrix_row_pins[matrix_rows];
constexpr uint8_t KeyScannerProps::matrix_col_pins[matrix_columns];
constexpr uint8_t LEDDriverProps::key_led_map[];

// Initialize static members
Preonic::TimerState Preonic::timer_state_ = {};
Preonic::TWIState Preonic::twi_state_ = {};
uint32_t Preonic::last_activity_time_ = 0;
volatile bool Preonic::input_event_pending_ = false;

// Battery state tracking
uint8_t Preonic::last_battery_level_ = 0;

//// `KeyScanner` here refers to the alias set up above, just like in the
//// `KeyScannerProps` case above.
template<>
KeyScanner::row_state_t
  kaleidoscope::driver::keyscanner::NRF52KeyScanner<kaleidoscope::device::keyboardio::PreonicKeyScannerProps>::matrix_state_
    [KeyScannerProps::matrix_rows] = {};

template<>
uint32_t kaleidoscope::driver::keyscanner::NRF52KeyScanner<kaleidoscope::device::keyboardio::PreonicKeyScannerProps>::next_scan_at_ = 0;

}  // namespace keyboardio
}  // namespace device
}  // namespace kaleidoscope

#endif /* ARDUINO_ARCH_NRF52 */
