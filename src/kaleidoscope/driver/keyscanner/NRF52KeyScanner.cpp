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

#ifdef ARDUINO_ARCH_NRF52

#include "NRF52KeyScanner.h"

namespace kaleidoscope {
namespace driver {
namespace keyscanner {

// Initialize static pointer to active scanner
TimerHandlerInterface* active_scanner_ = nullptr;

} // namespace keyscanner
} // namespace driver
} // namespace kaleidoscope

// Timer1 IRQ handler implementation
extern "C" {
void TIMER1_IRQHandler(void) {
  if (NRF_TIMER1->EVENTS_COMPARE[0]) {
    NRF_TIMER1->EVENTS_COMPARE[0] = 0;
    if (kaleidoscope::driver::keyscanner::active_scanner_) {
      kaleidoscope::driver::keyscanner::active_scanner_->handleTimer();
    }
  }
}
}

#endif // ARDUINO_ARCH_NRF52
