/* Kaleidoscope-Hardware-Keyboardio-Preonic -- Keyboardio Preonic hardware support for Kaleidoscope
 * Copyright 2013-2025 Keyboard.io, inc.
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



/**
 * PPI-based GPIO PORT event handler
 * 
 * This implementation is based on the solution by jgartrel:
 * https://github.com/adafruit/Adafruit_nRF52_Arduino/issues/754#issuecomment-1437329605
 * 
 * This file implements the SWI3_EGU3 interrupt handler that receives GPIO PORT events
 * via the PPI (Programmable Peripheral Interconnect) system. This approach allows us
 * to handle PORT events without conflicting with the Arduino core's GPIOTE handler.
 * 
 * The flow is:
 * 1. A key is pressed or rotary encoder turned, pulling a GPIO pin LOW
 * 2. This triggers a PORT event in the GPIOTE peripheral
 * 3. Via PPI, the PORT event triggers EGU3 channel 0
 * 4. This causes the SWI3_EGU3_IRQHandler to be called
 * 5. The handler calls setInputEventPending which tells Kaleidoscope to start scanning for events again
 * 
 * Note: The handler must be declared with extern "C" to prevent name mangling,
 * ensuring it matches the interrupt vector table entry exactly.
 */

#include "Preonic.h"

extern "C" void SWI3_EGU3_IRQHandler(void) {
  // Disable interrupts to prevent race conditions
  noInterrupts();

#if CFG_SYSVIEW
  SEGGER_SYSVIEW_RecordEnterISR();
#endif

  // Call our handler to process the wakeup
  kaleidoscope::device::keyboardio::Preonic::setInputEventPending();

  // Clear both event registers to prevent repeated triggers:
  // - EGU3 event that caused this interrupt
  // - GPIOTE PORT event that triggered the EGU3 event
  NRF_EGU3->EVENTS_TRIGGERED[0] = 0;
  NRF_GPIOTE->EVENTS_PORT       = 0;

#if CFG_SYSVIEW
  SEGGER_SYSVIEW_RecordExitISR();
#endif

  // Re-enable interrupts
  interrupts();
}
