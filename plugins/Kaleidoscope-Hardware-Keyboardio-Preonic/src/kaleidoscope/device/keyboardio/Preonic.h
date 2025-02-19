/* Kaleidoscope-Hardware-Keyboardio-Preonic -- Keyboardio Preonic hardware support for Kaleidoscope
 * Copyright 2017-2025 Keyboard.io, inc.
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

#ifdef ARDUINO_PREONIC

#define KALEIDOSCOPE_HYBRID 1

#include <Arduino.h>

#define CRGB(r, g, b) \
  (cRGB) {            \
    b, g, r           \
  }

struct cRGB {
  uint8_t b;
  uint8_t g;
  uint8_t r;
};

#include "RotaryEncoder.h"

#include "kaleidoscope/device/Base.h"
#include "kaleidoscope/driver/bootloader/nrf52/uf2.h"

#include "kaleidoscope/driver/hid/Base.h"
#include "kaleidoscope/driver/hid/Bluefruit.h"
#include "kaleidoscope/driver/hid/Hybrid.h"
#include "kaleidoscope/driver/hid/TinyUSB.h"
#include "kaleidoscope/driver/keyscanner/Base.h"
#include "kaleidoscope/driver/keyscanner/NRF52KeyScanner.h"
#include "kaleidoscope/driver/led/WS2812.h"
#include "kaleidoscope/driver/mcu/TinyUSB.h"
#include "kaleidoscope/driver/storage/NRF52Flash.h"
#include "kaleidoscope/driver/ble/Bluefruit.h"
#include "kaleidoscope/driver/speaker/Piezo.h"
#include "nrfx_gpiote.h"
#include "kaleidoscope/driver/battery_gauge/MAX17048.h"

namespace kaleidoscope {
namespace device {
namespace keyboardio {

using Color = kaleidoscope::driver::led::color::GRB;

// Structure to define rotary encoder configuration
struct EncoderConfig {
  uint8_t pinA;
  uint8_t pinB;
  struct {
    uint8_t row;
    uint8_t col;
  } ccw;  // Counter-clockwise key address
  struct {
    uint8_t row;
    uint8_t col;
  } cw;  // Clockwise key address
};

// Move encoder definitions to namespace scope for shared access
static constexpr size_t NUM_ENCODERS = 3;
static const EncoderConfig ENCODER_CONFIGS[NUM_ENCODERS] = {
  {PIN_ENC1_A, PIN_ENC1_B, {0, 0}, {0, 1}},  // Encoder 1
  {PIN_ENC2_A, PIN_ENC2_B, {0, 2}, {0, 3}},  // Encoder 2
  {PIN_ENC3_A, PIN_ENC3_B, {0, 4}, {0, 5}}   // Encoder 3
};

// Custom keyscanner for Preonic that adds rotary encoder support
template<typename _KeyScannerProps>
class PreonicKeyScanner : public kaleidoscope::driver::keyscanner::NRF52KeyScanner<_KeyScannerProps> {
 private:
  typedef kaleidoscope::driver::keyscanner::NRF52KeyScanner<_KeyScannerProps> Parent;
  static int last_encoder_values_[NUM_ENCODERS];
  SwRotaryEncoder encoders_[NUM_ENCODERS];

 public:
  void setup() {
    Parent::setup();
    initializeEncoders();
  }

  /// @brief Reinitialize encoder pins and state
  void reinitializeEncoders() {
    initializeEncoders();
  }

  void scanMatrix() {
    Parent::scanMatrix();
    postReadMatrix();
  }



 private:
  void initializeEncoders() {
    for (size_t i = 0; i < NUM_ENCODERS; i++) {
      pinMode(ENCODER_CONFIGS[i].pinA, INPUT_PULLUP);
      pinMode(ENCODER_CONFIGS[i].pinB, INPUT_PULLUP);
      encoders_[i].begin(ENCODER_CONFIGS[i].pinA, ENCODER_CONFIGS[i].pinB);
      last_encoder_values_[i] = encoders_[i].read();
    }
  }

 protected:
  void postReadMatrix() override {
    for (size_t i = 0; i < NUM_ENCODERS; i++) {
      int current_value = encoders_[i].read();
      if (current_value != 0) {
        if (current_value < 0) {
          // Counter-clockwise
          this->setMatrixState(ENCODER_CONFIGS[i].ccw.row, ENCODER_CONFIGS[i].ccw.col, true);
          this->setMatrixState(ENCODER_CONFIGS[i].cw.row, ENCODER_CONFIGS[i].cw.col, false);
        } else {
          // Clockwise
          this->setMatrixState(ENCODER_CONFIGS[i].ccw.row, ENCODER_CONFIGS[i].ccw.col, false);
          this->setMatrixState(ENCODER_CONFIGS[i].cw.row, ENCODER_CONFIGS[i].cw.col, true);
        }
        last_encoder_values_[i] = current_value;
      } else {
        // Clear the encoder bits if no movement
        this->setMatrixState(ENCODER_CONFIGS[i].ccw.row, ENCODER_CONFIGS[i].ccw.col, false);
        this->setMatrixState(ENCODER_CONFIGS[i].cw.row, ENCODER_CONFIGS[i].cw.col, false);
      }
    }
  }
};

template<typename _KeyScannerProps>
int PreonicKeyScanner<_KeyScannerProps>::last_encoder_values_[kaleidoscope::device::keyboardio::NUM_ENCODERS];

struct PreonicStorageProps : public kaleidoscope::driver::storage::NRF52FlashProps {
  static constexpr uint16_t length = 16384;
};

struct PreonicKeyScannerProps : public kaleidoscope::driver::keyscanner::NRF52KeyScannerProps {
  static constexpr uint32_t keyscan_interval_micros = 750;
  static constexpr uint8_t matrix_rows    = 6;
  static constexpr uint8_t matrix_columns = 12;
  typedef MatrixAddr<matrix_rows, matrix_columns> KeyAddr;
  typedef uint16_t RowState;
  static constexpr uint8_t matrix_row_pins[matrix_rows]    = {0, 1, 2, 3, 4, 5};
  static constexpr uint8_t matrix_col_pins[matrix_columns] = {6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17};
};

// If we need to override HID props:
struct PreonicHIDProps : public kaleidoscope::driver::hid::HybridProps {
  //typedef kaleidoscope::driver::hid::base::AbsoluteMouseProps AbsoluteMouseProps;
  //typedef kaleidoscope::driver::hid::base::AbsoluteMouse<AbsoluteMouseProps> AbsoluteMouse;
};

struct PreonicLEDProps : public kaleidoscope::driver::led::WS2812Props {
  static constexpr uint8_t led_count     = 4;
  static constexpr uint8_t pin           = 19;
  static constexpr uint8_t key_led_map[] = {
    2, 3, 1, 0  // The logical order of the LEDS should be clockwise, starting in the top left corner
  };
};

struct PreonicSpeakerProps : public kaleidoscope::driver::speaker::PiezoProps {
  static constexpr uint8_t pin = PIN_SPEAKER;
};

struct PreonicBatteryGaugeProps : public kaleidoscope::driver::battery_gauge::MAX17048Props {
  static constexpr uint8_t alert_pin = PIN_BATT_ALERT;  // Pin connected to MAX17048 ALERT output
  // Battery configuration for Preonic's 3.7V LiPo
  static constexpr uint16_t battery_voltage_min    = 3300;  // 3.3V cutoff
  static constexpr uint16_t battery_voltage_max    = 4200;  // 4.2V max
  static constexpr uint8_t battery_alert_threshold = 15;    // Alert at 15%
};

struct PreonicProps : public kaleidoscope::device::BaseProps {
  typedef PreonicHIDProps HIDProps;
  typedef kaleidoscope::driver::hid::Hybrid<HIDProps> HID;

  typedef PreonicLEDProps LEDDriverProps;
  typedef kaleidoscope::driver::led::WS2812<LEDDriverProps> LEDDriver;

  typedef PreonicKeyScannerProps KeyScannerProps;
  typedef PreonicKeyScanner<KeyScannerProps> KeyScanner;

  typedef PreonicStorageProps StorageProps;
  typedef kaleidoscope::driver::storage::NRF52Flash<StorageProps> Storage;

  typedef kaleidoscope::driver::bootloader::nrf52::UF2 Bootloader;
  static constexpr const char *short_name = "preonic";

  typedef kaleidoscope::driver::mcu::TinyUSBProps MCUProps;
  typedef kaleidoscope::driver::mcu::TinyUSB<MCUProps> MCU;

  typedef kaleidoscope::driver::ble::BLEBluefruit BLE;

  static constexpr const bool isHybridHostConnection = true;

  typedef PreonicSpeakerProps SpeakerProps;
  typedef kaleidoscope::driver::speaker::Piezo<SpeakerProps> Speaker;

  typedef PreonicBatteryGaugeProps BatteryGaugeProps;
  typedef kaleidoscope::driver::battery_gauge::MAX17048<BatteryGaugeProps> BatteryGauge;
};

class Preonic : public kaleidoscope::device::Base<PreonicProps> {
 private:
  static uint32_t last_activity_time_;                    // Used for deep sleep
  static constexpr uint32_t DEEP_SLEEP_TIMEOUT_MS = 50;  // Enter deep sleep after 100ms
  static volatile bool input_event_pending_;

  /**
   * @brief Structure to track timer and RTC states for sleep/wake
   */
  struct TimerState {
    bool timer0_enabled;
    bool timer1_enabled;
    bool timer2_enabled;
    bool timer3_enabled;
    bool timer4_enabled;
    
    // RTC state
    bool rtc0_enabled;
    bool rtc1_enabled;
    bool rtc2_enabled;
    
    // RTC register backups
    uint32_t rtc0_intenset;
    uint32_t rtc0_evten;
    uint32_t rtc1_intenset;
    uint32_t rtc1_evten;
    uint32_t rtc2_intenset;
    uint32_t rtc2_evten;
  };
  static TimerState timer_state_;

  // Battery state tracking
  static uint8_t last_battery_level_;

  /// @brief Configure a pin for wake-on-low sensing
  /// @param arduino_pin The Arduino pin number to configure
  static void enablePinSensing(uint8_t arduino_pin) {
    uint32_t nrf_pin = g_ADigitalPinMap[arduino_pin];
    if (nrf_pin >= 32) {
      NRF_P1->PIN_CNF[nrf_pin - 32] |= (GPIO_PIN_CNF_SENSE_Low << GPIO_PIN_CNF_SENSE_Pos);
    } else {
      NRF_GPIO->PIN_CNF[nrf_pin] |= (GPIO_PIN_CNF_SENSE_Low << GPIO_PIN_CNF_SENSE_Pos);
    }
  }

  /// @brief Disable wake-on-low sensing for a pin
  /// @param arduino_pin The Arduino pin number to configure
  static void disablePinSensing(uint8_t arduino_pin) {
    uint32_t nrf_pin = g_ADigitalPinMap[arduino_pin];
    if (nrf_pin >= 32) {
      NRF_P1->PIN_CNF[nrf_pin - 32] &= ~(GPIO_PIN_CNF_SENSE_Msk);
    } else {
      NRF_GPIO->PIN_CNF[nrf_pin] &= ~(GPIO_PIN_CNF_SENSE_Msk);
    }
  }

  // Configure all column pins for PORT event detection
  static void configureColumnsForSensing() {
    for (uint8_t i = 0; i < KeyScannerProps::matrix_columns; i++) {
      enablePinSensing(KeyScannerProps::matrix_col_pins[i]);
    }
  }

  // Configure all column pins to stop sensing
  static void disableColumnSensing() {
    for (uint8_t i = 0; i < KeyScannerProps::matrix_columns; i++) {
      disablePinSensing(KeyScannerProps::matrix_col_pins[i]);
    }
  }

  // Configure encoder pins for wake-on-low sensing
  static void enableEncoderSensing() {
    for (uint8_t i = 0; i < kaleidoscope::device::keyboardio::NUM_ENCODERS; i++) {
      enablePinSensing(ENCODER_CONFIGS[i].pinA);
      enablePinSensing(ENCODER_CONFIGS[i].pinB);
    }
  }

  // Disable sense detection on encoder pins
  static void disableEncoderSensing() {
    for (uint8_t i = 0; i < NUM_ENCODERS; i++) {
      disablePinSensing(ENCODER_CONFIGS[i].pinA);
      disablePinSensing(ENCODER_CONFIGS[i].pinB);
    }
  }

  // Drive all rows LOW to enable key detection across the entire matrix
  static void prepareMatrixForSleep() {
    // Drive all rows LOW so we can detect any keypress
    for (uint8_t i = 0; i < KeyScannerProps::matrix_rows; i++) {
      digitalWrite(KeyScannerProps::matrix_row_pins[i], LOW);
    }
  }

  // Restore matrix pins to their normal scanning state
  static void restoreMatrixAfterSleep() {
    // Reset all rows to HIGH
    for (uint8_t i = 0; i < KeyScannerProps::matrix_rows; i++) {
      digitalWrite(KeyScannerProps::matrix_row_pins[i], HIGH);
    }
  }

 private:
  /**
   * Sets up the Programmable Peripheral Interconnect (PPI) system to handle GPIO PORT events
   * 
   * This implementation is based on the solution by jgartrel:
   * https://github.com/adafruit/Adafruit_nRF52_Arduino/issues/754#issuecomment-1437329605
   * 
   * The nRF52 PPI system allows events from one peripheral to trigger actions in another
   * peripheral without CPU intervention. Here we use it to:
   * 
   * 1. Route GPIOTE PORT events to a Software Interrupt (SWI3) via EGU3
   * 2. Avoid conflicts with the Arduino core's GPIOTE interrupt handler
   * 
   * The setup process:
   * a) Configure SWI3_EGU3 interrupt with priority 6 (same as SDK drivers)
   * b) Enable EGU3 interrupt for channel 0
   * c) Connect GPIOTE PORT event to EGU3 trigger via PPI channel 0
   * d) Enable the PPI channel
   */
  static void setupPPIInterrupt() {
    // Set up SWI3_EGU3 interrupt with priority 6 (same as SDK drivers)
    NVIC_SetPriority(SWI3_EGU3_IRQn, 6);
    NVIC_ClearPendingIRQ(SWI3_EGU3_IRQn);
    NVIC_EnableIRQ(SWI3_EGU3_IRQn);

    // Enable EGU3 (Event Generator Unit) interrupt for channel 0
    // EGU is used to trigger the software interrupt
    NRF_EGU3->INTENSET = EGU_INTEN_TRIGGERED0_Msk;

    // Configure PPI channel 0:
    // - Event End Point (EEP): GPIOTE PORT event
    // - Task End Point (TEP): EGU3 trigger task
    NRF_PPI->CH[0].EEP = (uint32_t)&NRF_GPIOTE->EVENTS_PORT;
    NRF_PPI->CH[0].TEP = (uint32_t)&NRF_EGU3->TASKS_TRIGGER[0];

    // Enable PPI channel 0
    NRF_PPI->CHENSET = PPI_CHENSET_CH0_Msk;
  }

  /**
   * Sets up GPIO and GPIOTE for matrix column sensing and wake-on-keypress
   * 
   * This function:
   * 1. Configures all matrix columns as inputs with pull-ups
   * 2. Enables sense detection (LOW) on all columns for wake-on-keypress
   * 3. Sets up PPI to route PORT events to our interrupt handler
   * 
   * The nRF52's PORT event is triggered when any pin's sense condition is met,
   * making it perfect for wake-on-keypress from any key. However, we can't use
   * the standard GPIOTE interrupt handler as it's already used by the Arduino
   * core. Instead, we use PPI to route PORT events to a separate interrupt.
   */
  static void setupGPIOTE() {
    // Configure each column pin for sense detection
    for (uint8_t i = 0; i < KeyScannerProps::matrix_columns; i++) {
      uint8_t arduino_pin = KeyScannerProps::matrix_col_pins[i];
      uint32_t nrf_pin    = g_ADigitalPinMap[arduino_pin];

      // Configure pin as input with pull-up and LOW sense detection
      // When the pin goes LOW (key pressed), it will trigger a PORT event
      nrf_gpio_cfg_sense_input(nrf_pin, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
    }

    // Configure all rotary encoder pins for sense detection
    for (uint8_t i = 0; i < NUM_ENCODERS; i++) {
      // Configure both A and B pins for each encoder
      uint32_t encoder_a = g_ADigitalPinMap[ENCODER_CONFIGS[i].pinA];
      uint32_t encoder_b = g_ADigitalPinMap[ENCODER_CONFIGS[i].pinB];

      // Configure encoder pins with pull-ups and LOW sense detection
      // This won't interfere with SwEncoder because we're just adding sense detection
      // to the existing pin configuration
      nrf_gpio_cfg_sense_input(encoder_a, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
      nrf_gpio_cfg_sense_input(encoder_b, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
    }

    // Set up PPI to route PORT events to our interrupt handler
    setupPPIInterrupt();

    input_event_pending_ = false;
  }

  /**
   * Enter deep sleep mode. Will only wake on GPIO activity.
   * In this mode:
   * - Main loop is suspended
   * - CPU enters low power mode
   * - GPIO sense remains active to detect key presses
   * 
   * @returns true if woken by GPIO activity
   */
  /**
   * @brief Safely disable non-critical timers
   * @details Only disables timers that are safe to stop, preserving system-critical ones
   */
  static void disableTimers() {
    // Save current timer states
    timer_state_.timer2_enabled = (NRF_TIMER2->INTENSET != 0);
    timer_state_.timer3_enabled = (NRF_TIMER3->INTENSET != 0);
    timer_state_.timer4_enabled = (NRF_TIMER4->INTENSET != 0);

    // TIMER0: System timer - don't touch
    // TIMER1: Keyscanner - managed by NRF52KeyScanner
    
    // TIMER2: Application timer
    if (timer_state_.timer2_enabled) {
      // Stop the timer
      NRF_TIMER2->TASKS_STOP = 1;
      // Disable timer interrupt
      NVIC_DisableIRQ(TIMER2_IRQn);
      // Clear any pending interrupts
      NVIC_ClearPendingIRQ(TIMER2_IRQn);
    }

    // TIMER3: Application timer
    if (timer_state_.timer3_enabled) {
      NRF_TIMER3->TASKS_STOP = 1;
      NVIC_DisableIRQ(TIMER3_IRQn);
      NVIC_ClearPendingIRQ(TIMER3_IRQn);
    }
    
    // TIMER4: Application timer
    if (timer_state_.timer4_enabled) {
      NRF_TIMER4->TASKS_STOP = 1;
      NVIC_DisableIRQ(TIMER4_IRQn);
      NVIC_ClearPendingIRQ(TIMER4_IRQn);
    }
  }

  /**
   * @brief Helper function to disable a specific RTC
   * @param rtc_num RTC number (0, 1, or 2)
   * @returns true if RTC was disabled, false if invalid RTC number
   */
  static bool disableRTCHelper(uint8_t rtc_num) {
    NRF_RTC_Type* rtc;
    IRQn_Type irq;
    bool* enabled_state;
    uint32_t* intenset_backup;
    uint32_t* evten_backup;

    switch (rtc_num) {
      case 0:
        rtc = NRF_RTC0;
        irq = RTC0_IRQn;
        enabled_state = &timer_state_.rtc0_enabled;
        intenset_backup = &timer_state_.rtc0_intenset;
        evten_backup = &timer_state_.rtc0_evten;
        break;
      case 1:
        rtc = NRF_RTC1;
        irq = RTC1_IRQn;
        enabled_state = &timer_state_.rtc1_enabled;
        intenset_backup = &timer_state_.rtc1_intenset;
        evten_backup = &timer_state_.rtc1_evten;
        break;
      case 2:
        rtc = NRF_RTC2;
        irq = RTC2_IRQn;
        enabled_state = &timer_state_.rtc2_enabled;
        intenset_backup = &timer_state_.rtc2_intenset;
        evten_backup = &timer_state_.rtc2_evten;
        break;
      default:
        return false;
    }

    // Save current state
    *enabled_state = (rtc->INTENSET != 0);
    *intenset_backup = rtc->INTENSET;
    *evten_backup = rtc->EVTEN;

    // Clear all interrupts and events
    rtc->INTENCLR = 0xFFFFFFFF;
    rtc->EVTENCLR = 0xFFFFFFFF;
    
    // Clear all event flags
    rtc->EVENTS_TICK = 0;
    rtc->EVENTS_OVRFLW = 0;
    rtc->EVENTS_COMPARE[0] = 0;
    rtc->EVENTS_COMPARE[1] = 0;
    rtc->EVENTS_COMPARE[2] = 0;
    rtc->EVENTS_COMPARE[3] = 0;

    // Stop the RTC
    rtc->TASKS_STOP = 1;
    
    // Disable RTC interrupt
    NVIC_DisableIRQ(irq);
    // Clear any pending interrupts
    NVIC_ClearPendingIRQ(irq);

    return true;
  }

  /**
   * @brief Helper function to restore a specific RTC
   * @param rtc_num RTC number (0, 1, or 2)
   * @returns true if RTC was restored, false if invalid RTC number
   */
  static bool restoreRTCHelper(uint8_t rtc_num) {
    NRF_RTC_Type* rtc;
    IRQn_Type irq;
    bool enabled_state;
    uint32_t intenset_backup;
    uint32_t evten_backup;

    switch (rtc_num) {
      case 0:
        rtc = NRF_RTC0;
        irq = RTC0_IRQn;
        enabled_state = timer_state_.rtc0_enabled;
        intenset_backup = timer_state_.rtc0_intenset;
        evten_backup = timer_state_.rtc0_evten;
        break;
      case 1:
        rtc = NRF_RTC1;
        irq = RTC1_IRQn;
        enabled_state = timer_state_.rtc1_enabled;
        intenset_backup = timer_state_.rtc1_intenset;
        evten_backup = timer_state_.rtc1_evten;
        break;
      case 2:
        rtc = NRF_RTC2;
        irq = RTC2_IRQn;
        enabled_state = timer_state_.rtc2_enabled;
        intenset_backup = timer_state_.rtc2_intenset;
        evten_backup = timer_state_.rtc2_evten;
        break;
      default:
        return false;
    }

    // Restore interrupt and event configuration
    rtc->INTENSET = intenset_backup;
    rtc->EVTEN = evten_backup;

    if (enabled_state) {
      // Clear any pending interrupts
      NVIC_ClearPendingIRQ(irq);
      // Re-enable RTC interrupt
      NVIC_EnableIRQ(irq);
      // Start the RTC
      rtc->TASKS_START = 1;
    }

    return true;
  }

  /**
   * @brief Disable all RTC functions
   */
  static void disableRTC() {
   // disableRTCHelper(0);  // RTC0
    disableRTCHelper(1);  // RTC1
    disableRTCHelper(2);  // RTC2
  }


  /**
   * @brief Restore all previously disabled RTC functions
   */
  static void restoreRTC() {
  // restoreRTCHelper(0);  // RTC0
    restoreRTCHelper(1);  // RTC1
    restoreRTCHelper(2);  // RTC2
  }

  /**
   * @brief Restore previously enabled timers and RTCs
   * @details Restores timer states saved by disableTimersAndRTC()
   */
  /** 
   * @brief Restore previously disabled timers
   * @details Only restores TIMER3 and TIMER4 that we explicitly disabled
   */
  static void restoreTimers() {
    // TIMER0: System timer - don't touch
    // TIMER1: Keyscanner - managed by NRF52KeyScanner

    // TIMER2: Application timer
    if (timer_state_.timer2_enabled) {
      // Clear any pending interrupts
      NVIC_ClearPendingIRQ(TIMER2_IRQn);
      // Re-enable timer interrupt
      NVIC_EnableIRQ(TIMER2_IRQn);
      // Start the timer
      NRF_TIMER2->TASKS_START = 1;
    }

    // TIMER3: Application timer
    if (timer_state_.timer3_enabled) {
      NVIC_ClearPendingIRQ(TIMER3_IRQn);
      NVIC_EnableIRQ(TIMER3_IRQn);
      NRF_TIMER3->TASKS_START = 1;
    }
    
    // TIMER4: Application timer
    if (timer_state_.timer4_enabled) {
      NVIC_ClearPendingIRQ(TIMER4_IRQn);
      NVIC_EnableIRQ(TIMER4_IRQn);
      NRF_TIMER4->TASKS_START = 1;
    }
  }






  /**
   * @brief Structure to store TWI state during sleep
   */
  struct TWIState {
    uint32_t frequency;
    uint32_t pin_scl;
    uint32_t pin_sda;
    bool enabled;
  };
  static TWIState twi_state_;

  /**
   * @brief Disable TWI for sleep
   * @details Properly disables TWI and clears events before sleep
   */
  static void disableTWIForSleep() {
    // Stop any ongoing TWI transactions
    NRF_TWI0->TASKS_STOP = 1;
    
    // Clear all TWI events
    NRF_TWI0->EVENTS_STOPPED = 0;
    NRF_TWI0->EVENTS_RXDREADY = 0;
    NRF_TWI0->EVENTS_TXDSENT = 0;
    NRF_TWI0->EVENTS_ERROR = 0;
    NRF_TWI0->EVENTS_BB = 0;  // Bus busy event
    
    // Disable TWI
    NRF_TWI0->ENABLE = 0;

    // Save current TWI state for restoration
    twi_state_.frequency = NRF_TWI0->FREQUENCY;
    twi_state_.pin_scl = NRF_TWI0->PSEL.SCL;
    twi_state_.pin_sda = NRF_TWI0->PSEL.SDA;
    twi_state_.enabled = true;
  }

  /**
   * @brief Restore TWI after sleep
   * @details Restores TWI configuration if it was previously enabled
   */
  static void restoreTWIAfterSleep() {
    if (!twi_state_.enabled) return;

    // Restore TWI configuration
    NRF_TWI0->FREQUENCY = twi_state_.frequency;
    NRF_TWI0->PSEL.SCL = twi_state_.pin_scl;
    NRF_TWI0->PSEL.SDA = twi_state_.pin_sda;
    
    // Enable TWI
    NRF_TWI0->ENABLE = TWI_ENABLE_ENABLE_Enabled << TWI_ENABLE_ENABLE_Pos;

    twi_state_.enabled = false;
  }


  bool enterDeepSleep() {
    disableLEDPower();
    keyScanner().suspendTimer();
    prepareMatrixForSleep();
    configureColumnsForSensing();
    enableEncoderSensing();
    setupGPIOTE();
   // disableTWIForSleep();
    disableRTC();
    //disableTimers();
    sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
  
    while (!input_event_pending_) {
      waitForEvent();
    }

    // Wake up sequence:
    // 1. Switch back to constant latency mode
    sd_power_mode_set(NRF_POWER_MODE_CONSTLAT);
    last_activity_time_ = millis();

    // restoreTWIAfterSleep();
    // restoreTimers();
    restoreRTC();
    restoreMatrixAfterSleep();
    disableColumnSensing();
    disableEncoderSensing();
    keyScanner().reinitializeEncoders();
    keyScanner().resumeTimer();

    enableLEDPower();
    return true;
  }

  /**
   * Check if we should enter deep sleep based on inactivity time
   * Never enter deep sleep if USB is connected
   */
  bool shouldEnterDeepSleep() {
    // Never sleep if USB is connected
    if (mcu_.USBConfigured()) {
      return false;
    }
    if (keyScanner().pressedKeyswitchCount()) {
      return false;
    }

    // Don't sleep if the keyscanner has events in queue
    if (keyScanner().hasQueuedEvents()) {
      return false;
    }
    // Don't sleep if LEDs were recently active
    if ((millis() - ledDriver().LEDsLastOn()) < 1000) {
      return false;
    }

    if (millis() - last_activity_time_ >= DEEP_SLEEP_TIMEOUT_MS) {
      return true;
    }
    return false;
  }

  // Battery monitoring methods
  void updateBatteryLevel() {
    uint8_t new_level = batteryGauge().getBatteryLevel();
    if (new_level != last_battery_level_) {
      last_battery_level_ = new_level;
      ble().setBatteryLevel(new_level);
    }
  }


 public:
  Preonic() {
  }


  void betweenCycles() {
    // Handle speaker updates
    // TODO(jesse): move this into a hook
    updateSpeaker();

    // Manage LED power based on LED activity
    if (ledDriver().areAnyLEDsOn() || ((millis() - ledDriver().LEDsLastOn()) < 1000)) {
      enableLEDPower();
    } else {
      disableLEDPower();
    }

    // Handle any pending GPIOTE events
    if (input_event_pending_) {
      input_event_pending_ = false;
      last_activity_time_  = millis();  // Update activity time on GPIOTE event

      // Check if the battery gauge has an alert
    //  if (batteryGauge().hasAlert()) {
    //    updateBatteryLevel();
     //   batteryGauge().clearAlert();
     // }
    }

    // Check if we should enter deep sleep
    // Only enter deep sleep if no keys are pressed and we're not connected via USB
    else if (shouldEnterDeepSleep()) {
     // TODO FIX ME Temporarily disable enterDeepSleep because we're not yet properly
     // checking to make sure the bluetooth event queue is empty 
     // and we also need to deal with the rotary encoder library generating extra events when we're going to sleep
     // enterDeepSleep();
    }

    // In the future, this should run in response to a USB connect/disconnect event rather than on every cycle
    // Enabling this method will cause the device to automatically switch to USB mode if it's connected to 
    // a USB host, even if the user is explicitlytrying to connect to a bluetooth host
    // autoHostConnectionMode();
  }


  void enableLEDPower() {
    digitalWrite(PIN_LED_ENABLE, HIGH);
  }

  void disableLEDPower() {
    digitalWrite(PIN_LED_ENABLE, LOW);
  }


  /**
   * @brief Disable unused peripherals at startup
   * @details Disables and properly cleans up UART, ADC, TWI Slave, SPI, and NFC
   */
  static void disableUnusedPeripherals() {
    // Disable UART
    NRF_UARTE0->TASKS_STOPTX = 1;
    NRF_UARTE0->TASKS_STOPRX = 1;
    NRF_UARTE0->EVENTS_TXDRDY = 0;
    NRF_UARTE0->EVENTS_RXDRDY = 0;
    NRF_UARTE0->ENABLE = 0;

    // Disable ADC
    NRF_SAADC->TASKS_STOP = 1;
    NRF_SAADC->EVENTS_DONE = 0;
    NRF_SAADC->ENABLE = 0;

    // Disable TWI Slave
    NRF_TWIS0->TASKS_STOP = 1;
    NRF_TWIS0->EVENTS_READ = 0;
    NRF_TWIS0->EVENTS_WRITE = 0;
    NRF_TWIS0->ENABLE = 0;

    // Disable SPI
    NRF_SPI0->EVENTS_READY = 0;
    NRF_SPI0->ENABLE = 0;

    // Disable NFC
    NRF_NFCT->TASKS_DISABLE = 1;
    NRF_NFCT->EVENTS_FIELDDETECTED = 0;
    NRF_NFCT->EVENTS_FIELDLOST = 0;
    NRF_NFCT->EVENTS_READY = 0;
  }


  void setup() {


    //while (!Serial) delay(10);  // Wait for Serial
    //Serial.println("Preonic starting up...");
    // setupGPIOTE();

    // Work around some BSP problems with TinyUSB and Serial1 logging
#if CFG_LOGGER == 1
    Serial1.begin(115200);
#endif

    // Disable debug interface if not actively debugging
    NRF_CLOCK->TRACECONFIG = 0;

    //    disableUnusedPeripherals(); // As of this writing, disableUnusedPeripherals() does not provide a measurable power efficiency improvement
    // Turn on the LED power
    pinMode(PIN_LED_ENABLE, OUTPUT);
    enableLEDPower();

    device::Base<PreonicProps>::setup();
    last_activity_time_ = millis();
  }

  Stream &serialPort() {
    return Serial;  // For now, we *always* use USB Serial
    if (getHostConnectionMode() == MODE_USB && mcu_.USBConfigured()) {
      return Serial;
    } else {
      return ble().serialPort();
    }
  }

  void initSerial() {
    Serial.begin(9600);
  }

 public:
  static void setInputEventPending() {
    input_event_pending_ = true;
  }
};

}  // namespace keyboardio
}  // namespace device


EXPORT_DEVICE(kaleidoscope::device::keyboardio::Preonic)

}  // namespace kaleidoscope

// clang-format off

#define PER_LED_DATA(dft, r0c0, r0c1, r0c2, r0c3, ...)\
		          r0c0, r0c1, r0c2, RESTRICT_ARGS_COUNT((r0c3), 4, LEDMAP, ##__VA_ARGS__)

#define PER_KEY_DATA(dflt,                                                                                 \
                                r0c4, r0c5,                  r0c9, r0c10, r0c11,  \
        r1c0, r1c1, r1c2, r1c3, r1c4, r1c5, r1c6, r1c7, r1c8, r1c9, r1c10, r1c11,  \
        r2c0, r2c1, r2c2, r2c3, r2c4, r2c5, r2c6, r2c7, r2c8, r2c9, r2c10, r2c11,  \
        r3c0, r3c1, r3c2, r3c3, r3c4, r3c5, r3c6, r3c7, r3c8, r3c9, r3c10, r3c11, \
        r4c0, r4c1, r4c2, r4c3, r4c4, r4c5, r4c6, r4c7, r4c8, r4c9, r4c10, r4c11,  \
        r5c0, r5c1, r5c2, r5c3, r5c4, r5c5, r5c6, r5c7, r5c8, r5c9, r5c10, r5c11, ...)\
         XXX,  XXX,  XXX,  XXX, r0c4, r0c5, XXX,   XXX,  XXX, r0c9, r0c10, r0c11, \
        r1c0, r1c1, r1c2, r1c3, r1c4, r1c5, r1c6, r1c7, r1c8, r1c9, r1c10, r1c11, \
        r2c0, r2c1, r2c2, r2c3, r2c4, r2c5, r2c6, r2c7, r2c8, r2c9, r2c10, r2c11, \
        r3c0, r3c1, r3c2, r3c3, r3c4, r3c5, r3c6, r3c7, r3c8, r3c9, r3c10, r3c11, \
        r4c0, r4c1, r4c2, r4c3, r4c4, r4c5, r4c6, r4c7, r4c8, r4c9, r4c10, r4c11, \
        r5c0, r5c1, r5c2, r5c3, r5c4, r5c5,  r5c6, r5c7, r5c8, r5c9, r5c10, RESTRICT_ARGS_COUNT((r5c11), 63, KEYMAP, ##__VA_ARGS__)

#endif /* ARDUINO_ARCH_NRF52 */
