/* Kaleidoscope-SonicThemes -- Audio feedback themes for Kaleidoscope
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



#include <Kaleidoscope.h>
#include <Kaleidoscope-SonicThemes.h>
#include "testing/setup-googletest.h"

namespace kaleidoscope {
namespace testing {

class SonicThemes : public ::testing::Test {
 protected:
  void SetUp() override {
    ::SonicThemes.enable();
  }

  void TearDown() override {
    ::SonicThemes.disable();
  }
};

TEST_F(SonicThemes, EnableDisable) {
  EXPECT_TRUE(::SonicThemes.isEnabled());
  ::SonicThemes.disable();
  EXPECT_FALSE(::SonicThemes.isEnabled());
}

TEST_F(SonicThemes, ThemeSwitching) {
  uint8_t initial_theme = Runtime.storage().next_theme;
  ::SonicThemes.nextTheme();
  EXPECT_NE(initial_theme, Runtime.storage().next_theme);
}

TEST_F(SonicThemes, LayerChange) {
  // Mock a layer change
  Layer.activate(1);
  cycle();
  // Verify tone was played (need to add mock tone interface)
}

TEST_F(SonicThemes, BatteryEvents) {
  // Mock battery level changes
  Runtime.device().battery.level(20);
  cycle();
  // Verify low battery tone
}

TEST_F(SonicThemes, ConnectionEvents) {
  // Mock connection state changes
  Runtime.device().ble.disconnect();
  cycle();
  // Verify disconnect tone
}

}  // namespace testing
}  // namespace kaleidoscope