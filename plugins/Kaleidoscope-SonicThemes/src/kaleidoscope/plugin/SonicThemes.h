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



#pragma once

#include "kaleidoscope/Runtime.h"
#include "kaleidoscope/plugin.h"
#include "kaleidoscope/plugin/MelodyPlayer.h"
#include "kaleidoscope/plugin/Theme.h"
#include "kaleidoscope/plugin/themes/StationMasterTheme.h"
#include <Kaleidoscope-FocusSerial.h>

namespace kaleidoscope {
namespace plugin {

class SonicThemes : public kaleidoscope::Plugin {
 public:
  SonicThemes() : enabled_(true), current_theme_index_(0) {}

  EventHandlerResult onSetup();
  EventHandlerResult beforeReportingState(const KeyEvent &event);
  EventHandlerResult afterEachCycle();
  EventHandlerResult onLayerChange();
  EventHandlerResult onFocusEvent(const char *command);
  EventHandlerResult onNameQuery();

  void playEvent(SoundEvent event);

 private:
  bool enabled_;
  uint8_t current_theme_index_;
  MelodyPlayer melody_player_;

  static constexpr uint8_t THEME_COUNT = 1;
  static const Theme *const themes_[THEME_COUNT];
};

}  // namespace plugin
}  // namespace kaleidoscope

extern kaleidoscope::plugin::SonicThemes SonicThemes;
