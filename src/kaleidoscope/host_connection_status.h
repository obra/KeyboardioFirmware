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

namespace kaleidoscope {

enum class HostConnectionStatus {
  Disconnected,      // Not connected to any host
  Advertising,       // Currently advertising
  DeviceSelected,    // Device has been selected
  DeviceUnselected,  // Device has been unselected
  Connecting,        // In the process of connecting (including pairing)
  Connected,         // Successfully connected and ready
  PairingFailed,     // Failed to pair with host
  PairingSuccess     // Successfully paired with host
};

}  // namespace kaleidoscope
