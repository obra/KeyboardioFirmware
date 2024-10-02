/* Kaleidoscope - Firmware for computer input devices
 * Copyright (C) 2023-2025 Keyboard.io, inc.
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

#if defined(ARDUINO_ARCH_NRF52) || defined(KALEIDOSCOPE_VIRTUAL_BUILD)

#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include "kaleidoscope/driver/storage/Base.h"

namespace kaleidoscope {
namespace driver {
namespace storage {

#define NRF52FLASH_DEBUG 1  // Set to 0 to disable debug messages

#if NRF52FLASH_DEBUG
#define DEBUG_MSG(...) Serial.println(F(__VA_ARGS__))
#else
#define DEBUG_MSG(...)
#endif

struct NRF52FlashProps : kaleidoscope::driver::storage::BaseProps {
  static constexpr uint16_t length = 16384;
};

template<typename _StorageProps>
class NRF52Flash : public kaleidoscope::driver::storage::Base<_StorageProps> {
 private:
  static constexpr const char *EEPROM_PATH = "/eeprom.dat";
  static Adafruit_LittleFS_Namespace::File file;
  static bool is_initialized;
  static uint8_t *contents;  // Add buffer to store file contents
  static bool dirty_;        // Add flag to track if contents have changed

  static bool init() {
    if (is_initialized) return true;

    // Allocate memory for contents
    if (contents == nullptr) {
      contents = new uint8_t[_StorageProps::length];
    }

    InternalFS.end();
    if (!InternalFS.begin()) {
      DEBUG_MSG("[NRF52Flash] Failed to initialize filesystem");
      return false;
    }

    // Load file contents into memory
    if (InternalFS.exists(EEPROM_PATH)) {
      if (!file.open(EEPROM_PATH, Adafruit_LittleFS_Namespace::FILE_O_READ)) {
        DEBUG_MSG("[NRF52Flash] Failed to open file");
        return false;
      }
      size_t read_size = file.read(contents, _StorageProps::length);
      file.close();

      if (read_size != _StorageProps::length) {
        DEBUG_MSG("[NRF52Flash] Failed to read entire file");
        return false;
      }
    } else {
      // Initialize with uninitialized_byte if file doesn't exist
      memset(contents, _StorageProps::uninitialized_byte, _StorageProps::length);
    }

    is_initialized = true;
    return true;
  }

  static bool verifyAndFixFile() {
    if (!init()) return false;

    // Check if file exists and has correct size
    if (InternalFS.exists(EEPROM_PATH)) {
      if (!file.open(EEPROM_PATH, Adafruit_LittleFS_Namespace::FILE_O_READ)) {
        DEBUG_MSG("[NRF52Flash] Failed to open file");
        return false;
      }

      size_t size = file.size();
      file.close();

      if (size == _StorageProps::length) {
        return true;
      }

      DEBUG_MSG("[NRF52Flash] File exists but has wrong size, removing it");
      InternalFS.remove(EEPROM_PATH);
    }

    // Create new file
    if (!file.open(EEPROM_PATH, Adafruit_LittleFS_Namespace::FILE_O_WRITE)) {
      DEBUG_MSG("[NRF52Flash] Failed to create file");
      return false;
    }

    // Initialize with uninitialized_byte
    uint8_t buffer[64];
    memset(buffer, _StorageProps::uninitialized_byte, sizeof(buffer));

    size_t remaining = _StorageProps::length;
    while (remaining > 0) {
      size_t to_write = (remaining < sizeof(buffer)) ? remaining : sizeof(buffer);
      if (file.write(buffer, to_write) != to_write) {
        DEBUG_MSG("[NRF52Flash] Failed to write to file");
        file.close();
        return false;
      }
      remaining -= to_write;
    }

    file.close();
    return true;
  }

  static bool checkBounds(uint16_t offset, uint16_t size) {
    bool result = (offset + size <= _StorageProps::length);
    return result;
  }

  static bool writeToFile() {

    //TODO(jesse): I'd rather truncate the file, but there's a bug in truncate(0)
    // That causes "assertion "pcache->block == 0xffffffff" failed"
    // Delete existing file first
    InternalFS.remove(EEPROM_PATH);

    // Create new file
    if (!file.open(EEPROM_PATH, Adafruit_LittleFS_Namespace::FILE_O_WRITE)) {
      DEBUG_MSG("[NRF52Flash] Failed to open file for writing");
      return false;
    }

    size_t written = file.write(contents, _StorageProps::length);
    file.close();
    if (written != _StorageProps::length) {
      DEBUG_MSG("[NRF52Flash] Write failed");
      return false;
    }

    return true;
  }

 public:
  template<typename T>
  static T &get(uint16_t offset, T &t) {
    if (!init() || !checkBounds(offset, sizeof(T))) {
      return t;
    }

    memcpy(&t, contents + offset, sizeof(T));
    return t;
  }

  template<typename T>
  static const T &put(uint16_t offset, T &t) {
    if (!init() || !checkBounds(offset, sizeof(T))) {
      return t;
    }

    // Update in-memory contents only
    memcpy(contents + offset, &t, sizeof(T));
    dirty_ = true;

    return t;
  }

  uint8_t read(int idx) {
    if (!init() || !checkBounds(idx, 1)) {
      return 0;
    }
    return contents[idx];
  }

  void write(int idx, uint8_t val) {
    if (!init() || !checkBounds(idx, 1)) {
      return;
    }
    contents[idx] = val;
    dirty_        = true;
  }

  void update(int idx, uint8_t val) {
    if (read(idx) != val) {
      write(idx, val);
    }
  }

  bool isSliceUninitialized(uint16_t offset, uint16_t size) {
    if (!file.open(EEPROM_PATH, Adafruit_LittleFS_Namespace::FILE_O_READ)) {
      return true;
    }
    if (!file.seek(offset)) {
      file.close();
      return true;
    }

    uint8_t *buffer  = new uint8_t[size];
    size_t read_size = file.read(buffer, size);
    file.close();

    bool uninitialized = true;
    if (read_size == size) {
      for (uint16_t i = 0; i < size; i++) {
        if (buffer[i] != _StorageProps::uninitialized_byte) {
          uninitialized = false;
          break;
        }
      }
    }

    delete[] buffer;
    return uninitialized;
  }

  const uint16_t length() {
    return _StorageProps::length;
  }

  void setup() {
    if (!is_initialized) {
      verifyAndFixFile();
      is_initialized = true;
    }
  }

  void commit() {
    if (dirty_) {
      if (writeToFile()) {
        dirty_ = false;
      } else {
        DEBUG_MSG("[NRF52Flash] Write failed");
      }
    }
  }

  void erase() {
    // format the internal filesystem
    InternalFS.format();
    // erase the in-memory contents
    memset(contents, _StorageProps::uninitialized_byte, _StorageProps::length);
  }
};

template<typename _StorageProps>
bool NRF52Flash<_StorageProps>::is_initialized = false;

template<typename _StorageProps>
Adafruit_LittleFS_Namespace::File NRF52Flash<_StorageProps>::file(InternalFS);

template<typename _StorageProps>
uint8_t *NRF52Flash<_StorageProps>::contents = nullptr;

template<typename _StorageProps>
bool NRF52Flash<_StorageProps>::dirty_ = false;

}  // namespace storage
}  // namespace driver
}  // namespace kaleidoscope

#endif
