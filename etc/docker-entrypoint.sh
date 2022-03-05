#! /bin/bash
set -e

install -d /kaleidoscope/ \
           /kaleidoscope/.arduino/user/hardware/keyboardio \
           /kaleidoscope-persist/temp  \
           /kaleidoscope-persist/ccache/cache


echo "Syncing Kaleidoscope..."
tar xf /kaleidoscope-src/kaleidoscope.tar -C /kaleidoscope/

ln -s  /kaleidoscope /kaleidoscope/.arduino/user/hardware/keyboardio/avr/libraries/Kaleidoscope
ln -s  /kaleidoscope /kaleidoscope/.arduino/user/hardware/keyboardio/gd32/libraries/Kaleidoscope

cd /kaleidoscope/
export ARDUINO_DIRECTORIES_DATA=/arduino-cli/data
export KALEIDOSCOPE_CCACHE=1

/bin/bash -c "$*"
