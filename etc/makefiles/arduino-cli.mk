mkfile_dir := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))

ifeq ($(VERBOSE),)

ifeq ($(QUIET),)
$(info Building in quiet mode. For a lot more information, add 'VERBOSE=1' to the beginning of your call to $(MAKE))
export QUIET = @
endif

export ARDUINO_VERBOSE ?=
else
export ARDUINO_VERBOSE ?= --verbose
endif



# Build path config
TMPDIR ?= /tmp

ifeq ($(KALEIDOSCOPE_DIR),)
export KALEIDOSCOPE_DIR := $(abspath $(mkfile_dir)/../..)
endif

KALEIDOSCOPE_BIN_DIR ?= $(KALEIDOSCOPE_DIR)/bin
KALEIDOSCOPE_ETC_DIR ?= $(KALEIDOSCOPE_DIR)/etc

KALEIDOSCOPE_TEMP_PATH ?= $(TMPDIR)/kaleidoscope-$(USER)
KALEIDOSCOPE_BUILD_PATH ?= $(KALEIDOSCOPE_TEMP_PATH)/build
KALEIDOSCOPE_OUTPUT_PATH ?= $(KALEIDOSCOPE_TEMP_PATH)/output

CORE_CACHE_PATH ?= $(KALEIDOSCOPE_TEMP_PATH)/arduino-cores


ARDUINO_CONTENT ?= $(KALEIDOSCOPE_DIR)/.arduino
export ARDUINO_DIRECTORIES_DATA ?= $(ARDUINO_CONTENT)/data
export ARDUINO_DIRECTORIES_DOWNLOADS ?= $(ARDUINO_CONTENT)/downloads
export ARDUINO_CLI_CONFIG ?= $(ARDUINO_DIRECTORIES_DATA)/arduino-cli.yaml
export ARDUINO_BOARD_MANAGER_ADDITIONAL_URLS ?= https://raw.githubusercontent.com/keyboardio/boardsmanager/master/devel/package_kaleidoscope_devel_index.json


# If it looks like Kaleidoscope is inside a "traditional" Arduino hardware directory 
# in the user's homedir, let's use that.

ifeq ($(shell uname -s),Darwin)
traditional_path = $(HOME)/Documents/Arduino/
else
traditional_path = $(HOME)/Arduino/
endif

# use realpath to compare the real absolute path of the kaleidoscope dir
# and the arduino bundle, even if they're symlinked into the Arudino/hardware dir
ifeq ($(realpath $(traditional_path)/hardware/keyboardio/avr/libraries/Kaleidoscope), $(realpath $(KALEIDOSCOPE_DIR))) 
export ARDUINO_DIRECTORIES_USER ?= $(traditional_path)
endif
# Otherwise, use the arduino-cli bundle
export ARDUINO_DIRECTORIES_USER ?= $(ARDUINO_CONTENT)/user



arduino_env = ARDUINO_DIRECTORIES_USER=$(ARDUINO_DIRECTORIES_USER) \
	      ARDUINO_DIRECTORIES_DATA=$(ARDUINO_DIRECTORIES_DATA)

ifeq ($(ARDUINO_CLI_PATH),) 

system_arduino_cli := $(shell command -v arduino-cli || true)

ifeq ($(system_arduino_cli),) 
export ARDUINO_CLI_PATH ?= $(KALEIDOSCOPE_BIN_DIR)/arduino-cli
else
export ARDUINO_CLI_PATH ?= $(system_arduino_cli)
endif

endif

export ARDUINO_CLI ?= $(arduino_env) $(ARDUINO_CLI_PATH)

ifneq ($(VERBOSE),) 
$(info Using arduino-cli from $(ARDUINO_CLI_PATH))
endif

ifneq ($(FQBN),)
fqbn_arg = --fqbn $(FQBN)
endif

# if we don't have a sketch, make a pretend one so we can run --show properties
# This is because arduino-cli doesn't currently allow us to get props with
# just an FQBN. We've filed a bug with them
ifeq ($(SKETCH_FILE_PATH),) 
_arduino_props_sketch_arg	=	$(KALEIDOSCOPE_ETC_DIR)/dummy-sketch/
else
_arduino_props_sketch_arg	=	$(SKETCH_FILE_PATH)
endif

# This is horrible. But because make doesn't really support
# multi-line variables and we want to cache the full
# _arduino_props in a variable, which means letting make 
# split the properties on space, which is what it converts
# newlines into. To make this go, we we need to replace interior
# spaces in the variables with something. We chose the fire 
# emoji, since it accurately represents our feelings on this 
# state of affairs. Later, when finding props, we need to reverse 
# this process, turning fire into space.
_arduino_props := $(shell ${ARDUINO_CLI}  compile $(fqbn_arg) --show-properties=expanded "$(_arduino_props_sketch_arg)"|perl -p -e"s/ /🔥/g")

_arduino_prop = $(subst $1=,,$(subst 🔥, ,$(filter $1=%,$(_arduino_props))))

# How to use_arduino_prop
# $(call _arduino_prop,recipe.hooks.sketch.prebuild.2.pattern)

ifneq ($(KALEIDOSCOPE_CCACHE),) 
ccache_wrapper_property := --build-property compiler.wrapper.cmd=ccache
endif

.PHONY: configure-arduino-cli install-arduino-core-kaleidoscope install-arduino-core-avr install-arduino-core-nrf52
.PHONY: stupid-workaround-for-make-inclusion-semantics

stupid-workaround-for-make-inclusion-semantics:
	@: # This is here so that the sketch makefile including this file doesn't
	@: # default to arduino-cli installation as its priamry target
	@echo "No default target specified; aborting. Please set the .DEFAULT_GOAL variable in your makefile."

$(KALEIDOSCOPE_BIN_DIR)/arduino-cli:
	$(QUIET) curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR="$(KALEIDOSCOPE_BIN_DIR)" sh

install-arduino-cli: $(KALEIDOSCOPE_BIN_DIR)/arduino-cli

configure-arduino-cli: $(ARDUINO_DIRECTORIES_DATA)/arduino-cli.yaml

$(ARDUINO_DIRECTORIES_DATA)/arduino-cli.yaml:
	$(QUIET) $(ARDUINO_CLI) config init

arduino-update-cores:
	$(QUIET) $(ARDUINO_CLI) core update-index

install-arduino-core-kaleidoscope: arduino-update-cores
	$(QUIET) $(ARDUINO_CLI) core install "keyboardio:avr-tools-only"
	$(QUIET) $(ARDUINO_CLI) core install "keyboardio:gd32-tools-only"

install-arduino-core-avr: arduino-update-cores
	$(QUIET) $(ARDUINO_CLI) core install "arduino:avr"


install-arduino-core-nrf52: arduino-update-cores
	$(QUIET) ARDUINO_BOARD_MANAGER_ADDITIONAL_URLS=https://raw.githubusercontent.com/keyboardio/arduino-kaleidoscope-master/refs/heads/main/package_kaleidoscope_master_index.json \
	$(ARDUINO_CLI) core update-index
	$(QUIET) ARDUINO_BOARD_MANAGER_ADDITIONAL_URLS=https://raw.githubusercontent.com/keyboardio/arduino-kaleidoscope-master/refs/heads/main/package_kaleidoscope_master_index.json \
	$(ARDUINO_CLI) core install "keyboardio:nrf52"

install-arduino-core-deps:
	$(QUIET) $(ARDUINO_CLI) core install "keyboardio:avr-tools-only"
	$(QUIET) $(ARDUINO_CLI) core install "keyboardio:gd32-tools-only"


# On arm64 macs, we need rosetta to function
.PHONY: check-rosetta
check-rosetta:
	$(QUIET) if [ "$$(uname)" = "Darwin" ]; then \
		if [ "$$(uname -m)" = "arm64" ]; then \
			if ! pgrep oahd >/dev/null 2>&1; then \
				echo "Rosetta 2 is required but not installed. Please install Rosetta 2 and try again."; \
				exit 1; \
			fi; \
		fi; \
	fi



# If we're not calling setup, we should freak out if the hardware
# definitions don't exist

.PHONY: kaleidoscope-hardware-configured

kaleidoscope-hardware-configured: $(ARDUINO_DIRECTORIES_USER)/hardware/keyboardio/avr/platform.txt

$(ARDUINO_DIRECTORIES_USER)/hardware/keyboardio/avr/platform.txt:
	$(info Kaleidoscope hardware definitions not found in)
	$(info $(ARDUINO_DIRECTORIES_USER))
	$(info )
	$(info You may be able to resolve this issue by running the following command)
	$(info to initialize Kaleidoscope )
	$(info )
	$(info $(MAKE) -C $(KALEIDOSCOPE_DIR) setup )
	$(info )
	$(error )
