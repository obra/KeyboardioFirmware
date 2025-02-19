# This makefile for a Kaleidoscope sketch pulls in all the targets
# required to build the example




ifneq ($(KALEIDOSCOPE_DIR),)
search_path += $(KALEIDOSCOPE_DIR)
endif

ifneq ($(ARDUINO_DIRECTORIES_USER),)
search_path += $(ARDUINO_DIRECTORIES_USER)/hardware/keyboardio/avr/libraries/Kaleidoscope
endif

ifeq ($(shell uname -s),Darwin)
search_path += $(HOME)/Documents/Arduino/hardware/keyboardio/avr/libraries/Kaleidoscope
else
search_path += $(HOME)/Arduino/hardware/keyboardio/avr/libraries/Kaleidoscope
endif

sketch_makefile := etc/makefiles/sketch.mk

$(foreach candidate, $(search_path), $(if $(wildcard $(candidate)/$(sketch_makefile)), $(eval ks_dir ?= $(candidate))))

ifneq ($(ks_dir),)

$(info Using Kaleidoscope from $(ks_dir))

export KALEIDOSCOPE_DIR := $(ks_dir)
include $(ks_dir)/$(sketch_makefile)

else

$(info I can't find your Kaleidoscope installation.)
$(info )
$(info I tried looking in:)
$(info )
$(foreach candidate, $(search_path), $(info $(candidate)))
$(info )
$(info The easiest way to fix this is to set the 'KALEIDOSCOPE_DIR' environment)
$(info variable to the location of your Kaleidoscope directory.)

endif


null-target:
	$(info You should never see this message)
	@:
