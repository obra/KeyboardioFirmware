# Reset a bunch of historical GNU make implicit rules that we never
# use, but which have a disastrous impact on performance
#
# --no-builtin-rules in MAKEFLAGS apparently came in with GNU Make 4,
# which is newer than what Apple ships
MAKEFLAGS += --no-builtin-rules

# These lines reset the implicit rules we really care about
%:: %,v

%:: RCS/%,v

%:: RCS/%

%:: s.%

%:: SCCS/s.%

.SUFFIXES:

# GNU Make earlier than 4.0 don't have the output-sync option, but we need it
# to make parallel simulator test output readable. (otherwise it gets interleaved)
ifeq ($(shell test $(firstword $(subst ., ,$(MAKE_VERSION))) -ge 4; echo $$?),0)
MAKEFLAGS+=--output-sync=target
else
_using_old_make=1
endif

include $(dir $(abspath $(lastword $(MAKEFILE_LIST))))/etc/makefiles/arduino-cli.mk

# Set up an argument for passing to the simulator tests in docker
# but if the var isn't set, don't even pass the definition
# since this messes with downstream makefiles

ifneq ($(TEST_PATH),) 
TEST_PATH_ARG="TEST_PATH='$(TEST_PATH)'"
endif

.DEFAULT_GOAL := smoke-sketches

.PHONY: setup
setup: $(ARDUINO_CLI_PATH) $(ARDUINO_DIRECTORIES_DATA)/arduino-cli.yaml install-arduino-core-avr install-arduino-core-kaleidoscope checkout-platform update prepare-virtual  install-arduino-core-nrf52
	@:

.PHONY: checkout-platform
checkout-platform: $(ARDUINO_DIRECTORIES_USER)/hardware/keyboardio/avr/boards.txt
	@:

$(ARDUINO_DIRECTORIES_USER)/hardware/keyboardio/avr/boards.txt:
	git clone -c core.symlinks=true \
		--recurse-submodules \
		--shallow-submodules \
		--recurse-submodules=':(exclude)avr/bootloaders' \
		--recurse-submodules=':(exclude)avr/libraries/Kaleidoscope' \
		https://github.com/keyboardio/Kaleidoscope-Bundle-Keyboardio \
		$(ARDUINO_DIRECTORIES_USER)/hardware/keyboardio
	git clone -c core.symlinks=true \
		--recurse-submodules \
		--shallow-submodules \
		--recurse-submodules=':(exclude)libraries/Kaleidoscope' \
		https://github.com/keyboardio/ArduinoCore-GD32-Keyboardio $(ARDUINO_DIRECTORIES_USER)/hardware/keyboardio/gd32

.PHONY: prepare-virtual
prepare-virtual: $(ARDUINO_DIRECTORIES_USER)/hardware/keyboardio/virtual/boards.txt
	@:

$(ARDUINO_DIRECTORIES_USER)/hardware/keyboardio/virtual/boards.txt:
	$(MAKE) -C $(ARDUINO_DIRECTORIES_USER)/hardware/keyboardio prepare-virtual

.PHONY: update
update:
	cd $(ARDUINO_DIRECTORIES_USER)/hardware/keyboardio; git pull; \
		git submodule update --init --recursive
	cd $(ARDUINO_DIRECTORIES_USER)/hardware/keyboardio/gd32; git pull; \
		git submodule update --init --recursive

.PHONY: simulator-tests
simulator-tests:
ifneq ($(_using_old_make),)
	$(info You're using an older version of GNU Make that doesn't offer the --output-sync option. If you're running the test suite in parallel, output may be garbled. You might consider using GNU Make 4.0 or later instead)
endif
	$(MAKE) -C tests all

.PHONY: docker-simulator-tests
docker-simulator-tests:
	ARDUINO_DIRECTORIES_USER="$(ARDUINO_DIRECTORIES_USER)" ./bin/run-docker "make simulator-tests $(TEST_PATH_ARG)"

.PHONY: docker-clean
docker-clean:
	_NO_SYNC_KALEIDOSCOPE=1 ARDUINO_DIRECTORIES_USER="$(ARDUINO_DIRECTORIES_USER)" ./bin/run-docker "rm -rf -- testing/googletest/build/* _build/* /kaleidoscope-persist/temp/*"

.PHONY: docker-bash
docker-bash:
	_NO_SYNC_KALEIDOSCOPE=1 DOCKER_LIVE_KALEIDOSCOPE_DIR=1 ARDUINO_DIRECTORIES_USER="$(ARDUINO_DIRECTORIES_USER)" ./bin/run-docker "bash"

build-gtest-gmock:
	(cd testing/googletest && cmake -H. -Bbuild -DCMAKE_C_COMPILER=$(call _arduino_prop,compiler.path)$(call _arduino_prop,compiler.c.cmd) -DCMAKE_CXX_COMPILER=$(call _arduino_prop,compiler.path)$(call _arduino_prop,compiler.cpp.cmd)  .)
	$(MAKE) -C testing/googletest

.PHONY: find-filename-conflicts
find-filename-conflicts:
	bin/find-filename-conflicts.py src plugins/*

.PHONY: format
format:
	bin/format-code.py \
		--exclude-dir 'testing/googletest' \
		--exclude-file 'generated-testcase.cpp' \
		src plugins examples testing

.PHONY: check-code-style
check-code-style:
	bin/format-code.py \
		--exclude-dir 'testing/googletest' \
		--exclude-file 'generated-testcase.cpp' \
		--check \
		--verbose \
		src plugins examples testing

.PHONY: check-includes
check-includes:
	bin/fix-header-includes

.PHONY: check-all-includes
check-all-includes:
	bin/iwyu.py -v src plugins
	bin/iwyu.py -v \
		-I=$(KALEIDOSCOPE_DIR) \
		-I=$(KALEIDOSCOPE_DIR)/testing/googletest/googlemock/include \
		-I=$(KALEIDOSCOPE_DIR)/testing/googletest/googletest/include \
		testing
	bin/format-code.py -f -v --check \
		--exclude-dir 'testing/googletest' \
		src plugins testing

.PHONY: cpplint-noisy
cpplint-noisy:
	-bin/cpplint.py --config=.cpplint-noisy --recursive src plugins examples

.PHONY: cpplint
cpplint:
	bin/cpplint.py --config=.cpplint --quiet --recursive src plugins examples

.PHONY: shellcheck
shellcheck:
	bin/check-shell-scripts.sh

SMOKE_SKETCHES := $(sort $(shell if [ -d ./examples ]; then find ./examples -type f -name \*ino | xargs -n 1 dirname; fi))

smoke-sketches: $(SMOKE_SKETCHES)
	@echo "Smoke-tested all the sketches"

.PHONY: force
$(SMOKE_SKETCHES): force
	$(MAKE) -C $@ -f $(KALEIDOSCOPE_ETC_DIR)/makefiles/sketch.mk compile

.PHONY: clean
clean: 
	$(MAKE) -C tests clean
	rm -rf -- "testing/googletest/build/*"
	rm -rf -- "_build/*"

build-arduino-nightly-package:
	perl bin/build-arduino-package \
		--kaleidoscope-tag=master \
		--version `date +%Y.%-m.%-d%H%M%S` \
		--index-filename-slug=kaleidoscope_master \
		--only-one-platform-revision \
		--push \
		--output-repo=ssh://git@github.com/keyboardio/arduino-kaleidoscope-master
