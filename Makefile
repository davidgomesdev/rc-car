APP_NAME ?= rc-car
CARGO ?= cargo
ESPFLASH ?= espflash
TARGET ?= xtensa-esp32s3-espidf
HOST_TARGET ?= $(shell rustc -vV | sed -n 's/^host: //p')
PROFILE ?= release
BAUD ?= 460800
MONITOR_BAUD ?= 115200
PORT ?=
EXTRA_FLASH_ARGS ?=
ESP_ENV_SCRIPT ?= ${HOME}/export-esp.sh

ifeq ($(PROFILE),release)
PROFILE_FLAG := --release
PROFILE_DIR := release
else
PROFILE_FLAG :=
PROFILE_DIR := debug
endif

PORT_ARG := $(if $(PORT),--port $(PORT),)
ESP_BIN := target/$(TARGET)/$(PROFILE_DIR)/$(APP_NAME)
HOST_BIN := target/$(HOST_TARGET)/debug/$(APP_NAME)

.PHONY: help source-tool check-rust check-target check-espflash build-host build-esp flash monitor clean

source-tool:
	@. "$(ESP_ENV_SCRIPT)" && export ESP_IDF_TOOLS_INSTALL_DIR=global

check-espflash:
	@command -v $(ESPFLASH) >/dev/null || \
		(echo "Error: $(ESPFLASH) not found in PATH" && \
		echo "Install with: cargo install espflash" && exit 1)

build-host:
	$(CARGO) build --target $(HOST_TARGET)
	@echo "Host binary: $(HOST_BIN)"

build-esp:
	@. "$(ESP_ENV_SCRIPT)" && \
	export ESP_IDF_TOOLS_INSTALL_DIR=global && \
	$(CARGO) +esp build -Zbuild-std=std,panic_abort --target $(TARGET) $(PROFILE_FLAG)
	@echo "Firmware binary: $(ESP_BIN)"

flash: build-esp check-espflash
	$(ESPFLASH) flash $(PORT_ARG) --baud $(BAUD) $(EXTRA_FLASH_ARGS) $(ESP_BIN)

monitor: check-espflash
	$(ESPFLASH) monitor $(PORT_ARG) --baud $(MONITOR_BAUD)

clean:
	$(CARGO) clean
