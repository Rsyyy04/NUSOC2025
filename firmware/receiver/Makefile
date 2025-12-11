# Makefile for Anti-Loss System Receiver Firmware
# Based on nRF5 SDK

PROJECT_NAME := antiloss_receiver
TARGETS      := $(PROJECT_NAME)
OUTPUT_DIRECTORY := _build

# nRF5 SDK路径（需要根据实际情况修改）
SDK_ROOT := ../../../nRF5_SDK_17.1.0_ddde560
PROJ_DIR := .

# 源文件
SRC_FILES += \
  $(PROJ_DIR)/src/main.c \
  $(PROJ_DIR)/src/ble_central.c \
  $(PROJ_DIR)/src/tag_manager.c \
  $(PROJ_DIR)/src/alarm.c \
  $(PROJ_DIR)/src/search_mode.c \
  $(PROJ_DIR)/src/oled_ui.c \
  $(PROJ_DIR)/src/storage.c \

# nRF5 SDK源文件
SRC_FILES += \
  $(SDK_ROOT)/modules/nrfx/mdk/gcc_startup_nrf52.S \
  $(SDK_ROOT)/modules/nrfx/mdk/system_nrf52.c \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_backend_rtt.c \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_backend_serial.c \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_default_backends.c \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_frontend.c \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_str_formatter.c \
  $(SDK_ROOT)/components/libraries/util/app_error.c \
  $(SDK_ROOT)/components/libraries/util/app_error_handler_gcc.c \
  $(SDK_ROOT)/components/libraries/util/app_error_weak.c \
  $(SDK_ROOT)/components/libraries/util/app_util_platform.c \
  $(SDK_ROOT)/components/libraries/util/nrf_assert.c \
  $(SDK_ROOT)/components/libraries/atomic/nrf_atomic.c \
  $(SDK_ROOT)/components/libraries/balloc/nrf_balloc.c \
  $(SDK_ROOT)/components/libraries/memobj/nrf_memobj.c \
  $(SDK_ROOT)/components/libraries/ringbuf/nrf_ringbuf.c \
  $(SDK_ROOT)/components/libraries/scheduler/app_scheduler.c \
  $(SDK_ROOT)/components/libraries/timer/app_timer2.c \
  $(SDK_ROOT)/components/libraries/sortlist/nrf_sortlist.c \
  $(SDK_ROOT)/components/libraries/strerror/nrf_strerror.c \
  $(SDK_ROOT)/components/libraries/pwr_mgmt/nrf_pwr_mgmt.c \

# 包含目录
INC_FOLDERS += \
  $(PROJ_DIR)/inc \
  $(PROJ_DIR)/config \
  $(SDK_ROOT)/components \
  $(SDK_ROOT)/modules/nrfx/mdk \
  $(SDK_ROOT)/components/libraries/strerror \
  $(SDK_ROOT)/components/libraries/util \
  $(SDK_ROOT)/components/libraries/log \
  $(SDK_ROOT)/components/libraries/log/src \
  $(SDK_ROOT)/components/libraries/scheduler \
  $(SDK_ROOT)/components/libraries/timer \
  $(SDK_ROOT)/components/libraries/atomic \
  $(SDK_ROOT)/components/libraries/balloc \
  $(SDK_ROOT)/components/libraries/memobj \
  $(SDK_ROOT)/components/libraries/ringbuf \
  $(SDK_ROOT)/components/libraries/sortlist \
  $(SDK_ROOT)/components/libraries/pwr_mgmt \
  $(SDK_ROOT)/components/softdevice/s132/headers \
  $(SDK_ROOT)/components/softdevice/s132/headers/nrf52 \
  $(SDK_ROOT)/components/ble/common \
  $(SDK_ROOT)/integration/nrfx \
  $(SDK_ROOT)/modules/nrfx \
  $(SDK_ROOT)/modules/nrfx/hal \
  $(SDK_ROOT)/modules/nrfx/drivers/include \

# 库文件
LIB_FILES += \

# C编译标志
CFLAGS += -DBOARD_CUSTOM
CFLAGS += -DSOFTDEVICE_PRESENT
CFLAGS += -DNRF52832_XXAA
CFLAGS += -DNRF_SD_BLE_API_VERSION=7
CFLAGS += -DS132
CFLAGS += -DCONFIG_GPIO_AS_PINRESET
CFLAGS += -DFLOAT_ABI_HARD
CFLAGS += -mcpu=cortex-m4
CFLAGS += -mthumb -mabi=aapcs
CFLAGS += -Wall -Werror
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin -fshort-enums

# C++编译标志
CXXFLAGS += $(CFLAGS)

# 汇编标志
ASMFLAGS += -g3
ASMFLAGS += -mcpu=cortex-m4
ASMFLAGS += -mthumb -mabi=aapcs
ASMFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
ASMFLAGS += -DBOARD_CUSTOM
ASMFLAGS += -DSOFTDEVICE_PRESENT
ASMFLAGS += -DNRF52832_XXAA

# 链接标志
LDFLAGS += -mthumb -mabi=aapcs -L$(SDK_ROOT)/modules/nrfx/mdk -T$(PROJ_DIR)/linker_script.ld
LDFLAGS += -mcpu=cortex-m4
LDFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
LDFLAGS += -Wl,--gc-sections
LDFLAGS += --specs=nano.specs

# 优化级别
OPT = -O3 -g3

# 包含SDK Makefile
include $(SDK_ROOT)/components/toolchain/gcc/Makefile.common

# 默认目标
default: $(TARGETS)

# 清除编译文件
clean:
	rm -rf $(OUTPUT_DIRECTORY)

# Flash固件
flash: default
	@echo Flashing: $(OUTPUT_DIRECTORY)/$(PROJECT_NAME).hex
	nrfjprog -f nrf52 --program $(OUTPUT_DIRECTORY)/$(PROJECT_NAME).hex --sectorerase
	nrfjprog -f nrf52 --reset

# Flash SoftDevice
flash_softdevice:
	@echo Flashing: s132_nrf52_7.3.0_softdevice.hex
	nrfjprog -f nrf52 --program $(SDK_ROOT)/components/softdevice/s132/hex/s132_nrf52_7.3.0_softdevice.hex --chiperase
	nrfjprog -f nrf52 --reset

# 擦除芯片
erase:
	nrfjprog -f nrf52 --eraseall

.PHONY: default clean flash flash_softdevice erase
