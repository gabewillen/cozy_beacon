PROJECT_NAME     := nrf52811_xxaa
TARGETS          := nrf52811_xxaa
OUTPUT_DIRECTORY := _build

SDK_ROOT := ./nRF5_SDK_17.0.2
PROJ_DIR := ./


$(OUTPUT_DIRECTORY)/nrf52811_xxaa.out: \
  LINKER_SCRIPT  := generic_gcc_nrf52.ld

# SDK_CONFIG BEGIN
SEGGER_RTT := yes
BLE_ADVERTISING := no
LIB_LOG := rtt
LIB_PWR_MGMT := yes
NRFX_TEMP := yes
# SDK_CONFIG END

INC_FOLDERS :=
SRC_FILES :=



ifeq ($(SEGGER_RTT), yes)
	SRC_FILES += \
		$(SDK_ROOT)/external/segger_rtt/SEGGER_RTT_printf.c \
		$(SDK_ROOT)/external/segger_rtt/SEGGER_RTT.c \
      	$(SDK_ROOT)/external/segger_rtt/SEGGER_RTT_Syscalls_GCC.c \

	INC_FOLDERS += \
		  $(SDK_ROOT)/external/segger_rtt \

endif

ifdef $(BLE_ADVERTISING)
	SRC_FILES += \
		$(SDK_ROOT)/components/ble/ble_advertising.c
	INC_FOLDERS += \
		$(SDK_ROOT)/components/ble/ble_advertising
endif

ifdef $(LIB_LOG)
	CFLAGS += -DNRF_LOG_ENABLED=1
	INC_FOLDERS += \
		$(SDK_ROOT)/components/libraries/log
endif

ifeq ($(LIB_LOG), rtt)
	CFLAGS += -DNRF_LOG_BACKEND_RTT_ENABLED=1
	SRC_FILES += \
		$(SDK_ROOT)/components/libraries/log/src/nrf_log_default_backends.c \
		$(SDK_ROOT)/components/libraries/log/src/nrf_log_backend_rtt.c \
		$(SDK_ROOT)/components/libraries/log/src/nrf_log_backend_serial.c
endif

ifeq ($(LIB_PWR_MGMT), yes)
	CFLAGS += -DNRF_PWR_MGMT_ENABLED=1
	SRC_FILES += \
		$(SDK_ROOT)/components/libraries/pwr_mgmt/nrf_pwr_mgmt.c
	INC_FOLDERS += \
		$(SDK_ROOT)/components/libraries/pwr_mgmt/
endif


ifeq ($(NRFX_TEMP), yes)
	SRC_FILES += \
		$(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_temp.c
endif

# Source files common to all targets
EXTERNAL_SRC_FILES := \
	  $(SDK_ROOT)/external/fprintf/nrf_fprintf.c \
      $(SDK_ROOT)/external/fprintf/nrf_fprintf_format.c \



SRC_FILES += \
  $(EXTERNAL_SRC_FILES) \
  $(SDK_ROOT)/components/ble/common/ble_advdata.c \
  $(SDK_ROOT)/components/ble/common/ble_conn_params.c \
  $(SDK_ROOT)/components/ble/common/ble_srv_common.c \
  $(SDK_ROOT)/components/libraries/util/app_error.c \
    $(SDK_ROOT)/components/libraries/util/app_error_handler_gcc.c \
    $(SDK_ROOT)/components/libraries/util/app_error_weak.c \
  $(SDK_ROOT)/components/softdevice/common/nrf_sdh.c \
  $(SDK_ROOT)/components/softdevice/common/nrf_sdh_ble.c \
  $(SDK_ROOT)/components/softdevice/common/nrf_sdh_soc.c \
  $(SDK_ROOT)/modules/nrfx/mdk/gcc_startup_nrf52811.S \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_frontend.c \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_str_formatter.c \
  $(SDK_ROOT)/components/libraries/scheduler/app_scheduler.c \
  $(SDK_ROOT)/components/libraries/timer/app_timer2.c \
  $(SDK_ROOT)/components/libraries/util/app_util_platform.c \
  $(SDK_ROOT)/components/libraries/timer/drv_rtc.c \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_backend_rtt.c \
  $(SDK_ROOT)/components/libraries/hardfault/hardfault_implementation.c \
  $(SDK_ROOT)/components/libraries/util/nrf_assert.c \
  $(SDK_ROOT)/components/libraries/atomic_fifo/nrf_atfifo.c \
  $(SDK_ROOT)/components/libraries/atomic/nrf_atomic.c \
  $(SDK_ROOT)/components/libraries/balloc/nrf_balloc.c \
  $(SDK_ROOT)/components/libraries/memobj/nrf_memobj.c \
  $(SDK_ROOT)/components/libraries/ringbuf/nrf_ringbuf.c \
  $(SDK_ROOT)/components/libraries/experimental_section_vars/nrf_section_iter.c \
  $(SDK_ROOT)/components/libraries/sortlist/nrf_sortlist.c \
  $(SDK_ROOT)/components/libraries/strerror/nrf_strerror.c \
  $(SDK_ROOT)/integration/nrfx/legacy/nrf_drv_clock.c \
  $(SDK_ROOT)/modules/nrfx/soc/nrfx_atomic.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_clock.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_saadc.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_twi.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_gpiote.c \
  $(PROJ_DIR)/main.c \
  $(PROJ_DIR)/src/app_saadc.c \
  $(PROJ_DIR)/src/app_advertising.c \
  $(PROJ_DIR)/src/app_sensors.c \
  $(PROJ_DIR)/src/spl06_007.cpp \
  $(SDK_ROOT)/modules/nrfx/mdk/system_nrf52811.c \
  $(SDK_ROOT)/external/utf_converter/utf.c \
  $(SDK_ROOT)/components/softdevice/common/nrf_sdh.c \
  $(SDK_ROOT)/components/softdevice/common/nrf_sdh_ble.c \
  $(SDK_ROOT)/components/softdevice/common/nrf_sdh_soc.c \

EXTERNAL_INC_FOLDERS := \
	  $(SDK_ROOT)/external/utf_converter \

# Include folders common to all targets
INC_FOLDERS += \
	$(EXTERNAL_INC_FOLDERS) \
  $(SDK_ROOT)/components/libraries/sdcard \
  $(SDK_ROOT)/components/libraries/bootloader/ble_dfu \
  $(SDK_ROOT)/components/ble/ble_services/ble_tps \
  $(SDK_ROOT)/components/libraries/experimental_section_vars \
  $(SDK_ROOT)/components/libraries/crc32 \
  $(SDK_ROOT)/components/libraries/csense_drv \
  $(SDK_ROOT)/components/libraries/fstorage \
  $(SDK_ROOT)/components/libraries/cli \
  $(SDK_ROOT)/components/ble/ble_services/ble_lbs \
  $(SDK_ROOT)/components/libraries/atomic_fifo \
  $(SDK_ROOT)/components/ble/ble_services/ble_cts_c \
  $(SDK_ROOT)/components/softdevice/s112/headers/nrf52 \
  $(SDK_ROOT)/components/ble/ble_services/ble_rscs_c \
  $(SDK_ROOT)/components/libraries/scheduler \
  $(SDK_ROOT)/components/libraries/memobj \
  $(SDK_ROOT)/components/ble/ble_services/ble_nus \
  $(SDK_ROOT)/components/libraries/hci \
  $(SDK_ROOT)/components/libraries/queue \
  $(SDK_ROOT)/components/ble/ble_services/ble_hrs \
  $(SDK_ROOT)/components/libraries/mem_manager \
  $(SDK_ROOT)/components/libraries/usbd/class/hid \
  $(SDK_ROOT)/components/libraries/mutex \
  $(SDK_ROOT)/components/ble/peer_manager \
  $(SDK_ROOT)/components/ble/ble_services/ble_bas \
  $(SDK_ROOT)/components/ble/ble_services/ble_ias_c \
  $(SDK_ROOT)/components/softdevice/common \
  $(SDK_ROOT)/components/libraries/mpu \
  $(SDK_ROOT)/components/libraries/usbd/class/audio \
  $(SDK_ROOT)/components/libraries/experimental_task_manager \
  $(SDK_ROOT)/components/libraries/stack_guard \
  $(SDK_ROOT)/components/libraries/usbd/class/cdc/acm \
  $(SDK_ROOT)/components/libraries/util \
  $(SDK_ROOT)/components/libraries/timer \
  $(SDK_ROOT)/integration/nrfx \
  $(SDK_ROOT)/components/libraries/spi_mngr \
  $(SDK_ROOT)/components/libraries/usbd/class/cdc \
  $(SDK_ROOT)/components/libraries/gfx \
  $(SDK_ROOT)/components/ble/ble_services/ble_dfu \
  $(SDK_ROOT)/components/ble/ble_dtm \
  $(SDK_ROOT)/components/libraries/usbd/class/hid/generic \
  $(SDK_ROOT)/components/libraries/usbd/class/hid/kbd \
  $(SDK_ROOT)/modules/nrfx/drivers/include \
  $(SDK_ROOT)/components/toolchain/cmsis/include \
  $(SDK_ROOT)/components/ble/ble_services/ble_rscs \
  $(SDK_ROOT)/components/libraries/balloc \
  $(SDK_ROOT)/components/libraries/sortlist \
  $(SDK_ROOT)/components/ble/nrf_ble_qwr \
  $(SDK_ROOT)/components/libraries/usbd/class/hid/mouse \
  $(SDK_ROOT)/components/libraries/pwm \
  $(SDK_ROOT)/components/libraries/ringbuf \
  $(SDK_ROOT)/components/libraries/bsp \
  $(SDK_ROOT)/components/boards \
  $(SDK_ROOT)/components/libraries/slip \
  $(SDK_ROOT)/components/ble/ble_services/ble_hids \
  $(SDK_ROOT)/components/ble/ble_advertising \
  ./include \
  $(SDK_ROOT)/modules/nrfx/hal \
  $(SDK_ROOT)/components/libraries/pwr_mgmt \
  $(SDK_ROOT)/components/ble/ble_services/ble_nus_c \
  $(SDK_ROOT)/modules/nrfx \
  $(SDK_ROOT)/components/libraries/csense \
  $(SDK_ROOT)/components/libraries/fds \
  $(SDK_ROOT)/components/libraries/delay \
  $(SDK_ROOT)/components/ble/ble_services/ble_lbs_c \
  $(SDK_ROOT)/integration/nrfx/legacy \
  $(SDK_ROOT)/components/libraries/usbd \
  $(SDK_ROOT)/components/libraries/log \
  $(SDK_ROOT)/components/ble/common \
  $(SDK_ROOT)/components/libraries/usbd/class/msc \
  $(SDK_ROOT)/external/fprintf \
  $(SDK_ROOT)/components/libraries/crypto \
  $(SDK_ROOT)/components \
  $(SDK_ROOT)/components/libraries/log/src \
  $(SDK_ROOT)/components/ble/ble_services/ble_hrs_c \
  $(SDK_ROOT)/components/ble/ble_services/ble_ancs_c \
  $(SDK_ROOT)/components/libraries/ecc \
  $(SDK_ROOT)/components/ble/ble_services/ble_cscs \
  $(SDK_ROOT)/components/ble/ble_services/ble_dis \
  $(SDK_ROOT)/components/ble/ble_services/ble_ans_c \
  $(SDK_ROOT)/components/libraries/crc16 \
  $(SDK_ROOT)/components/libraries/low_power_pwm \
  $(SDK_ROOT)/components/softdevice/s112/headers \
  $(SDK_ROOT)/components/libraries/led_softblink \
  $(SDK_ROOT)/components/ble/ble_services/ble_bas_c \
  $(SDK_ROOT)/modules/nrfx/mdk \
  $(SDK_ROOT)/components/libraries/atomic \
  $(SDK_ROOT)/components/ble/ble_services/ble_gls \
  $(SDK_ROOT)/components/ble/ble_racp \
  $(SDK_ROOT)/components/ble/ble_services/ble_hts \
  $(SDK_ROOT)/components/ble/ble_services/ble_ias \
  $(SDK_ROOT)/components/libraries/svc \
  $(SDK_ROOT)/components/libraries/button \
  $(SDK_ROOT)/components/libraries/strerror \
  $(SDK_ROOT)/components/ble/ble_services/ble_lls \
  $(SDK_ROOT)/components/libraries/gpiote \
  $(SDK_ROOT)/components/libraries/hardfault \


$(info $(INC_FOLDERS))
$(info $(SRC_FILES))
# Libraries common to all targets
LIB_FILES += \

# Optimization flags
OPT = -g3

# Uncomment the line below to enable link time optimization
#OPT += -flto

# C flags common to all targets
CFLAGS += $(OPT)
CFLAGS += -DAPP_TIMER_V2
CFLAGS += -DUSE_APP_CONFIG
CFLAGS += -DAPP_TIMER_V2_RTC1_ENABLED
CFLAGS += -DFLOAT_ABI_SOFT
CFLAGS += -DNRF52811_XXAA
CFLAGS += -DNRF_SD_BLE_API_VERSION=7
CFLAGS += -DS112
CFLAGS += -DSOFTDEVICE_PRESENT
CFLAGS += -mcpu=cortex-m4
CFLAGS += -DBOARD_CUSTOM
CFLAGS += -mthumb -mabi=aapcs
CFLAGS += -Wall -Werror
CFLAGS += -mfloat-abi=soft
# keep every function in a separate section, this allows linker to discard unused ones
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin -fshort-enums

# C++ flags common to all targets
CXXFLAGS += $(OPT)
# Assembler flags common to all targets
ASMFLAGS += -g3
ASMFLAGS += -mcpu=cortex-m4
ASMFLAGS += -mthumb -mabi=aapcs
ASMFLAGS += -mfloat-abi=soft
ASMFLAGS += -DAPP_TIMER_V2
ASMFLAGS += -DAPP_TIMER_V2_RTC1_ENABLED
ASMFLAGS += -DFLOAT_ABI_SOFT
ASMFLAGS += -DNRF52811_XXAA
ASMFLAGS += -DNRF_SD_BLE_API_VERSION=7
ASMFLAGS += -DS112
ASMFLAGS += -DSOFTDEVICE_PRESENT

# Linker flags
LDFLAGS += $(OPT)
LDFLAGS += -mthumb -mabi=aapcs -L$(SDK_ROOT)/modules/nrfx/mdk -T$(LINKER_SCRIPT)
LDFLAGS += -mcpu=cortex-m4
# let linker dump unused sections
LDFLAGS += -Wl,--gc-sections
# use newlib in nano version
LDFLAGS += --specs=nano.specs

nrf52811_xxaa: CFLAGS += -D__HEAP_SIZE=4096
nrf52811_xxaa: CFLAGS += -D__STACK_SIZE=4096
nrf52811_xxaa: ASMFLAGS += -D__HEAP_SIZE=4096
nrf52811_xxaa: ASMFLAGS += -D__STACK_SIZE=4096

# Add standard libraries at the very end of the linker input, after all objects
# that may need symbols provided by these libraries.
LIB_FILES += -lc -lnosys -lm


.PHONY: default help

# Default target - first one defined
default: nrf52811_xxaa

# Print all targets that can be built
help:
	@echo following targets are available:
	@echo		nrf52811_xxaa
	@echo		flash_softdevice
	@echo		sdk_config - starting external tool for editing sdk_config.h
	@echo		flash      - flashing binary

TEMPLATE_PATH := $(SDK_ROOT)/components/toolchain/gcc


include $(TEMPLATE_PATH)/Makefile.common

$(foreach target, $(TARGETS), $(call define_target, $(target)))

.PHONY: flash flash_softdevice erase

# Flash the program
flash: default
	@echo Flashing: $(OUTPUT_DIRECTORY)/nrf52811_xxaa.hex
	JFlashExe -open$(OUTPUT_DIRECTORY)/nrf52811_xxaa.hex -auto -verify -startapp -exit

rtt: flash
	JLinkRTTViewerExe

# Flash softdevice
flash_softdevice:
	@echo Flashing: s112_nrf52_7.0.1_softdevice.hex
	JFlashExe -open$(SDK_ROOT)/components/softdevice/s112/hex/s112_nrf52_7.2.0_softdevice.hex -auto -startapp -exit

erase:
	nrfjprog -f nrf52 --eraseall

SDK_CONFIG_FILE := ./include/sdk_config.h
CMSIS_CONFIG_TOOL := $(SDK_ROOT)/external_tools/cmsisconfig/CMSIS_Configuration_Wizard.jar
sdk_config:
	java -jar $(CMSIS_CONFIG_TOOL) $(SDK_CONFIG_FILE)
