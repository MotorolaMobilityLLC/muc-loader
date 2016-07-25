# Copyright (c) 2016 Motorola Mobility, LLC.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from this
# software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
# OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
#
TOPDIR := ${shell pwd}

OUT_DIR      = out
DEP_DIR      = dep
INC_DIR      = include
SRC_DIR      = src
CFG_DIR      = configs
MAN_DIR      = manifests

include $(TOPDIR)/.config
include $(TOPDIR)/Make.defs

CUBE_DIR         = $(CONFIG_CUBE_DIR)
CHIPSET          = $(CONFIG_CHIPSET)
CHIPSET_LC       = $(CONFIG_CHIPSET_LC)
TARGET_DEVICE    = $(CONFIG_TARGET_DEVICE)
TARGET_DEVICE_LC = $(CONFIG_TARGET_DEVICE_LC)
BOARD            = $(CONFIG_BOARD)

HAL_DIR      = $(CUBE_DIR)/Drivers/$(CHIPSET)_HAL_Driver
CMSIS_DIR    = $(CUBE_DIR)/Drivers/CMSIS
CMSIS_DIR_ST = $(CMSIS_DIR)/Device/ST/$(CHIPSET)

PREFIX     = arm-none-eabi
CC         = $(PREFIX)-gcc
AR         = $(PREFIX)-ar
OBJCOPY    = $(PREFIX)-objcopy
OBJDUMP    = $(PREFIX)-objdump
SIZE       = $(PREFIX)-size
NM         = $(PREFIX)-nm

# Defines
DEFS       = -D$(TARGET_DEVICE)
DEFS      += -DUSE_HAL_DRIVER
DEFS      += -DUSE_DBPRINTF
DEFS      += $(SIG_VALIDATION)

ifeq ($(CONFIG_DEBUG),y)
DEFS      += -D_DEBUG
endif

# core version represents the slego part configuration
CONFIG_ROOT_VERSION ?= 0

DEFS      += -DMOD_TYPE_$(CONFIG_MOD_TYPE)
DEFS      += -DMOD_BOARDID_PID=$(CONFIG_ARCH_BOARDID_PID)
DEFS      += -DMOD_BOARDID_VID=$(CONFIG_ARCH_BOARDID_VID)
ifeq ($(CONFIG_SLAVE_APBE),y)
DEFS      += -DMOD_SLAVE_APBE
endif
# do we set the interrupt vector in flash or ram
# default is to run from ram (normal bootloader
# behavior)
ifneq ($(CONFIG_RUN_FROM_FLASH),y)
 DEFS      += -DVECT_TAB_SRAM
endif

INCS       = -I$(HAL_DIR)/Inc/
INCS      += -I$(CFG_DIR)/$(CONFIG_MOD_TYPE)/$(INC_DIR)/
INCS      += -I.
INCS      += -I$(INC_DIR)/
INCS      += -I$(CMSIS_DIR)/Include/
INCS      += -I$(CMSIS_DIR)/Device/ST/$(CHIPSET)/Include
INCS      += -I$(OUT_DIR)/include
INCS      += -I$(SRC_DIR)/

CFLAGS     = -Wall -g -std=c99 -Os -Werror
CFLAGS    += -mlittle-endian -mcpu=cortex-m4 -march=armv7e-m -mthumb
CFLAGS    += -include $(OUT_DIR)/include/config.h
ifeq ($(CONFIG_ARCH_HAS_HW_FLOAT),y)
 CFLAGS    += -mfpu=fpv4-sp-d16 -mfloat-abi=hard
else
 CFLAGS    += -mfloat-abi=soft
endif
CFLAGS    += -ffunction-sections -fdata-sections
CFLAGS    += $(INCS) $(DEFS)

CSRCS       = main.c
CSRCS      += system_$(CHIPSET_LC).c
CSRCS      += $(CHIPSET_LC)_hal_msp.c
CSRCS      += $(CHIPSET_LC)_it.c
CSRCS      += $(CHIPSET_LC)_flash.c

CSRCS      += \
	      utils.c \
	      gbcore.c \
	      ctrl.c \
	      modsctrl.c \
	      gbfirmware.c \
	      network.c \
	      tftf.c \
	      crypto.c \
	      public_keys.c \
	      mod_ids.c \
	      mnfb.c \
	      $(CHIPSET_LC)_muc.c \
	      $(CHIPSET_LC)_mod_device.c \
	      $(CHIPSET_LC)_hal_mod.c

ifeq ($(CONFIG_DATALINK_SPI),y)
CSRCS      += dl_spi.c
endif

ifeq ($(CONFIG_DATALINK_I2C),y)
CSRCS      += dl_i2c.c \
              lib_crc16_poly8005.c
endif

ifeq ($(CONFIG_APBE_FLASH),y)
CSRCS      += spi_write_w25q40.c \
	      spi_flash.c
endif

ifeq ($(CONFIG_SLAVE_APBE),y)
CSRCS      += apbe.c
endif

# Basic HAL libraries
CSRCS      += $(CHIPSET_LC)_hal_rcc.c \
              $(CHIPSET_LC)_hal_rcc_ex.c \
              $(CHIPSET_LC)_hal.c \
              $(CHIPSET_LC)_hal_cortex.c \
              $(CHIPSET_LC)_hal_gpio.c  \
              $(CHIPSET_LC)_hal_i2c.c \
              $(CHIPSET_LC)_hal_spi.c  \
              $(CHIPSET_LC)_hal_dma.c  \
              $(CHIPSET_LC)_hal_flash.c \
              $(CHIPSET_LC)_hal_flash_ex.c \
              $(CHIPSET_LC)_hal_uart_ex.c \
              $(CHIPSET_LC)_hal_pwr_ex.c

ifdef DEVICE_CSRCS
 CSRCS += $(DEVICE_CSRCS)
endif

ifeq ($(CONFIG_DEBUG),y)
CSRCS      += debug.c
CSRCS      += $(CHIPSET_LC)_hal_uart.c

ifeq ($(CONFIG_RAMLOG),y)
CSRCS      += ramlog.c
endif
endif

# STARTUP_S should be defined in the product Make.defs file
SSRCS       = $(STARTUP_S)

VPATH      = ./src
VPATH     += ./src/greybus
VPATH     += $(HAL_DIR)/Src
VPATH     += $(DEV_DIR)/Source
VPATH     += $(CFG_DIR)/$(CONFIG_MOD_TYPE)/src
VPATH     += $(MAN_DIR)/

LIBS       = -L$(CMSIS_DIR)/Lib

TARGET       = boot_$(CONFIG_MOD_TYPE)

# Linker flags
LDFLAGS    = -Wl,--gc-sections -g
LDFLAGS   += $(LIBS) -T$(CFG_DIR)/$(CONFIG_MOD_TYPE)/scripts/$(LDSCRIPT)

# Enable Semihosting
LDFLAGS   += --specs=rdimon.specs -lc -lrdimon

OBJS    = $(addprefix $(OUT_DIR)/,$(CSRCS:.c=.o)) $(addprefix $(OUT_DIR)/,$(SSRCS:.s=.o))
DEPS    = $(addprefix $(DEP_DIR)/,$(CSRCS:.c=.d)) $(addprefix $(OUT_DIR)/,$(SSRCS:.s=.d))

$(OUT_DIR)/%.o : %.c $(DEP_DIR) $(OUT_DIR) $(OUT_DIR)/include/version.h $(OUT_DIR)/include/config.h
	@echo "CC:      $(notdir $<)"
	$(CC) $(CFLAGS) -c -o $@ $< -MMD -MF $(DEP_DIR)/$(*F).d

$(OUT_DIR)/%.o : %.s $(DEP_DIR) $(OUT_DIR)
	@echo "CC:      $(notdir $<)"
	$(CC) $(CFLAGS) -c -o $@ $< -MMD -MF $(DEP_DIR)/$(*F).d

all:  $(OUT_DIR)/$(TARGET).bin $(OUT_DIR)/$(TARGET).hex $(OUT_DIR)/$(TARGET).lst $(OUT_DIR)/System.map

.PHONY: FORCE tftf

ifeq (${INCREMENTAL_RELEASE_NUMBER},)
  OVERRIDE_VERSION = 0.1
else
  OVERRIDE_VERSION = 0.${INCREMENTAL_RELEASE_NUMBER}
endif

ifeq (${INCREMENTAL_BUILD_NUMBER},)
  OVERRIDE_BUILD_NUM = ${shell git describe --always --dirty --match nothing 2>/dev/null}
else
  OVERRIDE_BUILD_NUM = ${INCREMENTAL_BUILD_NUMBER}
endif

.version: FORCE
	./tools/version.sh -v $(OVERRIDE_VERSION) -b $(OVERRIDE_BUILD_NUM) $@

$(OUT_DIR)/mnfb.o : $(OUT_DIR)/manifest.inc

ifdef EXT_MANIFEST_INC
  ifneq ("$(wildcard $(EXT_MANIFEST_INC))","")
   $(info "using externally signed manifest " $(EXT_MANIFEST_INC))
   USE_EXT_MANIFEST_INC = y
  else
   $(error "EXT_MANIFEST_INC path invalid " $(EXT_MANIFEST_INC))
  endif
else
  USE_EXT_MANIFEST_INC = n
endif

ifeq ($(USE_EXT_MANIFEST_INC), y)
$(OUT_DIR)/manifest.inc: $(EXT_MANIFEST_INC)
	cp $(EXT_MANIFEST_INC) $@
else
$(OUT_DIR)/manifest.inc: manifests/firmware.inc
	cp $< $@
endif

$(OUT_DIR)/include/version.h: .version $(OUT_DIR) $(OUT_DIR)/include
	@echo "#ifndef __VERSION_H__"       > $@
	@echo "#define __VERSION_H__"      >> $@
	@grep CONFIG_VERSION .version | sed -e 's/^/\#define /' -e 's/=/ /'  >> $@
	@echo "#endif /* __VERSION_H__ */" >> $@

$(OUT_DIR)/include/config.h: .config $(OUT_DIR) $(OUT_DIR)/include
	@echo '#ifndef _MUCLOADER_CONFIG_H__'       > $@
	@echo '#define _MUCLOADER_CONFIG_H__'      >> $@
	@sed -e '/^#.*/d' -e 's/^/#define /' -e 's/=/\ /' < $< >> $@
	@echo                                      >> $@
	@echo '#endif /* _MUCLOADER_CONFIG_H__ */' >> $@

$(DEP_DIR) $(OUT_DIR) $(OUT_DIR)/include:
	@echo "MKDIR:   $@"; mkdir -p $@

$(OUT_DIR)/$(TARGET).bin: $(OUT_DIR)/$(TARGET).elf
	@echo "CP: $(TARGET).bin"
	$(OBJCOPY) -O binary $< $@

$(OUT_DIR)/$(TARGET).hex: $(OUT_DIR)/$(TARGET).elf
	@echo "CP: $(TARGET).hex"
	$(OBJCOPY) -O ihex $< $@

$(OUT_DIR)/$(TARGET).lst: $(OUT_DIR)/$(TARGET).elf
	@echo "OBJDUMP: $(@)"
	$(OBJDUMP) -St $^ > $@

$(OUT_DIR)/$(TARGET).elf: $(OBJS) $(OUT_DIR)/manifest.inc
	@echo "LD:      $(TARGET).elf"
	$(CC) $(CFLAGS) $(LDFLAGS) $(OBJS) -o $@
	@echo "SIZE:    $(TARGET).elf"
	$(SIZE) $@

$(OUT_DIR)/System.map: $(OUT_DIR)/$(TARGET).elf
	$(NM) $(OUT_DIR)/$(TARGET).elf | \
	grep -v '\(compiled\)\|\(\$(OBJEXT)$$\)\|\( [aUw] \)\|\(\.\.ng$$\)\|\(LASH[RL]DI\)' | \
	sort > $(OUT_DIR)/System.map

tftf: $(OUT_DIR)/$(TARGET).bin  $(OUT_DIR)/System.map
	bash ./tools/tftf.sh $(OUT_DIR)/$(TARGET).bin

clean:
	@echo "RMDIR:   dep"          ; rm -fr dep
	@echo "RMDIR:   out"          ; rm -fr out

distclean: clean
	-rm -rf .config
