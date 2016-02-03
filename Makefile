TOPDIR := ${shell pwd}

OUT_DIR      = out
DEP_DIR      = dep
INC_DIR      = include
SRC_DIR      = src
CFG_DIR      = configs
MAN_DIR      = manifests

include $(TOPDIR)/.config

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
DEFS      += -DVECT_TAB_SRAM

INCS       = -I$(HAL_DIR)/Inc/
INCS      += -I$(CFG_DIR)/$(CONFIG_MOD_TYPE)/$(INC_DIR)/
INCS      += -I.
INCS      += -I$(INC_DIR)/
INCS      += -I$(CMSIS_DIR)/Include/
INCS      += -I$(CMSIS_DIR)/Device/ST/$(CHIPSET)/Include
INCS      += -I$(OUT_DIR)/include
INCS      += -I$(SRC_DIR)/

CFLAGS     = -Wall -g -std=c99 -Os
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

CSRCS      += debug.c \
	      utils.c \
	      gbcore.c \
	      ctrl.c \
	      modsctrl.c \
	      gbfirmware.c \
	      datalink.c \
	      es3_unipro.c \
	      tftf.c \
	      crypto.c \
	      public_keys.c \
	      $(CHIPSET_LC)_muc.c \
	      $(CHIPSET_LC)_mod_device.c \
	      $(CHIPSET_LC)_hal_mod.c

# Basic HAL libraries
CSRCS      += $(CHIPSET_LC)_hal_rcc.c \
              $(CHIPSET_LC)_hal_rcc_ex.c \
              $(CHIPSET_LC)_hal.c \
              $(CHIPSET_LC)_hal_cortex.c \
              $(CHIPSET_LC)_hal_gpio.c  \
              $(CHIPSET_LC)_hal_spi.c  \
              $(CHIPSET_LC)_hal_dma.c  \
              $(CHIPSET_LC)_hal_flash.c \
              $(CHIPSET_LC)_hal_flash_ex.c \
              $(CHIPSET_LC)_hal_uart.c \
              $(CHIPSET_LC)_hal_uart_ex.c \
              $(CHIPSET_LC)_hal_pwr_ex.c

SSRCS       = startup_$(TARGET_DEVICE_LC).s

VPATH      = ./src
VPATH     += $(HAL_DIR)/Src
VPATH     += $(DEV_DIR)/Source/
VPATH     += $(CFG_DIR)/$(CONFIG_MOD_TYPE)/src
VPATH     += $(MAN_DIR)/

LIBS       = -L$(CMSIS_DIR)/Lib

TARGET       = boot_$(CONFIG_MOD_TYPE)

# Linker flags
LDFLAGS    = -Wl,--gc-sections -g -Wl,-Map=$(OUT_DIR)/$(TARGET).map $(LIBS) -T$(SRC_DIR)/$(TARGET_DEVICE_LC).ld
#
# # Enable Semihosting
LDFLAGS   += --specs=rdimon.specs -lc -lrdimon

OBJS    = $(addprefix $(OUT_DIR)/,$(CSRCS:.c=.o)) $(addprefix $(OUT_DIR)/,$(SSRCS:.s=.o))
DEPS    = $(addprefix $(DEP_DIR)/,$(CSRCS:.c=.d)) $(addprefix $(OUT_DIR)/,$(SSRCS:.s=.d))

$(OUT_DIR)/%.o : %.c $(DEP_DIR) $(OUT_DIR) $(OUT_DIR)/include/version.h $(OUT_DIR)/include/config.h
	@echo "CC:      $(notdir $<)"
	$(CC) $(CFLAGS) -c -o $@ $< -MMD -MF $(DEP_DIR)/$(*F).d

$(OUT_DIR)/%.o : %.s $(DEP_DIR) $(OUT_DIR)
	@echo "CC:      $(notdir $<)"
	$(CC) $(CFLAGS) -c -o $@ $< -MMD -MF $(DEP_DIR)/$(*F).d

all:  $(OUT_DIR)/$(TARGET).bin $(OUT_DIR)/$(TARGET).hex $(OUT_DIR)/$(TARGET).lst

.PHONY: FORCE


ifeq (${INCREMENTAL_RELEASE_NUMBER},)
  OVERRIDE_VERSION = 0.1
else
  OVERRIDE_VERSION = 0.${INCREMENTAL_RELEASE_NUMBER}
endif

ifeq (${INCREMENTAL_BUILD_NUMBER},)
  OVERRIDE_BUILD_NUM = 5
else
  OVERRIDE_BUILD_NUM = ${INCREMENTAL_BUILD_NUMBER}
endif

.version: FORCE
	./tools/version.sh -v $(OVERRIDE_VERSION) -b $(OVERRIDE_BUILD_NUM) $@

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

#	$(CC) $(CFLAGS) $(LDFLAGS) $(SRC_DIR)/startup_$(TARGET_DEVICE_LC).s $(OBJS) -o $@
$(OUT_DIR)/$(TARGET).elf: $(OBJS)
	@echo "LD:      $(TARGET).elf"
	$(CC) $(CFLAGS) $(LDFLAGS) $(OBJS) -o $@
	@echo "SIZE:    $(TARGET).elf"
	$(SIZE) $@

clean:
	@echo "RMDIR:   dep"          ; rm -fr dep
	@echo "RMDIR:   out"          ; rm -fr out

distclean: clean
	-rm -rf .config
