TOPDIR := ${shell pwd}

CUBE_DIR     = STM32Cube_FW_L4_V1.0.0
OUT_DIR      = out
DEP_DIR      = dep
INC_DIR      = include
SRC_DIR      = src
COM_DIR      = common
CFG_DIR      = configs

CHIPSET          = STM32L4xx
CHIPSET_LC       = stm32l4xx
TARGET_DEVICE    = STM32L476xx
TARGET_DEVICE_LC = stm32l476xx
BOARD            = STM32L476G-Discovery

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

include $(TOPDIR)/.config

ifeq ($(CONFIG_DEBUG),y)
DEFS      += -D_DEBUG
endif

DEFS      += -DMOD_TYPE_$(CONFIG_MOD_TYPE)
DEFS      += -DMOD_BOARDID_PID=$(CONFIG_ARCH_BOARDID_PID)
DEFS      += -DMOD_BOARDID_VID=$(CONFIG_ARCH_BOARDID_VID)
DEFS      += -DMOD_BOOT_VERSION=$(CONFIG_BOOT_VERSION)
DEFS      += -DVECT_TAB_SRAM

INCS       = -I$(HAL_DIR)/Inc/
INCS      += -I$(CFG_DIR)/$(CONFIG_MOD_TYPE)/$(INC_DIR)/
INCS      += -I.
INCS      += -I$(INC_DIR)/
INCS      += -I$(COM_DIR)/$(INC_DIR)/
INCS      += -I$(CMSIS_DIR)/Include/
INCS      += -I$(CMSIS_DIR)/Device/ST/$(CHIPSET)/Include
INCS      += -I$(SRC_DIR)/

CFLAGS     = -Wall -g -std=c99 -Os
CFLAGS    += -mlittle-endian -mcpu=cortex-m4 -march=armv7e-m -mthumb
CFLAGS    += -mfpu=fpv4-sp-d16 -mfloat-abi=hard
CFLAGS    += -ffunction-sections -fdata-sections
CFLAGS    += $(INCS) $(DEFS)

CSRCS       = main.c
CSRCS      += system_stm32l4xx.c
CSRCS      += stm32l4xx_hal_msp.c
CSRCS      += stm32l4xx_it.c
CSRCS      += stm32l4xx_flash.c

CSRCS      += debug.c \
	      utils.c \
	      gbcore.c \
	      gbfirmware.c \
	      datalink.c \
	      es3_unipro.c

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

SSRCS       = startup_stm32l476xx.s

VPATH      = ./src
VPATH     += $(COM_DIR)/src
VPATH     += $(HAL_DIR)/Src
VPATH     += $(DEV_DIR)/Source/

LIBS       = -L$(CMSIS_DIR)/Lib

TARGET       = boot_$(CONFIG_MOD_TYPE)

# Linker flags
LDFLAGS    = -Wl,--gc-sections -g -Wl,-Map=$(OUT_DIR)/$(TARGET).map $(LIBS) -T$(SRC_DIR)/$(TARGET_DEVICE_LC).ld
#
# # Enable Semihosting
LDFLAGS   += --specs=rdimon.specs -lc -lrdimon

OBJS    = $(addprefix $(OUT_DIR)/,$(CSRCS:.c=.o)) $(addprefix $(OUT_DIR)/,$(SSRCS:.s=.o))
DEPS    = $(addprefix $(DEP_DIR)/,$(CSRCS:.c=.d)) $(addprefix $(OUT_DIR)/,$(SSRCS:.s=.d))

$(OUT_DIR)/%.o : %.c $(DEP_DIR) $(OUT_DIR)
	@echo "CC:      $(notdir $<)"
	$(CC) $(CFLAGS) -c -o $@ $< -MMD -MF $(DEP_DIR)/$(*F).d

$(OUT_DIR)/%.o : %.s $(DEP_DIR) $(OUT_DIR)
	@echo "CC:      $(notdir $<)"
	$(CC) $(CFLAGS) -c -o $@ $< -MMD -MF $(DEP_DIR)/$(*F).d

all:  $(OUT_DIR)/$(TARGET).bin $(OUT_DIR)/$(TARGET).hex $(OUT_DIR)/$(TARGET).lst

$(DEP_DIR) $(OUT_DIR):
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
