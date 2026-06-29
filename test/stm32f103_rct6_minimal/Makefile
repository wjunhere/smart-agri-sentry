# STM32F103RCT6 minimal test Makefile

PREFIX  ?= $(HOME)/scoop/apps/gcc-arm-none-eabi/current/bin/arm-none-eabi
CC      = $(PREFIX)-gcc
LD      = $(PREFIX)-gcc
OBJCOPY = $(PREFIX)-objcopy
OBJDUMP = $(PREFIX)-objdump
SIZE    = $(PREFIX)-size

# Project name
TARGET  = rct6_minimal

# Directories
BUILD_DIR = build
SRC_DIR   = src
INC_DIR   = inc

# Includes
INCLUDES = -I$(INC_DIR)

# Defines
DEFINES = -DSTM32F10X_HD -DUSE_STDPERIPH_DRIVER

# Source files
C_SOURCES = \
    $(SRC_DIR)/main.c \
    $(SRC_DIR)/stm32f10x_it.c \
    $(SRC_DIR)/system_stm32f10x.c \
    $(SRC_DIR)/stm32f10x_gpio.c \
    $(SRC_DIR)/stm32f10x_rcc.c \
    $(SRC_DIR)/stm32f10x_usart.c \
    $(SRC_DIR)/stm32f10x_exti.c \
    $(SRC_DIR)/misc.c

ASM_SOURCES = $(SRC_DIR)/startup_stm32f10x_hd.s

OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))

# Compiler flags
CPU = -mcpu=cortex-m3 -mthumb
CFLAGS = $(CPU) $(DEFINES) $(INCLUDES) -O0 -g -Wall -ffunction-sections -fdata-sections
CFLAGS += -std=gnu99 -fverbose-asm

# Linker flags
LDSCRIPT = $(SRC_DIR)/stm32f103rct6_flash.ld
LDFLAGS = $(CPU) -T$(LDSCRIPT) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections -specs=nano.specs -specs=nosys.specs

# Default target
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).bin $(BUILD_DIR)/$(TARGET).hex
	$(SIZE) $(BUILD_DIR)/$(TARGET).elf

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.s Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(LD) $(OBJECTS) $(LDFLAGS) -o $@

$(BUILD_DIR)/$(TARGET).bin: $(BUILD_DIR)/$(TARGET).elf
	$(OBJCOPY) -O binary $< $@

$(BUILD_DIR)/$(TARGET).hex: $(BUILD_DIR)/$(TARGET).elf
	$(OBJCOPY) -O ihex $< $@

$(BUILD_DIR):
	mkdir -p $@

flash: $(BUILD_DIR)/$(TARGET).bin
	python -m stm32loader -p COM9 -b 115200 -e -w -v $(BUILD_DIR)/$(TARGET).bin

clean:
	rm -rf $(BUILD_DIR)

.PHONY: all clean flash
