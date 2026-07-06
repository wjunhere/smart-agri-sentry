# Makefile for STM32F407ZGTx chassis firmware (GCC ARM Embedded)
#
# Usage:
#   make                — build firmware.elf / firmware.hex / firmware.bin
#   make clean          — remove build artifacts
#   make flash          — flash via STM32_Programmer_CLI (SWD)
#
# Requires: arm-none-eabi-gcc, STM32_Programmer_CLI

PROJ := firmware
TARGET := $(PROJ)

# ---- Toolchain ----
CROSS  := arm-none-eabi-
CC     := $(CROSS)gcc
CXX    := $(CROSS)g++
AS     := $(CROSS)gcc -x assembler-with-cpp
OBJCPY := $(CROSS)objcopy
OBJDUMP:= $(CROSS)objdump
SIZE   := $(CROSS)size

# ---- MCU ----
MCU    := -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16
DEFS   := -DSTM32F407xx -DUSE_HAL_DRIVER
OPT    := -Og  # debug-optimised (safe, debuggable)

# ---- Directories ----
CORE    := Core/Src
USERS   := Users
HAL_SRC := Drivers/STM32F4xx_HAL_Driver/Src
STARTUP := Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/gcc

INC := -ICore/Inc
INC += -IUsers
INC += -IDrivers/STM32F4xx_HAL_Driver/Inc
INC += -IDrivers/CMSIS/Device/ST/STM32F4xx/Include
INC += -IDrivers/CMSIS/Include

LDSCRIPT := STM32F407ZGTx.ld

# ---- Sources ----
C_SRCS  := $(wildcard $(CORE)/*.c)
C_SRCS  += $(wildcard $(USERS)/*.c)
C_SRCS  += $(HAL_SRC)/stm32f4xx_hal.c
C_SRCS  += $(HAL_SRC)/stm32f4xx_hal_uart.c
C_SRCS  += $(HAL_SRC)/stm32f4xx_hal_gpio.c
C_SRCS  += $(HAL_SRC)/stm32f4xx_hal_dma.c
C_SRCS  += $(HAL_SRC)/stm32f4xx_hal_dma_ex.c
C_SRCS  += $(HAL_SRC)/stm32f4xx_hal_tim.c
C_SRCS  += $(HAL_SRC)/stm32f4xx_hal_tim_ex.c
C_SRCS  += $(HAL_SRC)/stm32f4xx_hal_adc.c
C_SRCS  += $(HAL_SRC)/stm32f4xx_hal_adc_ex.c
C_SRCS  += $(HAL_SRC)/stm32f4xx_hal_rcc.c
C_SRCS  += $(HAL_SRC)/stm32f4xx_hal_rcc_ex.c
C_SRCS  += $(HAL_SRC)/stm32f4xx_hal_cortex.c
C_SRCS  += $(HAL_SRC)/stm32f4xx_hal_flash.c
C_SRCS  += $(HAL_SRC)/stm32f4xx_hal_flash_ex.c
C_SRCS  += $(HAL_SRC)/stm32f4xx_hal_flash_ramfunc.c
C_SRCS  += $(HAL_SRC)/stm32f4xx_hal_pwr.c
C_SRCS  += $(HAL_SRC)/stm32f4xx_hal_pwr_ex.c
C_SRCS  += $(HAL_SRC)/stm32f4xx_hal_iwdg.c
C_SRCS  += $(HAL_SRC)/stm32f4xx_hal_exti.c

AS_SRCS := $(STARTUP)/startup_stm32f407xx.s

# ---- Objects ----
BUILD   := build
OBJS    := $(C_SRCS:%.c=$(BUILD)/%.o)
OBJS    += $(AS_SRCS:%.s=$(BUILD)/%.o)

# ---- Flags ----
CFLAGS  := $(MCU) $(DEFS) $(INC) $(OPT)
CFLAGS  += -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers
CFLAGS  += -ffunction-sections -fdata-sections
CFLAGS  += -fno-common

ASFLAGS := $(MCU) $(DEFS) $(INC)

LDFLAGS := $(MCU) -T$(LDSCRIPT)
LDFLAGS += -Wl,--gc-sections
LDFLAGS += -Wl,-Map=$(BUILD)/$(TARGET).map
LDFLAGS += --specs=nosys.specs -lc -lm -lnosys

# ---- Build ----
.PHONY: all clean flash

all: $(BUILD)/$(TARGET).elf $(BUILD)/$(TARGET).hex $(BUILD)/$(TARGET).bin

$(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/%.o: %.s
	@mkdir -p $(dir $@)
	$(AS) $(ASFLAGS) -c $< -o $@

$(BUILD)/$(TARGET).elf: $(OBJS)
	$(CC) $(MCU) $(OBJS) $(LDFLAGS) -o $@
	$(SIZE) $@

$(BUILD)/%.hex: $(BUILD)/%.elf
	$(OBJCPY) -O ihex $< $@

$(BUILD)/%.bin: $(BUILD)/%.elf
	$(OBJCPY) -O binary $< $@

clean:
	rm -rf $(BUILD)

flash: $(BUILD)/$(TARGET).hex
	STM32_Programmer_CLI -c port=SWD freq=4000 -w $< -v -hardRst
