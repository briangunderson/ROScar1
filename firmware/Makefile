# ROScar1 firmware — STM32F103RCT6 (Yahboom YB-ERF01-V3.0)
# Build: make
# Flash: make flash
# Debug: make debug

TARGET   = roscar_fw
MCU      = STM32F103xE
CPU      = cortex-m3
ARCH     = thumb

# ─── Toolchain ───────────────────────────────────────────────────────────────
CROSS  ?= arm-none-eabi-
CC      = $(CROSS)gcc
AS      = $(CROSS)gcc -x assembler-with-cpp
CP      = $(CROSS)objcopy
SZ      = $(CROSS)size
GDB     = $(CROSS)gdb

# ─── Paths ───────────────────────────────────────────────────────────────────
BUILD_DIR = build
HAL_DIR   = vendor/STM32CubeF1/Drivers/STM32F1xx_HAL_Driver
CMSIS_DIR = vendor/STM32CubeF1/Drivers/CMSIS

# ─── Sources ─────────────────────────────────────────────────────────────────
C_SOURCES = \
    src/main.c \
    src/stm32f1xx_it.c \
    src/system_stm32f1xx.c \
    src/bsp_systime.c \
    src/bsp_led.c \
    src/bsp_uart.c \
    src/bsp_motor.c \
    src/bsp_encoder.c \
    src/bsp_icm20948.c \
    src/bsp_adc.c \
    src/kinematics.c \
    src/pid.c \
    src/control_loop.c \
    src/protocol.c \
    src/stubs.c

# Slim subset of HAL — only what we actually use
HAL_SRC = \
    $(HAL_DIR)/Src/stm32f1xx_hal.c \
    $(HAL_DIR)/Src/stm32f1xx_hal_cortex.c \
    $(HAL_DIR)/Src/stm32f1xx_hal_rcc.c \
    $(HAL_DIR)/Src/stm32f1xx_hal_rcc_ex.c \
    $(HAL_DIR)/Src/stm32f1xx_hal_pwr.c \
    $(HAL_DIR)/Src/stm32f1xx_hal_gpio.c \
    $(HAL_DIR)/Src/stm32f1xx_hal_gpio_ex.c \
    $(HAL_DIR)/Src/stm32f1xx_hal_dma.c \
    $(HAL_DIR)/Src/stm32f1xx_hal_tim.c \
    $(HAL_DIR)/Src/stm32f1xx_hal_tim_ex.c \
    $(HAL_DIR)/Src/stm32f1xx_hal_uart.c \
    $(HAL_DIR)/Src/stm32f1xx_hal_spi.c \
    $(HAL_DIR)/Src/stm32f1xx_hal_adc.c \
    $(HAL_DIR)/Src/stm32f1xx_hal_adc_ex.c \
    $(HAL_DIR)/Src/stm32f1xx_hal_iwdg.c \
    $(HAL_DIR)/Src/stm32f1xx_hal_flash.c \
    $(HAL_DIR)/Src/stm32f1xx_hal_flash_ex.c

C_SOURCES += $(HAL_SRC)

ASM_SOURCES = startup/startup_stm32f103xe.s

# ─── Includes ────────────────────────────────────────────────────────────────
INCLUDES = \
    -Iinc \
    -I$(HAL_DIR)/Inc \
    -I$(CMSIS_DIR)/Device/ST/STM32F1xx/Include \
    -I$(CMSIS_DIR)/Include

# ─── Defines ─────────────────────────────────────────────────────────────────
DEFINES = -D$(MCU) -DUSE_HAL_DRIVER

# ─── Flags ───────────────────────────────────────────────────────────────────
MCUFLAGS = -mcpu=$(CPU) -m$(ARCH) -mfloat-abi=soft

CFLAGS = $(MCUFLAGS) $(DEFINES) $(INCLUDES) \
    -Wall -Wextra -Wno-unused-parameter -Wno-unused-function \
    -fdata-sections -ffunction-sections \
    -Os -g3 -std=gnu11 \
    -MMD -MP -MF"$(@:%.o=%.d)"

ASFLAGS = $(MCUFLAGS) -Wall -fdata-sections -ffunction-sections -g3

LDSCRIPT = linker/stm32f103rc.ld
LDFLAGS  = $(MCUFLAGS) -specs=nano.specs -specs=nosys.specs \
    -T$(LDSCRIPT) \
    -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref \
    -Wl,--gc-sections -Wl,--print-memory-usage \
    -lc -lm

# ─── Object lists ────────────────────────────────────────────────────────────
OBJECTS  = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))

# Tell make where to find sources
vpath %.c src $(HAL_DIR)/Src
vpath %.s startup

# ─── Targets ─────────────────────────────────────────────────────────────────
.PHONY: all hal flash erase reset debug clean size

all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin
	@$(SZ) $<

$(BUILD_DIR)/%.o: %.c | $(BUILD_DIR)
	@echo "CC  $<"
	@$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/%.o: %.s | $(BUILD_DIR)
	@echo "AS  $<"
	@$(AS) -c $(ASFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) $(LDSCRIPT)
	@echo "LD  $@"
	@$(CC) $(OBJECTS) $(LDFLAGS) -o $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf
	@$(CP) -O ihex $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf
	@$(CP) -O binary -S $< $@

$(BUILD_DIR):
	@mkdir -p $@

hal:
	@if [ ! -d vendor/STM32CubeF1 ]; then \
	    git clone --depth 1 --branch v1.8.5 https://github.com/STMicroelectronics/STM32CubeF1.git vendor/STM32CubeF1; \
	fi

flash: $(BUILD_DIR)/$(TARGET).elf
	openocd -f interface/stlink.cfg -f target/stm32f1x.cfg \
	    -c "program $< verify reset exit"

erase:
	openocd -f interface/stlink.cfg -f target/stm32f1x.cfg \
	    -c "init; reset halt; stm32f1x mass_erase 0; exit"

reset:
	openocd -f interface/stlink.cfg -f target/stm32f1x.cfg \
	    -c "init; reset run; exit"

debug: $(BUILD_DIR)/$(TARGET).elf
	@bash scripts/debug.sh $<

size: $(BUILD_DIR)/$(TARGET).elf
	$(SZ) -A -x $<

clean:
	rm -rf $(BUILD_DIR)

-include $(OBJECTS:.o=.d)
