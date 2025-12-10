################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include tinyusb/src/portable/wch/ch32v307/subdir.mk
-include tinyusb/src/portable/valentyusb/eptri/subdir.mk
-include tinyusb/src/portable/ti/msp430x5xx/subdir.mk
-include tinyusb/src/portable/template/subdir.mk
-include tinyusb/src/portable/synopsys/dwc2/subdir.mk
-include tinyusb/src/portable/sunxi/subdir.mk
-include tinyusb/src/portable/st/stm32_fsdev/subdir.mk
-include tinyusb/src/portable/sony/cxd56/subdir.mk
-include tinyusb/src/portable/renesas/rusb2/subdir.mk
-include tinyusb/src/portable/raspberrypi/rp2040/subdir.mk
-include tinyusb/src/portable/raspberrypi/pio_usb/subdir.mk
-include tinyusb/src/portable/ohci/subdir.mk
-include tinyusb/src/portable/nxp/transdimension/subdir.mk
-include tinyusb/src/portable/nxp/lpc_ip3511/subdir.mk
-include tinyusb/src/portable/nxp/lpc17_40/subdir.mk
-include tinyusb/src/portable/nxp/khci/subdir.mk
-include tinyusb/src/portable/nuvoton/nuc505/subdir.mk
-include tinyusb/src/portable/nuvoton/nuc121/subdir.mk
-include tinyusb/src/portable/nuvoton/nuc120/subdir.mk
-include tinyusb/src/portable/nordic/nrf5x/subdir.mk
-include tinyusb/src/portable/mindmotion/mm32/subdir.mk
-include tinyusb/src/portable/microchip/samx7x/subdir.mk
-include tinyusb/src/portable/microchip/samg/subdir.mk
-include tinyusb/src/portable/microchip/samd/subdir.mk
-include tinyusb/src/portable/microchip/pic32mz/subdir.mk
-include tinyusb/src/portable/microchip/pic/subdir.mk
-include tinyusb/src/portable/mentor/musb/subdir.mk
-include tinyusb/src/portable/espressif/esp32sx/subdir.mk
-include tinyusb/src/portable/ehci/subdir.mk
-include tinyusb/src/portable/dialog/da146xx/subdir.mk
-include tinyusb/src/portable/chipidea/ci_hs/subdir.mk
-include tinyusb/src/portable/bridgetek/ft9xx/subdir.mk
-include tinyusb/src/host/subdir.mk
-include tinyusb/src/device/subdir.mk
-include tinyusb/src/common/subdir.mk
-include tinyusb/src/class/video/subdir.mk
-include tinyusb/src/class/vendor/subdir.mk
-include tinyusb/src/class/usbtmc/subdir.mk
-include tinyusb/src/class/net/subdir.mk
-include tinyusb/src/class/msc/subdir.mk
-include tinyusb/src/class/midi/subdir.mk
-include tinyusb/src/class/hid/subdir.mk
-include tinyusb/src/class/dfu/subdir.mk
-include tinyusb/src/class/cdc/subdir.mk
-include tinyusb/src/class/bth/subdir.mk
-include tinyusb/src/class/audio/subdir.mk
-include tinyusb/src/subdir.mk
-include Drivers/bhy2/subdir.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/usb/subdir.mk
-include Core/Src/glove/subdir.mk
-include Core/Src/comms/protobuf/subdir.mk
-include Core/Src/comms/subdir.mk
-include Core/Src/subdir.mk
-include Core/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := BowieGlove
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
BowieGlove.elf \

MAP_FILES += \
BowieGlove.map \

SIZE_OUTPUT += \
BowieGlove.size.stdout \

OBJDUMP_LIST += \
BowieGlove.list \


# All Target
all: main-build

# Main-build Target
main-build: BowieGlove.elf secondary-outputs

# Tool invocations
BowieGlove.elf BowieGlove.map: $(OBJS) $(USER_OBJS) C:\Users\kundusayantan\Documents\GitHub\GUM\hardware\glove\fw\BowieGlove\STM32F446RETX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "BowieGlove.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\kundusayantan\Documents\GitHub\GUM\hardware\glove\fw\BowieGlove\STM32F446RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="BowieGlove.map" -Wl,--gc-sections -static -u _printf_float --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

BowieGlove.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

BowieGlove.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "BowieGlove.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) BowieGlove.elf BowieGlove.list BowieGlove.map BowieGlove.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
