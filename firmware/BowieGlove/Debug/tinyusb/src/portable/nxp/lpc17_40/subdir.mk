################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../tinyusb/src/portable/nxp/lpc17_40/dcd_lpc17_40.c \
../tinyusb/src/portable/nxp/lpc17_40/hcd_lpc17_40.c 

OBJS += \
./tinyusb/src/portable/nxp/lpc17_40/dcd_lpc17_40.o \
./tinyusb/src/portable/nxp/lpc17_40/hcd_lpc17_40.o 

C_DEPS += \
./tinyusb/src/portable/nxp/lpc17_40/dcd_lpc17_40.d \
./tinyusb/src/portable/nxp/lpc17_40/hcd_lpc17_40.d 


# Each subdirectory must supply rules for building sources it contributes
tinyusb/src/portable/nxp/lpc17_40/%.o tinyusb/src/portable/nxp/lpc17_40/%.su tinyusb/src/portable/nxp/lpc17_40/%.cyclo: ../tinyusb/src/portable/nxp/lpc17_40/%.c tinyusb/src/portable/nxp/lpc17_40/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I"C:/Users/kundusayantan/Documents/GitHub/GUM/hardware/glove/fw/BowieGlove/Core/Inc/comms" -I"C:/Users/kundusayantan/Documents/GitHub/GUM/hardware/glove/fw/BowieGlove/Core/Inc/comms/protobuf" -I"C:/Users/kundusayantan/Documents/GitHub/GUM/hardware/glove/fw/BowieGlove/tinyusb/src" -I../Drivers/bhy2 -I../Core/Inc -I"C:/Users/kundusayantan/Documents/GitHub/GUM/hardware/glove/fw/BowieGlove/Core/Inc/glove" -I"C:/Users/kundusayantan/Documents/GitHub/GUM/hardware/glove/fw/BowieGlove/Core/Inc/usb" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-tinyusb-2f-src-2f-portable-2f-nxp-2f-lpc17_40

clean-tinyusb-2f-src-2f-portable-2f-nxp-2f-lpc17_40:
	-$(RM) ./tinyusb/src/portable/nxp/lpc17_40/dcd_lpc17_40.cyclo ./tinyusb/src/portable/nxp/lpc17_40/dcd_lpc17_40.d ./tinyusb/src/portable/nxp/lpc17_40/dcd_lpc17_40.o ./tinyusb/src/portable/nxp/lpc17_40/dcd_lpc17_40.su ./tinyusb/src/portable/nxp/lpc17_40/hcd_lpc17_40.cyclo ./tinyusb/src/portable/nxp/lpc17_40/hcd_lpc17_40.d ./tinyusb/src/portable/nxp/lpc17_40/hcd_lpc17_40.o ./tinyusb/src/portable/nxp/lpc17_40/hcd_lpc17_40.su

.PHONY: clean-tinyusb-2f-src-2f-portable-2f-nxp-2f-lpc17_40

