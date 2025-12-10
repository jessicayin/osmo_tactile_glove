################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../tinyusb/src/portable/template/dcd_template.c 

OBJS += \
./tinyusb/src/portable/template/dcd_template.o 

C_DEPS += \
./tinyusb/src/portable/template/dcd_template.d 


# Each subdirectory must supply rules for building sources it contributes
tinyusb/src/portable/template/%.o tinyusb/src/portable/template/%.su tinyusb/src/portable/template/%.cyclo: ../tinyusb/src/portable/template/%.c tinyusb/src/portable/template/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I"C:/Users/kundusayantan/Documents/GitHub/GUM/hardware/glove/fw/BowieGlove/Core/Inc/comms" -I"C:/Users/kundusayantan/Documents/GitHub/GUM/hardware/glove/fw/BowieGlove/Core/Inc/comms/protobuf" -I"C:/Users/kundusayantan/Documents/GitHub/GUM/hardware/glove/fw/BowieGlove/tinyusb/src" -I../Drivers/bhy2 -I../Core/Inc -I"C:/Users/kundusayantan/Documents/GitHub/GUM/hardware/glove/fw/BowieGlove/Core/Inc/glove" -I"C:/Users/kundusayantan/Documents/GitHub/GUM/hardware/glove/fw/BowieGlove/Core/Inc/usb" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-tinyusb-2f-src-2f-portable-2f-template

clean-tinyusb-2f-src-2f-portable-2f-template:
	-$(RM) ./tinyusb/src/portable/template/dcd_template.cyclo ./tinyusb/src/portable/template/dcd_template.d ./tinyusb/src/portable/template/dcd_template.o ./tinyusb/src/portable/template/dcd_template.su

.PHONY: clean-tinyusb-2f-src-2f-portable-2f-template

