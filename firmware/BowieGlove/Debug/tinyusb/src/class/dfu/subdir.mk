################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../tinyusb/src/class/dfu/dfu_device.c \
../tinyusb/src/class/dfu/dfu_rt_device.c 

OBJS += \
./tinyusb/src/class/dfu/dfu_device.o \
./tinyusb/src/class/dfu/dfu_rt_device.o 

C_DEPS += \
./tinyusb/src/class/dfu/dfu_device.d \
./tinyusb/src/class/dfu/dfu_rt_device.d 


# Each subdirectory must supply rules for building sources it contributes
tinyusb/src/class/dfu/%.o tinyusb/src/class/dfu/%.su tinyusb/src/class/dfu/%.cyclo: ../tinyusb/src/class/dfu/%.c tinyusb/src/class/dfu/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I"C:/Users/kundusayantan/Documents/GitHub/GUM/hardware/glove/fw/BowieGlove/Core/Inc/comms" -I"C:/Users/kundusayantan/Documents/GitHub/GUM/hardware/glove/fw/BowieGlove/Core/Inc/comms/protobuf" -I"C:/Users/kundusayantan/Documents/GitHub/GUM/hardware/glove/fw/BowieGlove/tinyusb/src" -I../Drivers/bhy2 -I../Core/Inc -I"C:/Users/kundusayantan/Documents/GitHub/GUM/hardware/glove/fw/BowieGlove/Core/Inc/glove" -I"C:/Users/kundusayantan/Documents/GitHub/GUM/hardware/glove/fw/BowieGlove/Core/Inc/usb" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-tinyusb-2f-src-2f-class-2f-dfu

clean-tinyusb-2f-src-2f-class-2f-dfu:
	-$(RM) ./tinyusb/src/class/dfu/dfu_device.cyclo ./tinyusb/src/class/dfu/dfu_device.d ./tinyusb/src/class/dfu/dfu_device.o ./tinyusb/src/class/dfu/dfu_device.su ./tinyusb/src/class/dfu/dfu_rt_device.cyclo ./tinyusb/src/class/dfu/dfu_rt_device.d ./tinyusb/src/class/dfu/dfu_rt_device.o ./tinyusb/src/class/dfu/dfu_rt_device.su

.PHONY: clean-tinyusb-2f-src-2f-class-2f-dfu

