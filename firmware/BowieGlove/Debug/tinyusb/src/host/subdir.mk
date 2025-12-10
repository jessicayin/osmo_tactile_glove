################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../tinyusb/src/host/hub.c \
../tinyusb/src/host/usbh.c 

OBJS += \
./tinyusb/src/host/hub.o \
./tinyusb/src/host/usbh.o 

C_DEPS += \
./tinyusb/src/host/hub.d \
./tinyusb/src/host/usbh.d 


# Each subdirectory must supply rules for building sources it contributes
tinyusb/src/host/%.o tinyusb/src/host/%.su tinyusb/src/host/%.cyclo: ../tinyusb/src/host/%.c tinyusb/src/host/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I"C:/Users/kundusayantan/Documents/GitHub/GUM/hardware/glove/fw/BowieGlove/Core/Inc/comms" -I"C:/Users/kundusayantan/Documents/GitHub/GUM/hardware/glove/fw/BowieGlove/Core/Inc/comms/protobuf" -I"C:/Users/kundusayantan/Documents/GitHub/GUM/hardware/glove/fw/BowieGlove/tinyusb/src" -I../Drivers/bhy2 -I../Core/Inc -I"C:/Users/kundusayantan/Documents/GitHub/GUM/hardware/glove/fw/BowieGlove/Core/Inc/glove" -I"C:/Users/kundusayantan/Documents/GitHub/GUM/hardware/glove/fw/BowieGlove/Core/Inc/usb" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-tinyusb-2f-src-2f-host

clean-tinyusb-2f-src-2f-host:
	-$(RM) ./tinyusb/src/host/hub.cyclo ./tinyusb/src/host/hub.d ./tinyusb/src/host/hub.o ./tinyusb/src/host/hub.su ./tinyusb/src/host/usbh.cyclo ./tinyusb/src/host/usbh.d ./tinyusb/src/host/usbh.o ./tinyusb/src/host/usbh.su

.PHONY: clean-tinyusb-2f-src-2f-host

