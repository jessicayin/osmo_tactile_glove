################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/usb/usb_descriptors.c \
../Core/Src/usb/usb_dfu_rt.c 

OBJS += \
./Core/Src/usb/usb_descriptors.o \
./Core/Src/usb/usb_dfu_rt.o 

C_DEPS += \
./Core/Src/usb/usb_descriptors.d \
./Core/Src/usb/usb_dfu_rt.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/usb/%.o Core/Src/usb/%.su Core/Src/usb/%.cyclo: ../Core/Src/usb/%.c Core/Src/usb/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I"C:/Users/kundusayantan/Documents/GitHub/GUM/hardware/glove/fw/BowieGlove/Core/Inc/comms" -I"C:/Users/kundusayantan/Documents/GitHub/GUM/hardware/glove/fw/BowieGlove/Core/Inc/comms/protobuf" -I"C:/Users/kundusayantan/Documents/GitHub/GUM/hardware/glove/fw/BowieGlove/tinyusb/src" -I../Drivers/bhy2 -I../Core/Inc -I"C:/Users/kundusayantan/Documents/GitHub/GUM/hardware/glove/fw/BowieGlove/Core/Inc/glove" -I"C:/Users/kundusayantan/Documents/GitHub/GUM/hardware/glove/fw/BowieGlove/Core/Inc/usb" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-usb

clean-Core-2f-Src-2f-usb:
	-$(RM) ./Core/Src/usb/usb_descriptors.cyclo ./Core/Src/usb/usb_descriptors.d ./Core/Src/usb/usb_descriptors.o ./Core/Src/usb/usb_descriptors.su ./Core/Src/usb/usb_dfu_rt.cyclo ./Core/Src/usb/usb_dfu_rt.d ./Core/Src/usb/usb_dfu_rt.o ./Core/Src/usb/usb_dfu_rt.su

.PHONY: clean-Core-2f-Src-2f-usb

