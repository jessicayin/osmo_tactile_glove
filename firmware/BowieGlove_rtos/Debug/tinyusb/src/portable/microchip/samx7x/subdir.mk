################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../tinyusb/src/portable/microchip/samx7x/dcd_samx7x.c 

OBJS += \
./tinyusb/src/portable/microchip/samx7x/dcd_samx7x.o 

C_DEPS += \
./tinyusb/src/portable/microchip/samx7x/dcd_samx7x.d 


# Each subdirectory must supply rules for building sources it contributes
tinyusb/src/portable/microchip/samx7x/%.o tinyusb/src/portable/microchip/samx7x/%.su tinyusb/src/portable/microchip/samx7x/%.cyclo: ../tinyusb/src/portable/microchip/samx7x/%.c tinyusb/src/portable/microchip/samx7x/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I"C:/Users/kundusayantan/Documents/GitHub/GUM/hardware/glove/fw/BowieGlove_rtos/Middlewares/Third_Party/FreeRTOS/Source/include" -I../Core/Inc -I../Drivers/bhy2 -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"C:/Users/kundusayantan/Documents/GitHub/GUM/hardware/glove/fw/BowieGlove_rtos/tinyusb/src" -I"C:/Users/kundusayantan/Documents/GitHub/GUM/hardware/glove/fw/BowieGlove_rtos/Core/Inc/comms" -I"C:/Users/kundusayantan/Documents/GitHub/GUM/hardware/glove/fw/BowieGlove_rtos/Core/Inc/comms/protobuf" -I"C:/Users/kundusayantan/Documents/GitHub/GUM/hardware/glove/fw/BowieGlove_rtos/Core/Inc/glove" -I"C:/Users/kundusayantan/Documents/GitHub/GUM/hardware/glove/fw/BowieGlove_rtos/Core/Inc/usb" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-tinyusb-2f-src-2f-portable-2f-microchip-2f-samx7x

clean-tinyusb-2f-src-2f-portable-2f-microchip-2f-samx7x:
	-$(RM) ./tinyusb/src/portable/microchip/samx7x/dcd_samx7x.cyclo ./tinyusb/src/portable/microchip/samx7x/dcd_samx7x.d ./tinyusb/src/portable/microchip/samx7x/dcd_samx7x.o ./tinyusb/src/portable/microchip/samx7x/dcd_samx7x.su

.PHONY: clean-tinyusb-2f-src-2f-portable-2f-microchip-2f-samx7x

