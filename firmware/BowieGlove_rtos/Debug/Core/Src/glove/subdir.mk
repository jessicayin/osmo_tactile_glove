################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/glove/bhi360.c \
../Core/Src/glove/common.c \
../Core/Src/glove/glove.c 

OBJS += \
./Core/Src/glove/bhi360.o \
./Core/Src/glove/common.o \
./Core/Src/glove/glove.o 

C_DEPS += \
./Core/Src/glove/bhi360.d \
./Core/Src/glove/common.d \
./Core/Src/glove/glove.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/glove/%.o Core/Src/glove/%.su Core/Src/glove/%.cyclo: ../Core/Src/glove/%.c Core/Src/glove/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I"C:/Users/kundusayantan/Documents/GitHub/GUM/hardware/glove/fw/BowieGlove_rtos/Core/Inc/comms" -I"C:/Users/kundusayantan/Documents/GitHub/GUM/hardware/glove/fw/BowieGlove_rtos/Core/Inc/comms/protobuf" -I"C:/Users/kundusayantan/Documents/GitHub/GUM/hardware/glove/fw/BowieGlove_rtos/tinyusb/src" -I../Drivers/bhy2 -I../Core/Inc -I"C:/Users/kundusayantan/Documents/GitHub/GUM/hardware/glove/fw/BowieGlove_rtos/Core/Inc/glove" -I"C:/Users/kundusayantan/Documents/GitHub/GUM/hardware/glove/fw/BowieGlove_rtos/Core/Inc/usb" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-glove

clean-Core-2f-Src-2f-glove:
	-$(RM) ./Core/Src/glove/bhi360.cyclo ./Core/Src/glove/bhi360.d ./Core/Src/glove/bhi360.o ./Core/Src/glove/bhi360.su ./Core/Src/glove/common.cyclo ./Core/Src/glove/common.d ./Core/Src/glove/common.o ./Core/Src/glove/common.su ./Core/Src/glove/glove.cyclo ./Core/Src/glove/glove.d ./Core/Src/glove/glove.o ./Core/Src/glove/glove.su

.PHONY: clean-Core-2f-Src-2f-glove

