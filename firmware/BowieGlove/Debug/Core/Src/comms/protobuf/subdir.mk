################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/comms/protobuf/pb_common.c \
../Core/Src/comms/protobuf/pb_decode.c \
../Core/Src/comms/protobuf/pb_encode.c 

OBJS += \
./Core/Src/comms/protobuf/pb_common.o \
./Core/Src/comms/protobuf/pb_decode.o \
./Core/Src/comms/protobuf/pb_encode.o 

C_DEPS += \
./Core/Src/comms/protobuf/pb_common.d \
./Core/Src/comms/protobuf/pb_decode.d \
./Core/Src/comms/protobuf/pb_encode.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/comms/protobuf/pb_common.o: ../Core/Src/comms/protobuf/pb_common.c Core/Src/comms/protobuf/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I"C:/Users/kundusayantan/Documents/GitHub/GUM/hardware/glove/fw/BowieGlove/Core/Inc/comms" -I"C:/Users/kundusayantan/Documents/GitHub/GUM/hardware/glove/fw/BowieGlove/Core/Inc/comms/protobuf" -I"C:/Users/kundusayantan/Documents/GitHub/GUM/hardware/glove/fw/BowieGlove/tinyusb/src" -I../Drivers/bhy2 -I../Core/Inc -I"C:/Users/kundusayantan/Documents/GitHub/GUM/hardware/glove/fw/BowieGlove/Core/Inc/glove" -I"C:/Users/kundusayantan/Documents/GitHub/GUM/hardware/glove/fw/BowieGlove/Core/Inc/usb" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/comms/protobuf/%.o Core/Src/comms/protobuf/%.su Core/Src/comms/protobuf/%.cyclo: ../Core/Src/comms/protobuf/%.c Core/Src/comms/protobuf/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I"C:/Users/kundusayantan/Documents/GitHub/GUM/hardware/glove/fw/BowieGlove/Core/Inc/comms" -I"C:/Users/kundusayantan/Documents/GitHub/GUM/hardware/glove/fw/BowieGlove/Core/Inc/comms/protobuf" -I"C:/Users/kundusayantan/Documents/GitHub/GUM/hardware/glove/fw/BowieGlove/tinyusb/src" -I../Drivers/bhy2 -I../Core/Inc -I"C:/Users/kundusayantan/Documents/GitHub/GUM/hardware/glove/fw/BowieGlove/Core/Inc/glove" -I"C:/Users/kundusayantan/Documents/GitHub/GUM/hardware/glove/fw/BowieGlove/Core/Inc/usb" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-comms-2f-protobuf

clean-Core-2f-Src-2f-comms-2f-protobuf:
	-$(RM) ./Core/Src/comms/protobuf/pb_common.cyclo ./Core/Src/comms/protobuf/pb_common.d ./Core/Src/comms/protobuf/pb_common.o ./Core/Src/comms/protobuf/pb_common.su ./Core/Src/comms/protobuf/pb_decode.cyclo ./Core/Src/comms/protobuf/pb_decode.d ./Core/Src/comms/protobuf/pb_decode.o ./Core/Src/comms/protobuf/pb_decode.su ./Core/Src/comms/protobuf/pb_encode.cyclo ./Core/Src/comms/protobuf/pb_encode.d ./Core/Src/comms/protobuf/pb_encode.o ./Core/Src/comms/protobuf/pb_encode.su

.PHONY: clean-Core-2f-Src-2f-comms-2f-protobuf

