################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/bhy2/bhi3.c \
../Drivers/bhy2/bhi3_multi_tap.c \
../Drivers/bhy2/bhy2.c \
../Drivers/bhy2/bhy2_bsec.c \
../Drivers/bhy2/bhy2_head_tracker.c \
../Drivers/bhy2/bhy2_hif.c \
../Drivers/bhy2/bhy2_klio.c \
../Drivers/bhy2/bhy2_parse.c \
../Drivers/bhy2/bhy2_swim.c 

OBJS += \
./Drivers/bhy2/bhi3.o \
./Drivers/bhy2/bhi3_multi_tap.o \
./Drivers/bhy2/bhy2.o \
./Drivers/bhy2/bhy2_bsec.o \
./Drivers/bhy2/bhy2_head_tracker.o \
./Drivers/bhy2/bhy2_hif.o \
./Drivers/bhy2/bhy2_klio.o \
./Drivers/bhy2/bhy2_parse.o \
./Drivers/bhy2/bhy2_swim.o 

C_DEPS += \
./Drivers/bhy2/bhi3.d \
./Drivers/bhy2/bhi3_multi_tap.d \
./Drivers/bhy2/bhy2.d \
./Drivers/bhy2/bhy2_bsec.d \
./Drivers/bhy2/bhy2_head_tracker.d \
./Drivers/bhy2/bhy2_hif.d \
./Drivers/bhy2/bhy2_klio.d \
./Drivers/bhy2/bhy2_parse.d \
./Drivers/bhy2/bhy2_swim.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/bhy2/%.o Drivers/bhy2/%.su Drivers/bhy2/%.cyclo: ../Drivers/bhy2/%.c Drivers/bhy2/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I"C:/Users/kundusayantan/Documents/GitHub/GUM/hardware/glove/fw/BowieGlove_rtos/Middlewares/Third_Party/FreeRTOS/Source/include" -I../Core/Inc -I../Drivers/bhy2 -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"C:/Users/kundusayantan/Documents/GitHub/GUM/hardware/glove/fw/BowieGlove_rtos/tinyusb/src" -I"C:/Users/kundusayantan/Documents/GitHub/GUM/hardware/glove/fw/BowieGlove_rtos/Core/Inc/comms" -I"C:/Users/kundusayantan/Documents/GitHub/GUM/hardware/glove/fw/BowieGlove_rtos/Core/Inc/comms/protobuf" -I"C:/Users/kundusayantan/Documents/GitHub/GUM/hardware/glove/fw/BowieGlove_rtos/Core/Inc/glove" -I"C:/Users/kundusayantan/Documents/GitHub/GUM/hardware/glove/fw/BowieGlove_rtos/Core/Inc/usb" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-bhy2

clean-Drivers-2f-bhy2:
	-$(RM) ./Drivers/bhy2/bhi3.cyclo ./Drivers/bhy2/bhi3.d ./Drivers/bhy2/bhi3.o ./Drivers/bhy2/bhi3.su ./Drivers/bhy2/bhi3_multi_tap.cyclo ./Drivers/bhy2/bhi3_multi_tap.d ./Drivers/bhy2/bhi3_multi_tap.o ./Drivers/bhy2/bhi3_multi_tap.su ./Drivers/bhy2/bhy2.cyclo ./Drivers/bhy2/bhy2.d ./Drivers/bhy2/bhy2.o ./Drivers/bhy2/bhy2.su ./Drivers/bhy2/bhy2_bsec.cyclo ./Drivers/bhy2/bhy2_bsec.d ./Drivers/bhy2/bhy2_bsec.o ./Drivers/bhy2/bhy2_bsec.su ./Drivers/bhy2/bhy2_head_tracker.cyclo ./Drivers/bhy2/bhy2_head_tracker.d ./Drivers/bhy2/bhy2_head_tracker.o ./Drivers/bhy2/bhy2_head_tracker.su ./Drivers/bhy2/bhy2_hif.cyclo ./Drivers/bhy2/bhy2_hif.d ./Drivers/bhy2/bhy2_hif.o ./Drivers/bhy2/bhy2_hif.su ./Drivers/bhy2/bhy2_klio.cyclo ./Drivers/bhy2/bhy2_klio.d ./Drivers/bhy2/bhy2_klio.o ./Drivers/bhy2/bhy2_klio.su ./Drivers/bhy2/bhy2_parse.cyclo ./Drivers/bhy2/bhy2_parse.d ./Drivers/bhy2/bhy2_parse.o ./Drivers/bhy2/bhy2_parse.su ./Drivers/bhy2/bhy2_swim.cyclo ./Drivers/bhy2/bhy2_swim.d ./Drivers/bhy2/bhy2_swim.o ./Drivers/bhy2/bhy2_swim.su

.PHONY: clean-Drivers-2f-bhy2

