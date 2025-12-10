cmake_minimum_required(VERSION 3.22)
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/gcc-arm-none-eabi.cmake)

project(BowieGlove)

enable_language(C ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(CPU_PARAMETERS
    -mthumb
    -mcpu=cortex-m4
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
)

set(symbols_SYMB
        DEBUG
        USE_HAL_DRIVER
        STM32F446xx
)

set(source_SRCS
        
        ./Core/Src/comms/protobuf/pb_common.c
        ./Core/Src/comms/protobuf/pb_decode.c
        ./Core/Src/comms/protobuf/pb_encode.c


        ./Core/Src/comms/bowie.pb.c
        ./Core/Src/comms/cobs.c

        ./Core/Src/glove/bhi360.c
        ./Core/Src/glove/common.c
        ./Core/Src/glove/glove.c

        ./Core/Src/usb/usb_descriptors.c
        ./Core/Src/usb/usb_dfu_rt.c

        Core/Src/gpio.c
        Core/Src/i2c.c
        Core/Src/main.c
        Core/Src/sdio.c
        Core/Src/stm32f4xx_hal_msp.c
        Core/Src/stm32f4xx_it.c
        Core/Src/system_stm32f4xx.c
        Core/Src/tim.c
        Core/Src/usb_otg.c
    
        Core/Startup/startup_stm32f446retx.s
    
        Drivers/bhy2/bhi3_multi_tap.c
        Drivers/bhy2/bhi3.c
        Drivers/bhy2/bhy2_bsec.c
        Drivers/bhy2/bhy2_head_tracker.c
        Drivers/bhy2/bhy2_hif.c
        Drivers/bhy2/bhy2_klio.c
        Drivers/bhy2/bhy2_parse.c
        Drivers/bhy2/bhy2_swim.c
        Drivers/bhy2/bhy2.c
    
        tinyusb/src/tusb.c 
        tinyusb/src/common/tusb_fifo.c 
	tinyusb/src/device/usbd.c 
	tinyusb/src/device/usbd_control.c 
	tinyusb/src/class/cdc/cdc_device.c 
	tinyusb/src/class/dfu/dfu_rt_device.c 
        tinyusb/src/portable/synopsys/dwc2/dcd_dwc2.c

        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c_ex.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd_ex.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_usb.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_sd.c
        Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_sdmmc.c
)

set(include_DIRS
        

        ./Core/Inc
        ./Core/Inc/usb
        ./Core/Inc/glove
        ./Core/Inc/comms
        ./Core/Inc/comms/protobuf
        Drivers/bhy2
        Drivers/bhy2/firmware/bhi360
        Drivers/CMSIS/Include
        Drivers/CMSIS/Device/ST/STM32F4xx/Include
        Drivers/STM32F4xx_HAL_Driver/Inc
        Drivers/STM32F4xx_HAL_Driver/Inc/Legacy

        tinyusb/src
        tinyusb/src/device
        tinyusb/src/portable/synopsys/dwc2
    
)

set(EXECUTABLE ${PROJECT_NAME}.out)
set(EXECUTABLE ${PROJECT_NAME}.elf)
add_executable(${EXECUTABLE} ${source_SRCS})

target_include_directories(${EXECUTABLE} PRIVATE
        ${include_DIRS}
)

target_compile_definitions(${EXECUTABLE} PRIVATE
        ${symbols_SYMB}
)

target_compile_options(${EXECUTABLE} PRIVATE
        ${CPU_PARAMETERS}
        -fdata-sections
        -ffunction-sections
        -Wall
        -O0
        -fstack-usage 
        #-fcyclomatic-complexity
        --specs=nano.specs
        $<$<CONFIG:Debug>:-g3>
)

target_link_options(${EXECUTABLE} PRIVATE
        -T${CMAKE_SOURCE_DIR}/STM32F446RETX_FLASH.ld
        ${CPU_PARAMETERS}
        -specs=nosys.specs
        -u _printf_float
        -Wl,--gc-sections
        -Wl,--start-group
        -lc
        -lm
        # -lnosys
        -Wl,--end-group
        -Wl,-Map=${PROJECT_NAME}.map
)

#Print executable size
add_custom_command(TARGET ${EXECUTABLE}
        POST_BUILD
        COMMAND arm-none-eabi-size ${EXECUTABLE}
)

#Create hex file
add_custom_command(TARGET ${EXECUTABLE}
        POST_BUILD
        COMMAND arm-none-eabi-objcopy -O ihex ${EXECUTABLE} ${PROJECT_NAME}.hex
        COMMAND arm-none-eabi-objcopy -O binary ${EXECUTABLE} ${PROJECT_NAME}.bin
)
