/**
 * Copyright (c) 2023 Bosch Sensortec GmbH. All rights reserved.
 *
 * BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * @file    common.c
 * @brief   Common source file for the BHy260 examples
 *
 */

#include "common.h"

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>

#include "bhy2_parse.h"
#include "i2c.h"
#include "tim.h"
#include "bhi360.h"

//#define BHA260_SHUTTLE_ID  0x139
//#define BHI260_SHUTTLE_ID  0x119
//#define DEV_ADDRESS_A 0x3A
//#define DEV_ADDRESS_B 0x3C


int8_t bhy2_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    (void)intf_ptr;
    struct mag_data_dev* dev;
    dev = intf_ptr;

    uint16_t DevAddress = dev->dev_addr;
    return HAL_I2C_Mem_Read(&hi2c1, (DevAddress<<1), (uint16_t) reg_addr, sizeof(reg_addr), reg_data, (uint16_t) length, 25);
//    return HAL_I2C_Mem_Read_IT(&hi2c1, (DevAddress<<1), (uint16_t) reg_addr, sizeof(reg_addr), reg_data, (uint16_t) length);

}

//HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)

int8_t bhy2_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
//    (void)intf_ptr;
    struct mag_data_dev* dev;
    dev = intf_ptr;

    uint16_t DevAddress = dev->dev_addr;
    uint16_t MemAddress = (uint16_t) reg_addr;
    uint16_t MemAddSize = sizeof(MemAddress);
    uint16_t Size = (uint16_t) length;
    uint32_t Timeout = 50;
    HAL_StatusTypeDef result = HAL_I2C_Mem_Write(&hi2c1, (DevAddress<<1), MemAddress, 1, reg_data, Size, Timeout);
    return result;
//    return HAL_I2C_Mem_Write_IT(&hi2c1, (DevAddress<<1), (uint16_t) reg_addr, sizeof(reg_addr), reg_data, (uint16_t) length);


}

void bhy2_delay_us(uint32_t us, void *private_data)
{
	(void)private_data;
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    while (__HAL_TIM_GET_COUNTER(&htim1) <  us)
    	;
//    HAL_Delay((uint16_t) period/1000); //10,000 us --> 10 ms

}

char *get_sensor_error_text(uint8_t sensor_error)
{
    char *ret;

    switch (sensor_error)
    {
        case 0x00:
            break;
        case 0x10:
            ret = "[Sensor error] Bootloader reports: Firmware Expected Version Mismatch";
            break;
        case 0x11:
            ret = "[Sensor error] Bootloader reports: Firmware Upload Failed: Bad Header CRC";
            break;
        case 0x12:
            ret = "[Sensor error] Bootloader reports: Firmware Upload Failed: SHA Hash Mismatch";
            break;
        case 0x13:
            ret = "[Sensor error] Bootloader reports: Firmware Upload Failed: Bad Image CRC";
            break;
        case 0x14:
            ret = "[Sensor error] Bootloader reports: Firmware Upload Failed: ECDSA Signature Verification Failed";
            break;
        case 0x15:
            ret = "[Sensor error] Bootloader reports: Firmware Upload Failed: Bad Public Key CRC";
            break;
        case 0x16:
            ret = "[Sensor error] Bootloader reports: Firmware Upload Failed: Signed Firmware Required";
            break;
        case 0x17:
            ret = "[Sensor error] Bootloader reports: Firmware Upload Failed: FW Header Missing";
            break;
        case 0x19:
            ret = "[Sensor error] Bootloader reports: Unexpected Watchdog Reset";
            break;
        case 0x1A:
            ret = "[Sensor error] ROM Version Mismatch";
            break;
        case 0x1B:
            ret = "[Sensor error] Bootloader reports: Fatal Firmware Error";
            break;
        case 0x1C:
            ret = "[Sensor error] Chained Firmware Error: Next Payload Not Found";
            break;
        case 0x1D:
            ret = "[Sensor error] Chained Firmware Error: Payload Not Valid";
            break;
        case 0x1E:
            ret = "[Sensor error] Chained Firmware Error: Payload Entries Invalid";
            break;
        case 0x1F:
            ret = "[Sensor error] Bootloader reports: Bootloader Error: OTP CRC Invalid";
            break;
        case 0x20:
            ret = "[Sensor error] Firmware Init Failed";
            break;
        case 0x21:
            ret = "[Sensor error] Sensor Init Failed: Unexpected Device ID";
            break;
        case 0x22:
            ret = "[Sensor error] Sensor Init Failed: No Response from Device";
            break;
        case 0x23:
            ret = "[Sensor error] Sensor Init Failed: Unknown";
            break;
        case 0x24:
            ret = "[Sensor error] Sensor Error: No Valid Data";
            break;
        case 0x25:
            ret = "[Sensor error] Slow Sample Rate";
            break;
        case 0x26:
            ret = "[Sensor error] Data Overflow (saturated sensor data)";
            break;
        case 0x27:
            ret = "[Sensor error] Stack Overflow";
            break;
        case 0x28:
            ret = "[Sensor error] Insufficient Free RAM";
            break;
        case 0x29:
            ret = "[Sensor error] Sensor Init Failed: Driver Parsing Error";
            break;
        case 0x2A:
            ret = "[Sensor error] Too Many RAM Banks Required";
            break;
        case 0x2B:
            ret = "[Sensor error] Invalid Event Specified";
            break;
        case 0x2C:
            ret = "[Sensor error] More than 32 On Change";
            break;
        case 0x2D:
            ret = "[Sensor error] Firmware Too Large";
            break;
        case 0x2F:
            ret = "[Sensor error] Invalid RAM Banks";
            break;
        case 0x30:
            ret = "[Sensor error] Math Error";
            break;
        case 0x40:
            ret = "[Sensor error] Memory Error";
            break;
        case 0x41:
            ret = "[Sensor error] SWI3 Error";
            break;
        case 0x42:
            ret = "[Sensor error] SWI4 Error";
            break;
        case 0x43:
            ret = "[Sensor error] Illegal Instruction Error";
            break;
        case 0x44:
            ret = "[Sensor error] Bootloader reports: Unhandled Interrupt Error / Exception / Postmortem Available";
            break;
        case 0x45:
            ret = "[Sensor error] Invalid Memory Access";
            break;
        case 0x50:
            ret = "[Sensor error] Algorithm Error: BSX Init";
            break;
        case 0x51:
            ret = "[Sensor error] Algorithm Error: BSX Do Step";
            break;
        case 0x52:
            ret = "[Sensor error] Algorithm Error: Update Sub";
            break;
        case 0x53:
            ret = "[Sensor error] Algorithm Error: Get Sub";
            break;
        case 0x54:
            ret = "[Sensor error] Algorithm Error: Get Phys";
            break;
        case 0x55:
            ret = "[Sensor error] Algorithm Error: Unsupported Phys Rate";
            break;
        case 0x56:
            ret = "[Sensor error] Algorithm Error: Cannot find BSX Driver";
            break;
        case 0x60:
            ret = "[Sensor error] Sensor Self-Test Failure";
            break;
        case 0x61:
            ret = "[Sensor error] Sensor Self-Test X Axis Failure";
            break;
        case 0x62:
            ret = "[Sensor error] Sensor Self-Test Y Axis Failure";
            break;
        case 0x64:
            ret = "[Sensor error] Sensor Self-Test Z Axis Failure";
            break;
        case 0x65:
            ret = "[Sensor error] FOC Failure";
            break;
        case 0x66:
            ret = "[Sensor error] Sensor Busy";
            break;
        case 0x6F:
            ret = "[Sensor error] Self-Test or FOC Test Unsupported";
            break;
        case 0x72:
            ret = "[Sensor error] No Host Interrupt Set";
            break;
        case 0x73:
            ret = "[Sensor error] Event ID Passed to Host Interface Has No Known Size";
            break;
        case 0x75:
            ret = "[Sensor error] Host Download Channel Underflow (Host Read Too Fast)";
            break;
        case 0x76:
            ret = "[Sensor error] Host Upload Channel Overflow (Host Wrote Too Fast)";
            break;
        case 0x77:
            ret = "[Sensor error] Host Download Channel Empty";
            break;
        case 0x78:
            ret = "[Sensor error] DMA Error";
            break;
        case 0x79:
            ret = "[Sensor error] Corrupted Input Block Chain";
            break;
        case 0x7A:
            ret = "[Sensor error] Corrupted Output Block Chain";
            break;
        case 0x7B:
            ret = "[Sensor error] Buffer Block Manager Error";
            break;
        case 0x7C:
            ret = "[Sensor error] Input Channel Not Word Aligned";
            break;
        case 0x7D:
            ret = "[Sensor error] Too Many Flush Events";
            break;
        case 0x7E:
            ret = "[Sensor error] Unknown Host Channel Error";
            break;
        case 0x81:
            ret = "[Sensor error] Decimation Too Large";
            break;
        case 0x90:
            ret = "[Sensor error] Master SPI/I2C Queue Overflow";
            break;
        case 0x91:
            ret = "[Sensor error] SPI/I2C Callback Error";
            break;
        case 0xA0:
            ret = "[Sensor error] Timer Scheduling Error";
            break;
        case 0xB0:
            ret = "[Sensor error] Invalid GPIO for Host IRQ";
            break;
        case 0xB1:
            ret = "[Sensor error] Error Sending Initialized Meta Events";
            break;
        case 0xC0:
            ret = "[Sensor error] Bootloader reports: Command Error";
            break;
        case 0xC1:
            ret = "[Sensor error] Bootloader reports: Command Too Long";
            break;
        case 0xC2:
            ret = "[Sensor error] Bootloader reports: Command Buffer Overflow";
            break;
        case 0xD0:
            ret = "[Sensor error] User Mode Error: Sys Call Invalid";
            break;
        case 0xD1:
            ret = "[Sensor error] User Mode Error: Trap Invalid";
            break;
        case 0xE1:
            ret = "[Sensor error] Firmware Upload Failed: Firmware header corrupt";
            break;
        case 0xE2:
            ret = "[Sensor error] Sensor Data Injection: Invalid input stream";
            break;
        default:
            ret = "[Sensor error] Unknown error code";
    }

    return ret;
}

char *get_sensor_name(uint8_t sensor_id)
{
    char *ret;

    switch (sensor_id)
    {
        case BHY2_SENSOR_ID_ACC_PASS:
            ret = "Accelerometer passthrough";
            break;
        case BHY2_SENSOR_ID_ACC_RAW:
            ret = "Accelerometer uncalibrated";
            break;
        case BHY2_SENSOR_ID_ACC:
            ret = "Accelerometer corrected";
            break;
        case BHY2_SENSOR_ID_ACC_BIAS:
            ret = "Accelerometer offset";
            break;
        case BHY2_SENSOR_ID_ACC_WU:
            ret = "Accelerometer corrected wake up";
            break;
        case BHY2_SENSOR_ID_ACC_RAW_WU:
            ret = "Accelerometer uncalibrated wake up";
            break;
        case BHY2_SENSOR_ID_GYRO_PASS:
            ret = "Gyroscope passthrough";
            break;
        case BHY2_SENSOR_ID_GYRO_RAW:
            ret = "Gyroscope uncalibrated";
            break;
        case BHY2_SENSOR_ID_GYRO:
            ret = "Gyroscope corrected";
            break;
        case BHY2_SENSOR_ID_GYRO_BIAS:
            ret = "Gyroscope offset";
            break;
        case BHY2_SENSOR_ID_GYRO_WU:
            ret = "Gyroscope wake up";
            break;
        case BHY2_SENSOR_ID_GYRO_RAW_WU:
            ret = "Gyroscope uncalibrated wake up";
            break;
        case BHY2_SENSOR_ID_MAG_PASS:
            ret = "Magnetometer passthrough";
            break;
        case BHY2_SENSOR_ID_MAG_RAW:
            ret = "Magnetometer uncalibrated";
            break;
        case BHY2_SENSOR_ID_MAG:
            ret = "Magnetometer corrected";
            break;
        case BHY2_SENSOR_ID_MAG_BIAS:
            ret = "Magnetometer offset";
            break;
        case BHY2_SENSOR_ID_MAG_WU:
            ret = "Magnetometer wake up";
            break;
        case BHY2_SENSOR_ID_MAG_RAW_WU:
            ret = "Magnetometer uncalibrated wake up";
            break;
        case BHY2_SENSOR_ID_GRA:
            ret = "Gravity vector";
            break;
        case BHY2_SENSOR_ID_GRA_WU:
            ret = "Gravity vector wake up";
            break;
        case BHY2_SENSOR_ID_LACC:
            ret = "Linear acceleration";
            break;
        case BHY2_SENSOR_ID_LACC_WU:
            ret = "Linear acceleration wake up";
            break;
        case BHY2_SENSOR_ID_RV:
            ret = "Rotation vector";
            break;
        case BHY2_SENSOR_ID_RV_WU:
            ret = "Rotation vector wake up";
            break;
        case BHY2_SENSOR_ID_GAMERV:
            ret = "Game rotation vector";
            break;
        case BHY2_SENSOR_ID_GAMERV_WU:
            ret = "Game rotation vector wake up";
            break;
        case BHY2_SENSOR_ID_GEORV:
            ret = "Geo-magnetic rotation vector";
            break;
        case BHY2_SENSOR_ID_GEORV_WU:
            ret = "Geo-magnetic rotation vector wake up";
            break;
        case BHY2_SENSOR_ID_ORI:
            ret = "Orientation";
            break;
        case BHY2_SENSOR_ID_ORI_WU:
            ret = "Orientation wake up";
            break;
        case BHY2_SENSOR_ID_ACC_BIAS_WU:
            ret = "Accelerometer offset wake up";
            break;
        case BHY2_SENSOR_ID_GYRO_BIAS_WU:
            ret = "Gyroscope offset wake up";
            break;
        case BHY2_SENSOR_ID_MAG_BIAS_WU:
            ret = "Magnetometer offset wake up";
            break;
        case BHY2_SENSOR_ID_TEMP:
            ret = "Temperature";
            break;
        case BHY2_SENSOR_ID_BARO:
            ret = "Barometer";
            break;
        case BHY2_SENSOR_ID_HUM:
            ret = "Humidity";
            break;
        case BHY2_SENSOR_ID_GAS:
            ret = "Gas";
            break;
        case BHY2_SENSOR_ID_TEMP_WU:
            ret = "Temperature wake up";
            break;
        case BHY2_SENSOR_ID_BARO_WU:
            ret = "Barometer wake up";
            break;
        case BHY2_SENSOR_ID_HUM_WU:
            ret = "Humidity wake up";
            break;
        case BHY2_SENSOR_ID_GAS_WU:
            ret = "Gas wake up";
            break;
        case BHY2_SENSOR_ID_KLIO:
            ret = "Klio";
            break;
        case BHY2_SENSOR_ID_KLIO_LOG:
            ret = "Klio log";
            break;
        case BHY2_SENSOR_ID_SWIM:
            ret = "Swim recognition";
            break;
        case BHY2_SENSOR_ID_SI_ACCEL:
            ret = "SI Accel";
            break;
        case BHY2_SENSOR_ID_SI_GYROS:
            ret = "SI Gyro";
            break;
        case BHY2_SENSOR_ID_LIGHT:
            ret = "Light";
            break;
        case BHY2_SENSOR_ID_LIGHT_WU:
            ret = "Light wake up";
            break;
        case BHY2_SENSOR_ID_PROX:
            ret = "Proximity";
            break;
        case BHY2_SENSOR_ID_PROX_WU:
            ret = "Proximity wake up";
            break;
        case BHY2_SENSOR_ID_STC:
            ret = "Step counter";
            break;
        case BHY2_SENSOR_ID_STC_WU:
            ret = "Step counter wake up";
            break;
        case BHY2_SENSOR_ID_STC_LP:
            ret = "Low Power Step counter";
            break;
        case BHY2_SENSOR_ID_STC_LP_WU:
            ret = "Low Power Step counter wake up";
            break;
        case BHY2_SENSOR_ID_SIG:
            ret = "Significant motion";
            break;
        case BHY2_SENSOR_ID_STD:
            ret = "Step detector";
            break;
        case BHY2_SENSOR_ID_STD_WU:
            ret = "Step detector wake up";
            break;
        case BHY2_SENSOR_ID_TILT_DETECTOR:
            ret = "Tilt detector";
            break;
        case BHY2_SENSOR_ID_WAKE_GESTURE:
            ret = "Wake gesture";
            break;
        case BHY2_SENSOR_ID_GLANCE_GESTURE:
            ret = "Glance gesture";
            break;
        case BHY2_SENSOR_ID_PICKUP_GESTURE:
            ret = "Pickup gesture";
            break;
        case BHY2_SENSOR_ID_SIG_LP:
            ret = "Low Power Significant motion";
            break;
        case BHY2_SENSOR_ID_SIG_LP_WU:
            ret = "Low Power Significant motion wake up";
            break;
        case BHY2_SENSOR_ID_STD_LP:
            ret = "Low Power Step detector";
            break;
        case BHY2_SENSOR_ID_STD_LP_WU:
            ret = "Low Power Step detector wake up";
            break;
        case BHY2_SENSOR_ID_AR:
            ret = "Activity recognition";
            break;
        case BHY2_SENSOR_ID_EXCAMERA:
            ret = "External camera trigger";
            break;
        case BHY2_SENSOR_ID_GPS:
            ret = "GPS";
            break;
        case BHY2_SENSOR_ID_WRIST_TILT_GESTURE:
            ret = "Wrist tilt gesture";
            break;
        case BHY2_SENSOR_ID_DEVICE_ORI:
            ret = "Device orientation";
            break;
        case BHY2_SENSOR_ID_DEVICE_ORI_WU:
            ret = "Device orientation wake up";
            break;
        case BHY2_SENSOR_ID_STATIONARY_DET:
            ret = "Stationary detect";
            break;
        case BHY2_SENSOR_ID_ANY_MOTION_LP:
            ret = "Low Power Any motion";
            break;
        case BHY2_SENSOR_ID_ANY_MOTION_LP_WU:
            ret = "Low Power Any motion wake up";
            break;
        case BHI3_SENSOR_ID_NO_MOTION_LP_WU:
            ret = "Low Power No Motion wake up";
            break;
        case BHY2_SENSOR_ID_MOTION_DET:
            ret = "Motion detect";
            break;
        case BHI3_SENSOR_ID_AR_WEAR_WU:
            ret = "Activity recognition for Wearables";
            break;
        case BHI3_SENSOR_ID_WRIST_WEAR_LP_WU:
            ret = "Low Power Wrist Wear wake up";
            break;
        case BHI3_SENSOR_ID_WRIST_GEST_DETECT_LP_WU:
            ret = "Low Power Wrist Gesture wake up";
            break;
        case BHI3_SENSOR_ID_MULTI_TAP:
            ret = "Multi Tap Detector";
            break;
        case BHY2_SENSOR_ID_AIR_QUALITY:
            ret = "Air Quality";
            break;
        case BHY2_SENSOR_ID_HEAD_ORI_MIS_ALG:
            ret = "Head Misalignment Calibrator";
            break;
        case BHY2_SENSOR_ID_IMU_HEAD_ORI_Q:
            ret = "IMU Head Orientation Quaternion";
            break;
        case BHY2_SENSOR_ID_NDOF_HEAD_ORI_Q:
            ret = "NDOF Head Orientation Quaternion";
            break;
        case BHY2_SENSOR_ID_IMU_HEAD_ORI_E:
            ret = "IMU Head Orientation Euler";
            break;
        case BHY2_SENSOR_ID_NDOF_HEAD_ORI_E:
            ret = "NDOF Head Orientation Euler";
            break;
        default:
            if ((sensor_id >= BHY2_SENSOR_ID_CUSTOM_START) && (sensor_id <= BHY2_SENSOR_ID_CUSTOM_END))
            {
                ret = "Custom sensor ID ";
            }
            else
            {
                ret = "Undefined sensor ID ";
            }
    }

    return ret;
}

char *get_api_error(int8_t error_code)
{
    char *ret = " ";

    switch (error_code)
    {
        case BHY2_OK:
            break;
        case BHY2_E_NULL_PTR:
            ret = "[API Error] Null pointer";
            break;
        case BHY2_E_INVALID_PARAM:
            ret = "[API Error] Invalid parameter";
            break;
        case BHY2_E_IO:
            ret = "[API Error] IO error";
            break;
        case BHY2_E_MAGIC:
            ret = "[API Error] Invalid firmware";
            break;
        case BHY2_E_TIMEOUT:
            ret = "[API Error] Timed out";
            break;
        case BHY2_E_BUFFER:
            ret = "[API Error] Invalid buffer";
            break;
        case BHY2_E_INVALID_FIFO_TYPE:
            ret = "[API Error] Invalid FIFO type";
            break;
        case BHY2_E_INVALID_EVENT_SIZE:
            ret = "[API Error] Invalid Event size";
            break;
        case BHY2_E_PARAM_NOT_SET:
            ret = "[API Error] Parameter not set";
            break;
        default:
            ret = "[API Error] Unknown API error code";
    }

    return ret;
}

void print_api_error(int8_t rslt, struct bhy2_dev *dev)
{
    if (rslt != BHY2_OK)
    {
        printf("%s\r\n", get_api_error(rslt));

        exit(0);
    }
}

bool get_interrupt_status(void)
{

}

