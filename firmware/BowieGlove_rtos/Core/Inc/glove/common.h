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
 * @file    common.h
 * @brief   Common header file for the BHI260/BHA260 examples, adapted for BHI360
 *
 */

#ifndef _COMMON_H_
#define _COMMON_H_

#include <stdbool.h>

#include "bhy2.h"
#include "bhi3.h"
#include "bhi3_multi_tap.h"
#include "i2c.h"

#include "bhy2_klio.h"
#include "bhy2_swim.h"
#include "bhy2_bsec.h"
#include "bhy2_head_tracker.h"

//#define BHY260_APP20_CS_PIN     COINES_SHUTTLE_PIN_7
//#define BHY260_APP20_INT_PIN    COINES_SHUTTLE_PIN_21
//#define BHY260_APP20_RESET_PIN  COINES_SHUTTLE_PIN_8
//#define BHY260_APP30_CS_PIN     COINES_MINI_SHUTTLE_PIN_2_1
//#define BHY260_APP30_INT_PIN    COINES_MINI_SHUTTLE_PIN_1_6
//#define BHY260_APP30_RESET_PIN  COINES_MINI_SHUTTLE_PIN_2_6

//#ifdef PC
//#ifdef COINES_BRIDGE
//#define BHY2_RD_WR_LEN          256    /* Coines bridge maximum read write length */
//#else
#define BHY2_RD_WR_LEN          256     /* USB maximum read write length(DD firmware) */
//#endif
//#else
//#define BHY2_RD_WR_LEN          256    /* MCU maximum read write length */
//#endif

//char *get_coines_error(int16_t rslt);
char *get_api_error(int8_t error_code);
void print_api_error(int8_t rslt, struct bhy2_dev *dev);
char *get_sensor_error_text(uint8_t sensor_error);
char *get_sensor_name(uint8_t sensor_id);
//float get_sensor_default_scaling(uint8_t sensor_id);
//char *get_sensor_parse_format(uint8_t sensor_id);
//char *get_sensor_axis_names(uint8_t sensor_id);
//char *get_klio_error(bhy2_klio_driver_error_state_t error);

int8_t bhy2_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);
int8_t bhy2_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);

//int8_t bowie_i2c_write_it(I2C_HandleTypeDef *hi2c,uint16_t addr, uint8_t *data, uint16_t size);

int8_t bowie_i2c_mem_write_it(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);
int8_t bowie_i2c_mem_read_it(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);


void setup_interfaces(bool reset_power, enum bhy2_intf intf);
void close_interfaces(enum bhy2_intf intf);
int8_t bhy2_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);
uint8_t bhi360_I2C_read(uint8_t *reg_data, uint32_t length);
int8_t bhy2_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);
void bhy2_delay_us(uint32_t us, void *private_data);
bool get_interrupt_status(void);

#endif /* _COMMON_H_ */
