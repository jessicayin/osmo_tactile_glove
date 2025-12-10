/*
 * glove.h
 *
 *      Author: Tess Hellebrekers
 */

#ifndef INC_GLOVE_H_
#define INC_GLOVE_H_

#include "bowie.pb.h"
#include "bhi360.h"


#define MUX_ADDR 		0x70
//#define NUM_DEVICES 	1
#define NUM_DEVICES 	1

typedef enum {
	COMM_OK = 0x00,
	COMM_ERROR = 0x01,
	COMM_PB_ERROR = 0x02,
	COMM_COBS_ERROR = 0x03,
} Comms_StatusTypedef;

/*
 * Bowie glove sensor map
 * {finger, link, sensor_id, mux_channel, device address, bhy2_dev}
 */
extern struct mag_data_dev glove_devices[NUM_DEVICES];

HAL_StatusTypeDef glove_mux_set_channel(uint8_t channel);
void glove_mux_reset();

void glove_load_firmware();
void scan_mux_channel(uint8_t channel);


void glove_send_quat_data(uint32_t index, uint32_t s, uint32_t ns, uint8_t finger, uint8_t link, uint8_t sensor_id, int16_t x, int16_t y, int16_t z, int16_t w, uint16_t accuracy);
void glove_send_mag_data(uint32_t index, uint32_t s, uint32_t ns, uint8_t finger, uint8_t link, uint8_t sensor_id, int16_t mag_x, int16_t mag_y, int16_t mag_z);
Comms_StatusTypedef glove_coms_write(bowie_Data *msg);

int8_t init_island(uint8_t idx);
void init_glove();
void flush_buffer();
void glove_update_data();


#endif /* INC_GLOVE_H_ */
