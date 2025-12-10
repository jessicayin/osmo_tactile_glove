#ifndef __BHI360_H__
#define __BHI360_H__

#ifdef __cplusplus
extern "C" {
#endif

//#include "main.h"
#include "bhy2.h"
#include "bhy2_parse.h"
#include <stdbool.h>

#define WORK_BUFFER_SIZE  1024

#define QUAT_SENSOR_ID    BHY2_SENSOR_ID_GAMERV // ACC+GYR
#define MAG_ID			  BHY2_SENSOR_ID_MAG // Mag Corrected uT

typedef struct mag_data_dev {

	uint8_t finger;
	uint8_t link;
	uint8_t sensor_id;
	uint8_t mux_chan;
	uint8_t dev_addr;
	uint32_t count;
	bool init;
	struct bhy2_dev *dev;
} t_mag_info;

//int8_t bhi360_init(struct bhy2_dev *dev);
int8_t bhi360_init(struct bhy2_dev *dev, struct mag_data_dev *device);

int8_t bhi360_upload_firmware(uint8_t boot_stat, struct bhy2_dev *dev);
int8_t upload_firmware_partly(struct bhy2_dev *dev);


void parse_quaternion(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);
void parse_magnetometer(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);
void parse_magnetometer_meta(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);
void parse_meta_event(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);

void parse_acc_gyro_mag_3axis_s16(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);

#ifdef __cplusplus
}
#endif

#endif /* __BHI360_H__ */
