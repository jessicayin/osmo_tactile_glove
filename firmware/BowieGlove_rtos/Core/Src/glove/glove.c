/*
 * glove.c
 *
 *      Author: Tess Hellebrekers
 *      Based of off metahand.h by Mike Lambeta
 */

#include "main.h"
//#include "fmpi2c.h"
#include "i2c.h"

#include "glove/glove.h"
#include "glove/bhi360.h"
#include "glove/common.h"
#include "comms/bowie.pb.h"
#include "comms/cobs.h"
#include "pb.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "tusb.h"


//easy reference for init below
//typedef struct mag_data_dev {
//
//	uint8_t finger;
//	uint8_t link;
//	uint8_t sensor_id; //remove?
//	uint8_t mux_chan;
//	uint8_t dev_addr;
//	uint8_t count;
//	bool init;
//	struct bhy2_dev *dev;
//} t_mag_info;


struct bhy2_dev bhy2_devices[NUM_DEVICES] = {0};
// Note on sensor_id, until bosch updates firmware at end of month, we do not have access to sensor_id=2
//struct mag_data_dev glove_devices[NUM_DEVICES] = {
//		{bowie_Finger_pinky, 	1, 1, 0, 0x1E, 0, 0, NULL},
//		{bowie_Finger_ring, 	1, 1, 2, 0x1B, 0, 0, NULL},
//};

// working on blue board 2/24/24
//struct mag_data_dev glove_devices[NUM_DEVICES] = {
//		{bowie_Finger_thumb, 	3, 1, 0, 0x0D, 0, 0, NULL},
//		{bowie_Finger_index, 	3, 1, 2, 0x3A, 0, 0, NULL},
//		{bowie_Finger_middle, 	1, 1, 3, 0x3A, 0, 0, NULL},
//		{bowie_Finger_index, 	1, 1, 5, 0x3A, 0, 0, NULL}
//};

//#define BOWIE_LINK_PER_FINGER	4
//#define BOWIE_FINGER_PINKY	0
//#define BOWIE_FINGER_RING	4
//#define BOWIE_FINGER_MIDDLE	8
//#define BOWIE_FINGER_INDEX	12
//#define BOWIE_FINGER_THUMB	16

////this is the final layout for full glove
struct mag_data_dev glove_devices[NUM_DEVICES] = {
		{bowie_Finger_pinky, 	1, 1, 0, 0x1B, 0, 0, NULL}, // 0
//		{bowie_Finger_pinky, 	2, 2, 0, 0x1C, 0, 0, NULL},
//		{bowie_Finger_pinky, 	3, 3, 0, 0x1D, 0, 0, NULL},
//		{bowie_Finger_pinky, 	4, 4, 0, 0x1E, 0, 0, NULL},
//		{bowie_Finger_ring, 	1, 5, 2, 0x1B, 0, 0, NULL}, // 4
//		{bowie_Finger_ring, 	2, 6, 2, 0x1C, 0, 0, NULL},
//		{bowie_Finger_ring, 	3, 7, 2, 0x1D, 0, 0, NULL},
//		{bowie_Finger_ring, 	4, 8, 2, 0x1E, 0, 0, NULL},
//		{bowie_Finger_middle, 	1, 9, 3, 0x1B, 0, 0, NULL}, // 8
//		{bowie_Finger_middle, 	2, 10, 3, 0x1C, 0, 0, NULL},
//		{bowie_Finger_middle, 	3, 11, 3, 0x1D, 0, 0, NULL},
//		{bowie_Finger_middle, 	4, 12, 3, 0x1E, 0, 0, NULL},
//		{bowie_Finger_index, 	1, 13, 5, 0x1B, 0, 0, NULL}, // 12
//		{bowie_Finger_index, 	2, 14, 5, 0x1C, 0, 0, NULL},
//		{bowie_Finger_index, 	3, 15, 5, 0x1D, 0, 0, NULL},
//		{bowie_Finger_index, 	4, 16, 5, 0x1E, 0, 0, NULL},
//		{bowie_Finger_thumb, 	1, 17, 7, 0x1B, 0, 0, NULL}, // 16
//		{bowie_Finger_thumb, 	2, 18, 7, 0x1C, 0, 0, NULL},
//		{bowie_Finger_thumb, 	3, 19, 7, 0x1D, 0, 0, NULL},
//		{bowie_Finger_thumb, 	4, 20, 7, 0x1E, 0, 0, NULL}
};

//It takes about 4.5 seconds to initialize ONE sensor. For 20 total, that is almost 2 min :(

//uint8_t work_buffer[WORK_BUFFER_SIZE];

//struct mag_data_dev dev;
// this will turn off all mux i2c channels
void glove_mux_reset() {
	HAL_GPIO_WritePin(MUX_RESET_GPIO_Port, MUX_RESET_GPIO_Pin, GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(MUX_RESET_GPIO_Port, MUX_RESET_GPIO_Pin, GPIO_PIN_SET);
	HAL_Delay(100);
}

void scan_mux_channel(uint8_t channel)
{
	uint16_t i, ret;
	ret = glove_mux_set_channel(channel);
	HAL_Delay(10);


	for (i = 0; i < 128; i++) {
		ret = HAL_I2C_IsDeviceReady(&hi2c1, (i << 1), 3, 25);
		if (ret == HAL_OK) /* No ACK Received At That Address */
		{
			printf("Channel %d found device at: %X \r\n", channel, i);
		}
	}
}

/*
 * channel input can be decimal 0-7 inclusive only
 *
 */
HAL_StatusTypeDef glove_mux_set_channel(uint8_t channel) {
	HAL_StatusTypeDef result = HAL_OK;
	if(channel < 8)
	{
		uint8_t mux_addr = (MUX_ADDR<<1);
		uint8_t mux_channel = (1 << channel);
		result = HAL_I2C_Master_Transmit(
				&hi2c1,
				mux_addr,
				&mux_channel,
				1,
				100
			);
		return result;
	}
	else
	{
		return HAL_ERROR;
	}

}

void init_glove(){

	struct bhy2_phys_sensor_info info;
	uint8_t phys_sensor_id = 0x5;
	uint8_t results[NUM_DEVICES];
	uint8_t i, channel, result;
	for (i = 0; i < NUM_DEVICES; i++) {
		channel = glove_devices[i].mux_chan;
 		result = glove_mux_set_channel(channel);
		glove_devices[i].dev = &bhy2_devices[i];
		glove_devices[i].init = false;
		if(result == HAL_OK)
		{
			// init and save pointer to bhy2_dev
			result = init_island(i); // initialize device, load firmware, and setup sensors
			if(result == 0)
			{
				glove_devices[i].init = true;
			}
		}

		result = bhy2_get_phys_sensor_info(phys_sensor_id, &info, glove_devices[0].dev);

		printf("[%s] Finger: %d, Link: %d, Mux_Chan: %d, Dev_addr: %X, Init: %s \r\n",
				(glove_devices[i].init) ? "PASS" : "FAIL",
				glove_devices[i].finger,
				glove_devices[i].link,
				glove_devices[i].mux_chan,
				glove_devices[i].dev_addr,
				(glove_devices[i].init) ? "true" : "false");
		//both gyro_acc and accel_acc should be true

	}
//
//	for(i=0; i< NUM_DEVICES; i++)
//	{
//		result = result | results[i];
//	}

}

int8_t init_island(uint8_t idx){

	uint8_t result = 0;
	uint8_t NUM_RESULTS = 21;
	int8_t results[NUM_RESULTS], rslt;
	uint16_t version = 0;
	uint8_t product_id = 0;
	uint8_t chip_id = 0;
	uint8_t boot_status = 0;
	uint8_t hintr_ctrl, hif_ctrl;
	uint8_t chip_ctrl;

	uint8_t acc_accuracy, gyro_accuracy, accuracy;
//	uint8_t acc_cal_flag = 0;
//	uint8_t gyro_cal_flag = 0;

	float sample_rate = 100; /* Read out data measured at 100Hz */
	uint32_t report_latency_ms = 0; /* Report immediately */

	// init struct pointers, soft reset and confirm chip and product IDs
	results[0] = bhi360_init(glove_devices[idx].dev, &glove_devices[idx]);
	results[1] = bhy2_soft_reset(glove_devices[idx].dev);
 	results[2] = bhy2_get_product_id(&product_id, glove_devices[idx].dev);
	results[3] = bhy2_get_chip_id(&chip_id, glove_devices[idx].dev);

//	printf("\tlink %d, init: %d, soft_reset: %d, product_id: %d, chip_id: %d\r\n", idx, results[0], results[1], results[2], results[3]);

	if (product_id != BHY2_PRODUCT_ID)
	{
	   results[3] = ERROR;
	}

	// load firmware in low-speed mode
	chip_ctrl = BHY2_CHIP_CTRL_TURBO_ENABLE; //BHY2_CHIP_CTRL_TURBO_ENABLE; // //
	results[4] = bhy2_set_chip_ctrl(chip_ctrl, glove_devices[idx].dev);

	results[19] = bhy2_set_virt_sensor_cfg(BHY2_SENSOR_ID_ACC, sample_rate, report_latency_ms, glove_devices[idx].dev);
	results[20] = bhy2_set_virt_sensor_cfg(BHY2_SENSOR_ID_GYRO, sample_rate, report_latency_ms, glove_devices[idx].dev);


//	printf("\tlink %d, set_chip_ctrl: %d\r\n", idx, results[4]);

	// host interrupt control, TODO confirm settings
//	hintr_ctrl = BHY2_ICTL_DISABLE_FAULT|BHY2_ICTL_DISABLE_FIFO_W| BHY2_ICTL_DISABLE_FIFO_NW |BHY2_ICTL_DISABLE_STATUS_FIFO | BHY2_ICTL_DISABLE_DEBUG;
	hintr_ctrl = BHY2_ICTL_DISABLE_STATUS_FIFO | BHY2_ICTL_DISABLE_DEBUG;
	results[5] = bhy2_set_host_interrupt_ctrl(hintr_ctrl, glove_devices[idx].dev);
//	printf("\tlink %d, set_host_interrupt_ctrl: %d\r\n", idx, results[5]);
//	print_api_error(result1, &glove_devices[idx].dev);
//	bhy2_get_host_interrupt_ctrl(&hintr_ctrl, glove_devices[idx].dev);
//	print_api_error(result1, &glove_devices[idx].dev);
//
////	printf("Host interrupt control\r\n");
////	printf("    Wake up FIFO %s.\r\n", (hintr_ctrl & BHY2_ICTL_DISABLE_FIFO_W) ? "disabled" : "enabled");
////	printf("    Non wake up FIFO %s.\r\n", (hintr_ctrl & BHY2_ICTL_DISABLE_FIFO_NW) ? "disabled" : "enabled");
////	printf("    Status FIFO %s.\r\n", (hintr_ctrl & BHY2_ICTL_DISABLE_STATUS_FIFO) ? "disabled" : "enabled");
////	printf("    Debugging %s.\r\n", (hintr_ctrl & BHY2_ICTL_DISABLE_DEBUG) ? "disabled" : "enabled");
////	printf("    Fault %s.\r\n", (hintr_ctrl & BHY2_ICTL_DISABLE_FAULT) ? "disabled" : "enabled");
////	printf("    Interrupt is %s.\r\n", (hintr_ctrl & BHY2_ICTL_ACTIVE_LOW) ? "active low" : "active high");
////	printf("    Interrupt is %s triggered.\r\n", (hintr_ctrl & BHY2_ICTL_EDGE) ? "pulse" : "level");
////	printf("    Interrupt pin drive is %s.\r\n", (hintr_ctrl & BHY2_ICTL_OPEN_DRAIN) ? "open drain" : "push-pull");
//
	/* Configure the host interface */
	hif_ctrl = 0;
	results[6] = bhy2_set_host_intf_ctrl(hif_ctrl, glove_devices[idx].dev);
//	printf("\tlink %d, set_host_intf_ctrl: %d\r\n", idx, results[6]);

	/* Check the host control settings */
	uint8_t host_ctrl = 0;
//	results[7] = bhy2_set_host_ctrl(host_ctrl, glove_devices[idx].dev);
//	printf("\tlink %d, sset_host_ctrl: %d\r\n", idx, results[7]);

	/* Check if the sensor is ready to load firmware */
	results[8] = bhy2_get_boot_status(&boot_status, glove_devices[idx].dev);
//	printf("\tlink %d, get_boot_status: %d\r\n", idx, results[8]);

	if (boot_status & BHY2_BST_HOST_INTERFACE_READY)
	{
//		results[8] = bhi360_upload_firmware(boot_status, glove_devices[idx].dev);
		results[8] = upload_firmware_partly(glove_devices[idx].dev);

	    if(results[8] != BHY2_OK)
	    {
	    	printf("BAD UPLOAD! ");
	    	return results[8];
	    }

		results[8] = bhy2_boot_from_ram(glove_devices[idx].dev);

	    if(results[8] != BHY2_OK)
	    {
	    	printf("BAD BOOT! ");
	    	return results[8];
	    }


//	    rslt = bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT, parse_meta_event, (void*)&accuracy , glove_devices[idx].dev);
//		printf("\t meta event %d,  %d\r\n", idx, rslt);

//		rslt = bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_ACC, parse_acc_gyro_mag_3axis_s16, (void*)&acc_accuracy, glove_devices[idx].dev);
//		printf("\t Acc %d,  %d\r\n", idx, rslt);

//		rslt = bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_GYRO, parse_acc_gyro_mag_3axis_s16, (void*)&gyro_accuracy, glove_devices[idx].dev);
//		printf("\t gyro %d,  %d\r\n", idx, rslt);

		results[9] = bhy2_get_kernel_version(&version, glove_devices[idx].dev);
		results[10] = bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT, parse_meta_event, NULL, glove_devices[idx].dev);
		results[11] = bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT_WU, parse_meta_event, NULL, glove_devices[idx].dev);
		results[12] = bhy2_register_fifo_parse_callback(QUAT_SENSOR_ID, parse_quaternion, &glove_devices[idx], glove_devices[idx].dev);
		results[13] = bhy2_register_fifo_parse_callback(MAG_ID, parse_magnetometer, &glove_devices[idx], glove_devices[idx].dev);
		results[14] = bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_MAG_PASS_META, parse_magnetometer_meta, &glove_devices[idx], glove_devices[idx].dev);

//		printf("\tlink %d, bhi360_upload_firmware: %d\r\n", idx, results[8]);
//		printf("\tlink %d, bhy2_get_kernel_version: %d\r\n", idx, results[9]);
//		printf("\tlink %d, 1register_fifo_parse_callback: %d\r\n", idx, results[10]);
//		printf("\tlink %d, 2register_fifo_parse_callback: %d\r\n", idx, results[11]);
//		printf("\tlink %d, 3register_fifo_parse_callback: %d\r\n", idx, results[12]);
//		printf("\tlink %d, 4register_fifo_parse_callback: %d\r\n", idx, results[14]);


	}
	else
	{
//		printf("Host interface not ready. Exiting\r\n");
		//TODO: turn off all the LEDS?
		return ERROR;
	}

	/* Update the callback table to enable parsing of sensor data */
	results[15] = bhy2_update_virtual_sensor_list(glove_devices[idx].dev);
//	printf("\tlink %d, bhy2_update_virtual_sensor_list: %d\r\n", idx, results[14]);

//	results[14] = bhy2_get_virt_sensor_list(glove_devices[idx].dev);
//	bhy2_get_virt_sensor_list(glove_devices[1].dev);

	// we can allow sample rate as an input to function, or keep it fixed like this

//	results[15] = 0;
	results[16] = bhy2_set_virt_sensor_cfg(QUAT_SENSOR_ID, sample_rate, report_latency_ms, glove_devices[idx].dev);
//	printf("\tlink %d, bhy2_set_virt_sensor_cfg: %d\r\n", idx, results[15]);

//	printf("Enable %s at %.2fHz.\r\n", get_sensor_name(QUAT_SENSOR_ID), sample_rate);

	results[17] = bhy2_set_virt_sensor_cfg(MAG_ID, sample_rate, report_latency_ms, glove_devices[idx].dev);
//	printf("\tlink %d, bhy2_set_virt_sensor_cfg: %d\r\n", idx, results[16]);

//	printf("Enable %s at %.2fHz.\r\n", get_sensor_name(MAG_ID), sample_rate);

//	results[18] = bhy2_set_virt_sensor_cfg(BHY2_SENSOR_ID_MAG_PASS_META, sample_rate, report_latency_ms, glove_devices[idx].dev);
//	results[19] = bhy2_set_virt_sensor_cfg(BHY2_SENSOR_ID_ACC, sample_rate, report_latency_ms, glove_devices[idx].dev);
//	results[20] = bhy2_set_virt_sensor_cfg(BHY2_SENSOR_ID_GYRO, sample_rate, report_latency_ms, glove_devices[idx].dev);

	uint8_t i;
	for(i=0; i< NUM_RESULTS; i++)
	{
		// combine all the error reports into one. Will need to check this function for more detail on the failure mode
		result = result | results[i];
	}

//	printf("FINAL RESULT: %d\r\n", result);
	return BHY2_OK;

}
void flush_buffer()
{
	uint8_t idx;
	for(idx=0; idx < NUM_DEVICES; idx++)
	{
		bhy2_clear_fifo(0xFF,glove_devices[idx].dev);
//		printf("successful");
	}
}

void glove_update_data()
{

	// run through all the properly initialized devices
	// check fifos for available data
	// if data is available, the registered callbacks will be activated
	// NOTE: if we cannot scan through this list FASTER than the next data ready, it will get stuck in a loop on one sensor only.
	uint8_t work_buffer[WORK_BUFFER_SIZE];
	uint8_t idx;
	uint8_t channel, result;
//
//	uint32_t count = 0;
//	uint32_t NUM_READS = 100;

//	HAL_GPIO_WritePin(GPIOA, RED_LED_STATUS_Pin, GPIO_PIN_RESET);
//	int32_t start = HAL_GetTick();
	for(idx=0; idx < NUM_DEVICES; idx++)
	{

		// TODO: add error checking to skip sensor if data line is corrupted during movement
		if(glove_devices[idx].init == true)
		{
			// TODO: can optimize by checking current mux channel before sending another command
			channel = glove_devices[idx].mux_chan;
			result = glove_mux_set_channel(channel);
			if(result==HAL_OK)
			{
 				result = bhy2_get_and_process_fifo(work_buffer, WORK_BUFFER_SIZE, glove_devices[idx].dev);
			}
		}

	}
//	int32_t stop =  HAL_GetTick();
//	int32_t elapsed = stop - start;
//	printf("elapsed time: %d\r\n", elapsed);
//	HAL_GPIO_WritePin(GPIOA, RED_LED_STATUS_Pin, GPIO_PIN_SET);


}



void glove_send_quat_data(uint32_t index, uint32_t s, uint32_t ns, uint8_t finger, uint8_t link, uint8_t sensor_id, int16_t x, int16_t y, int16_t z, int16_t w, uint16_t accuracy)
{
//	typedef struct _bowie_Quat {
//	    int32_t seconds;
//	    int32_t nanoseconds;
//	    float x;
//	    float y;
//	    float z;
//	    float w;
//	    float accuracy;
//	} bowie_Quat;

	bowie_Data msg = bowie_Data_init_zero;

	msg.index = index; //TODO update to system ticks
	msg.finger = finger;
	msg.link = link;
	msg.sensor_id = sensor_id; //TODO: update to 1
	msg.has_quat = true;
	msg.quat.seconds = s;  //TODO: validate
	msg.quat.nanoseconds = ns; //TODO: validate output rate
	msg.quat.x = x / 16384.0f;
	msg.quat.y = y / 16384.0f;
	msg.quat.z = z / 16384.0f;
	msg.quat.w = w / 16384.0f;
	msg.quat.accuracy = accuracy;// (((accuracy * 180.0f) / 16384.0f) / 3.141592653589793f);

	glove_coms_write(&msg);
}
// save sensor into protobuf structure message
void glove_send_mag_data(uint32_t index, uint32_t s, uint32_t ns, uint8_t finger, uint8_t link, uint8_t sensor_id, int16_t mag_x, int16_t mag_y, int16_t mag_z)
{

//	typedef struct _bowie_Data {
//	    int32_t timestamp;
//	    bowie_Finger finger;
//	    int32_t link;
//	    int32_t sensor_id;
//	    bool has_mag;
//	    bowie_Mag mag;
//	    bool has_quat;
//	    bowie_Quat quat;
//	} bowie_Data;

	bowie_Data msg = bowie_Data_init_zero;

	msg.index = index; //TODO update to system ticks
	msg.finger = finger;
	msg.link = link;
	msg.sensor_id = sensor_id;
	msg.has_mag = true;
	msg.mag.seconds = s;  //TODO: validate
	msg.mag.nanoseconds = ns; //TODO: validate output rate
	msg.mag.x = mag_x ; //* (5000.0f / 32768.0f); // / 16384.0f;
	msg.mag.y = mag_y; //* (5000.0f / 32768.0f); // / 16384.0f;
	msg.mag.z = mag_z; //* (5000.0f / 32768.0f); // / 16384.0f;

	// send empty quat data to maintain packet size

	glove_coms_write(&msg);

}

// encode into protobuf, encode into cobs, then send to USB
Comms_StatusTypedef glove_coms_write(bowie_Data *msg)
{
	//
	cobs_encode_result res;
	static uint8_t out_buffer[256];
	static uint8_t pb_buffer[128];

	pb_ostream_t pb_ostream = pb_ostream_from_buffer(pb_buffer, sizeof(pb_buffer));

	if(!pb_encode(&pb_ostream, bowie_Data_fields, msg)) {
		return COMM_PB_ERROR;
	}

	res = cobs_encode(out_buffer, sizeof(out_buffer),
				pb_buffer, pb_ostream.bytes_written);

	if(res.status != COBS_ENCODE_OK) {
		return COMM_COBS_ERROR;
	}

	tud_cdc_write(&out_buffer, res.out_len);
	tud_cdc_write_flush();
	HAL_GPIO_TogglePin(GPIOA, BLUE_LED_STOP_Pin);
	return COMM_OK;
}
