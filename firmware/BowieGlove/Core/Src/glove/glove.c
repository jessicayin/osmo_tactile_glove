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
#include "glove/debug_log.h"
#include "glove/watchdog.h"
#include "glove/usb_service.h"


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
static uint8_t glove_work_buffer[WORK_BUFFER_SIZE];
static uint8_t mux_current_channel = 0xFFU;
static uint8_t device_error_count[NUM_DEVICES] = {0};
static uint32_t device_last_reinit_ms[NUM_DEVICES] = {0};

#define GLOVE_I2C_RECOVER_AFTER 3U
#define GLOVE_REINIT_AFTER       8U
#define GLOVE_REINIT_COOLDOWN_MS 10000U
#define GLOVE_RETRY_AFTER_FAILED_INIT_MS 2000U
// Note on sensor_id, until bosch updates firmware at end of month, we do not have access to sensor_id=2
//struct mag_data_dev glove_devices[NUM_DEVICES] = {
//		{bowie_Finger_pinky, 	1, 1, 0, 0x3B, 0, 0, NULL},
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
		{bowie_Finger_pinky, 	2, 2, 0, 0x1C, 0, 0, NULL},
		{bowie_Finger_pinky, 	3, 3, 0, 0x1D, 0, 0, NULL},
		{bowie_Finger_pinky, 	4, 4, 0, 0x1E, 0, 0, NULL},
		{bowie_Finger_ring, 	1, 5, 2, 0x1B, 0, 0, NULL}, // 4
		{bowie_Finger_ring, 	2, 6, 2, 0x1C, 0, 0, NULL},
		{bowie_Finger_ring, 	3, 7, 2, 0x1D, 0, 0, NULL},
		{bowie_Finger_ring, 	4, 8, 2, 0x1E, 0, 0, NULL},
		{bowie_Finger_middle, 	1, 9, 3, 0x1B, 0, 0, NULL}, // 8
		{bowie_Finger_middle, 	2, 10, 3, 0x1C, 0, 0, NULL},
		{bowie_Finger_middle, 	3, 11, 3, 0x1D, 0, 0, NULL},
		{bowie_Finger_middle, 	4, 12, 3, 0x1E, 0, 0, NULL},
		{bowie_Finger_index, 	1, 13, 5, 0x1B, 0, 0, NULL}, // 12
		{bowie_Finger_index, 	2, 14, 5, 0x1C, 0, 0, NULL},
		{bowie_Finger_index, 	3, 15, 5, 0x1D, 0, 0, NULL},
		{bowie_Finger_index, 	4, 16, 5, 0x1E, 0, 0, NULL},
		{bowie_Finger_thumb, 	1, 17, 7, 0x1B, 0, 0, NULL}, // 16
		{bowie_Finger_thumb, 	2, 18, 7, 0x1C, 0, 0, NULL},
		{bowie_Finger_thumb, 	3, 19, 7, 0x1D, 0, 0, NULL},
		{bowie_Finger_thumb, 	4, 20, 7, 0x1E, 0, 0, NULL}
};

//It takes about 4.5 seconds to initialize ONE sensor. For 20 total, that is almost 2 min :(

//uint8_t work_buffer[WORK_BUFFER_SIZE];

//struct mag_data_dev dev;
// this will turn off all mux i2c channels
void glove_mux_reset() {
	uint16_t mux_addr = (MUX_ADDR)<<1;
	uint8_t mux_chan[1];
	mux_chan[0] = 0x00;

//	uint8_t mux_reset_addr = (MUX_ADDR)<<1;
	(void)HAL_I2C_Master_Transmit(&hi2c1, mux_addr, mux_chan, 1, 100);
    mux_current_channel = 0xFFU;
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
			BOWIE_LOG("Channel %d found device at: %X \r\n", channel, i);
		}
	}
}

/*
 * channel input can be decimal 0-7 inclusive only
 *
 */
HAL_StatusTypeDef glove_mux_set_channel(uint8_t channel) {
    uint8_t control_register[1];
    uint16_t mux_addr = (MUX_ADDR) << 1;
    HAL_StatusTypeDef result;

    if (channel >= 8U)
    {
        return HAL_ERROR;
    }
    if (channel == mux_current_channel)
    {
        return HAL_OK;
    }

    control_register[0] = (uint8_t)((uint8_t)1U << channel);
    result = HAL_I2C_Master_Transmit(&hi2c1, mux_addr, control_register, 1U, 100U);
    if (result == HAL_OK)
    {
        mux_current_channel = channel;
        bhy2_delay_us(100U, NULL);
    }
    else
    {
        mux_current_channel = 0xFFU;
    }
    return result;
}

void init_glove(){

	uint8_t i, channel;
	HAL_StatusTypeDef mux_result;
	HAL_StatusTypeDef probe_result;
	int8_t init_result;

	for (i = 0; i < NUM_DEVICES; i++) {
		bowie_watchdog_feed();
		bowie_usb_poll();
		channel = glove_devices[i].mux_chan;
		glove_devices[i].dev = &bhy2_devices[i];
		glove_devices[i].init = false;
		glove_devices[i].present = false;
		glove_devices[i].recover_requested = 0U;
		probe_result = HAL_ERROR;

		mux_result = glove_mux_set_channel(channel);
		if (mux_result == HAL_OK)
		{
			HAL_Delay(2);
			/* Do not spend time loading firmware into an empty slot. */
			probe_result = HAL_I2C_IsDeviceReady(
				&hi2c1,
				(uint16_t)(glove_devices[i].dev_addr << 1),
				2,
				20);
		}

		if (probe_result == HAL_OK)
		{
			glove_devices[i].present = true;
			device_last_reinit_ms[i] = HAL_GetTick();
			init_result = init_island(i);
			if (init_result == 0)
			{
				glove_devices[i].init = true;
				glove_devices[i].last_data_ms = HAL_GetTick();
			}
		}

		BOWIE_LOG("[%s] Finger: %d, Link: %d, Mux_Chan: %d, Dev_addr: %X, Init: %s \r\n",
				(glove_devices[i].init) ? "PASS" : ((probe_result == HAL_OK) ? "FAIL" : "SKIP"),
				glove_devices[i].finger,
				glove_devices[i].link,
				glove_devices[i].mux_chan,
				glove_devices[i].dev_addr,
				(glove_devices[i].init) ? "true" : "false");
	}
}

int8_t init_island(uint8_t idx){
	bowie_watchdog_feed();

	uint8_t result = 0;
	uint8_t NUM_RESULTS = 19;
	int8_t results[NUM_RESULTS];
	uint16_t version = 0;
	uint8_t product_id = 0;
	uint8_t chip_id = 0;
	uint8_t boot_status = 0;
	uint8_t hintr_ctrl, hif_ctrl;
	uint8_t chip_ctrl;

	// init struct pointers, soft reset and confirm chip and product IDs
	results[0] = bhi360_init(glove_devices[idx].dev, &glove_devices[idx]);
	results[1] = bhy2_soft_reset(glove_devices[idx].dev);
 	results[2] = bhy2_get_product_id(&product_id, glove_devices[idx].dev);
	results[3] = bhy2_get_chip_id(&chip_id, glove_devices[idx].dev);

//	BOWIE_LOG("\tlink %d, init: %d, soft_reset: %d, product_id: %d, chip_id: %d\r\n", idx, results[0], results[1], results[2], results[3]);

	if (product_id != BHY2_PRODUCT_ID)
	{
	   results[3] = ERROR;
	}

	// load firmware in low-speed mode
	chip_ctrl = BHY2_CHIP_CTRL_TURBO_ENABLE; //BHY2_CHIP_CTRL_TURBO_ENABLE; // //
	results[4] = bhy2_set_chip_ctrl(chip_ctrl, glove_devices[idx].dev);

//	BOWIE_LOG("\tlink %d, set_chip_ctrl: %d\r\n", idx, results[4]);

	// host interrupt control, TODO confirm settings
//	hintr_ctrl = BHY2_ICTL_DISABLE_FAULT|BHY2_ICTL_DISABLE_FIFO_W| BHY2_ICTL_DISABLE_FIFO_NW |BHY2_ICTL_DISABLE_STATUS_FIFO | BHY2_ICTL_DISABLE_DEBUG;
	hintr_ctrl = BHY2_ICTL_ACTIVE_LOW;
//	hintr_ctrl = BHY2_ICTL_DISABLE_STATUS_FIFO | BHY2_ICTL_DISABLE_DEBUG;
	results[5] = bhy2_set_host_interrupt_ctrl(hintr_ctrl, glove_devices[idx].dev);
//	BOWIE_LOG("\tlink %d, set_host_interrupt_ctrl: %d\r\n", idx, results[5]);
//	print_api_error(result1, &glove_devices[idx].dev);
//	bhy2_get_host_interrupt_ctrl(&hintr_ctrl, glove_devices[idx].dev);
//	print_api_error(result1, &glove_devices[idx].dev);

//	BOWIE_LOG("Host interrupt control\r\n");
//	BOWIE_LOG("    Wake up FIFO %s.\r\n", (hintr_ctrl & BHY2_ICTL_DISABLE_FIFO_W) ? "disabled" : "enabled");
//	BOWIE_LOG("    Non wake up FIFO %s.\r\n", (hintr_ctrl & BHY2_ICTL_DISABLE_FIFO_NW) ? "disabled" : "enabled");
//	BOWIE_LOG("    Status FIFO %s.\r\n", (hintr_ctrl & BHY2_ICTL_DISABLE_STATUS_FIFO) ? "disabled" : "enabled");
//	BOWIE_LOG("    Debugging %s.\r\n", (hintr_ctrl & BHY2_ICTL_DISABLE_DEBUG) ? "disabled" : "enabled");
//	BOWIE_LOG("    Fault %s.\r\n", (hintr_ctrl & BHY2_ICTL_DISABLE_FAULT) ? "disabled" : "enabled");
//	BOWIE_LOG("    Interrupt is %s.\r\n", (hintr_ctrl & BHY2_ICTL_ACTIVE_LOW) ? "active low" : "active high");
//	BOWIE_LOG("    Interrupt is %s triggered.\r\n", (hintr_ctrl & BHY2_ICTL_EDGE) ? "pulse" : "level");
//	BOWIE_LOG("    Interrupt pin drive is %s.\r\n", (hintr_ctrl & BHY2_ICTL_OPEN_DRAIN) ? "open drain" : "push-pull");
//
	/* Configure the host interface */
	hif_ctrl = 0;
	results[6] = bhy2_set_host_intf_ctrl(hif_ctrl, glove_devices[idx].dev);
//	BOWIE_LOG("\tlink %d, set_host_intf_ctrl: %d\r\n", idx, results[6]);

	/* Check the host control settings */
	uint8_t host_ctrl = 0;
	results[7] = bhy2_set_host_ctrl(host_ctrl, glove_devices[idx].dev);
//	BOWIE_LOG("\tlink %d, sset_host_ctrl: %d\r\n", idx, results[7]);


//	results[7] = bhy2_get_virt_sensor_list(glove_devices[idx].dev);
	/* Check if the sensor is ready to load firmware */
	results[8] = bhy2_get_boot_status(&boot_status, glove_devices[idx].dev);
//	BOWIE_LOG("\tlink %d, get_boot_status: %d\r\n", idx, results[8]);

	if (boot_status & BHY2_BST_HOST_INTERFACE_READY)
	{
//		results[8] = bhi360_upload_firmware(boot_status, glove_devices[idx].dev);
		results[8] = upload_firmware_partly(glove_devices[idx].dev);

	    if(results[8] != BHY2_OK)
	    {
	    	BOWIE_LOG("BAD UPLOAD! ");
	    	return results[8];
	    }

		results[8] = bhy2_boot_from_ram(glove_devices[idx].dev);

	    if(results[8] != BHY2_OK)
	    {
	    	BOWIE_LOG("BAD BOOT! ");
	    	return results[8];
	    }

		results[9] = bhy2_get_kernel_version(&version, glove_devices[idx].dev);
		results[10] = bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT, parse_meta_event, &glove_devices[idx], glove_devices[idx].dev);
		results[11] = bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT_WU, parse_meta_event, &glove_devices[idx], glove_devices[idx].dev);
		results[12] = bhy2_register_fifo_parse_callback(QUAT_SENSOR_ID, parse_quaternion, &glove_devices[idx], glove_devices[idx].dev);
		results[13] = bhy2_register_fifo_parse_callback(MAG_ID, parse_magnetometer, &glove_devices[idx], glove_devices[idx].dev);
		results[14] = bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_MAG_PASS_META, parse_magnetometer_meta, &glove_devices[idx], glove_devices[idx].dev);

//		BOWIE_LOG("\tlink %d, bhi360_upload_firmware: %d\r\n", idx, results[8]);
//		BOWIE_LOG("\tlink %d, bhy2_get_kernel_version: %d\r\n", idx, results[9]);
//		BOWIE_LOG("\tlink %d, 1register_fifo_parse_callback: %d\r\n", idx, results[10]);
//		BOWIE_LOG("\tlink %d, 2register_fifo_parse_callback: %d\r\n", idx, results[11]);
//		BOWIE_LOG("\tlink %d, 3register_fifo_parse_callback: %d\r\n", idx, results[12]);
//		BOWIE_LOG("\tlink %d, 4register_fifo_parse_callback: %d\r\n", idx, results[13]);


	}
	else
	{
//		BOWIE_LOG("Host interface not ready. Exiting\r\n");
		//TODO: turn off all the LEDS?
		return ERROR;
	}

	/* Update the callback table to enable parsing of sensor data */
	results[15] = bhy2_update_virtual_sensor_list(glove_devices[idx].dev);
//	BOWIE_LOG("\tlink %d, bhy2_update_virtual_sensor_list: %d\r\n", idx, results[14]);

//	results[14] = bhy2_get_virt_sensor_list(glove_devices[idx].dev);

//		uint8_t phys_sensor_id = 0x5;
//	for(uint8_t i=0; i< 64; i++)
//	{
//		struct bhy2_phys_sensor_info info;
//		result = bhy2_get_phys_sensor_info(i, &info, glove_devices[idx].dev);
//		if(result==HAL_OK)
//		{
//			BOWIE_LOG("found phys sensor!");
//		}
//	}


//	bhy2_get_virt_sensor_list(glove_devices[0].dev);
//	struct bhy2_sensor_info info;
//	bhy2_get_sensor_info(MAG_ID, &info, glove_devices[0].dev);
//	bhy2_get_sensor_info(101, &info, glove_devices[0].dev);
//	bhy2_get_sensor_info(5, &info, glove_devices[0].dev);

	// we can allow sample rate as an input to function, or keep it fixed like this
	float sample_rate = 25; /* Read out data measured at 100Hz */
	uint32_t report_latency_ms = 0; /* Report immediately */
//	results[15] = 0;
	results[16] = bhy2_set_virt_sensor_cfg(QUAT_SENSOR_ID, sample_rate, report_latency_ms, glove_devices[idx].dev);
//	BOWIE_LOG("\tlink %d, bhy2_set_virt_sensor_cfg: %d\r\n", idx, results[15]);

//	BOWIE_LOG("Enable %s at %.2fHz.\r\n", get_sensor_name(QUAT_SENSOR_ID), sample_rate);

	results[17] = bhy2_set_virt_sensor_cfg(MAG_ID, sample_rate, report_latency_ms, glove_devices[idx].dev);

	results[18] = bhy2_set_virt_sensor_cfg(BHY2_SENSOR_ID_MAG_PASS_META, sample_rate, report_latency_ms, glove_devices[idx].dev);
//	BOWIE_LOG("\tlink %d, bhy2_set_virt_sensor_cfg: %d\r\n", idx, results[16]);

//	BOWIE_LOG("Enable %s at %.2fHz.\r\n", get_sensor_name(MAG_ID), sample_rate);

	uint8_t i;
	for(i=0; i< NUM_RESULTS; i++)
	{
		// combine all the error reports into one. Will need to check this function for more detail on the failure mode
		result = result | results[i];
	}
//	BOWIE_LOG("FINAL RESULT: %d\r\n", result);
	return result;

}

static void glove_handle_device_error(uint8_t idx)
{
    uint32_t now;

    if (idx >= NUM_DEVICES)
    {
        return;
    }

    if (device_error_count[idx] < 0xFFU)
    {
        device_error_count[idx]++;
    }

    if (device_error_count[idx] == GLOVE_I2C_RECOVER_AFTER)
    {
        BOWIE_LOG("link %u: I2C recovery after repeated errors\r\n", (unsigned)idx);
        bowie_i2c_recover();
        mux_current_channel = 0xFFU;
    }

    if (device_error_count[idx] >= GLOVE_REINIT_AFTER)
    {
        now = HAL_GetTick();
        if ((device_last_reinit_ms[idx] == 0U) ||
            ((uint32_t)(now - device_last_reinit_ms[idx]) >= GLOVE_REINIT_COOLDOWN_MS))
        {
            device_last_reinit_ms[idx] = now;
            BOWIE_LOG("link %u: reinitializing BHI360\r\n", (unsigned)idx);
            glove_devices[idx].init = false;
            HAL_Delay(2);
            if (init_island(idx) == 0)
            {
                glove_devices[idx].init = true;
                glove_devices[idx].last_data_ms = HAL_GetTick();
                BOWIE_LOG("link %u: reinitialization OK\r\n", (unsigned)idx);
            }
            else
            {
                BOWIE_LOG("link %u: reinitialization FAILED\r\n", (unsigned)idx);
            }
            device_error_count[idx] = 0U;
            mux_current_channel = 0xFFU;
        }
    }
}

void glove_update_data()
{
    uint8_t idx;
    uint8_t channel;
    int8_t result;

    for (idx = 0U; idx < NUM_DEVICES; idx++)
    {
        bowie_watchdog_feed();
        bowie_usb_poll();

        if (glove_devices[idx].init != true)
        {
            uint32_t retry_now;
            if (glove_devices[idx].present != true)
            {
                continue;
            }

            retry_now = HAL_GetTick();
            if ((device_last_reinit_ms[idx] != 0U) &&
                ((uint32_t)(retry_now - device_last_reinit_ms[idx]) < GLOVE_RETRY_AFTER_FAILED_INIT_MS))
            {
                continue;
            }

            device_last_reinit_ms[idx] = retry_now;
            bowie_i2c_recover();
            bowie_watchdog_feed();
            if (init_island(idx) == 0)
            {
                glove_devices[idx].init = true;
                glove_devices[idx].last_data_ms = HAL_GetTick();
                device_error_count[idx] = 0U;
            }
            continue;
        }

        if (glove_devices[idx].recover_requested)
        {
            glove_devices[idx].recover_requested = 0U;
            glove_devices[idx].present = 1U;
            device_last_reinit_ms[idx] = HAL_GetTick();
            bowie_i2c_recover();
            if (init_island(idx) == 0)
            {
                glove_devices[idx].init = true;
                glove_devices[idx].last_data_ms = HAL_GetTick();
            }
            else
            {
                glove_devices[idx].init = false;
            }
            device_error_count[idx] = 0U;
            mux_current_channel = 0xFFU;
            continue;
        }

        channel = glove_devices[idx].mux_chan;
        if (glove_mux_set_channel(channel) != HAL_OK)
        {
            glove_handle_device_error(idx);
            continue;
        }

        result = bhy2_get_and_process_fifo(glove_work_buffer, WORK_BUFFER_SIZE, glove_devices[idx].dev);
        if (result != BHY2_OK)
        {
            glove_handle_device_error(idx);
        }
        else
        {
            uint32_t now_ms = HAL_GetTick();
            if (glove_devices[idx].last_data_ms == 0U)
            {
                glove_devices[idx].last_data_ms = now_ms;
                device_error_count[idx] = 0U;
            }
            else if ((uint32_t)(now_ms - glove_devices[idx].last_data_ms) > 500U)
            {
                glove_handle_device_error(idx);
            }
            else
            {
                device_error_count[idx] = 0U;
            }
        }
    }
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
//	typedef struct _bowie_Data {
//	    int32_t index;
//	    bowie_Finger finger;
//	    int32_t link;
//	    int32_t sensor_id;
//	    bool has_mag;
//	    bowie_Mag mag;
//	    bool has_quat;
//	    bowie_Quat quat;
//	} bowie_Data;

	bowie_Data msg = bowie_Data_init_zero;


//	msg.timestamp = HAL_GetTick();
	msg.index = index; //TODO update to system ticks
	msg.finger = finger;
	msg.link = link;
	msg.sensor_id = sensor_id; //TODO: update to 1

	msg.has_quat = true;
	msg.quat.seconds = s;
	msg.quat.nanoseconds = ns;
	msg.quat.x = x / 16384.0f;
	msg.quat.y = y / 16384.0f;
	msg.quat.z = z / 16384.0f;
	msg.quat.w = w / 16384.0f;
	msg.quat.accuracy = (((accuracy * 180.0f) / 16384.0f) / 3.141592653589793f);

//	msg.which_payload = bowie_Data_quat_tag;
//	msg.payload.quat.seconds = s;
//	msg.payload.quat.nanoseconds = ns; //TODO: validate output rate
//	msg.payload.quat.x = x / 16384.0f;
//	msg.payload.quat.y = y / 16384.0f;
//	msg.payload.quat.z = z / 16384.0f;
//	msg.payload.quat.w = w / 16384.0f;
//	msg.payload.quat.accuracy = (((accuracy * 180.0f) / 16384.0f) / 3.141592653589793f);

	// send empty mag data to maintain packet size
//	msg.has_mag = false;
//	msg.mag.seconds = 0;  //TODO: validate
//	msg.mag.nanoseconds = 0; //TODO: validate output rate
//	msg.mag.x = 0;
//	msg.mag.y = 0;
//	msg.mag.z = 0;

//	BOWIE_LOG("quat msg %d\r\n", sizeof(msg));
	glove_coms_write(&msg);
}
// save sensor into protobuf structure message
void glove_send_mag_data(uint32_t index, uint32_t s, uint32_t ns, uint8_t finger, uint8_t link, uint8_t sensor_id, int16_t mag_x, int16_t mag_y, int16_t mag_z)
{

//	typedef struct _bowie_Data {
//	    int32_t index;
//	    bowie_Finger finger;
//	    int32_t link;
//	    int32_t sensor_id;
//	    bool has_mag;
//	    bowie_Mag mag;
//	    bool has_quat;
//	    bowie_Quat quat;
//	} bowie_Data;

	bowie_Data msg = bowie_Data_init_zero;

//	msg.timestamp = HAL_GetTick();
	msg.index = index; //TODO update to system ticks
	msg.finger = finger;
	msg.link = link;
	msg.sensor_id = sensor_id;

	msg.has_mag = true;
	msg.mag.seconds = s;  //TODO: validate
	msg.mag.nanoseconds = ns; //TODO: validate output rate

	msg.mag.x = mag_x;
	msg.mag.y = mag_y;
	msg.mag.z = mag_z;

//	msg.mag.x = mag_x * (2500.0f / 32768.0f);
//	msg.mag.y = mag_y * (2500.0f / 32768.0f);
//	msg.mag.z = mag_z * (2500.0f / 32768.0f);

//	msg.which_payload = bowie_Data_mag_tag;
//	msg.payload.mag.seconds = s;  //TODO: validate
//	msg.payload.mag.nanoseconds = ns; //TODO: validate output rate
//	msg.payload.mag.x = mag_x / 2000.0f;
//	msg.payload.mag.y = mag_y / 2000.0f;
//	msg.payload.mag.z = mag_z / 2000.0f;


	// send empty quat data to maintain packet size
//	msg.has_quat = false;
//	msg.quat.seconds = 0;  //TODO: validate
//	msg.quat.nanoseconds = 0; //TODO: validate output rate
//	msg.quat.x = 0;
//	msg.quat.y = 0;
//	msg.quat.z = 0;
//	msg.quat.w = 0;
//	msg.quat.accuracy = 0;

//	BOWIE_LOG("mag msg %d\r\n", sizeof(msg));
	glove_coms_write(&msg);

}

// encode into protobuf, encode into cobs, then send to USB
Comms_StatusTypedef glove_coms_write(bowie_Data *msg)
{
	//
	cobs_encode_result res;
	static uint8_t out_buffer[128];
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
//	BOWIE_LOG("pb_ostream %d\r\n", pb_ostream.bytes_written);
	if (!tud_ready()) {
		bowie_usb_note_tx_failure();
		return COMM_ERROR;
	}
	if (tud_cdc_write_available() < res.out_len) {
		bowie_usb_note_tx_failure();
		return COMM_ERROR;
	}
	{
		uint32_t written = tud_cdc_write(out_buffer, res.out_len);
		if (written != res.out_len) {
			bowie_usb_note_tx_failure();
			return COMM_ERROR;
		}
		bowie_usb_note_tx_success();
		(void)tud_cdc_write_flush();
	}
//	tud_cdc_write_flush();
	HAL_GPIO_TogglePin(GPIOA, BLUE_LED_STOP_Pin);
	return COMM_OK;
}

uint8_t flush_glove()
{
	uint8_t i, channel, result, result_mag, result_quat;
	for (i = 0; i < NUM_DEVICES; i++) {
		channel = glove_devices[i].mux_chan;
		result = glove_mux_set_channel(channel);

		if(result == HAL_OK)
		{
			result_quat = flush_quat_island(i);
			if(result_quat != HAL_OK)
			{
				return ERROR;
			}
			result_mag = flush_mag_island(i);
			if(result_mag != HAL_OK)
			{
				return ERROR;
			}
		}

	}
	return HAL_OK;

}

// Clear any data in the mag FIFO
uint8_t flush_mag_island(uint8_t index)
{
	uint8_t result = 0;

	result = bhy2_flush_fifo(0xFE, glove_devices[index].dev);

	return result;
}

uint8_t flush_quat_island(uint8_t index)
{
	uint8_t result = 0;

	result = bhy2_flush_fifo(0xFE, glove_devices[index].dev);

	return result;
}

