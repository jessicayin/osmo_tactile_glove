/* Includes ------------------------------------------------------------------*/
#include "i2c.h"
#define __STDC_FORMAT_MACROS
#include <inttypes.h>

#include "bhy2_defs.h"
#include "bhy2.h"
#include "bhy2_parse.h"
//#include "firmware/bhi360/BHI360.fw.h"
//#include "firmware/bhi360/BHI360_BMM350C.fw.h"
//#include "firmware/bhi360/new_version/Bosch_Shuttle3_BHI360_BMM350C_Poll_Meta.fw.h"
#include "firmware/bhi360/latest_11_13/Bosch_Shuttle3_BHI360_BMM350C_Poll_Meta_12.fw.h"
#include "tusb.h"
#include "glove/common.h"
#include "bhi360.h"
#include "comms/bowie.pb.h"
#include "glove.h"


uint8_t acc_accuracy, gyro_accuracy, quaternion_accuracy;
uint8_t acc_cal_flag = 0;
uint8_t gyro_cal_flag = 0;

int8_t bhi360_init(struct bhy2_dev *dev, struct mag_data_dev *device)
{
	int8_t rslt = BHY2_OK;

	rslt = bhy2_init(BHY2_I2C_INTERFACE, bowie_i2c_mem_read_it, bowie_i2c_mem_write_it, bhy2_delay_us, BHY2_RD_WR_LEN, device, dev);

	return rslt;
}

// Upload firmware to the device in parts

int8_t upload_firmware_partly(struct bhy2_dev *dev)
{
    uint32_t incr = 256; /* Max command packet size */
    uint32_t len = sizeof(bhy2_firmware_image);
    int8_t rslt = BHY2_OK;

    if ((incr % 4) != 0) /* Round off to higher 4 bytes */
    {
        incr = ((incr >> 2) + 1) << 2;
    }

    for (uint32_t i = 0; (i < len) && (rslt == BHY2_OK); i += incr)
    {
        if (incr > (len - i)) /* If last payload */
        {
            incr = len - i;
            if ((incr % 4) != 0) /* Round off to higher 4 bytes */
            {
                incr = ((incr >> 2) + 1) << 2;
            }
        }

        rslt = bhy2_upload_firmware_to_ram_partly(&bhy2_firmware_image[i], len, i, incr, dev);

//        printf("%.2f%% complete\r", (float)(i + incr) / (float)len * 100.0f);
    }

//    printf("\n");

    return rslt;
}

// Upload firmware to the device and boot from RAM

int8_t bhi360_upload_firmware(uint8_t boot_stat, struct bhy2_dev *dev)
{
    uint8_t sensor_error;
    int8_t temp_rslt;
    int8_t rslt = BHY2_OK;


//    results[8] = upload_firmware_partly(glove_devices[idx].dev);
//    results[8] = bhy2_boot_from_ram(glove_devices[idx].dev);
//    printf("Loading firmware into RAM.\r\n");

    rslt = upload_firmware_partly(dev);

//    rslt = bhy2_upload_firmware_to_ram(bhy2_firmware_image, sizeof(bhy2_firmware_image), dev);

//    temp_rslt = bhy2_get_error_value(&sensor_error, dev);
//    if (sensor_error)
//    {
//        printf("%s\r\n", get_sensor_error_text(sensor_error));
//    }

    if(rslt != BHY2_OK)
    {
    	return rslt;
    }

//    print_api_error(rslt, dev);
//    print_api_error(temp_rslt, dev);


//    printf("Booting from RAM.\r\n");
    rslt = bhy2_boot_from_ram(dev);
    printf("boot_from_ram: %d\r\n", rslt);

//    temp_rslt = bhy2_get_error_value(&sensor_error, dev);
//    if (sensor_error)
//    {
//        printf("%s\r\n", get_sensor_error_text(sensor_error));
//    }

//    print_api_error(rslt, dev);
//    print_api_error(temp_rslt, dev);
    return rslt;
}

// Parse quaternion data from the FIFO

void parse_quaternion(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
	struct mag_data_dev* dev = callback_ref;
    struct bhy2_data_quaternion data;
    uint32_t s, ns;

    if (callback_info->data_size != 11) /* Check for a valid payload size. Includes sensor ID */
    {
        return;
    }

    bhy2_parse_quaternion(callback_info->data_ptr, &data);

    uint64_t timestamp = *callback_info->time_stamp; /* Store the last timestamp */

    // Datasheet bhi360 ref pg 110
    timestamp = timestamp * 15625; /* Timestamp is now in nanoseconds */
    s = (uint32_t)(timestamp / UINT64_C(1000000000));
    ns = (uint32_t)(timestamp - (s * UINT64_C(1000000000)));

    //encode and send to usb
    glove_send_quat_data(dev->count, s, ns, dev->finger, dev->link, dev->sensor_id, data.x, data.y, data.z, data.w, data.accuracy);

//    glove_send_quat_data(dev->count, s, ns, dev->finger-1, dev->link, dev->sensor_id+4, data.x, data.y, data.z, data.w, data.accuracy);
//    glove_send_quat_data(dev->count, s, ns, dev->finger-2, dev->link, dev->sensor_id+4, data.x, data.y, data.z, data.w, data.accuracy);
//    glove_send_quat_data(dev->count, s, ns, dev->finger-3, dev->link, dev->sensor_id+4, data.x, data.y, data.z, data.w, data.accuracy);
//    glove_send_quat_data(dev->count, s, ns, dev->finger-4, dev->link, dev->sensor_id+4, data.x, data.y, data.z, data.w, data.accuracy);
    dev->count = dev->count + 1;
}

// Parse magnetometer data from the FIFO

void parse_magnetometer(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
	struct mag_data_dev* dev = callback_ref;
	struct bhy2_data_xyz data;
	uint32_t s, ns;

	if (callback_info->data_size != 7) /* Check for a valid payload size. Includes sensor ID */
	{
		return;
	}

	bhy2_parse_xyz(callback_info->data_ptr, &data);

	uint64_t timestamp = *callback_info->time_stamp; /* Store the last timestamp */

	timestamp = timestamp * 15625; /* Timestamp is now in nanoseconds */
	s = (uint32_t)(timestamp / UINT64_C(1000000000));
	ns = (uint32_t)(timestamp - (s * UINT64_C(1000000000)));

	//encode and send to usb
	glove_send_mag_data(dev->count, s, ns, dev->finger, dev->link, dev->sensor_id, data.x, data.y, data.z);
//	glove_send_mag_data(dev->count, s, ns, dev->finger-1, dev->link, dev->sensor_id+4, data.x, data.y, data.z);
//	glove_send_mag_data(dev->count, s, ns, dev->finger-2, dev->link, dev->sensor_id+4, data.x, data.y, data.z);
//	glove_send_mag_data(dev->count, s, ns, dev->finger-3, dev->link, dev->sensor_id+4, data.x, data.y, data.z);
//	glove_send_mag_data(dev->count, s, ns, dev->finger-4, dev->link, dev->sensor_id+4, data.x, data.y, data.z);

}

void parse_magnetometer_meta(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
	struct mag_data_dev* dev;
	dev = callback_ref;

	struct bhy2_data_xyz data;
	uint32_t s, ns;
	if (callback_info->data_size != 7) /* Check for a valid payload size. Includes sensor ID */
	{
		return;
	}

	bhy2_parse_xyz(callback_info->data_ptr, &data);

	uint64_t timestamp = *callback_info->time_stamp; /* Store the last timestamp */

	timestamp = timestamp * 15625; /* Timestamp is now in nanoseconds */
	s = (uint32_t)(timestamp / UINT64_C(1000000000));
	ns = (uint32_t)(timestamp - (s * UINT64_C(1000000000)));

	//encode and send to usb
//	printf("meta parse event\r\n");
	glove_send_mag_data(dev->count, s, ns, dev->finger, dev->link, dev->sensor_id+20, data.x, data.y, data.z);

}

//Meta Events of sensor and fifo buffer




void parse_acc_gyro_mag_3axis_s16(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
	struct bhy2_data_xyz data;
	int i;

	uint8_t bhi360_calib_prof[68] = {0};
	struct mag_data_dev* dev;
	dev = callback_ref;
	uint32_t bhi360_prof_len;

	uint32_t actual_len;
	uint8_t *accuracy = (uint8_t*)callback_ref;
	int8_t rslt;
	float sample_rate = 0.0; /* Read out data measured at 10Hz */
	uint32_t report_latency_ms = 0; /* Report immediately */
	uint32_t tick;
	tick = HAL_GetTick();

	if (accuracy)
	{
		bhy2_parse_xyz(callback_info->data_ptr, &data);

		if(callback_info->sensor_id == BHY2_SENSOR_ID_ACC)
		{
			if(*accuracy == 3)
			{

				if(acc_cal_flag == 0)
				{
					memset(bhi360_calib_prof, 0, sizeof(bhi360_calib_prof));
					actual_len = 0;
					bhy2_get_calibration_profile(BHY2_PHYS_SENSOR_ID_ACCELEROMETER, bhi360_calib_prof, bhi360_prof_len, &actual_len, dev);
					printf("BHY2_PHYS_SENSOR_ID_ACCELEROMETER profile----------------------------bhi360_prof_len=%d, actual_len=%d---------------------\r\n", bhi360_prof_len, actual_len);
					for(i=0; i < actual_len; i++)
					{
						printf("0x%02x,", bhi360_calib_prof[i]);
					}
					printf("\r\n");
					acc_cal_flag = 1;
				}
			}
		}
		else if(callback_info->sensor_id == BHY2_SENSOR_ID_GYRO)
		{
			if(*accuracy == 3)
			{
				if(gyro_cal_flag == 0)
				{
					memset(bhi360_calib_prof, 0, sizeof(bhi360_calib_prof));
					actual_len = 0;
					bhy2_get_calibration_profile(BHY2_PHYS_SENSOR_ID_GYROSCOPE, bhi360_calib_prof, bhi360_prof_len, &actual_len, dev);
					printf("BHY2_PHYS_SENSOR_ID_GYROSCOPE profile----------------------------bhi360_prof_len=%d, actual_len=%d---------------------\r\n", bhi360_prof_len, actual_len);
					for(i=0; i < actual_len; i++)
					{
						printf("0x%02x,", bhi360_calib_prof[i]);
					}
					printf("\r\n");
					gyro_cal_flag = 1;
				}
			}
		}
	}
	else
	{
		printf("Null reference\r\r\n");
	}
}








//void parse_meta_event(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
//{
//	(void)callback_ref;
//	uint8_t meta_event_type = callback_info->data_ptr[0];
//	uint8_t byte1 = callback_info->data_ptr[1];
//	uint8_t byte2 = callback_info->data_ptr[2];
//	uint8_t *accuracy = (uint8_t*)callback_ref;
//	char *event_text;
//
//	if (callback_info->sensor_id == BHY2_SYS_ID_META_EVENT)
//	{
//		event_text = "[META EVENT]";
//	}
//	else if (callback_info->sensor_id == BHY2_SYS_ID_META_EVENT_WU)
//	{
//		event_text = "[META EVENT WAKE UP]";
//	}
//	else
//	{
//		return;
//	}
//
//	switch (meta_event_type)
//	{
//		case BHY2_META_EVENT_FLUSH_COMPLETE:
//			printf("%s Flush complete for sensor id %u\r\n", event_text, byte1);
//			break;
//		case BHY2_META_EVENT_SAMPLE_RATE_CHANGED:
//			printf("%s Sample rate changed for sensor id %u\r\n", event_text, byte1);
//			break;
//		case BHY2_META_EVENT_POWER_MODE_CHANGED:
//			printf("%s Power mode changed for sensor id %u\r\n", event_text, byte1);
//			break;
//		case BHY2_META_EVENT_ALGORITHM_EVENTS:
//			printf("%s Algorithm event\r\n", event_text);
//			break;
//		case BHY2_META_EVENT_SENSOR_STATUS:
//			printf("%s Accuracy for sensor id %u changed to %u\r\n", event_text, byte1, byte2);
//			if (accuracy)
//			{
//				if(byte1 == BHY2_SENSOR_ID_ACC)
//				{
////					printf("accel_acc!");
//					acc_accuracy = byte2;
//				}
//				else if(byte1 == BHY2_SENSOR_ID_GYRO)
//				{
////					printf("gyro_acc!");
//					gyro_accuracy = byte2;
//				}
//				//else if((byte1 == BHY2_SENSOR_ID_GAMERV) || (byte1 == BHY2_SENSOR_ID_RV_WU) || (byte1 == BHY2_SENSOR_ID_RV))
//				else if(byte1 == BHY2_SENSOR_ID_GAMERV)
//				{
////					printf("quat_acc!");
//					quaternion_accuracy = byte2;
//				}
//				else
//				{
////					printf("don't,know");
//					*accuracy = byte2;
//				}
//			}
////			if(acc_accuracy == 3 && gyro_accuracy == 3)
////				accuracy_flag = true;
//			break;
//		case BHY2_META_EVENT_BSX_DO_STEPS_MAIN:
//			printf("%s BSX event (do steps main)\r\n", event_text);
//			break;
//		case BHY2_META_EVENT_BSX_DO_STEPS_CALIB:
//			printf("%s BSX event (do steps calib)\r\n", event_text);
//			break;
//		case BHY2_META_EVENT_BSX_GET_OUTPUT_SIGNAL:
//			printf("%s BSX event (get output signal)\r\n", event_text);
//			break;
//		case BHY2_META_EVENT_SENSOR_ERROR:
//			printf("%s Sensor id %u reported error 0x%02X\r\n", event_text, byte1, byte2);
//			break;
//		case BHY2_META_EVENT_FIFO_OVERFLOW:
//			printf("%s FIFO overflow\r\n", event_text);
//			break;
//		case BHY2_META_EVENT_DYNAMIC_RANGE_CHANGED:
//			printf("%s Dynamic range changed for sensor id %u\r\n", event_text, byte1);
//			break;
//		case BHY2_META_EVENT_FIFO_WATERMARK:
//			printf("%s FIFO watermark reached\r\n", event_text);
//			break;
//		case BHY2_META_EVENT_INITIALIZED:
//			printf("%s Firmware initialized. Firmware version %u\r\n", event_text, ((uint16_t)byte2 << 8) | byte1);
//			//disable_all_sensor();
//			break;
//		case BHY2_META_TRANSFER_CAUSE:
//			printf("%s Transfer cause for sensor id %u\r\n", event_text, byte1);
//			break;
//		case BHY2_META_EVENT_SENSOR_FRAMEWORK:
//			printf("%s Sensor framework event for sensor id %u\r\n", event_text, byte1);
//			break;
//		case BHY2_META_EVENT_RESET:
//			printf("%s Reset event\r\n", event_text);
//			break;
//		case BHY2_META_EVENT_SPACER:
//			break;
//		default:
//			printf("%s Unknown meta event with id: %u\r\n", event_text, meta_event_type);
//			break;
//	}
//}





void parse_meta_event(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    (void)callback_ref;
    uint8_t meta_event_type = callback_info->data_ptr[0];
    uint8_t byte1 = callback_info->data_ptr[1];
    uint8_t byte2 = callback_info->data_ptr[2];
    char *event_text;

    if (callback_info->sensor_id == BHY2_SYS_ID_META_EVENT)
    {
        event_text = "[META EVENT]";
    }
    else if (callback_info->sensor_id == BHY2_SYS_ID_META_EVENT_WU)
    {
        event_text = "[META EVENT WAKE UP]";
    }
    else
    {
        return;
    }

    switch (meta_event_type)
    {
        case BHY2_META_EVENT_FLUSH_COMPLETE:
            printf("%s Flush complete for sensor id %u\r\n", event_text, byte1);
            break;
        case BHY2_META_EVENT_SAMPLE_RATE_CHANGED:
            printf("%s Sample rate changed for sensor id %u\r\n", event_text, byte1);
            break;
        case BHY2_META_EVENT_POWER_MODE_CHANGED:
            printf("%s Power mode changed for sensor id %u\r\n", event_text, byte1);
            break;
        case BHY2_META_EVENT_ALGORITHM_EVENTS:
            printf("%s Algorithm event\r\n", event_text);
            break;
        case BHY2_META_EVENT_SENSOR_STATUS:
            printf("%s Accuracy for sensor id %u changed to %u\r\n", event_text, byte1, byte2);
            break;
        case BHY2_META_EVENT_BSX_DO_STEPS_MAIN:
            printf("%s BSX event (do steps main)\r\n", event_text);
            break;
        case BHY2_META_EVENT_BSX_DO_STEPS_CALIB:
            printf("%s BSX event (do steps calib)\r\n", event_text);
            break;
        case BHY2_META_EVENT_BSX_GET_OUTPUT_SIGNAL:
            printf("%s BSX event (get output signal)\r\n", event_text);
            break;
        case BHY2_META_EVENT_SENSOR_ERROR:
            printf("%s Sensor id %u reported error 0x%02X\r\n", event_text, byte1, byte2);
            break;
        case BHY2_META_EVENT_FIFO_OVERFLOW:
            printf("%s FIFO overflow\r\n", event_text);
            break;
        case BHY2_META_EVENT_DYNAMIC_RANGE_CHANGED:
            printf("%s Dynamic range changed for sensor id %u\r\n", event_text, byte1);
            break;
        case BHY2_META_EVENT_FIFO_WATERMARK:
            printf("%s FIFO watermark reached\r\n", event_text);
            break;
        case BHY2_META_EVENT_INITIALIZED:
            printf("%s Firmware initialized. Firmware version %u\r\n", event_text, ((uint16_t)byte2 << 8) | byte1);
            break;
        case BHY2_META_TRANSFER_CAUSE:
            printf("%s Transfer cause for sensor id %u\r\n", event_text, byte1);
            break;
        case BHY2_META_EVENT_SENSOR_FRAMEWORK:
            printf("%s Sensor framework event for sensor id %u\r\n", event_text, byte1);
            break;
        case BHY2_META_EVENT_RESET:
            printf("%s Reset event\r\n", event_text);
            break;
        case BHY2_META_EVENT_SPACER:
            break;
        default:
            printf("%s Unknown meta event with id: %u\r\n", event_text, meta_event_type);
            break;
    }
}
