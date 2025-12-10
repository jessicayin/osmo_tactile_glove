/* Includes ------------------------------------------------------------------*/
#include "i2c.h"
#define __STDC_FORMAT_MACROS
#include <inttypes.h>

#include "bhy2_defs.h"
#include "bhy2.h"
#include "bhy2_parse.h"

#include "firmware/bhi360/latest_11_13/Bosch_Shuttle3_BHI360_BMM350C_Poll_Meta_12.fw.h"

#include "tusb.h"
#include "glove/common.h"
#include "bhi360.h"
#include "comms/bowie.pb.h"
#include "glove.h"

int8_t bhi360_init(struct bhy2_dev *dev, struct mag_data_dev *device){
	int8_t rslt = BHY2_OK;
	rslt = bhy2_init(BHY2_I2C_INTERFACE, bhy2_i2c_read, bhy2_i2c_write, bhy2_delay_us, BHY2_RD_WR_LEN, device, dev);

	return rslt;
}


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

int8_t bhi360_upload_firmware(uint8_t boot_stat, struct bhy2_dev *dev)
{
    uint8_t sensor_error;
    int8_t temp_rslt;
    int8_t rslt = BHY2_OK;


    //		results[8] = upload_firmware_partly(glove_devices[idx].dev);
    //		results[8] = bhy2_boot_from_ram(glove_devices[idx].dev);


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

void parse_quaternion(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
	struct mag_data_dev* dev;
	dev = callback_ref;

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

    glove_send_quat_data(dev->count, s, ns, dev->finger, dev->link, dev->sensor_id, data.x, data.y, data.z, data.w, data.accuracy);
    dev->count = dev->count + 1;

}

void parse_magnetometer(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
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
//	printf("original parse event\r\n");
	glove_send_mag_data(dev->count, s, ns, dev->finger, dev->link, dev->sensor_id, data.x, data.y, data.z);

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
            printf("%s Reset event %d, 0x%02X,  0x%01X\r\n", event_text, meta_event_type, byte1, byte2);
            ///error checking
            uint8_t error_code = 0;
            uint8_t result;
            result = bhy2_get_error_reg(BHY2_REG_INT_STATUS, &error_code, glove_devices[0].dev);
			printf("Register: 0x%02X, Error Code: %d, 0x%02X\r\n", BHY2_REG_INT_STATUS, result, error_code);

			result = bhy2_get_error_reg(BHY2_REG_ERROR_VALUE, &error_code, glove_devices[0].dev);
			printf("Register: 0x%02X, Error Code: %d, 0x%02X\r\n", BHY2_REG_ERROR_VALUE, result, error_code);

			result = bhy2_get_error_reg(BHY2_REG_ERROR_AUX, &error_code, glove_devices[0].dev);
			printf("Register: 0x%02X, Error Code: %d, 0x%02X\r\n", BHY2_REG_ERROR_AUX, result, error_code);

			result = bhy2_get_error_reg(BHY2_REG_DEBUG_VALUE, &error_code, glove_devices[0].dev);
			printf("Register: 0x%02X, Error Code: %d, 0x%02X\r\n", BHY2_REG_DEBUG_VALUE, result, error_code);

			result = bhy2_get_error_reg(BHY2_REG_DEBUG_STATE, &error_code, glove_devices[0].dev);
			printf("Register: 0x%02X, Error Code: %d, 0x%02X\r\n", BHY2_REG_DEBUG_STATE, result, error_code);

			/**
			 * @brief Function to get the post mortem data
			 * @param[out] post_mortem  : Reference to the data buffer to store the post mortem data
			 * @param[in] buffer_len    : Length of the data buffer
			 * @param[out] actual_len   : Actual length of the post mortem data
			 * @param[in] dev           : Device reference
			 * @return API error codes
			 */
//			int8_t bhy2_get_post_mortem_data(uint8_t *post_mortem, uint32_t buffer_len, uint32_t *actual_len, struct bhy2_dev *dev);

			struct bhy2_post_mortem post_mortem_data = { 0 };
			uint32_t pmlen = 0;
			printf("size of postmortem struct: %d", sizeof(struct bhy2_post_mortem));
			result = get_post_mortem_data(&post_mortem_data, glove_devices[0].dev);
//			result = bhy2_get_post_mortem_data(&post_mortem_data, sizeof(struct bhy2_post_mortem), &pmlen, glove_devices[0].dev);
			printf("Post mortem data: %d", result);
            break;
        case BHY2_META_EVENT_SPACER:
            break;
        default:
            printf("%s Unknown meta event with id: %u\r\n", event_text, meta_event_type);
            break;
    }
}

/**
* @brief Function to get the Post Mortem data
*/
int8_t get_post_mortem_data(struct bhy2_post_mortem *pminfo, struct bhy2_dev *bhy2)
{
    uint32_t pmlen = 0;
    int8_t rslt;

    rslt = bhy2_get_post_mortem_data((uint8_t*)pminfo, sizeof(struct bhy2_post_mortem), &pmlen, bhy2);

    return rslt;
}
