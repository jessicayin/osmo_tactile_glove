/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "queue.h"
#include "event_groups.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "i2c.h"
#include "tim.h"
#include "bhi360.h"
#include "usb_otg.h"
#include "gpio.h"
#include "tusb.h"
#include "stm32f4xx_hal_i2c.h"
#include "glove/glove.h"
#include "comms/bowie.pb.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USB_PACKET_QUEUE_SIZE	64

#define BOWIE_INIT_EVENT_BIT (1 << 0)

static bool g_host_connected = false;
static uint32_t g_led_delay = 500;
//static int flag = 0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
EventGroupHandle_t xBowieInitEventBits;
/* USER CODE END Variables */
/* Definitions for USBTask */
osThreadId_t USBTaskHandle;
uint32_t USBTaskBuffer[1024];
osStaticThreadDef_t USBTaskControlBlock;
const osThreadAttr_t USBTask_attributes = { .name = "USBTask", .cb_mem =
		&USBTaskControlBlock, .cb_size = sizeof(USBTaskControlBlock),
		.stack_mem = &USBTaskBuffer[0], .stack_size = sizeof(USBTaskBuffer),
		.priority = (osPriority_t) osPriorityRealtime7, };

/* Definitions for CDCTask */
osThreadId_t CDCTaskHandle;
uint32_t CDCTaskBuffer[1024];
osStaticThreadDef_t CDCTaskControlBlock;
const osThreadAttr_t CDCTask_attributes = { .name = "CDCTask", .cb_mem =
		&CDCTaskControlBlock, .cb_size = sizeof(CDCTaskControlBlock),
		.stack_mem = &CDCTaskBuffer[0], .stack_size = sizeof(CDCTaskBuffer),
		.priority = (osPriority_t) osPriorityRealtime3, };

/* Definitions for BowieTask */
osThreadId_t BowieTaskHandle;
uint32_t BowieTaskBuffer[1024];
osStaticThreadDef_t BowieTaskControlBlock;
const osThreadAttr_t BowieTask_attributes = { .name = "BowieTask", .cb_mem =
		&BowieTaskControlBlock, .cb_size = sizeof(BowieTaskControlBlock),
		.stack_mem = &BowieTaskBuffer[0], .stack_size = sizeof(BowieTaskBuffer),
		.priority = (osPriority_t) osPriorityNormal, };

/* Definitions for DeviceToHostCommsQueue */
osMessageQueueId_t DeviceToHostCommsQueueHandle;
const osMessageQueueAttr_t DeviceToHostCommsQueue_attributes = { .name =
		"DeviceToHostCommsQueue" };

/* Private function prototypes -----------------------------------------------*/
void usb_handle_connection_change(bool state);
/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartUSBTask(void *argument);
void StartBowieTask(void *argument);
void StartCDCTask(void *argument);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */
/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void) {
	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* Create the queue(s) */
	/* creation of DeviceToHostCommsQueue */
	DeviceToHostCommsQueueHandle = osMessageQueueNew(USB_PACKET_QUEUE_SIZE,
			sizeof(bowie_Data), &DeviceToHostCommsQueue_attributes);
	/* USER CODE END RTOS_QUEUES */

	/* creation of USBTask */
	USBTaskHandle = osThreadNew(StartUSBTask, NULL, &USBTask_attributes);

	/* creation of CDCTask */
	CDCTaskHandle = osThreadNew(StartCDCTask, NULL, &CDCTask_attributes);

	/* creation of BowieTask */
	BowieTaskHandle = osThreadNew(StartBowieTask, NULL, &BowieTask_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_EVENTS */
	/* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartUSBTask */
/**
 * @brief  Function implementing the USBTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartUSBTask */
void StartUSBTask(void *argument) {
	/* USER CODE BEGIN StartUSBTask */
	tud_init(BOARD_DEVICE_RHPORT_NUM);

	for (;;) {
		tud_task();
//		tud_cdc_write_flush();
	}
	/* USER CODE END StartUSBTask */
}

/* USER CODE BEGIN Header_StartCDCTask */
/**
 * @brief Function implementing the CDCTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartCDCTask */

void StartCDCTask(void *argument) {
	/* USER CODE BEGIN StartCDCTask */
	bowie_Data msg = bowie_Data_init_zero;
	flush_buffer();
	for (;;) {
		if (tud_cdc_connected()) {
			usb_handle_connection_change(true);
			if (xQueueReceive(DeviceToHostCommsQueueHandle, &msg, 100) == pdTRUE) {
				glove_coms_write(&msg);
			}

		} else {
			usb_handle_connection_change(false);
			HAL_GPIO_TogglePin(GPIOA, BLUE_LED_STOP_Pin);
			osDelay(g_led_delay);
//			flush_buffer();
		}
	}
	/* USER CODE END StartCDCTask */
}

/* USER CODE BEGIN Header_StartBowieTask */
/**
 * @brief Function implementing the BowieTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartBowieTask */
void StartBowieTask(void *argument) {
	/* USER CODE BEGIN StartFinger2Task */

	HAL_GPIO_WritePin(GPIOA, YELLOW_LED_START_Pin, GPIO_PIN_SET);
//	glove_mux_reset();
	init_glove();
//	flush_buffer();
	HAL_GPIO_WritePin(GPIOA, YELLOW_LED_START_Pin, GPIO_PIN_RESET);
	flush_buffer();
	for (;;)
	{
		while (!g_host_connected)
		{
			osDelay(5);
		}
//		while(flag)
		glove_update_data();
	}
	/* USER CODE END StartFinger2Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void usb_handle_connection_change(bool state) {
	if (state == g_host_connected) {
		return;
	}
	g_host_connected = state;
	if (!g_host_connected) {
		xQueueReset(DeviceToHostCommsQueueHandle);
	}
}

void tud_mount_cb(void) {
	usb_handle_connection_change(true);
	g_led_delay = 250;
}

void tud_umount_cb(void) {
	usb_handle_connection_change(false);
	g_led_delay = 500;
}

void tud_suspend_cb(bool remote_wakeup_en) {
	usb_handle_connection_change(false);
}

void tud_resume_cb(void) {
	if (tud_mounted()) {
		usb_handle_connection_change(true);
		//turn on the stop led and start led
		HAL_GPIO_WritePin(GPIOA,YELLOW_LED_START_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA,BLUE_LED_STOP_Pin,GPIO_PIN_SET);
	} else {
		usb_handle_connection_change(false);
	}
}


/* USER CODE END Application */

