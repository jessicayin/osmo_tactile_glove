/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "main.h"
#include "i2c.h"
#include "sdio.h"
#include "tim.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "stdio.h"
#include "stm32f4xx_hal_i2c.h"
#include "tusb.h"

#include "glove/glove.h"
//#include "glove/glove.c"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//#Physical Drivers
//#DriverID,Bus,Addr,GPIO,Cal0,Cal1,Cal2,Cal3,Cal4,Cal5,Cal6,Cal7,Cal8,Off0,Off1,Off2,maxRate,Range
//16,i2c0,20,20, 1, 0, 0, 0, -1, 0, 0, 0, -1, 0, 0, 0, 400.000000, 0   #BMM350Mag on M2
//18,i2c0,21,18, 1, 0, 0, 0, -1, 0, 0, 0, -1, 0, 0, 0, 400.000000, 0   #BMM350Mag on M2 for Meta
//26,spi0,25,2, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 800.000000, 0   #BHI360Accel on SPI0
//25,spi0,25,-, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 800.000000, 0   #BHI360Gyro on SPI0
//27,spi0,25,3, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1.000000, 0   #BHI360HW Activity on INT_GPIO3
//46,spi0,25,-, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1.000000, 0   #BHI360HW Step detector
//47,spi0,25,-, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1.000000, 0   #BHI360HW Step counter
//48,spi0,25,-, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1.000000, 0   #BHI360HW Wrist Gesture
//43,spi0,25,-, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1.000000, 0   #BHI360HW Anymotion
//44,spi0,25,-, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1.000000, 0   #BHI360HW No motion
//49,spi0,25,-, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1.000000, 0   #BHI360HW Wrist Wakeup
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USB_OTG_HS_PCD_Init();
//  MX_SDIO_SD_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_Base_Start_IT(&htim8);
//
  tud_init(BOARD_DEVICE_RHPORT_NUM);
  HAL_Delay(100);

  uint8_t result;

//   while(1)
//   {
//	  printf("----------------\r\n");
//	  scan_mux_channel(0);
//	  scan_mux_channel(2);
//	  scan_mux_channel(3);
//	  scan_mux_channel(5);
//	  scan_mux_channel(7);
//	  printf("----- init -----\r\n");
//	  init_glove();
//	  HAL_Delay(5000);
//	  printf("----init done---\r\n");
//   }


    HAL_Delay(1000);
  	init_glove();


//	uint8_t work_buffer[WORK_BUFFER_SIZE];
//   result = bhy2_get_virt_sensor_list(glove_devices[0].dev);
//   result = bhy2_get_virt_sensor_list(glove_devices[1].dev);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//    uint8_t rslt = BHY2_OK;
//    uint8_t idx = 0;
//    uint8_t work_buffer[WORK_BUFFER_SIZE];
//    uint8_t work_buffer2[WORK_BUFFER_SIZE];

  	flush_glove();

//  	uint16_t range = 5000;
//  	result = bhy2_set_virt_sensor_range(MAG_ID, range, glove_devices[0].dev);

//  	struct bhy2_phys_sensor_info info;
//  	uint8_t phys_sensor_id = 0x5;
//  	result = bhy2_get_phys_sensor_info(phys_sensor_id, &info, glove_devices[0].dev);

    while(1){

		glove_update_data();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    }
//    HAL_GPIO_WritePin(GPIOA, BLUE_LED_STOP_Pin, GPIO_PIN_SET);

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SDIO|RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.PLLSAI.PLLSAIM = 4;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 96;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV4;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48CLKSOURCE_PLLSAIP;
  PeriphClkInitStruct.SdioClockSelection = RCC_SDIOCLKSOURCE_CLK48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
int __io_putchar(int ch) {
	uint8_t c[1];
	c[0] = ch & 0x00FF;
	tud_cdc_write(&*c, 1);
	return ch;
}

int _write(int file, char *ptr, int len) {
	int idx;
	for (idx = 0; idx < len; idx++) {
		__io_putchar(*ptr++);
	}
	return len;
}

// Callback: timer has rolled over
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // Check which version of the timer triggered this callback and toggle LED
  //  HAL_GPIO_TogglePin(GPIOA, RED_LED_STATUS_Pin);
  if (htim->Instance == TIM8)
  {
	HAL_GPIO_TogglePin(GPIOA, YELLOW_LED_START_Pin);
	tud_task(); //send data over usb
	tud_cdc_write_flush();
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
