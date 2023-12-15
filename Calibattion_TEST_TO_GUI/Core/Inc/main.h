/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "UartRingbuffer_multi.h"
#include "function.h"
#include "variable.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "FLASH_SECTOR_F4.h"
#include "math.h"
#include <ctype.h>
#include "ADXL.h"
  /* Private includes ----------------------------------------------------------*/
  /* USER CODE BEGIN Includes */

  /* USER CODE END Includes */
  extern UART_HandleTypeDef huart1;
  extern UART_HandleTypeDef huart3;
  extern TIM_HandleTypeDef htim10;
  extern TIM_HandleTypeDef htim2;
  extern TIM_HandleTypeDef htim1;
  extern ADC_HandleTypeDef hadc1;
  extern SPI_HandleTypeDef hspi2;
  extern I2C_HandleTypeDef hi2c1;
/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#define Timer_PRESCALER_VALUE (uint32_t)(((SystemCoreClock) / 45000000) - 1)
#define Timer_PERIOD_VALUE (uint32_t)(10500 - 1) /* Period Value  */
/* USER CODE END ET */
#define device_uart &huart1
#define pc_uart &huart3
/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
// 記憶體儲存地???????
#define FLASH_USER_START_ADDR ADDR_FLASH_SECTOR_2 /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR ADDR_FLASH_SECTOR_7   //+  GetSectorSize(ADDR_FLASH_SECTOR_7) -1 /* End @ of user Flash area : sector start address + sector size -1 */

#define data_size_adc (4)

  /* USER CODE END EC */

  /* Exported macro ------------------------------------------------------------*/
  /* USER CODE BEGIN EM */

  /* USER CODE END EM */

  void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

  /* Exported functions prototypes ---------------------------------------------*/
  void Error_Handler(void);
/*傳輸buffer 大小配置*/
#define Uart_Buffer (200)
/* USER CODE BEGIN EFP */
/*Boolean define*/
#define True (1)
#define False (0)
/*Strncmp Boolean*/
#define true (0)
#define false (1)

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define Volt_12V_Pin GPIO_PIN_0
#define Volt_12V_GPIO_Port GPIOA
#define Volt_5V_Pin GPIO_PIN_1
#define Volt_5V_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define CS_PIN_Pin GPIO_PIN_8
#define CS_PIN_GPIO_Port GPIOC
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */
/*PWM Freq & DUTY*/
#define PRESCALER_VALUE (uint32_t)(((SystemCoreClock) / 45000000) - 1)
#define PERIOD_VALUE (uint32_t)(1000 - 1)          /* Period Value  */
#define PULSE1_VALUE (uint32_t)(1000 / 2)          /* Capture Compare 1 Value  */
#define PULSE2_VALUE (uint32_t)(1000 * 37.5 / 100) /* Capture Compare 2 Value  */
#define PULSE3_VALUE (uint32_t)(1000 / 4)          /* Capture Compare 3 Value  */
#define PULSE4_VALUE (uint32_t)(1000 * 12.5 / 100) /* Capture Compare 4 Value  */
/*PWM DUTY*/
#define MAX_DUTY (0x03E8)
#define MAX_DUTY_percentage (0x0064)
#define PWM_offset (0x032)
#define Freq_Gain (1000)
/*SineWave OffSET for negative side*/
#define SINE_PWM_OFFSET (1000)

/*SineWave*/
#define PI 3.1415926
#define Sine_Resltion (100)
#define Tri_Resltion (100)
#define Table_Size (100)
/*DAC 12bit limit 標腰化*/
#define DAC_Resltion (0xFFF)

/*Search char*/
#define Target ("W")
#define Target_Multi ("F")
#define Wave_len (4)
#define TriWave (1)
#define SineWave (2)
#define Freq_value_len (6)
#define M_PI		(3.14159265358979323846)
  /*條件編譯DEBUG區*/

  // #define DEBUG_MODE_FLASH 1
// #define DEBUG_MODE_UART 1
  // #define DEBUG_MODE_UART_ADC_Message 1
  //  #define ISR_DISPLAY 1
  // #define TestDac 1
  // #define Debug_Searcg_Element_target 1

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
