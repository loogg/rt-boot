/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_5_Pin GPIO_PIN_2
#define LED_5_GPIO_Port GPIOE
#define LED_4_Pin GPIO_PIN_3
#define LED_4_GPIO_Port GPIOE
#define LED_3_Pin GPIO_PIN_4
#define LED_3_GPIO_Port GPIOE
#define LED_2_Pin GPIO_PIN_5
#define LED_2_GPIO_Port GPIOE
#define LED_1_Pin GPIO_PIN_6
#define LED_1_GPIO_Port GPIOE
#define ETH_PHY_RST_Pin GPIO_PIN_2
#define ETH_PHY_RST_GPIO_Port GPIOC
#define SDA3_Pin GPIO_PIN_9
#define SDA3_GPIO_Port GPIOC
#define SCL3_Pin GPIO_PIN_8
#define SCL3_GPIO_Port GPIOA
#define W25Q64_CS_Pin GPIO_PIN_7
#define W25Q64_CS_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
