/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#define ES_Pin GPIO_PIN_2
#define ES_GPIO_Port GPIOE
#define UPB_LED_Pin GPIO_PIN_3
#define UPB_LED_GPIO_Port GPIOE
#define BREAK_EN_Pin GPIO_PIN_5
#define BREAK_EN_GPIO_Port GPIOE
#define HOIST_DOWN_Pin GPIO_PIN_3
#define HOIST_DOWN_GPIO_Port GPIOC
#define LDX_RR_Pin GPIO_PIN_0
#define LDX_RR_GPIO_Port GPIOA
#define RUN_Pin GPIO_PIN_2
#define RUN_GPIO_Port GPIOB
#define UL9_Pin GPIO_PIN_7
#define UL9_GPIO_Port GPIOE
#define DL9_Pin GPIO_PIN_8
#define DL9_GPIO_Port GPIOE
#define LDX_RF_Pin GPIO_PIN_9
#define LDX_RF_GPIO_Port GPIOE
#define UL10_Pin GPIO_PIN_10
#define UL10_GPIO_Port GPIOE
#define DL10_Pin GPIO_PIN_11
#define DL10_GPIO_Port GPIOE
#define UL11_Pin GPIO_PIN_12
#define UL11_GPIO_Port GPIOE
#define DL11_Pin GPIO_PIN_13
#define DL11_GPIO_Port GPIOE
#define UL12_Pin GPIO_PIN_14
#define UL12_GPIO_Port GPIOE
#define DL12_Pin GPIO_PIN_15
#define DL12_GPIO_Port GPIOE
#define LDX_LR_Pin GPIO_PIN_10
#define LDX_LR_GPIO_Port GPIOB
#define MOTORPOEWER_Pin GPIO_PIN_14
#define MOTORPOEWER_GPIO_Port GPIOB
#define DIR3_Pin GPIO_PIN_15
#define DIR3_GPIO_Port GPIOB
#define HC597_STB_Pin GPIO_PIN_10
#define HC597_STB_GPIO_Port GPIOD
#define HC597_LOAD_Pin GPIO_PIN_11
#define HC597_LOAD_GPIO_Port GPIOD
#define HC597_CS_Pin GPIO_PIN_10
#define HC597_CS_GPIO_Port GPIOA
#define DIR4_Pin GPIO_PIN_3
#define DIR4_GPIO_Port GPIOD
#define HC595_EN_Pin GPIO_PIN_7
#define HC595_EN_GPIO_Port GPIOD
#define LDX_LF_Pin GPIO_PIN_8
#define LDX_LF_GPIO_Port GPIOB
#define HC595_RK_Pin GPIO_PIN_0
#define HC595_RK_GPIO_Port GPIOE
#define UPF_LED_Pin GPIO_PIN_1
#define UPF_LED_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
