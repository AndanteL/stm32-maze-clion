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
#include "stm32f1xx_hal.h"

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
#define A0_Pin GPIO_PIN_0
#define A0_GPIO_Port GPIOC
#define A1_Pin GPIO_PIN_1
#define A1_GPIO_Port GPIOC
#define A2_Pin GPIO_PIN_2
#define A2_GPIO_Port GPIOC
#define L4_Pin GPIO_PIN_0
#define L4_GPIO_Port GPIOA
#define L3_Pin GPIO_PIN_1
#define L3_GPIO_Port GPIOA
#define L2_Pin GPIO_PIN_2
#define L2_GPIO_Port GPIOA
#define L1_Pin GPIO_PIN_3
#define L1_GPIO_Port GPIOA
#define IR5_Pin GPIO_PIN_4
#define IR5_GPIO_Port GPIOA
#define IR4_Pin GPIO_PIN_5
#define IR4_GPIO_Port GPIOA
#define IR3_Pin GPIO_PIN_6
#define IR3_GPIO_Port GPIOA
#define IR2_Pin GPIO_PIN_7
#define IR2_GPIO_Port GPIOA
#define IR1_Pin GPIO_PIN_4
#define IR1_GPIO_Port GPIOC
#define R4_Pin GPIO_PIN_8
#define R4_GPIO_Port GPIOA
#define R3_Pin GPIO_PIN_9
#define R3_GPIO_Port GPIOA
#define R2_Pin GPIO_PIN_10
#define R2_GPIO_Port GPIOA
#define R1_Pin GPIO_PIN_11
#define R1_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */
//相关变量初始量化
#define uchar unsigned char
#define uint unsigned int
#define SIZE 8
typedef struct {
    uchar x;
    uchar y;
}zb;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
