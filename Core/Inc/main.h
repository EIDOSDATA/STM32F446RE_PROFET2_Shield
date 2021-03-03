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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define BTN2_Pin GPIO_PIN_0
#define BTN2_GPIO_Port GPIOA
#define A1_Vbb_Pin GPIO_PIN_1
#define A1_Vbb_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define A2_Isense_1_2_Pin GPIO_PIN_4
#define A2_Isense_1_2_GPIO_Port GPIOA
#define LED4_Pin GPIO_PIN_5
#define LED4_GPIO_Port GPIOA
#define LED3_Pin GPIO_PIN_6
#define LED3_GPIO_Port GPIOA
#define IN3_Pin GPIO_PIN_7
#define IN3_GPIO_Port GPIOA
#define A3_Isense_3_4_Pin GPIO_PIN_0
#define A3_Isense_3_4_GPIO_Port GPIOB
#define DEN1_DEN3_Pin GPIO_PIN_10
#define DEN1_DEN3_GPIO_Port GPIOB
#define IN1_Pin GPIO_PIN_7
#define IN1_GPIO_Port GPIOC
#define OLOFF_Pin GPIO_PIN_8
#define OLOFF_GPIO_Port GPIOA
#define DEN2_DEN4_Pin GPIO_PIN_9
#define DEN2_DEN4_GPIO_Port GPIOA
#define BTN1_Pin GPIO_PIN_10
#define BTN1_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define IN4_Pin GPIO_PIN_3
#define IN4_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_4
#define LED2_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_5
#define LED1_GPIO_Port GPIOB
#define IN2_Pin GPIO_PIN_6
#define IN2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
