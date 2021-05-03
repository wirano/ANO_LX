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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USER_LED_R_Pin GPIO_PIN_2
#define USER_LED_R_GPIO_Port GPIOE
#define USER_LED_G_Pin GPIO_PIN_3
#define USER_LED_G_GPIO_Port GPIOE
#define key0_Pin GPIO_PIN_4
#define key0_GPIO_Port GPIOE
#define key1_Pin GPIO_PIN_13
#define key1_GPIO_Port GPIOC
#define key2_Pin GPIO_PIN_14
#define key2_GPIO_Port GPIOC
#define key3_Pin GPIO_PIN_15
#define key3_GPIO_Port GPIOC
#define BEEP_Pin GPIO_PIN_0
#define BEEP_GPIO_Port GPIOC
#define SPI2_CS1_Pin GPIO_PIN_1
#define SPI2_CS1_GPIO_Port GPIOC
#define IO1_Pin GPIO_PIN_4
#define IO1_GPIO_Port GPIOC
#define Power_Input_Pin GPIO_PIN_5
#define Power_Input_GPIO_Port GPIOC
#define IO2_Pin GPIO_PIN_2
#define IO2_GPIO_Port GPIOB
#define IO3_Pin GPIO_PIN_7
#define IO3_GPIO_Port GPIOE
#define IO4_Pin GPIO_PIN_8
#define IO4_GPIO_Port GPIOE
#define ANO_RGB_B_Pin GPIO_PIN_10
#define ANO_RGB_B_GPIO_Port GPIOE
#define ANO_RGB_R_Pin GPIO_PIN_12
#define ANO_RGB_R_GPIO_Port GPIOE
#define ANO_RGB_G_Pin GPIO_PIN_15
#define ANO_RGB_G_GPIO_Port GPIOE
#define IO5_Pin GPIO_PIN_12
#define IO5_GPIO_Port GPIOB
#define IO6_Pin GPIO_PIN_14
#define IO6_GPIO_Port GPIOB
#define IO7_Pin GPIO_PIN_15
#define IO7_GPIO_Port GPIOB
#define IO8_Pin GPIO_PIN_10
#define IO8_GPIO_Port GPIOD
#define IO9_Pin GPIO_PIN_11
#define IO9_GPIO_Port GPIOD
#define IO10_Pin GPIO_PIN_8
#define IO10_GPIO_Port GPIOA
#define IO11_Pin GPIO_PIN_11
#define IO11_GPIO_Port GPIOA
#define IO12_Pin GPIO_PIN_12
#define IO12_GPIO_Port GPIOA
#define IO13_Pin GPIO_PIN_15
#define IO13_GPIO_Port GPIOA
#define IO14_Pin GPIO_PIN_0
#define IO14_GPIO_Port GPIOD
#define IO15_Pin GPIO_PIN_1
#define IO15_GPIO_Port GPIOD
#define IO16_Pin GPIO_PIN_3
#define IO16_GPIO_Port GPIOD
#define IO17_Pin GPIO_PIN_4
#define IO17_GPIO_Port GPIOD
#define IO18_Pin GPIO_PIN_7
#define IO18_GPIO_Port GPIOD
#define IO19_Pin GPIO_PIN_3
#define IO19_GPIO_Port GPIOB
#define ANO_LED_OB_Pin GPIO_PIN_0
#define ANO_LED_OB_GPIO_Port GPIOE
#define USER_LED_B_Pin GPIO_PIN_1
#define USER_LED_B_GPIO_Port GPIOE
void   MX_GPIO_Init(void);
void   MX_DMA_Init(void);
void   MX_SPI1_Init(void);
void   MX_SPI2_Init(void);
void   MX_TIM5_Init(void);
void   MX_UART4_Init(void);
void   MX_UART5_Init(void);
void   MX_USART1_UART_Init(void);
void   MX_USART2_UART_Init(void);
void   MX_USART3_UART_Init(void);
void   MX_USART6_UART_Init(void);
void   MX_ADC1_Init(void);
void   MX_I2C1_Init(void);
void   MX_I2C2_Init(void);
void   MX_TIM1_Init(void);
void   MX_TIM3_Init(void);
void   MX_TIM4_Init(void);
void   MX_TIM8_Init(void);
void   MX_TIM9_Init(void);
void   MX_TIM10_Init(void);
void   MX_TIM11_Init(void);
void   MX_TIM7_Init(void);
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
