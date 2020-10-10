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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#define DW_NSS_Pin GPIO_PIN_0
#define DW_NSS_GPIO_Port GPIOB
#define DW_RSTn_Pin GPIO_PIN_1
#define DW_RSTn_GPIO_Port GPIOB
#define DW_IRQn_Pin GPIO_PIN_10
#define DW_IRQn_GPIO_Port GPIOB
#define DW_IRQn_EXTI_IRQn EXTI15_10_IRQn
#define RX_LED_Pin GPIO_PIN_11
#define RX_LED_GPIO_Port GPIOB
#define RANGING_LED_Pin GPIO_PIN_14
#define RANGING_LED_GPIO_Port GPIOB
#define TX_LED_Pin GPIO_PIN_15
#define TX_LED_GPIO_Port GPIOB
#define USB_PULLUP_Pin GPIO_PIN_6
#define USB_PULLUP_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define DW_IRQn DW_IRQn_EXTI_IRQn
#define DW_IRQn_Type DW_IRQn
#define DW_RESET_Pin DW_RSTn_Pin
#define DW_RESET_GPIO_Port DW_RSTn_GPIO_Port

#define DEBUG_BUF_SIZE 256
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
