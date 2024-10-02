/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#define Start_knop_Pin GPIO_PIN_13
#define Start_knop_GPIO_Port GPIOC
#define Pauzeer_knop_Pin GPIO_PIN_14
#define Pauzeer_knop_GPIO_Port GPIOC
#define Stop_knop_Pin GPIO_PIN_15
#define Stop_knop_GPIO_Port GPIOC
#define Blauwe_LED_Pin GPIO_PIN_1
#define Blauwe_LED_GPIO_Port GPIOF
#define Groene_LED_Pin GPIO_PIN_2
#define Groene_LED_GPIO_Port GPIOF
#define Rode_LED_Pin GPIO_PIN_3
#define Rode_LED_GPIO_Port GPIOF
#define VCOM_TX_STLINK_Pin GPIO_PIN_10
#define VCOM_TX_STLINK_GPIO_Port GPIOB
#define VCOM_RX_ST_LINK_Pin GPIO_PIN_11
#define VCOM_RX_ST_LINK_GPIO_Port GPIOB
#define SPI_SYNC_Pin GPIO_PIN_12
#define SPI_SYNC_GPIO_Port GPIOB
#define LDAC_Pin GPIO_PIN_10
#define LDAC_GPIO_Port GPIOD
#define RE_tranceiver_Pin GPIO_PIN_4
#define RE_tranceiver_GPIO_Port GPIOB
#define DE_tranceiver_Pin GPIO_PIN_5
#define DE_tranceiver_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
