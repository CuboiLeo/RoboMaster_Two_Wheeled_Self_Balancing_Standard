/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#define Orin_Pin GPIO_PIN_1
#define Orin_GPIO_Port GPIOE
#define OrinE0_Pin GPIO_PIN_0
#define OrinE0_GPIO_Port GPIOE
#define IMU_Heating_Pin GPIO_PIN_5
#define IMU_Heating_GPIO_Port GPIOB
#define Referee_Pin GPIO_PIN_14
#define Referee_GPIO_Port GPIOG
#define DR16_Pin GPIO_PIN_7
#define DR16_GPIO_Port GPIOB
#define DR16B6_Pin GPIO_PIN_6
#define DR16B6_GPIO_Port GPIOB
#define RefereeG9_Pin GPIO_PIN_9
#define RefereeG9_GPIO_Port GPIOG
#define SPI_NSS_Pin GPIO_PIN_6
#define SPI_NSS_GPIO_Port GPIOF
#define Buzzer_Pin GPIO_PIN_6
#define Buzzer_GPIO_Port GPIOH
#define Board_A_Pin GPIO_PIN_8
#define Board_A_GPIO_Port GPIOE
#define Board_AE7_Pin GPIO_PIN_7
#define Board_AE7_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
