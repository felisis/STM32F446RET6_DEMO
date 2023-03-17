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
#define BT_2_Pin GPIO_PIN_13
#define BT_2_GPIO_Port GPIOC
#define I2S_DIN_Pin GPIO_PIN_1
#define I2S_DIN_GPIO_Port GPIOC
#define KEY_1_Pin GPIO_PIN_1
#define KEY_1_GPIO_Port GPIOA
#define DAC_OUT1_Pin GPIO_PIN_4
#define DAC_OUT1_GPIO_Port GPIOA
#define SPI_SCK_Pin GPIO_PIN_5
#define SPI_SCK_GPIO_Port GPIOA
#define SPI_MISO_Pin GPIO_PIN_6
#define SPI_MISO_GPIO_Port GPIOA
#define SPI_MOSI_Pin GPIO_PIN_7
#define SPI_MOSI_GPIO_Port GPIOA
#define FLASH_SS_Pin GPIO_PIN_5
#define FLASH_SS_GPIO_Port GPIOC
#define I2S_DOUT_Pin GPIO_PIN_0
#define I2S_DOUT_GPIO_Port GPIOB
#define SEG_A_Pin GPIO_PIN_1
#define SEG_A_GPIO_Port GPIOB
#define SEG_B_Pin GPIO_PIN_2
#define SEG_B_GPIO_Port GPIOB
#define I2S_MASTER_CK_Pin GPIO_PIN_10
#define I2S_MASTER_CK_GPIO_Port GPIOB
#define I2S_WS_Pin GPIO_PIN_12
#define I2S_WS_GPIO_Port GPIOB
#define SEG_F_Pin GPIO_PIN_13
#define SEG_F_GPIO_Port GPIOB
#define SEG_G_Pin GPIO_PIN_14
#define SEG_G_GPIO_Port GPIOB
#define SEG_DP_Pin GPIO_PIN_15
#define SEG_DP_GPIO_Port GPIOB
#define MOD_UART_RX_Pin GPIO_PIN_6
#define MOD_UART_RX_GPIO_Port GPIOC
#define I2S_SLAVE_CK_Pin GPIO_PIN_10
#define I2S_SLAVE_CK_GPIO_Port GPIOC
#define FND_COM1_Pin GPIO_PIN_11
#define FND_COM1_GPIO_Port GPIOC
#define FND_COM2_Pin GPIO_PIN_12
#define FND_COM2_GPIO_Port GPIOC
#define KEY_2_Pin GPIO_PIN_2
#define KEY_2_GPIO_Port GPIOD
#define SEG_C_Pin GPIO_PIN_4
#define SEG_C_GPIO_Port GPIOB
#define SEG_D_Pin GPIO_PIN_8
#define SEG_D_GPIO_Port GPIOB
#define SEG_E_Pin GPIO_PIN_9
#define SEG_E_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
