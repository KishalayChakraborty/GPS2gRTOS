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
#include "stm32g0xx_hal.h"

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
#define OSC32_IN_Pin GPIO_PIN_14
#define OSC32_IN_GPIO_Port GPIOC
#define OSC32_OUT_Pin GPIO_PIN_15
#define OSC32_OUT_GPIO_Port GPIOC
#define OSC_IN_Pin GPIO_PIN_0
#define OSC_IN_GPIO_Port GPIOF
#define OSC_OUT_Pin GPIO_PIN_1
#define OSC_OUT_GPIO_Port GPIOF
#define AI_ADC1_Pin GPIO_PIN_0
#define AI_ADC1_GPIO_Port GPIOA
#define AI_ADC2_Pin GPIO_PIN_1
#define AI_ADC2_GPIO_Port GPIOA
#define mcu_TXD_GPS_Pin GPIO_PIN_2
#define mcu_TXD_GPS_GPIO_Port GPIOA
#define mcu_RXD_GPS_Pin GPIO_PIN_3
#define mcu_RXD_GPS_GPIO_Port GPIOA
#define SPI_CS_MEM_Pin GPIO_PIN_4
#define SPI_CS_MEM_GPIO_Port GPIOA
#define SPI_CLK_Pin GPIO_PIN_5
#define SPI_CLK_GPIO_Port GPIOA
#define SPI_MISO_Pin GPIO_PIN_6
#define SPI_MISO_GPIO_Port GPIOA
#define SPI_MOSI_Pin GPIO_PIN_7
#define SPI_MOSI_GPIO_Port GPIOA
#define mcu_RXD_232_Pin GPIO_PIN_0
#define mcu_RXD_232_GPIO_Port GPIOB
#define EXT_B_SENSE_Pin GPIO_PIN_1
#define EXT_B_SENSE_GPIO_Port GPIOB
#define MCU_TXD_232_Pin GPIO_PIN_2
#define MCU_TXD_232_GPIO_Port GPIOB
#define INT_B_SENSE_Pin GPIO_PIN_10
#define INT_B_SENSE_GPIO_Port GPIOB
#define DI_IN2_Pin GPIO_PIN_11
#define DI_IN2_GPIO_Port GPIOB
#define DI_IN3_Pin GPIO_PIN_12
#define DI_IN3_GPIO_Port GPIOB
#define DI_MAINS_STATE_Pin GPIO_PIN_13
#define DI_MAINS_STATE_GPIO_Port GPIOB
#define DI_ACC_STATE_Pin GPIO_PIN_14
#define DI_ACC_STATE_GPIO_Port GPIOB
#define DI_BOX_STATE_Pin GPIO_PIN_15
#define DI_BOX_STATE_GPIO_Port GPIOB
#define DI_SOS_STATE_Pin GPIO_PIN_8
#define DI_SOS_STATE_GPIO_Port GPIOA
#define mcu_TXD_GSM_Pin GPIO_PIN_9
#define mcu_TXD_GSM_GPIO_Port GPIOA
#define DO_GSM_VCC_EN_Pin GPIO_PIN_6
#define DO_GSM_VCC_EN_GPIO_Port GPIOC
#define DO_PWRKEY_Pin GPIO_PIN_7
#define DO_PWRKEY_GPIO_Port GPIOC
#define mcu_RXD_GSM_Pin GPIO_PIN_10
#define mcu_RXD_GSM_GPIO_Port GPIOA
#define DI_IN1_Pin GPIO_PIN_12
#define DI_IN1_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define DO_GPS_VCC_EN_Pin GPIO_PIN_15
#define DO_GPS_VCC_EN_GPIO_Port GPIOA
#define DO_LED_PWR_Pin GPIO_PIN_0
#define DO_LED_PWR_GPIO_Port GPIOD
#define DO_LED_NET_Pin GPIO_PIN_1
#define DO_LED_NET_GPIO_Port GPIOD
#define DO_LED_GPS_Pin GPIO_PIN_2
#define DO_LED_GPS_GPIO_Port GPIOD
#define DO_OUT1_Pin GPIO_PIN_3
#define DO_OUT1_GPIO_Port GPIOB
#define DO_OUT2_Pin GPIO_PIN_4
#define DO_OUT2_GPIO_Port GPIOB
#define DO_5V_OUT_EN_Pin GPIO_PIN_5
#define DO_5V_OUT_EN_GPIO_Port GPIOB
#define SCL_Pin GPIO_PIN_6
#define SCL_GPIO_Port GPIOB
#define SDA_Pin GPIO_PIN_7
#define SDA_GPIO_Port GPIOB
#define DI_INT1_ACCEL_Pin GPIO_PIN_8
#define DI_INT1_ACCEL_GPIO_Port GPIOB
#define DO_OUT3_P_LED_Pin GPIO_PIN_9
#define DO_OUT3_P_LED_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
