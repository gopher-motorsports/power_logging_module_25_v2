/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#define RFD_GPIO0_Pin GPIO_PIN_2
#define RFD_GPIO0_GPIO_Port GPIOE
#define LED3_Pin GPIO_PIN_3
#define LED3_GPIO_Port GPIOE
#define LED4_Pin GPIO_PIN_4
#define LED4_GPIO_Port GPIOE
#define RFD_GPIO5_Pin GPIO_PIN_5
#define RFD_GPIO5_GPIO_Port GPIOE
#define RFD_GPIO6_Pin GPIO_PIN_6
#define RFD_GPIO6_GPIO_Port GPIOE
#define ADC_5V_1_Pin GPIO_PIN_0
#define ADC_5V_1_GPIO_Port GPIOC
#define ADC_5V_2_Pin GPIO_PIN_1
#define ADC_5V_2_GPIO_Port GPIOC
#define ADC_5V_3_Pin GPIO_PIN_2
#define ADC_5V_3_GPIO_Port GPIOC
#define VBAT_MEAS_Pin GPIO_PIN_3
#define VBAT_MEAS_GPIO_Port GPIOC
#define ADC_12V_0_Pin GPIO_PIN_0
#define ADC_12V_0_GPIO_Port GPIOA
#define ADC_12V_1_Pin GPIO_PIN_2
#define ADC_12V_1_GPIO_Port GPIOA
#define ADC_12V_2_Pin GPIO_PIN_3
#define ADC_12V_2_GPIO_Port GPIOA
#define ADC_12V_3_Pin GPIO_PIN_4
#define ADC_12V_3_GPIO_Port GPIOA
#define ADC_12V_4_Pin GPIO_PIN_5
#define ADC_12V_4_GPIO_Port GPIOA
#define ADC_12V_5_Pin GPIO_PIN_6
#define ADC_12V_5_GPIO_Port GPIOA
#define ADC_12V_6_Pin GPIO_PIN_7
#define ADC_12V_6_GPIO_Port GPIOA
#define FV_MEAS_Pin GPIO_PIN_4
#define FV_MEAS_GPIO_Port GPIOC
#define EN_12V_0_Pin GPIO_PIN_5
#define EN_12V_0_GPIO_Port GPIOC
#define ADC_5V_0_Pin GPIO_PIN_0
#define ADC_5V_0_GPIO_Port GPIOB
#define USB_RESET_Pin GPIO_PIN_7
#define USB_RESET_GPIO_Port GPIOE
#define SDC_MCU_1_Pin GPIO_PIN_8
#define SDC_MCU_1_GPIO_Port GPIOE
#define SDC_MCU_2_Pin GPIO_PIN_9
#define SDC_MCU_2_GPIO_Port GPIOE
#define SDC_MCU_3_Pin GPIO_PIN_10
#define SDC_MCU_3_GPIO_Port GPIOE
#define SDC_MCU_4_Pin GPIO_PIN_11
#define SDC_MCU_4_GPIO_Port GPIOE
#define EN_5V_0_Pin GPIO_PIN_12
#define EN_5V_0_GPIO_Port GPIOE
#define EN_5V_1_Pin GPIO_PIN_13
#define EN_5V_1_GPIO_Port GPIOE
#define EN_5V_2_Pin GPIO_PIN_14
#define EN_5V_2_GPIO_Port GPIOE
#define LED5_Pin GPIO_PIN_15
#define LED5_GPIO_Port GPIOE
#define USB_BUS_SNS_Pin GPIO_PIN_12
#define USB_BUS_SNS_GPIO_Port GPIOB
#define EN_12V_2_Pin GPIO_PIN_14
#define EN_12V_2_GPIO_Port GPIOB
#define EN_12V_3_Pin GPIO_PIN_15
#define EN_12V_3_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_14
#define LED2_GPIO_Port GPIOD
#define LED0_Pin GPIO_PIN_15
#define LED0_GPIO_Port GPIOD
#define EN_12V_4_Pin GPIO_PIN_6
#define EN_12V_4_GPIO_Port GPIOC
#define EN_12V_5_Pin GPIO_PIN_7
#define EN_12V_5_GPIO_Port GPIOC
#define EN_12V_6_Pin GPIO_PIN_8
#define EN_12V_6_GPIO_Port GPIOA
#define SDIO_CD_Pin GPIO_PIN_15
#define SDIO_CD_GPIO_Port GPIOA
#define RFD_GPIO3_Pin GPIO_PIN_4
#define RFD_GPIO3_GPIO_Port GPIOD
#define RPD_GPIO2_Pin GPIO_PIN_5
#define RPD_GPIO2_GPIO_Port GPIOD
#define RFD_GPIO1_Pin GPIO_PIN_6
#define RFD_GPIO1_GPIO_Port GPIOD
#define EN_12V_1_Pin GPIO_PIN_4
#define EN_12V_1_GPIO_Port GPIOB
#define USB_RESETB6_Pin GPIO_PIN_6
#define USB_RESETB6_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_7
#define LED1_GPIO_Port GPIOB
#define BMS_LIGHT_CTRL_Pin GPIO_PIN_0
#define BMS_LIGHT_CTRL_GPIO_Port GPIOE
#define IMD_LIGHT_CTRL_Pin GPIO_PIN_1
#define IMD_LIGHT_CTRL_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
