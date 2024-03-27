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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define OLED_SCL_Pin GPIO_PIN_5
#define OLED_SCL_GPIO_Port GPIOE
#define OLED_SDA_Pin GPIO_PIN_6
#define OLED_SDA_GPIO_Port GPIOE
#define MotorA_IN2_Pin GPIO_PIN_2
#define MotorA_IN2_GPIO_Port GPIOA
#define MotorA_IN1_Pin GPIO_PIN_3
#define MotorA_IN1_GPIO_Port GPIOA
#define MotorB_IN1_Pin GPIO_PIN_4
#define MotorB_IN1_GPIO_Port GPIOA
#define MotorB_IN2_Pin GPIO_PIN_5
#define MotorB_IN2_GPIO_Port GPIOA
#define EncoderB_CH1_Pin GPIO_PIN_6
#define EncoderB_CH1_GPIO_Port GPIOA
#define EncoderB_CH2_Pin GPIO_PIN_7
#define EncoderB_CH2_GPIO_Port GPIOA
#define OLED_RST_Pin GPIO_PIN_7
#define OLED_RST_GPIO_Port GPIOE
#define OLED_DC_Pin GPIO_PIN_8
#define OLED_DC_GPIO_Port GPIOE
#define TRIG_Pin GPIO_PIN_13
#define TRIG_GPIO_Port GPIOE
#define ServoMotor_Pin GPIO_PIN_14
#define ServoMotor_GPIO_Port GPIOE
#define ultra_ECHO_Pin GPIO_PIN_12
#define ultra_ECHO_GPIO_Port GPIOD
#define MotorA_PWM_Pin GPIO_PIN_6
#define MotorA_PWM_GPIO_Port GPIOC
#define MotorB_PWM_Pin GPIO_PIN_7
#define MotorB_PWM_GPIO_Port GPIOC
#define EncoderA_CH1_Pin GPIO_PIN_15
#define EncoderA_CH1_GPIO_Port GPIOA
#define EncoderA_CH2_Pin GPIO_PIN_3
#define EncoderA_CH2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
