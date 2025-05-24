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
#define TANK_HEIGHT 14.8

//#define Setpoint 3

#define K1 1/14.8


#define e_NB 0
#define e_NS 1
#define e_ZE 2
#define e_PS 3
#define e_PB 4


#define edot_NB 0
#define edot_NS 1
#define edot_ZE 2
#define edot_PS 3
#define edot_PB 4

#define y_NB 0
#define y_NM 1
#define y_NS 2
#define y_ZE 3
#define y_PS 4
#define y_PM 5
#define y_PB 6
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

float fuzzyController(float a, float b);
float hlt_hinhthang(float data,float l, float cl, float cr, float r);
float hlt_tamgiac(float data,float l,float m, float r);
float sum_array (float data[][5],char n, char m);
float MIN(float a,float b);
float rule(float data[],float val,char n);
float Saturation(float x);
void Motor_Control(int32_t pwmValue);
float HCSR05_Read(void);

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define HCSR05_TRIG_Pin GPIO_PIN_7
#define HCSR05_TRIG_GPIO_Port GPIOE
#define L298N_IN3_Pin GPIO_PIN_3
#define L298N_IN3_GPIO_Port GPIOB
#define L298N_IN4_Pin GPIO_PIN_5
#define L298N_IN4_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
