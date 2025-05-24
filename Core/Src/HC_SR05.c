/*
 * HC_SR05.c
 *
 */

#include <HC_SR05.h>
#include "stm32f4xx_hal.h"

extern TIM_HandleTypeDef htim1;


uint32_t IC_Val1 = 0;    			// Gia tri xunng dau tien
uint32_t IC_Val2 = 0;    			// Gia tri xung thu hai
uint32_t Difference = 0;			// Chenh lech giua hai lan do
uint8_t Is_First_Captured = 0;  	// Co bat canh len hay canh xuong
float Distance = 0;  				// Khoang cach do duoc


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		if (Is_First_Captured == 0)
		{
			IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			Is_First_Captured = 1;  // Co bao bat duoc xung 1
			// doi bat tu canh len -> canh xuong
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
		}
		else if (Is_First_Captured == 1)
		{
			IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			// Reset counter
			__HAL_TIM_SET_COUNTER(htim, 0);

			if (IC_Val2 > IC_Val1)
			{
				Difference = IC_Val2 - IC_Val1;
			}
			else if (IC_Val1 > IC_Val2)
			{
				// truong hop bi tran bo dem
				Difference = (0xffff - IC_Val1) + IC_Val2;
			}
			// k/c do bang cm, van toc anh sang = 340 m/s = 0.034 cm/us
			Distance = Difference * 0.034 / 2;
			Is_First_Captured = 0;
			// doi bat tu canh len -> canh xuong
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC1);
		}
	}
}

//Tao xung 10us
void Delay(uint32_t time)
{
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while (__HAL_TIM_GET_COUNTER(&htim1) < time);
}

// Hàm đo khoảng cách bằng cảm biến HC-SR05
float HCSR05_GetDis(void)
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);
	Delay(10);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);
	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1); // Bat ngat de do k/c
	return Distance;
}


