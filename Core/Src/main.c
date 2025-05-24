/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "HC_SR05.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
float e_k;
float e_dot;

float e_k1;

float e_k_input;
float e_k_dot_input;
uint8_t flag = 0;

double pwmValue;
volatile float point = 10.0f;
volatile float K2 = 2;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float Dis = 0;
float docao;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */

	HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1); // HCSR05 ECHO
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);   // L298N ENA (PA15)
//  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);   // L298N IN3
//  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); // L298N IN4
  HAL_TIM_Base_Start_IT(&htim3); // Thoi gian dieu khien he thong

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  Dis = HCSR05_GetDis();
	  HAL_Delay(200);
	  flag = 1;
	  docao = 14.8 - Dis;

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 100-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xffff-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 100-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 100-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 5000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(HCSR05_TRIG_GPIO_Port, HCSR05_TRIG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, L298N_IN3_Pin|L298N_IN4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : HCSR05_TRIG_Pin */
  GPIO_InitStruct.Pin = HCSR05_TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(HCSR05_TRIG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : L298N_IN3_Pin L298N_IN4_Pin */
  GPIO_InitStruct.Pin = L298N_IN3_Pin|L298N_IN4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/*
 **** Chuong trinh giai mo bang pp trung binh co trong so cua thay ****
 % My_SugenoFIS : Sugeno Fuzzy Inference System
clear all

% Gia tri ro cua bien vao
x1=-5;
x2=9;

% Khai bao cac gia tri ngon ngu
x1_NE=hlt_hinhthang(x1,-13,-12,-12,0);
x1_ZE=hlt_hinhthang(x1,-12,0,0,12);
x1_PO=hlt_hinhthang(x1,0,12,12,13);

x2_LO=hlt_hinhthang(x2,-1,0,1,6);
x2_ME=hlt_hinhthang(x2,1,6,6,11);
x2_HI=hlt_hinhthang(x2,6,11,12,13);

y_NB=-2;
y_NS=-1;
y_ZE=0;
y_PS=1;
y_PB=2;

% Cac luat va suy luan mo
beta1 = min(x1_NE,x2_LO);
y1 = y_NB;

beta2 = min(x1_NE,x2_ME);
y2 = y_NS;

beta3 = min(x1_NE,x2_HI);
y3 = y_ZE;

beta4 = min(x1_ZE,x2_LO);
y4 = y_NB;

beta5 = min(x1_ZE,x2_ME);
y5 = y_ZE;

beta6 = min(x1_ZE,x2_HI);
y6 = y_PB;

beta7 = min(x1_PO,x2_LO);
y7 = y_ZE;

beta8 = min(x1_PO,x2_ME);
y8 = y_PS;

beta9 = min(x1_PO,x2_HI);
y9 = y_PB;

% Giai mo trung binh co trong so

TS = beta1*y1+beta2*y2+beta3*y3+...
     beta4*y4+beta5*y5+beta6*y6+...
     beta7*y7+beta8*y8+beta9*y9;
MS = beta1+beta2+beta3+...
     beta4+beta5+beta6+...
     beta7+beta8+beta9;

y=TS/MS
 */

float fuzzyController(float a, float b)
{
    float e[]={hlt_hinhthang(a,-2,-1,-0.25,-0.1),hlt_tamgiac(a,-0.25,-0.1,0),hlt_tamgiac(a,-0.1,0,0.1),
    		hlt_tamgiac(a,0,0.1,0.25),hlt_hinhthang(a,0.1,0.25,1,2)};

    float edot[]={hlt_hinhthang(b,-1.5,-1,-0.3,-0.2),hlt_tamgiac(b,-0.3,-0.2,0),hlt_tamgiac(b,-0.2,0,0.2),
    		hlt_tamgiac(b,0,0.2,0.3),hlt_hinhthang(b,0.2,0.3,1,2)};

    float y[]={-100,-40,-20,0,30,70,100};
    float beta[5][5];
    for (int i = 0; i < 5; i++)
    {
     for (int j = 0; j < 5 ;j++)
     {
       beta[i][j]= MIN(e[i],edot[j]);
     }
    }

    float NB[] = {beta[e_NB][edot_NB],beta[e_NB][edot_NS],beta[e_NS][edot_NB]};
    float NM[] = {beta[e_NB][edot_ZE],beta[e_NS][edot_NS],beta[e_ZE][edot_NB]};
    float NS[] = {beta[e_NB][edot_PS],beta[e_NS][edot_ZE],beta[e_ZE][edot_NS],beta[e_PS][edot_NB]};
    float ZE[] = {beta[e_NB][edot_PB],beta[e_NS][edot_PS],beta[e_ZE][edot_ZE],beta[e_PS][edot_NS],beta[e_PB][edot_NB]};
    float PS[] = {beta[e_NS][edot_PB],beta[e_ZE][edot_PS],beta[e_PS][edot_ZE],beta[e_PB][edot_NS]};
    float PM[] = {beta[e_ZE][edot_PB],beta[e_PS][edot_PS],beta[e_PB][edot_ZE]};
    float PB[] = {beta[e_PB][edot_PB],beta[e_PB][edot_PS],beta[e_PS][edot_PB]};

    float ans = (rule(NB,y[y_NB],3)+rule(NM,y[y_NM],3)+rule(NS,y[y_NS],4)+rule(ZE,y[y_ZE],5)+
    		rule(PS,y[y_PS],4)+rule(PM,y[y_PM],3)+rule(PB,y[y_PB],3))/sum_array(beta,5,5); //
    return ans;
}

float hlt_hinhthang(float data, float l, float cl, float cr, float r)
{
    if ((data < l)   && (data >= r)) return 0;
    if ((data >= l)  && (data < cl)) return (data - l)/(cl - l);
    if ((data >= cl) && (data < cr)) return 1;
    if ((data >= cr) && (data < r))  return (r - data)/(r - cr);
    return 0;
}


float hlt_tamgiac(float data,float l,float m, float r)
{
    if ((data < l)  || (data >= r)) return 0;
    if ((data >= l) && (data < m))  return (data - l)/(m - l);
    if ((data >= m) && (data < r))  return (r - data)/(r - m);
    return 0;
}

// Tinh tong theo cong thuc trung binh co trong so
float sum_array (float data[][5],char n, char m)
{
    float s=0;
    for (int i =0; i < n; i++)
    {
     for (int j=0; j<m; j++)
     {
       s += data[i][j];
     }
    }
    return s;
}


float MIN(float a,float b)
{
    if (a < b) return a;
    else return b;
}


float rule(float data[],float val,char n)
{
    float s = 0;
    for (int i = 0; i < n; i++)
    {
     s += data[i]*val;
    }
    return s;
}

//Ham chuan hoa cac gia tri -1<x<1
float Saturation(float x)
{
	if(x >= 1)
		x = 1;
	else if(x <= -1)
		x = -1;
	return x;
}

void Motor_Control(int32_t pwmValue) {
    if (pwmValue > 0) {
        // Bom
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);   // L298N IN3
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); // L298N IN4
    } else {
        // Dung Bom
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET); // L298N IN3
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); // L298N IN4
    }
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwmValue);
}

/*
 Thoi gian dieu khien cua chuong trinh la 200ms
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3) // Kiểm tra nếu là timer 3
    {
//    	Dis = HCSR05_GetDis();
//      Delay(200000); // 200ms
    	//HAL_Delay(200);
    	if (flag){

    	e_k1 = e_k;
    	e_k = point - (TANK_HEIGHT - Dis);
    	e_dot = (e_k - e_k1)/0.2;

    	e_k_input = e_k*K1;
    	e_k_dot_input = e_dot*K2;

    	pwmValue += (0.2*fuzzyController(Saturation(e_k_input), Saturation(e_k_dot_input)));  //xuat thang ra gia tri cua pwm

    	if (pwmValue >= 900)
    		pwmValue = 900;
    	if (pwmValue < 0)
    		pwmValue = 0;

    	Motor_Control((int32_t)pwmValue);
    	}
    	flag = 0;
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
