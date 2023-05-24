/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t pulse_number = 0;
uint16_t rpm = 0;
uint16_t rpm1 = 0;
uint16_t rpm0 = 0;
uint16_t rpmp = 0;
uint16_t rpmn = 0;
float rpm_t = 0;
uint16_t steadyState = 0;
int pwm_low = 40;
int pwm_high = 70;
uint16_t K = 0;
uint16_t TAU_value = 0;
uint16_t direction = 0;
uint16_t position_pulse = 0;
uint16_t position = 0;
uint16_t pwm_DB = 0;
uint16_t tmp = 0;
uint16_t Tau_rpm =0;

typedef enum {S0, S1, S2, S3} STATE_T;
STATE_T state = S0;
//uint16_t dir_control = 0;
//uint16_t temp = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM11_Init();
  MX_TIM7_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim7);
  HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
  TIM11->PSC = 55;
  TIM11->ARR = 199;
  TIM11->CCR1 = 50;



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  switch(state){
	  case S0:
		  TIM11->CCR1 = 1;
		  break;
	  case S1:
		  TIM11->CCR1 = 60;
		  HAL_Delay(1000);
		  pwm_DB = TIM11->CCR1;
		  rpm0 = rpm;
		  rpmn = rpm;
		  break;
	  case S2:
		  TIM11->CCR1 = 140;
		  HAL_Delay(1000);
		  rpm1 = rpm;
		  rpmp = rpmn;
		  rpmn = rpm;

		  K = (rpm1 - rpm0) / ((0.7f-0.3f) * 12);

		  if ((rpmn - rpmp) < 1 && (rpmn - rpmp) > -1 ){
			  steadyState = rpmn;
		  }


		  Tau_rpm = rpm0 +((rpm1-rpm0)*63)/100;

		  TIM11->CCR1 = 60;

		  __HAL_TIM_SET_COUNTER(&htim6,0);
		  HAL_TIM_Base_Start(&htim6);

		  while (rpm < Tau_rpm){

		  }
		  tmp = __HAL_TIM_GET_COUNTER(&htim6);
//		  TAU_value = (tmp*63)/100;
		  HAL_TIM_Base_Stop(&htim6);


//		  if (rpm > (1134 + rpm0)){
//			  TAU_value = __HAL_TIM_GET_COUNTER(&htim7);
//			  HAL_TIM_Base_Stop(&htim7);
//		  }

		  break;
	  case S3:
		  TIM11->CCR1 = 70;
		  break;
	  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim -> Instance == TIM7){
		rpm = (pulse_number*6000)/1024 ;
		pulse_number = 0;
	}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == a_signal_Pin){
		pulse_number++;

		if(HAL_GPIO_ReadPin(b_signal_GPIO_Port,b_signal_Pin) == GPIO_PIN_SET){
			direction = 1;
		}
		else {
			direction = 0;
		}
	}

	if(GPIO_Pin == GPIO_PIN_2){
//		HAL_GPIO_TogglePin(direction_GPIO_Port, direction_Pin);
//		if(dir_control == 0){
//			HAL_GPIO_WritePin(direction_GPIO_Port, direction_Pin, GPIO_PIN_SET);
//			dir_control = 1;
//		}
//		else{
//			HAL_GPIO_WritePin(direction_GPIO_Port, direction_Pin, GPIO_PIN_RESET);
//			dir_control = 0;
//		}
		if (state == S0){
			state = S1;
		}
		else if(state == S1){
			state = S2;
		}
		else if(state == S2){
			state = S3;
		}
		else{
			state = S0;
		}


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
