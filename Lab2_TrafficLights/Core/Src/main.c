/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "abuzz.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
enum event{
	ev_none = 0,
	ev_button_push,
	ev_state_timeout,
	ev_error = -99
};

#define EVQ_SIZE 10
enum event evq[ EVQ_SIZE ];
int evq_count     = 0;
int evq_front_ix  = 0;
int evq_rear_ix   = 0;

enum state{
	s_init = 1,
	s_all_stop,
	s_humans_go,
	s_prepare_car,
	s_RY_car,
	s_car_go,
	s_pushed_delay,
	s_cars_stopping
};

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void abuzz_start();
void abuzz_stop();
void abuzz_p_long();
void abuzz_p_short();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t ticks_left_in_state = 0;

void push_button_light_on(){
	GPIOC->BSRR = GPIO_BSRR_BS9;
}

void push_button_light_off(){
	GPIOC->BRR = GPIO_BRR_BR9;
}

// Returns 1 if button is pressed, else 0 //
int is_button_pressed(){
	return (GPIOC->IDR & GPIO_PIN_0) != 0;
}

void reset_traffic_lights(){
	GPIOA->BRR = GPIO_BRR_BR12 | GPIO_BRR_BR11 |
				 GPIO_BRR_BR10 | GPIO_BRR_BR9  | GPIO_BRR_BR8;
}

void set_traffic_lights(enum state s){
	reset_traffic_lights();

	switch(s){
		case s_init:
			GPIOA->BSRR = GPIO_BSRR_BS12 | GPIO_BSRR_BS11 |
						  GPIO_BSRR_BS10 | GPIO_BSRR_BS9  | GPIO_BSRR_BS8;
			break;
		case s_all_stop:
			GPIOA->BSRR = GPIO_BSRR_BS12 | GPIO_BSRR_BS9;
			break;
		case s_humans_go:
			GPIOA->BSRR = GPIO_BSRR_BS12 | GPIO_BSRR_BS8;
			break;
		case s_prepare_car:
			GPIOA->BSRR = GPIO_BSRR_BS12 | GPIO_BSRR_BS9;
			break;
		case s_RY_car:
			GPIOA->BSRR = GPIO_BSRR_BS12 | GPIO_BSRR_BS11 | GPIO_BSRR_BS9;
			break;
		case s_car_go:
			GPIOA->BSRR = GPIO_BSRR_BS10 | GPIO_BSRR_BS9;
			break;
		case s_pushed_delay:
			GPIOA->BSRR = GPIO_BSRR_BS10 | GPIO_BSRR_BS9;
			break;
		case s_cars_stopping:
			GPIOA->BSRR = GPIO_BSRR_BS11 | GPIO_BSRR_BS9;
			break;
		default:
				//Something went wrong, both green//
			GPIOA->BSRR = GPIO_BSRR_BS10 | GPIO_BSRR_BS8;
	}
}

void state_handler(enum state* st, enum event* ev, uint32_t* ticks_left_in_state){

	switch(*st){

	case s_init:
		if(*ev == ev_button_push){
			*ev = ev_none;
			*st = s_all_stop;
			*ticks_left_in_state = 3000;
			push_button_light_on();
			set_traffic_lights(s_all_stop);
		}
		break;
	case s_all_stop:
			if(*ev == ev_state_timeout){
				*ev = ev_none;
				*st = s_humans_go;
				*ticks_left_in_state = 10000;
				abuzz_start();
				abuzz_p_short();
				set_traffic_lights(s_humans_go);
			}
			break;

	case s_humans_go:
		if(*ev == ev_state_timeout){
			*ev = ev_none;
			*st = s_prepare_car;
			*ticks_left_in_state = 2000;
			push_button_light_off();
			abuzz_stop();
			set_traffic_lights(s_prepare_car);
		}
		break;

	case s_prepare_car:
		if(*ev == ev_state_timeout){
			*ev = ev_none;
			*st = s_RY_car;
			*ticks_left_in_state = 2000;
			set_traffic_lights(s_RY_car);
		}
		break;

	case s_RY_car:
		if(*ev == ev_state_timeout){
			*ev = ev_none;
			*st = s_car_go;
			*ticks_left_in_state = 0;
			set_traffic_lights(s_car_go);
		}
		break;

	case s_car_go:
		if(*ev == ev_button_push){
			*ev = ev_none;
			*st = s_pushed_delay;
			*ticks_left_in_state = 2000;
			push_button_light_on();
			set_traffic_lights(s_pushed_delay);
		}
		break;

	case s_pushed_delay:
		if(*ev == ev_state_timeout){
			*ev = ev_none;
			*st = s_cars_stopping;
			*ticks_left_in_state = 2000;
			set_traffic_lights(s_cars_stopping);
		}
		break;

	case s_cars_stopping:
		if(*ev == ev_state_timeout){
			*ev = ev_none;
			*st = s_all_stop;
			*ticks_left_in_state = 3000;
			set_traffic_lights(s_all_stop);
		}
		break;

	default :
		break;
	}
}

void evq_init(){
	for(int i = 0; i<10; ++i)
		evq[i] = ev_error;
}


void evq_push_back(enum event ev){
	evq[evq_rear_ix] = ev;

	evq_rear_ix = (evq_rear_ix + 1) % EVQ_SIZE;
	if(evq_count < EVQ_SIZE)
		++evq_count;
}

enum event evq_pop_front(){
	if(evq_count <= 0)
		return ev_none;

	enum event temp = evq[evq_front_ix];
	evq_front_ix = (evq_front_ix + 1) % EVQ_SIZE;
	--evq_count;
	return temp;
}

int systick_count = 0;
void my_systick_handler(){

	systick_count++;
	if(systick_count == 1000){
		HAL_GPIO_TogglePin (LD4_GPIO_Port, LD4_Pin);
		systick_count = 0;
	}
	if(ticks_left_in_state > 0){
		--ticks_left_in_state;
		if(ticks_left_in_state == 0)
			evq_push_back(ev_state_timeout);
	}
}

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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
  evq_init();

  enum state st = s_init;
  enum event ev = ev_none;
  set_traffic_lights(st);
#if 0

  int last_pressed_state = 0;
  int pressed = 0;


  uint32_t ticks_left_in_state = 0;
  uint32_t curr_tick = 0;
  uint32_t last_tick = 0;*/
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1){

#if 0
	  ev = ev_none;
	  pressed = is_button_pressed();

	  if(pressed && !last_pressed_state && (st == s_car_go || st == s_init)){
		  ev = ev_button_push;
	  }

	  curr_tick = HAL_GetTick();
	  if(curr_tick != last_tick && curr_tick > last_tick)
		  ticks_left_in_state -= (curr_tick-last_tick);

	  if((ticks_left_in_state == 0 && last_tick > 0) && (st != s_car_go || st != s_init)){
		  ev = ev_state_timeout;
	  }
	  last_tick = curr_tick;
	  last_pressed_state = pressed;
#else
	  ev = evq_pop_front();
	  state_handler(&st,&ev,&ticks_left_in_state);

#endif
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SMPS_EN_Pin|SMPS_V1_Pin|SMPS_SW_Pin|walker_green_Pin
                          |walker_red_Pin|car_green_Pin|car_yellow_Pin|car_red_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(crosswalk_button_light_GPIO_Port, crosswalk_button_light_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : crosswalk_button_Pin */
  GPIO_InitStruct.Pin = crosswalk_button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(crosswalk_button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SMPS_EN_Pin SMPS_V1_Pin SMPS_SW_Pin walker_green_Pin
                           walker_red_Pin car_green_Pin car_yellow_Pin car_red_Pin */
  GPIO_InitStruct.Pin = SMPS_EN_Pin|SMPS_V1_Pin|SMPS_SW_Pin|walker_green_Pin
                          |walker_red_Pin|car_green_Pin|car_yellow_Pin|car_red_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SMPS_PG_Pin */
  GPIO_InitStruct.Pin = SMPS_PG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SMPS_PG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD4_Pin */
  GPIO_InitStruct.Pin = LD4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : crosswalk_button_light_Pin */
  GPIO_InitStruct.Pin = crosswalk_button_light_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(crosswalk_button_light_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	if(GPIO_Pin == crosswalk_button_Pin){
		evq_push_back(ev_button_push);
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
