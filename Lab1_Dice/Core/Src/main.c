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
#include <stdlib.h>
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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
int is_blue_button_pressed();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
const uint16_t sseg[10] = {0x5F,0x06,0x9B,0x8F,0xC6,0xCD,0xDD,0x07,0xDF,0xCF};
const uint16_t sseg_err = 0x19C;



//test 7seg display function
void put_on_sseg(uint8_t dec_nbr){
	GPIOC->ODR = 0x00;

	for(uint8_t i = 0; i<10; ++i){
		if(dec_nbr == i){
			GPIOC->ODR = sseg[i];
			return;
		}
	}
	GPIOC->ODR = sseg_err;
}

// Returns 1 if button is pressed, else 0 //
int is_blue_button_pressed(){
	uint32_t reg_reading = GPIOC->IDR;
	return (reg_reading & GPIO_PIN_13) != 0;
}

// Restets all pins in the dice to 0
void reset_die_dots(){
	HAL_GPIO_WritePin(DI_A_GPIO_Port,DI_A_Pin ,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DI_B_GPIO_Port,DI_B_Pin ,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DI_C_GPIO_Port,DI_C_Pin ,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DI_D_GPIO_Port,DI_D_Pin ,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DI_E_GPIO_Port,DI_E_Pin ,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DI_F_GPIO_Port,DI_F_Pin ,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DI_G_GPIO_Port,DI_G_Pin ,GPIO_PIN_RESET);
}

//turns on dice diodes depending on paramiter number
void put_die_dots(uint8_t die_number){

	reset_die_dots();

	switch(die_number){
	case 1:
		HAL_GPIO_WritePin(DI_G_GPIO_Port,DI_G_Pin ,GPIO_PIN_SET);
		break;
	case 2:
		HAL_GPIO_WritePin(DI_C_GPIO_Port,DI_C_Pin ,GPIO_PIN_SET);
		HAL_GPIO_WritePin(DI_D_GPIO_Port,DI_D_Pin ,GPIO_PIN_SET);
		break;
	case 3:
		HAL_GPIO_WritePin(DI_D_GPIO_Port,DI_D_Pin ,GPIO_PIN_SET);
		HAL_GPIO_WritePin(DI_G_GPIO_Port,DI_G_Pin ,GPIO_PIN_SET);
		HAL_GPIO_WritePin(DI_C_GPIO_Port,DI_C_Pin ,GPIO_PIN_SET);
		break;
	case 4:
		HAL_GPIO_WritePin(DI_A_GPIO_Port,DI_A_Pin ,GPIO_PIN_SET);
		HAL_GPIO_WritePin(DI_C_GPIO_Port,DI_C_Pin ,GPIO_PIN_SET);
		HAL_GPIO_WritePin(DI_D_GPIO_Port,DI_D_Pin ,GPIO_PIN_SET);
		HAL_GPIO_WritePin(DI_F_GPIO_Port,DI_F_Pin ,GPIO_PIN_SET);
		break;
	case 5:
		HAL_GPIO_WritePin(DI_A_GPIO_Port,DI_A_Pin ,GPIO_PIN_SET);
		HAL_GPIO_WritePin(DI_C_GPIO_Port,DI_C_Pin ,GPIO_PIN_SET);
		HAL_GPIO_WritePin(DI_D_GPIO_Port,DI_D_Pin ,GPIO_PIN_SET);
		HAL_GPIO_WritePin(DI_F_GPIO_Port,DI_F_Pin ,GPIO_PIN_SET);
		HAL_GPIO_WritePin(DI_G_GPIO_Port,DI_G_Pin ,GPIO_PIN_SET);
		break;
	case 6:
		HAL_GPIO_WritePin(DI_A_GPIO_Port,DI_A_Pin ,GPIO_PIN_SET);
		HAL_GPIO_WritePin(DI_B_GPIO_Port,DI_B_Pin ,GPIO_PIN_SET);
		HAL_GPIO_WritePin(DI_C_GPIO_Port,DI_C_Pin ,GPIO_PIN_SET);
		HAL_GPIO_WritePin(DI_D_GPIO_Port,DI_D_Pin ,GPIO_PIN_SET);
		HAL_GPIO_WritePin(DI_E_GPIO_Port,DI_E_Pin ,GPIO_PIN_SET);
		HAL_GPIO_WritePin(DI_F_GPIO_Port,DI_F_Pin ,GPIO_PIN_SET);
		break;
	default:
		HAL_GPIO_WritePin(DI_A_GPIO_Port,DI_A_Pin ,GPIO_PIN_SET);
		HAL_GPIO_WritePin(DI_B_GPIO_Port,DI_B_Pin ,GPIO_PIN_SET);
		HAL_GPIO_WritePin(DI_C_GPIO_Port,DI_C_Pin ,GPIO_PIN_SET);
		HAL_GPIO_WritePin(DI_D_GPIO_Port,DI_D_Pin ,GPIO_PIN_SET);
		HAL_GPIO_WritePin(DI_E_GPIO_Port,DI_E_Pin ,GPIO_PIN_SET);
		HAL_GPIO_WritePin(DI_F_GPIO_Port,DI_F_Pin ,GPIO_PIN_SET);
		HAL_GPIO_WritePin(DI_G_GPIO_Port,DI_G_Pin ,GPIO_PIN_SET);
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
  /* USER CODE BEGIN 2 */

 /* for(uint8_t i = 1; i<8; ++i){
	  put_die_dots(i);
	  HAL_Delay(800);
  }
  HAL_Delay(1000);
  reset_die_dots();*/

  for(uint8_t i = 0; i < 10; ++i){
  		put_on_sseg(i);
  		HAL_Delay(333);
  	}
  	put_on_sseg(88);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  srand(HAL_GetTick());
  int pressed = 0;
  int last_pressed_state = 0;

  while (1)
  {
	  pressed = is_blue_button_pressed();

		  if(pressed && !last_pressed_state){
			  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);
			  uint8_t die_value = (rand() % 6) + 1 ;
			  //put_die_dots(die_value);
			  put_on_sseg(die_value);

		  }
		  else{
			  GPIO_TypeDef* ld4_gpio     = GPIOB;
			  uint16_t      ld4_pin_nbr  = 13;
			  uint16_t      ld4_pin      = 0x01 << ld4_pin_nbr;
			  HAL_GPIO_WritePin(ld4_gpio, ld4_pin, GPIO_PIN_RESET);
		  }
	  last_pressed_state = pressed;
	  HAL_Delay(100);

	//  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);


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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SMPS_EN_Pin|SMPS_V1_Pin|SMPS_SW_Pin|DI_A_Pin
                          |DI_B_Pin|DI_C_Pin|DI_D_Pin|DI_E_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD4_Pin|DI_F_Pin|DI_G_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3
                           PC4 PC6 PC7 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SMPS_EN_Pin SMPS_V1_Pin SMPS_SW_Pin DI_A_Pin
                           DI_B_Pin DI_C_Pin DI_D_Pin DI_E_Pin */
  GPIO_InitStruct.Pin = SMPS_EN_Pin|SMPS_V1_Pin|SMPS_SW_Pin|DI_A_Pin
                          |DI_B_Pin|DI_C_Pin|DI_D_Pin|DI_E_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SMPS_PG_Pin */
  GPIO_InitStruct.Pin = SMPS_PG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SMPS_PG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin DI_F_Pin DI_G_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|DI_F_Pin|DI_G_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/*void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == B1_Pin) {
        HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);
        uint8_t die_value = (rand() % 6) + 1;
        put_die_dots(die_value);
    }
}*/
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
