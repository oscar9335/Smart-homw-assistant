/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include"FreeRTOS.h"
#include"task.h"
#include"semphr.h"

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

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
BaseType_t xReturned;
TaskHandle_t xHandle=NULL;

// flags
int bedroom_out = 0;
int bedroom_in = 0;
int livingroom_out = 0;
int livingroom_in = 0;


// there are 2 counter
int number_of_people_livingroom = 0;
int number_of_people_bedroom = 0;
// there are 2 array to store sensor change state
// [out,in, out,in]
int bedroom_state[4] = {0,0,0,0};
int livingroom_state[4] = {0,0,0,0};

int counter_bed = 5;
int counter_living = 5;


int bedroom_btn = 0;


SemaphoreHandle_t xSemaphore;


//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
//void HAL_GPIO_EXTI_IRQHandler(uint16_t GPIO_Pin);

//void Sensor_LED( void );

void Sensor_bedroom_in( void );
void Sensor_bedroom_out( void );
void Sensor_livingroom_in( void );
void Sensor_livingroom_out( void );

void LED_task( void );


void LED_Task( void ){
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);
	for(;;){

		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET );
		vTaskDelay(500);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET );
		vTaskDelay(500);
	}
}

// for UART2
void USART_Test(void *pvParameters){
	uint32_t Monitortimer = 400;
	char MonitorTset[30];
	char num[15];
	int i = 0;
	while(1){
		 memset(MonitorTset,'\0',sizeof(MonitorTset));
		 memset(num,'\0',sizeof(num));
		 itoa(i,num,10);
		 strcat(num," ");
		 sprintf(MonitorTset,"The point is %s\n\r",num);
		 HAL_UART_Transmit(&huart2,(uint8_t *)MonitorTset,strlen(MonitorTset),0xffff);
		 vTaskDelay(Monitortimer);
		 Monitortimer += 1;
		 i += 1;
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
//  xTaskCreate(
//		  USART_Test,
//		  "USART_Test",
//		  128,
//		  NULL,
//		  1,
//		  &xHandle);

//  xTaskCreate(Sensor_bedroom_in,"Sensor_bedroom_in",128,NULL,3,&xHandle);
//  xTaskCreate(Sensor_bedroom_out,"Sensor_bedroom_out",128,NULL,3,&xHandle);
//  xTaskCreate(Sensor_livingroom_in,"Sensor_livingroom_in",128,NULL,3,&xHandle);
//  xTaskCreate(Sensor_livingroom_out,"Sensor_livingroom_out",128,NULL,3,&xHandle);
  xSemaphore = xSemaphoreCreateBinary();

  xTaskCreate(LED_task,"LED_task",128,NULL,1,&xHandle);

  vTaskStartScheduler();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |room_LED_Pin|Livingroom_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : button_Pin */
  GPIO_InitStruct.Pin = button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_WS_Pin */
  GPIO_InitStruct.Pin = I2S3_WS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_WS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_SCK_Pin SPI1_MISO_Pin SPI1_MOSI_Pin */
  GPIO_InitStruct.Pin = SPI1_SCK_Pin|SPI1_MISO_Pin|SPI1_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           room_LED_Pin Livingroom_LED_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |room_LED_Pin|Livingroom_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S3_MCK_Pin I2S3_SCK_Pin I2S3_SD_Pin */
  GPIO_InitStruct.Pin = I2S3_MCK_Pin|I2S3_SCK_Pin|I2S3_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_FS_Pin */
  GPIO_InitStruct.Pin = VBUS_FS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = OTG_FS_ID_Pin|OTG_FS_DM_Pin|OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : bedroom_out_Pin bedroom_in_Pin livingroom_out_Pin livingroom_in_Pin */
  GPIO_InitStruct.Pin = bedroom_out_Pin|bedroom_in_Pin|livingroom_out_Pin|livingroom_in_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : Audio_SCL_Pin Audio_SDA_Pin */
  GPIO_InitStruct.Pin = Audio_SCL_Pin|Audio_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 15, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 15, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 15, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 15, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 15, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */

/**
  * @brief This function handles EXTI line1 interrupt.
  */

void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */
////
  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(button_Pin);

  /* USER CODE BEGIN EXTI0_IRQn 1 */
////
  /* USER CODE END EXTI0_IRQn 1 */
}


void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */
	bedroom_out = 1;
  /* USER CODE END EXTI1_IRQn 0 */

  HAL_GPIO_EXTI_IRQHandler(bedroom_out_Pin);
  /* USER CODE BEGIN EXTI1_IRQn 1 */
  counter_bed = 5;
  /* USER CODE END EXTI1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line2 interrupt.
  */
void EXTI2_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_IRQn 0 */
	bedroom_in = 1;

  /* USER CODE END EXTI2_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(bedroom_in_Pin);
  /* USER CODE BEGIN EXTI2_IRQn 1 */
  counter_bed = 5;
  // Clear the EXTI line 0 pending interrupt flag
//  __HAL_GPIO_EXTI_CLEAR_IT(bedroom_in_Pin);

  /* USER CODE END EXTI2_IRQn 1 */
}

/**
  * @brief This function handles EXTI line3 interrupt.
  */
void EXTI3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI3_IRQn 0 */
	livingroom_out = 1;
  /* USER CODE END EXTI3_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(livingroom_out_Pin);
  /* USER CODE BEGIN EXTI3_IRQn 1 */
  counter_living = 5;
  /* USER CODE END EXTI3_IRQn 1 */
}

void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */
	livingroom_in = 1;
  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(livingroom_in_Pin);

  counter_living = 5;

  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}



void HAL_GPIO_EXTI_IRQHandler(uint16_t GPIO_Pin){
	/* EXTI line interrupt detected */
	if(__HAL_GPIO_EXTI_GET_IT(GPIO_Pin) != RESET)
	{

		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin);

		HAL_GPIO_EXTI_Callback(GPIO_Pin);

	}
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	BaseType_t *taskwoken;
	taskwoken=pdFALSE;

//	char MonitorTset[30];
//	memset(MonitorTset,'\0',sizeof(MonitorTset));


	if(GPIO_Pin == bedroom_in_Pin ||  GPIO_Pin == bedroom_out_Pin){
		// state change bedroom
		bedroom_state[0] = bedroom_state[2];
		bedroom_state[1] = bedroom_state[3];
		bedroom_state[2] = bedroom_out;
		bedroom_state[3] = bedroom_in;

		bedroom_in = 0;
		bedroom_out = 0;
	}
	else if(GPIO_Pin == button_Pin){
		if(bedroom_btn == 0)
			bedroom_btn = 1;
		else
			bedroom_btn = 0;
	}

	else{
		// state change living room
		livingroom_state[0] = livingroom_state[2];
		livingroom_state[1] = livingroom_state[3];
		livingroom_state[2] = livingroom_out;
		livingroom_state[3] = livingroom_in;

		livingroom_in = 0;
		livingroom_out = 0;
	}


	xSemaphoreGiveFromISR(xSemaphore,&taskwoken);
}

void LED_task ( void ){


	for(;;){
		if(counter_bed > 0){
			counter_bed--;
		}
		if(counter_living > 0){
			counter_living--;
		}

		char MonitorTset[30];
		memset(MonitorTset,'\0',sizeof(MonitorTset));


		if( xSemaphore != NULL ){
			if( xSemaphoreTake( xSemaphore, ( TickType_t ) 10  ) == pdTRUE ){

//				sprintf(MonitorTset,"bedroom_state %d %d %d %d\n\r",bedroom_state[0],bedroom_state[1],bedroom_state[2],bedroom_state[3]);
//				HAL_UART_Transmit(&huart2,(uint8_t *)MonitorTset,strlen(MonitorTset),0xffff);
//
//				sprintf(MonitorTset,"livingroom_state %d %d %d %d\n\r",livingroom_state[0],livingroom_state[1],livingroom_state[2],livingroom_state[3]);
//				HAL_UART_Transmit(&huart2,(uint8_t *)MonitorTset,strlen(MonitorTset),0xffff);


				// for the bedroom
				if(bedroom_state[0]== 1 && bedroom_state[1]== 0 && bedroom_state[2]== 0 && bedroom_state[3]== 1){
					number_of_people_bedroom++;
					number_of_people_livingroom--;

					for(int i = 0 ; i < 4 ; ++i){
						bedroom_state[i] = 0;
					}

				}
				else if(bedroom_state[0]== 0 && bedroom_state[1]== 1 && bedroom_state[2]== 1 && bedroom_state[3]== 0){
					number_of_people_bedroom--;
					number_of_people_livingroom++;
					for(int i = 0 ; i < 4 ; ++i){
						bedroom_state[i] = 0;
					}
				}

				// for the living room
				if(livingroom_state[0]== 1 && livingroom_state[1]== 0 && livingroom_state[2]== 0 && livingroom_state[3]== 1){
					number_of_people_livingroom++;

					for(int i = 0 ; i < 4 ; ++i){
						livingroom_state[i] = 0;
					}

				}
				else if(livingroom_state[0]== 0 && livingroom_state[1]== 1 && livingroom_state[2]== 1 && livingroom_state[3]== 0){
					number_of_people_livingroom--;

					for(int i = 0 ; i < 4 ; ++i){
						livingroom_state[i] = 0;
					}
				}

//
//				sprintf(MonitorTset,"number_of_people_bedroom %d\n\r",number_of_people_bedroom);
//				HAL_UART_Transmit(&huart2,(uint8_t *)MonitorTset,strlen(MonitorTset),0xffff);
//
//				sprintf(MonitorTset,"number_of_people_livingroom %d\n\r",number_of_people_livingroom);
//				HAL_UART_Transmit(&huart2,(uint8_t *)MonitorTset,strlen(MonitorTset),0xffff);

				if(bedroom_btn == 0){
					if(number_of_people_bedroom > 0){
						HAL_GPIO_WritePin(GPIOD, room_LED_Pin, GPIO_PIN_SET);
					}
					else{
						number_of_people_bedroom = 0;
						// turn off the bedroom LED
						HAL_GPIO_WritePin(GPIOD, room_LED_Pin, GPIO_PIN_RESET);
					}
				}
				else{
					HAL_GPIO_WritePin(GPIOD, room_LED_Pin, GPIO_PIN_RESET);
				}



				if(number_of_people_livingroom > 0){
					// turn on the bedroom LED
					HAL_GPIO_WritePin(GPIOD, Livingroom_LED_Pin, GPIO_PIN_SET);
				}
				else{
					number_of_people_livingroom = 0;
					// turn off the bedroom LED
					HAL_GPIO_WritePin(GPIOD, Livingroom_LED_Pin, GPIO_PIN_RESET);
				}

				if(counter_bed == 0){
					for(int i = 0 ; i < 4 ; ++i){
						bedroom_state[i] = 0;
					}
				}
				if(counter_living == 0){
					for(int i = 0 ; i < 4 ; ++i){
						livingroom_state[i] = 0;
					}
				}

//				sprintf(MonitorTset,"counter_bed, counter_living %d, %d\n\r",counter_bed,counter_living);
//				HAL_UART_Transmit(&huart2,(uint8_t *)MonitorTset,strlen(MonitorTset),0xffff);
//
//
//
//				sprintf(MonitorTset,"\n\r");
//				HAL_UART_Transmit(&huart2,(uint8_t *)MonitorTset,strlen(MonitorTset),0xffff);
			}
		}

		vTaskDelay(500);
		xSemaphoreGive( xSemaphore );
	}
}


/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
