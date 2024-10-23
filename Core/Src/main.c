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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdlib.h"
#include "string.h"
#include "MFRC522.h"
#include "stdio.h"
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
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//uint8_t status;
//uint8_t str[MAX_LEN];
//uint8_t sNum[5];
//uint8_t tx_buffer[] = "Is New Card!";
//uint8_t keyA[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
uint8_t data_byte_receive[1];
uint8_t ESP_Response[100];

char buffer[50]="";
char SSID[]="\"v\"";
char Pass[]="\"ccthanh123\"";

int count_data_come = 0;
int command_size = 0;
int Flag_Response = 0;
int idx = 0;
int client_list[5]= {0,0,0,0,0};
char* temp = NULL;

void SendCommand(char* str){
	command_size = strlen(str);
	if(HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str),HAL_MAX_DELAY)==HAL_OK){
		HAL_UART_Receive_IT(&huart1, data_byte_receive,1);
	}
}
void WaitForResponse(uint32_t timeout,char* OKE_response,char* Error_response){
	uint32_t tickStart = HAL_GetTick();
	while(Flag_Response == 0){
		if(HAL_GetTick()-tickStart > timeout){
			HAL_UART_Transmit(&huart3,ESP_Response,strlen((char*)ESP_Response),HAL_MAX_DELAY);
			count_data_come = 0;
			idx=0;
//			memset(ESP_Response,0,sizeof(ESP_Response));
			return;
		}
		if(strstr((char*)ESP_Response,OKE_response) != NULL){
			Flag_Response = 1;
			count_data_come = 0;
		}
		else if(strstr((char*)ESP_Response,Error_response) != NULL){ //strcmp((char*)ESP_Response,(char*)ERROR_response)
			Flag_Response = 2;
			count_data_come = 0;
		}
		if(temp==NULL) temp = strstr((char*)ESP_Response,"CIPSTATE:");
		else {
			client_list[(*(temp+9))-48] = 1;
		}
	}
	idx=0;
//	memset(ESP_Response,0,sizeof(ESP_Response));
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle){
	if(Flag_Response==0)HAL_UART_Receive_IT(&huart1, data_byte_receive,1);
	count_data_come++;
	if(count_data_come > command_size){
		ESP_Response[idx++] = data_byte_receive[0];
		if(idx>=100||data_byte_receive[0]=='+'){
			idx = 0;
			temp=NULL;
		}
	}
}
void ESP32_Init(){
	SendCommand("AT+RESTORE\r\n");
	WaitForResponse(5000,"ready","ERROR\r\n");
	if(Flag_Response==1){
		HAL_UART_Transmit(&huart3,(uint8_t*)"AT CHECK OKE\r",sizeof("AT CHECK OKE\r"),HAL_MAX_DELAY);
	}
	else if(Flag_Response==2){
		HAL_UART_Transmit(&huart3,(uint8_t*)"AT CHECK ERROR\r",sizeof("AT CHECK ERROR\r"),HAL_MAX_DELAY);
	}
	memset(ESP_Response,0,sizeof(ESP_Response));

	Flag_Response = 0;
	SendCommand("AT\r\n");
	WaitForResponse(5000,"OK\r\n","ERROR\r\n");
	if(Flag_Response==1){
		HAL_UART_Transmit(&huart3,(uint8_t*)"AT CHECK OKE\r",sizeof("AT CHECK OKE\r"),HAL_MAX_DELAY);
	}
	else if(Flag_Response==2){
		HAL_UART_Transmit(&huart3,(uint8_t*)"AT CHECK ERROR\r",sizeof("AT CHECK ERROR\r"),HAL_MAX_DELAY);
	}
	memset(ESP_Response,0,sizeof(ESP_Response));

	Flag_Response = 0;
	SendCommand("AT+CWMODE=1\r\n");
	WaitForResponse(5000,"OK\r\n","ERROR\r\n");
	if(Flag_Response==1){
		HAL_UART_Transmit(&huart3,(uint8_t*)"AT CWMODE OKE\r",sizeof("AT CWMODE OKE\r"),HAL_MAX_DELAY);
	}
	else if(Flag_Response==2){
		HAL_UART_Transmit(&huart3,(uint8_t*)"AT CWMODE ERROR\r",sizeof("AT CWMODE ERROR\r"),HAL_MAX_DELAY);
	}
	memset(ESP_Response,0,sizeof(ESP_Response));

	Flag_Response = 0;
	sprintf(buffer,"AT+CWJAP=%s,%s\r\n",SSID,Pass);
	SendCommand(buffer);
	memset(buffer,0,sizeof(buffer));
	WaitForResponse(10000,"OK","ERROR");
	if(Flag_Response==1){
		HAL_UART_Transmit(&huart3,(uint8_t*)"AT CWJAP OKE\r",sizeof("AT CWJAP OKE\r"),HAL_MAX_DELAY);
	}
	else if(Flag_Response==2){
		HAL_UART_Transmit(&huart3,(uint8_t*)"AT CWJAP ERROR\r",sizeof("AT CWJAP ERROR\r"),HAL_MAX_DELAY);
	}
	memset(ESP_Response,0,sizeof(ESP_Response));

	Flag_Response = 0;
	SendCommand("AT+CIPMUX=1\r\n");
	WaitForResponse(5000,"OK\r\n","ERROR\r\n");
	if(Flag_Response==1){
		HAL_UART_Transmit(&huart3,(uint8_t*)"AT CIPMUX OKE\r",sizeof("AT CIPMUX OKE\r"),HAL_MAX_DELAY);
		}
	else if(Flag_Response==2){
		HAL_UART_Transmit(&huart3,(uint8_t*)"AT CIPMUX ERROR\r",sizeof("AT CIPMUX ERROR\r"),HAL_MAX_DELAY);
	}
	memset(ESP_Response,0,sizeof(ESP_Response));

	Flag_Response = 0;
	SendCommand("AT+CIPSERVER=1,80\r\n");
	WaitForResponse(5000,"OK\r\n","ERROR\r\n");
	if(Flag_Response==1){
		HAL_UART_Transmit(&huart3,(uint8_t*)"AT CIPSERVER OKE\r",sizeof("AT CIPSERVER OKE\r"),HAL_MAX_DELAY);
	}
	else if(Flag_Response==2){
		HAL_UART_Transmit(&huart3,(uint8_t*)"AT CIPSERVER ERROR\r",sizeof("AT CIPSERVER ERROR\r"),HAL_MAX_DELAY);
	}
	memset(ESP_Response,0,sizeof(ESP_Response));
}
void Server_Send();
void Server_Handle(){}
void Server_On(){
	Flag_Response=0;
	SendCommand("AT+CIPSTATE?\r\n");
	WaitForResponse(5000,"OK\r\n", "ERROR\r\n");
	memset(ESP_Response,0,sizeof(ESP_Response));
	for(int i=0;i<5;i++){
		if(*(client_list+i)==1){
			Flag_Response=0;
			sprintf("AT+CIPSEND=%d,4\r\n",i);
			SendCommand(buffer);
			WaitForResponse(5000,"OK\r\n>", "ERROR\r\n");
			memset(ESP_Response,0,sizeof(ESP_Response));
			if(Flag_Response==1){
				HAL_UART_Transmit(&huart1, (uint8_t*)"hehe", 4, HAL_MAX_DELAY);
			}
			else{
				HAL_UART_Transmit(&huart3, (uint8_t*)"BUG", 3, HAL_MAX_DELAY);
			}
		}
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
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  MFRC522_Init();
  ESP32_Init();
  //HAL_TIM_OnePulse_Start(&htim2, TIM_CHANNEL_2);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 63999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim2, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_TI2FP2;
  sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_RISING;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|LD2_Pin|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 LD2_Pin PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|LD2_Pin|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
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
