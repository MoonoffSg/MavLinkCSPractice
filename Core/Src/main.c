/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include<stdio.h>
#include"mavlink.h"
#include<stdint.h>
#include"ring_buffer.h"
#include"common.h"

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
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define RX_BUF_SIZE 256
uint8_t byte_buf[RX_BUF_SIZE];
mavlink_message_t msg;
mavlink_heartbeat_t hb;
mavlink_status_t status;
RingBuf_t buf;



//The best view of data
void uart_send_uint(UART_HandleTypeDef *huart, uint32_t num)
{
    char buf[10];
    int i = 0;

    if (num == 0)
    {
        char c = '0';
        HAL_UART_Transmit(huart, (uint8_t*)&c, 1, 10);
        return;
    }

    while (num > 0)
    {
        buf[i++] = (num % 10) + '0';
        num /= 10;
    }

    for (int j = i - 1; j >= 0; j--)
    {
        HAL_UART_Transmit(huart, (uint8_t*)&buf[j], 1, 10);
    }
}
void print_heartbeat(UART_HandleTypeDef *huart, mavlink_heartbeat_t *hb)
{
    char txt1[] = "SYS:";
    HAL_UART_Transmit(huart, (uint8_t*)txt1, sizeof(txt1)-1, 10);

    uart_send_uint(huart, hb->type);

    char txt2[] = " MODE:";
    HAL_UART_Transmit(huart, (uint8_t*)txt2, sizeof(txt2)-1, 10);

    uart_send_uint(huart, hb->base_mode);

    char txt3[] = " Version:";
        HAL_UART_Transmit(huart, (uint8_t*)txt3, sizeof(txt3)-1, 10);

        uart_send_uint(huart, hb->mavlink_version);
    char txt4[] = " SYSSTAT:";
               HAL_UART_Transmit(huart, (uint8_t*)txt4, sizeof(txt4)-1, 10);

               uart_send_uint(huart, hb->mavlink_version);
    char txt5[] = " CUST_MODE:";
         HAL_UART_Transmit(huart, (uint8_t*)txt5, sizeof(txt5)-1, 10);

         uart_send_uint(huart, hb->custom_mode);
   char txt6[] = " autopilot:";
         HAL_UART_Transmit(huart, (uint8_t*)txt6, sizeof(txt6)-1, 10);

         uart_send_uint(huart, hb->autopilot);

    char nl[] = "\r\n";
    HAL_UART_Transmit(huart, (uint8_t*)nl, sizeof(nl)-1, 10);
}

//MavLink Byte Parse
void process_byte(uint8_t byte)
{

    if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status))
    {

        if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT)
        {
            mavlink_msg_heartbeat_decode(&msg, &hb);
            print_heartbeat(&huart2, &hb);
        }
    }
}


//Interrupt method
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if (huart->Instance == USART1){
		for(int i =0; i<RX_BUF_SIZE;i++)
			process_byte(byte_buf[i]);
		HAL_UART_Receive_IT(&huart1, byte_buf, RX_BUF_SIZE);
	}
}



void sendHeartBeat(mavlink_message_t *msge, UART_HandleTypeDef *huart){
	uint8_t buff[MAVLINK_MAX_PACKET_LEN];
	int len = mavlink_msg_to_send_buffer(buff, msge);
	if(!len) return;
	HAL_UART_Transmit(huart, buff, len,100);
}

void sendOpticalFlow(mavlink_message_t *msge, UART_HandleTypeDef *huart){
	uint8_t buff[MAVLINK_MAX_PACKET_LEN];
	int len = mavlink_msg_to_send_buffer(buff, msge);
	if(!len) return;
	HAL_UART_Transmit(huart, buff, len,100);
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
 // HAL_UART_Receive_IT(&huart1, byte_buf, RX_BUF_SIZE);
  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  mavlink_message_t messageH;
  mavlink_heartbeat_t heartbeat;

  mavlink_message_t messageO;
  mavlink_optical_flow_rad_t Optical;

  uint32_t lasttime = 0;
  uint32_t lasttime10 = 0;
  while (1)
  {
	      uint32_t Time = HAL_GetTick();
	  	  if(Time-lasttime >= 1000){
	  	  lasttime +=1000 ;

		  heartbeat.type = MAV_TYPE_ONBOARD_CONTROLLER;
		  heartbeat.autopilot = MAV_AUTOPILOT_INVALID;
		  heartbeat.base_mode = 0;
		  heartbeat.custom_mode = 0;
		  heartbeat.system_status = MAV_STATE_ACTIVE;
		  heartbeat.mavlink_version = 3;
		  mavlink_msg_heartbeat_encode(255, 0, &messageH, &heartbeat);
		  sendHeartBeat(&messageH, &huart1);

	  	  }

	  	  if(Time-lasttime10 >= 100){
	  		  lasttime10 += 100;
	  		Optical.time_usec = HAL_GetTick() * 1000;
	  		Optical.integration_time_us = 100000;
	  		Optical.integrated_x = 0.02f;
	  		Optical.integrated_y = 0.00f;
	  		Optical.integrated_xgyro = 0.0f;
	  		Optical.integrated_ygyro = 0.0f;
	  		Optical.integrated_zgyro = 0.0f;

	  		Optical.temperature = 25;

	  		Optical.quality = 200;
	  		Optical.distance = 1.5f;
	  		 mavlink_msg_optical_flow_rad_encode(255,0,&messageO,&Optical);
	  		 sendOpticalFlow(&messageO,&huart1);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
#ifdef USE_FULL_ASSERT
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
