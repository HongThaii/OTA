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
#include <string.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>
#include "flash.h"

#define ACK	0
#define NACK 1
#define MAX_DATA 128
#define ADDR_APP1_PROGRAM 0x8005000
#define ADDR_APP1_PROGRAM_SIZE 53
#define ADDR_APP2_PROGRAM (ADDR_APP1_PROGRAM + ADDR_APP1_PROGRAM_SIZE*1024) //0x8012400
#define ADDR_COMMON_ADDR 0x801F800

uint8_t uart_buff[MAX_DATA];
uint16_t uart_index;
bool flag = 1;
uint32_t ota_addr_start = ADDR_APP2_PROGRAM;

enum
{
    START_SEND_FILE = 0x01,
    SEND_DATA = 0x02,
    SEND_DONE = 0x03,
    SEND_ACK = 0x04,
};
enum
{
	NORMAL_RESET,
	OTA_REQUEST_RESET
};
enum
{
	APP1_CURRENT,
	APP2_CURRENT
};
typedef struct 
{
	uint8_t used1;
	uint8_t used2;
	uint8_t reset_cause;
	uint8_t app_current;
} ota_infor_t;
ota_infor_t ota_infor;

//=================================== my_printf ==============================

void my_printf(const char* str, ...)
{
	char stringArray[100];
	
  va_list args;
  va_start(args, str);
  uint8_t len_str = vsprintf(stringArray, str, args);
  va_end(args);
	
	HAL_UART_Transmit(&huart1,(uint8_t*)stringArray, len_str, 100);
}

//=================================== console ================================

void console(uint8_t *buff, uint8_t len)
{
  for(uint8_t i=0; i<len; i++)
	{
    my_printf("%02x ", buff[i]);
  }
}

//=================================== checksum ===============================

uint8_t checksum(uint8_t *buff, uint8_t len)
{
    uint8_t sum=0;
    for(uint8_t i=0;i<len;i++)
		{
       sum += buff[i];
    }
    return sum;
}

//=================================== HAL_UARTEx_RxEventCallback =============

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  flag = 1;
  uart_index = Size;
  HAL_UARTEx_ReceiveToIdle_IT(&huart2,uart_buff,MAX_DATA);
}

//=================================== uart_send_data =========================

void uart_send_data(uint8_t *data, uint8_t len)
{
  HAL_UART_Transmit(&huart2,data,len,100);
}

//=================================== stm_send_ack ===========================

void stm_send_ack(bool ack)
{
	uint8_t buff[4];
	buff[0] = SEND_ACK;
	buff[1] = 1;
	buff[2] = ack;
	buff[3] = checksum(buff, 3);
	uart_send_data(buff, 4);
}

//=================================== receive_data_process ===================

void receive_data_process(uint8_t *data, uint8_t len)
{
  uint8_t check;
  if(len >= 3)
	{
    my_printf("receive_data_process:len= %d\n", len);
    console(data,len);
    my_printf("\n");
		my_printf("len: %d\n", data[1]);
    check = checksum(data,len-1);
    if(check == data[len-1])
		{
      my_printf("checksum: ok\n");
			my_printf("cmd: %x\n",data[0]);
      switch(data[0])
			{
        case START_SEND_FILE:
				{
					stm_send_ack(ACK);
					flash_unlock();
          break;
				}
        case SEND_DATA:
				{
					uint8_t len = data[1];
					flash_write_arr(ota_addr_start, &data[2], len);
					ota_addr_start += len;
          stm_send_ack(ACK);
          break;
				}
        case SEND_DONE:
				{
					ota_infor.app_current = (ota_infor.app_current = APP1_CURRENT)? APP2_CURRENT:APP1_CURRENT;
					ota_infor.reset_cause = NORMAL_RESET;
					flash_erase(1, ADDR_COMMON_ADDR);
					flash_write_arr(ADDR_COMMON_ADDR, &ota_infor, sizeof(ota_infor));
					flash_lock();
          my_printf("SEND_DONE_CMD\n");
					ota_addr_start = ADDR_APP2_PROGRAM;
					NVIC_SystemReset();
          break;
				}
      }
    }
    else
		{
			my_printf("checksum: err\n");
			stm_send_ack(NACK);
		}
  }
}

//=====================================Run_app_program==============================================

typedef void(*run_app_handle)(void);
run_app_handle run_app;
void run_app_program(uint32_t addr)
{
	HAL_RCC_DeInit();
	HAL_DeInit();
	SCB->SHCSR &= ~(SCB_SHCSR_USGFAULTENA_Msk |
									SCB_SHCSR_BUSFAULTENA_Msk |
									SCB_SHCSR_MEMFAULTENA_Msk ) ; 
	__set_MSP(*((volatile uint32_t*) addr)); 
	run_app = (run_app_handle)*((volatile uint32_t*) (addr + 4));
	run_app();
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
	flash_read_arr(ADDR_COMMON_ADDR, &ota_infor, sizeof(ota_infor));
	
	if((ota_infor.used1 != 0x55) || (ota_infor.used2 != 0xaa))
	{
		my_printf("Boot: First\n");
		ota_infor.used1 = 0x55;
		ota_infor.used2 = 0xaa;
		ota_infor.app_current = APP1_CURRENT;
		ota_infor.reset_cause = NORMAL_RESET;
		flash_unlock();
		flash_erase(1, ADDR_COMMON_ADDR);
		flash_write_arr(ADDR_COMMON_ADDR, &ota_infor, sizeof(ota_infor));
		flash_lock();
	}
	else
	{
		my_printf("Boot: Not First\n");
	}
//	ota_infor.app_current = APP1_CURRENT;
//	flash_unlock();
//	flash_erase(1, ADDR_COMMON_ADDR);
//	flash_write_arr(ADDR_COMMON_ADDR, &ota_infor, sizeof(ota_infor));
//	flash_lock();
	switch(ota_infor.reset_cause)
	{
		case NORMAL_RESET:
		{
			my_printf("Boot: NORMAL_RESET\n");
			my_printf("Boot: Run_app_program: App: %d, Addr: %x\n", ota_infor.app_current,
								ota_infor.app_current == APP1_CURRENT? ADDR_APP1_PROGRAM:ADDR_APP2_PROGRAM);
			if(ota_infor.app_current == APP1_CURRENT)
			{
					run_app_program(ADDR_APP1_PROGRAM);

			}
			else
			{
					run_app_program(ADDR_APP2_PROGRAM);
			}
			break;
		}
		case OTA_REQUEST_RESET:
		{
			my_printf("Boot: OTA_REQUEST_RESET\n");
			ota_infor.reset_cause = NORMAL_RESET;
			flash_unlock();
			flash_erase(1, ADDR_COMMON_ADDR);
			flash_write_arr(ADDR_COMMON_ADDR, &ota_infor, sizeof(ota_infor));
			flash_lock();
			break;
		}
	}
	my_printf("Boot: Program Bootloader Start!\n");
	
  HAL_UARTEx_ReceiveToIdle_IT(&huart2,uart_buff,MAX_DATA);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if(flag == 1)
		{
     receive_data_process(uart_buff, uart_index);
     flag = 0;
    }
//		 HAL_UARTEx_ReceiveToIdle_IT(&huart2,uart_buff,MAX_DATA);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

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
