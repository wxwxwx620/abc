/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
  /* USER CODE END Header */

  /* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"	
#include "modbus_host.h"
#include "voice.h"
#include "Protocol.h"
#include "74HC595.h"
#include "74HC597.h"
#include "encoder.h"
#include "BMS.h"
#include "Alarm.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim10;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart4_tx;
extern DMA_HandleTypeDef hdma_uart5_rx;
extern DMA_HandleTypeDef hdma_uart5_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim6;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
	/* USER CODE BEGIN NonMaskableInt_IRQn 0 */

	/* USER CODE END NonMaskableInt_IRQn 0 */
	/* USER CODE BEGIN NonMaskableInt_IRQn 1 */
	while (1)
	{
	}
	/* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
	/* USER CODE BEGIN HardFault_IRQn 0 */

	/* USER CODE END HardFault_IRQn 0 */
	while (1)
	{
		/* USER CODE BEGIN W1_HardFault_IRQn 0 */
		/* USER CODE END W1_HardFault_IRQn 0 */
	}
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
	/* USER CODE BEGIN MemoryManagement_IRQn 0 */

	/* USER CODE END MemoryManagement_IRQn 0 */
	while (1)
	{
		/* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
		/* USER CODE END W1_MemoryManagement_IRQn 0 */
	}
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
	/* USER CODE BEGIN BusFault_IRQn 0 */

	/* USER CODE END BusFault_IRQn 0 */
	while (1)
	{
		/* USER CODE BEGIN W1_BusFault_IRQn 0 */
		/* USER CODE END W1_BusFault_IRQn 0 */
	}
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
	/* USER CODE BEGIN UsageFault_IRQn 0 */

	/* USER CODE END UsageFault_IRQn 0 */
	while (1)
	{
		/* USER CODE BEGIN W1_UsageFault_IRQn 0 */
		/* USER CODE END W1_UsageFault_IRQn 0 */
	}
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
	/* USER CODE BEGIN DebugMonitor_IRQn 0 */

	/* USER CODE END DebugMonitor_IRQn 0 */
	/* USER CODE BEGIN DebugMonitor_IRQn 1 */

	/* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream0 global interrupt.
  */
void DMA1_Stream0_IRQHandler(void)
{
	/* USER CODE BEGIN DMA1_Stream0_IRQn 0 */

	/* USER CODE END DMA1_Stream0_IRQn 0 */
	HAL_DMA_IRQHandler(&hdma_uart5_rx);
	/* USER CODE BEGIN DMA1_Stream0_IRQn 1 */

	/* USER CODE END DMA1_Stream0_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream1 global interrupt.
  */
void DMA1_Stream1_IRQHandler(void)
{
	/* USER CODE BEGIN DMA1_Stream1_IRQn 0 */

	/* USER CODE END DMA1_Stream1_IRQn 0 */
	HAL_DMA_IRQHandler(&hdma_usart3_rx);
	/* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

	/* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream2 global interrupt.
  */
void DMA1_Stream2_IRQHandler(void)
{
	/* USER CODE BEGIN DMA1_Stream2_IRQn 0 */

	/* USER CODE END DMA1_Stream2_IRQn 0 */
	HAL_DMA_IRQHandler(&hdma_uart4_rx);
	/* USER CODE BEGIN DMA1_Stream2_IRQn 1 */

	/* USER CODE END DMA1_Stream2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream3 global interrupt.
  */
void DMA1_Stream3_IRQHandler(void)
{
	/* USER CODE BEGIN DMA1_Stream3_IRQn 0 */

	/* USER CODE END DMA1_Stream3_IRQn 0 */
	HAL_DMA_IRQHandler(&hdma_usart3_tx);
	/* USER CODE BEGIN DMA1_Stream3_IRQn 1 */

	/* USER CODE END DMA1_Stream3_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream4 global interrupt.
  */
void DMA1_Stream4_IRQHandler(void)
{
	/* USER CODE BEGIN DMA1_Stream4_IRQn 0 */

	/* USER CODE END DMA1_Stream4_IRQn 0 */
	HAL_DMA_IRQHandler(&hdma_uart4_tx);
	/* USER CODE BEGIN DMA1_Stream4_IRQn 1 */

	/* USER CODE END DMA1_Stream4_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream5 global interrupt.
  */
void DMA1_Stream5_IRQHandler(void)
{
	/* USER CODE BEGIN DMA1_Stream5_IRQn 0 */
	PC_Receive_handler(&huart2);
	/* USER CODE END DMA1_Stream5_IRQn 0 */
	HAL_DMA_IRQHandler(&hdma_usart2_rx);
	/* USER CODE BEGIN DMA1_Stream5_IRQn 1 */

	/* USER CODE END DMA1_Stream5_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream6 global interrupt.
  */
void DMA1_Stream6_IRQHandler(void)
{
	/* USER CODE BEGIN DMA1_Stream6_IRQn 0 */

	/* USER CODE END DMA1_Stream6_IRQn 0 */
	HAL_DMA_IRQHandler(&hdma_usart2_tx);
	/* USER CODE BEGIN DMA1_Stream6_IRQn 1 */

	/* USER CODE END DMA1_Stream6_IRQn 1 */
}

/**
  * @brief This function handles CAN1 RX0 interrupts.
  */
void CAN1_RX0_IRQHandler(void)
{
	/* USER CODE BEGIN CAN1_RX0_IRQn 0 */

	/* USER CODE END CAN1_RX0_IRQn 0 */
	HAL_CAN_IRQHandler(&hcan1);
	/* USER CODE BEGIN CAN1_RX0_IRQn 1 */

	/* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
  */
int16_t Motor_Alarm_Timcnt = 0;//电机告警定时计数 20221008
uint8_t Count595=4;
void TIM1_UP_TIM10_IRQHandler(void)
{
	/* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */

	if(Count595==4)
	{
		HC595_updata();
	    exi_read();
		Count595=0;
	}
	Count595++;
	

	Motor_Alarm_Timcnt++;
	if (Motor_Alarm_Timcnt > 60000)//定时10分钟
	{
		Motor_Alarm_Timcnt = 0;
		if (Motor_Alarm_cnt)
		{
			Motor_Alarm_cnt--;//电机告警计数减一 20221008	
		}
	}

	/* USER CODE END TIM1_UP_TIM10_IRQn 0 */
	HAL_TIM_IRQHandler(&htim10);
	/* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */

	/* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
	/* USER CODE BEGIN USART1_IRQn 0 */

	/* USER CODE END USART1_IRQn 0 */
	HAL_UART_IRQHandler(&huart1);
	/* USER CODE BEGIN USART1_IRQn 1 */
	//RC_Receive_IDLE(&huart1);
	/* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
	/* USER CODE BEGIN USART2_IRQn 0 */
	//Usart2Receive_IDLE();
	/* USER CODE END USART2_IRQn 0 */
	HAL_UART_IRQHandler(&huart2);
	/* USER CODE BEGIN USART2_IRQn 1 */

	/* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
	/* USER CODE BEGIN USART3_IRQn 0 */
  //  Usart3Receive_IDLE();
	/* USER CODE END USART3_IRQn 0 */
	HAL_UART_IRQHandler(&huart3);
	/* USER CODE BEGIN USART3_IRQn 1 */
	Usart3Receive_encoder_IDLE();             //串口3接收函数编码器
  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream7 global interrupt.
  */
void DMA1_Stream7_IRQHandler(void)
{
	/* USER CODE BEGIN DMA1_Stream7_IRQn 0 */

	/* USER CODE END DMA1_Stream7_IRQn 0 */
	HAL_DMA_IRQHandler(&hdma_uart5_tx);
	/* USER CODE BEGIN DMA1_Stream7_IRQn 1 */

	/* USER CODE END DMA1_Stream7_IRQn 1 */
}

/**
  * @brief This function handles UART4 global interrupt.
  */
void UART4_IRQHandler(void)
{
	/* USER CODE BEGIN UART4_IRQn 0 */

	/* USER CODE END UART4_IRQn 0 */
	HAL_UART_IRQHandler(&huart4);
	/* USER CODE BEGIN UART4_IRQn 1 */
  //  MODH_Poll();
	Usart4Receive_BMS_IDLE();
	/* USER CODE END UART4_IRQn 1 */
}

/**
  * @brief This function handles UART5 global interrupt.
  */
void UART5_IRQHandler(void)
{
	/* USER CODE BEGIN UART5_IRQn 0 */

	/* USER CODE END UART5_IRQn 0 */
	HAL_UART_IRQHandler(&huart5);
	/* USER CODE BEGIN UART5_IRQn 1 */
  //  Usart5Receive_IDLE();
	/* USER CODE END UART5_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
	/* USER CODE BEGIN TIM6_DAC_IRQn 0 */

	/* USER CODE END TIM6_DAC_IRQn 0 */
	HAL_TIM_IRQHandler(&htim6);
	/* USER CODE BEGIN TIM6_DAC_IRQn 1 */

	/* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
	/* USER CODE BEGIN TIM7_IRQn 0 */
	static uint16_t cnt;
	cnt++;
	if (!(cnt % 500))
	{
		//		if(!get_exi(IN_24))
		//		{
		//			HC595_add(OUT_1);
		//		}
		//		else 
		//		{
		//			HC595_del(OUT_1);
		//		}
		//		
		//		HAL_GPIO_TogglePin(RUN_GPIO_Port, RUN_Pin);
	}
	else
	{

	}

	/* USER CODE END TIM7_IRQn 0 */
	HAL_TIM_IRQHandler(&htim7);
	/* USER CODE BEGIN TIM7_IRQn 1 */

	/* USER CODE END TIM7_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream2 global interrupt.
  */
void DMA2_Stream2_IRQHandler(void)
{
	/* USER CODE BEGIN DMA2_Stream2_IRQn 0 */

	RC_Receive_handler(&huart1);

	/* USER CODE END DMA2_Stream2_IRQn 0 */
	HAL_DMA_IRQHandler(&hdma_usart1_rx);
	/* USER CODE BEGIN DMA2_Stream2_IRQn 1 */

	/* USER CODE END DMA2_Stream2_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream7 global interrupt.
  */
void DMA2_Stream7_IRQHandler(void)
{
	/* USER CODE BEGIN DMA2_Stream7_IRQn 0 */

	/* USER CODE END DMA2_Stream7_IRQn 0 */
	HAL_DMA_IRQHandler(&hdma_usart1_tx);
	/* USER CODE BEGIN DMA2_Stream7_IRQn 1 */

	/* USER CODE END DMA2_Stream7_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

