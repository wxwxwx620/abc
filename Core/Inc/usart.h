/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "cmsis_os.h"
#include "motor.h"
#include "parameter.h"	 
#include "datacheck.h"
#include "modbus_host.h"
/* USER CODE END Includes */

extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern uint8_t temp_str[512];
/* USER CODE BEGIN Private defines */
#define RX_URB_Size 128	
#define TX_URB_Size 128
#define RX_RC_Size 36	
#define RX_PC_Size 36	

typedef struct
{
	uint8_t RX_FLAG;
	uint8_t TX_FLAG;	
	
	uint8_t RX_LEN;
	uint8_t RX_BUF[RX_URB_Size]; 
		
	uint8_t TX_LEN;
	uint8_t TX_BUF[TX_URB_Size]; 
	//yao start
	uint8_t TxCount;	
	uint8_t RxCount;
	uint8_t fAck01H;		/* 应答命令标志 0 表示执行失败 1表示执行成功 */
	uint8_t fAck02H;
	uint8_t fAck03H;
	uint8_t fAck04H;
	uint8_t fAck05H;		
	uint8_t fAck06H;		
	uint8_t fAck10H;
	
	uint16_t Reg01H;		/* 保存主机发送的寄存器首地址 */
	uint16_t Reg02H;
	uint16_t Reg03H;		
	uint16_t Reg04H;

	uint8_t RegNum;			/* 寄存器个数 */
	uint8_t Voice_RX;	
	//yao end
}USART_STA;	

typedef struct
{	
		uint16_t address;//地址
		uint16_t data;//数据	
} UI_Funtion_STA;

extern UI_Funtion_STA UI_STA;
extern USART_STA USART1_STA;	 
extern USART_STA USART2_STA;
extern USART_STA USART3_STA;
extern USART_STA UART4_STA;
extern USART_STA UART5_STA;
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
 

extern uint8_t Lift_Command; //yao
extern uint8_t Stat_Lift_Command; //yao
extern uint8_t scram_alarm_filter;//yao
extern uint8_t BMS_Relay;
extern uint16_t pc_updateCount;//yao
extern uint16_t BMS_time_out;//yao
extern uint16_t BMS_time_out_Flag;//yao
extern uint16_t rc_updateCount;
extern uint8_t Message1_Flag;
extern uint8_t Message1_Flag1;
/* USER CODE END Private defines */

void MX_UART4_Init(void);
void MX_UART5_Init(void);
void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_USART3_UART_Init(void);

/* USER CODE BEGIN Prototypes */
extern void UsartSendData_DMA(uint8_t channel,uint8_t *pdata,uint16_t Length);
extern void RC_Receive_IDLE(UART_HandleTypeDef *huart);
extern void Usart2Receive_IDLE(void);
extern void Usart3Receive_IDLE(void);
extern void Usart5Receive_IDLE(void);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

