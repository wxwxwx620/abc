/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __Protocol_H
#define __Protocol_H
#ifdef __cplusplus
extern "C" {
#endif

    /* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"
#include "usart.h"
#include "tim.h"
#include "motor.h"
#include <string.h>
    /* USER CODE BEGIN Includes */
#include "modbus_host.h"
#include "voice.h"
    /* USER CODE END Includes */

    /* USER CODE BEGIN Private defines */
#define RC_STA USART1_STA
#define PC_STA USART2_STA
	
#define LEN_PC_2_AGV 		18		//定义PCtoAGV串口数据包长度
#define	LEN_AGV_2_PC		31		//定义AGVtoPC串口数据包长度
	
//#define EC_STA USART3_STA	
    /* USER CODE END Private defines */
	
/* USER CODE BEGIN Prototypes */
 void rc_get_srcdata(uint8_t* temp);
 void pc_get_srcdata(uint8_t* temp);

//extern void ec_get_srcdata(uint8_t* temp);

extern void RC_Receive_handler(UART_HandleTypeDef *huart);
extern void PC_Receive_handler(UART_HandleTypeDef *huart);
//extern void EC_Receive_handler(UART_HandleTypeDef *huart);
extern void Debug_Communication(USART_STA *usart_struct);

void Usart2Send_PC(void);
void Usart1Send_RC(void);
void Usart5Send_RC(void);
//发送数据给上位机
void SendToPC(void);
void pc_clear_srcdata(void);
void rc_clear_srcdata(void);
//extern USART_STA UART5_STA;
    /* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif

/**
  * @}
  */

/**
  * @}
  */
