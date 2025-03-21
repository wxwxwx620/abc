/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
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
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN Private defines */
//typedef struct{
//	CAN_RxHeaderTypeDef m;
//}CANOpen_Message;
typedef struct
{
        uint8_t Data[8];
}CAN_RecvMsg;
extern CAN_RecvMsg RXbufer1;
extern CAN_RecvMsg RXbufer2;
typedef struct
{
        uint8_t Data[8];
}CAN_TranvMsg;
extern CAN_TranvMsg TXbufer1;
extern CAN_TranvMsg TXbufer2;
/* USER CODE END Private defines */

void MX_CAN1_Init(void);

/* USER CODE BEGIN Prototypes */
void CAN_User_Init_Can1(void);
extern uint8_t Can_Send_Msg(uint8_t* msg,uint8_t len,uint32_t t_Stdid);
extern uint8_t Can_Send_Msg_Ext(uint8_t* msg,uint8_t len,uint32_t t_Stdid);
extern uint8_t Can_Send_Msg_RPDO(uint8_t* msg, uint8_t len, uint32_t t_Stdid); //标准桢发送
extern int32_t CanDatalLossRate; //CAN总线丢包率

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

