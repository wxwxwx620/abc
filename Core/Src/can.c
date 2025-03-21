/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
#include "can.h"

/* USER CODE BEGIN 0 */
#include "modbus_host.h"
#include "remote_control.h"
#include "Alarm.h"

#define   CAN1FIFO   CAN_RX_FIFO0 
#define   CAN2FIFO   CAN_RX_FIFO1
CAN_TxHeaderTypeDef     TxMeg;
CAN_RxHeaderTypeDef     RxMeg;
CAN_RecvMsg RXbufer1;
CAN_RecvMsg RXbufer2;
CAN_TranvMsg TXbufer1;
CAN_TranvMsg TXbufer2;

int32_t CanDatalLossRate; //CAN总线丢包率

extern can_motor_status can_status;
/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

	/* USER CODE BEGIN CAN1_Init 0 */

	/* USER CODE END CAN1_Init 0 */

	/* USER CODE BEGIN CAN1_Init 1 */

	/* USER CODE END CAN1_Init 1 */
	hcan1.Instance = CAN1;
	hcan1.Init.Prescaler = 12;
	hcan1.Init.Mode = CAN_MODE_NORMAL;
	hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan1.Init.TimeSeg1 = CAN_BS1_3TQ;
	hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
	hcan1.Init.TimeTriggeredMode = DISABLE;
	hcan1.Init.AutoBusOff = DISABLE;
	hcan1.Init.AutoWakeUp = DISABLE;
	hcan1.Init.AutoRetransmission = DISABLE;
	hcan1.Init.ReceiveFifoLocked = DISABLE;
	hcan1.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN CAN1_Init 2 */

	/* USER CODE END CAN1_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	if (canHandle->Instance == CAN1)
	{
		/* USER CODE BEGIN CAN1_MspInit 0 */

		/* USER CODE END CAN1_MspInit 0 */
		  /* CAN1 clock enable */
		__HAL_RCC_CAN1_CLK_ENABLE();

		__HAL_RCC_GPIOD_CLK_ENABLE();
		/**CAN1 GPIO Configuration
		PD0     ------> CAN1_RX
		PD1     ------> CAN1_TX
		*/
		GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
		HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

		/* CAN1 interrupt Init */
		HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
		HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
		/* USER CODE BEGIN CAN1_MspInit 1 */

		/* USER CODE END CAN1_MspInit 1 */
	}
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

	if (canHandle->Instance == CAN1)
	{
		/* USER CODE BEGIN CAN1_MspDeInit 0 */

		/* USER CODE END CAN1_MspDeInit 0 */
		  /* Peripheral clock disable */
		__HAL_RCC_CAN1_CLK_DISABLE();

		/**CAN1 GPIO Configuration
		PD0     ------> CAN1_RX
		PD1     ------> CAN1_TX
		*/
		HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0 | GPIO_PIN_1);

		/* CAN1 interrupt Deinit */
		HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
		/* USER CODE BEGIN CAN1_MspDeInit 1 */

		/* USER CODE END CAN1_MspDeInit 1 */
	}
}

/* USER CODE BEGIN 1 */
void CAN_User_Init_Can1(void)
{
	CAN_FilterTypeDef  CAN_FilterInitStructure;
	HAL_StatusTypeDef  HAL_Status;
	/* 设置CAN筛选器序号14 ，CAN2筛选器序号 14--27，而CAN1的是0--13*/
	CAN_FilterInitStructure.FilterBank = 0;
	CAN_FilterInitStructure.FilterMode = CAN_FILTERMODE_IDMASK;//设置过滤器组未屏蔽位模式
	CAN_FilterInitStructure.FilterScale = CAN_FILTERSCALE_32BIT;//过滤器位宽为32位过滤器一个
	CAN_FilterInitStructure.FilterIdHigh = 0x0000;//过滤器标识符高位
	CAN_FilterInitStructure.FilterIdLow = 0x0000;//过滤器标识符低位
	CAN_FilterInitStructure.FilterMaskIdHigh = 0x0000;
	CAN_FilterInitStructure.FilterMaskIdLow = 0x0000;
	CAN_FilterInitStructure.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	CAN_FilterInitStructure.FilterActivation = ENABLE;//激活过滤器
	HAL_Status = HAL_CAN_ConfigFilter(&hcan1, &CAN_FilterInitStructure);

	HAL_Status = HAL_CAN_Start(&hcan1);//开启CAN
	HAL_Status = HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)//接收回调函数
{
	/*
#define INNER_LEFTFRONT 	1  //MotorFL
#define INNER_RIGHTFRONT 	2	 //MotorFR
#define INNER_LEFTBACK  	3  //MotorBL
#define INNER_RIGHTBACK 	4  //MotorBR
#define OUTER_LEFTFRONT 	5  //MotorCFL
#define OUTER_RIGHTFRONT 	6 //MotorCFR
#define OUTER_LEFTBACK 		7 //MotorCBL
#define OUTER_RIGHTBACK 	8 //MotorCBR
*/
	float rpm=0;
	HAL_StatusTypeDef   HAL_RetVal;
	if (hcan == &hcan1) { HAL_RetVal = HAL_CAN_GetRxMessage(&hcan1, CAN1FIFO, &RxMeg, RXbufer1.Data); }
	if (HAL_OK == HAL_RetVal)
	{
		switch (RxMeg.StdId)
		{
		case 0x280 + FRONT_LEFT:           //+280+节点地址
			MotorFL.ACTUAL_RPM = RXbufer1.Data[0] + (RXbufer1.Data[1] * 256) + (RXbufer1.Data[2] * 65536) + (RXbufer1.Data[3] * 16777216);			
			//ILF_x = ILF_x / 512 / 10000 * 1875;
			rpm = MotorFL.ACTUAL_RPM*PULSE_TO_RPM;
			MotorFL.ACTUAL_RPM = rpm;         //实际输出转速
			MotorFL.update++;
			MotorFL.alarm = RXbufer1.Data[4] + (RXbufer1.Data[5] * 256)/*+(RXbufer1.Data[6]*65536)+(RXbufer1.Data[7]*16777216)*/;
			break;
		case 0x280 + FRONT_RIGHT:
			MotorFR.ACTUAL_RPM = RXbufer1.Data[0] + (RXbufer1.Data[1] * 256) + (RXbufer1.Data[2] * 65536) + (RXbufer1.Data[3] * 16777216);
			//IRF_x = MotorFR.ACTUAL_RPM;
			//IRF_x = IRF_x / 512 / 10000 * 1875;
			rpm = MotorFR.ACTUAL_RPM*PULSE_TO_RPM;
			MotorFR.ACTUAL_RPM = rpm;
			MotorFR.update++;
			MotorFR.alarm = RXbufer1.Data[4] + (RXbufer1.Data[5] * 256);
			break;
		case 0x280 + BACK_LEFT:
			MotorBL.ACTUAL_RPM = RXbufer1.Data[0] + (RXbufer1.Data[1] * 256) + (RXbufer1.Data[2] * 65536) + (RXbufer1.Data[3] * 16777216);
			rpm = MotorBL.ACTUAL_RPM*PULSE_TO_RPM;;
			//ILB_x = ILB_x / 512 / 10000 * 1875;
			MotorBL.ACTUAL_RPM = rpm;
			MotorBL.update++;
			MotorBL.alarm = RXbufer1.Data[4] + (RXbufer1.Data[5] * 256);
			break;
		case 0x280 + BACK_RIGHT:
			MotorBR.ACTUAL_RPM = RXbufer1.Data[0] + (RXbufer1.Data[1] * 256) + (RXbufer1.Data[2] * 65536) + (RXbufer1.Data[3] * 16777216);
			rpm = MotorBR.ACTUAL_RPM*PULSE_TO_RPM;;
			//IRB_x = IRB_x / 512 / 10000 * 1875;
			MotorBR.ACTUAL_RPM = rpm;
			MotorBR.update++;
			MotorBR.alarm = RXbufer1.Data[4] + (RXbufer1.Data[5] * 256);
			break;

		case 0x280 + FL_Clip:
			MotorCFL.POSITION = RXbufer1.Data[0] + (RXbufer1.Data[1] * 256) + (RXbufer1.Data[2] * 65536) + (RXbufer1.Data[3] * 16777216);
			rpm = MotorCFL.POSITION*PULSE_TO_RPM;;
			//BJLF_x = BJLF_x / 512 / 10000 * 1875;
			MotorCFL.POSITION = rpm;
			MotorCFL.update++;
			MotorCFL.alarm = RXbufer1.Data[4] + (RXbufer1.Data[5] * 256);
			break;
		case 0x280 + FR_Clip:
			MotorCFR.POSITION = RXbufer1.Data[0] + (RXbufer1.Data[1] * 256) + (RXbufer1.Data[2] * 65536) + (RXbufer1.Data[3] * 16777216);
			rpm = MotorCFR.POSITION*PULSE_TO_RPM;;
			//BJRF_x = BJRF_x / 512 / 10000 * 1875;
			MotorCFR.POSITION = rpm;
			MotorCFR.update++;
			MotorCFR.alarm = RXbufer1.Data[4] + (RXbufer1.Data[5] * 256);
			break;
		case 0x280 + BL_Clip:
			MotorCBL.POSITION = RXbufer1.Data[0] + (RXbufer1.Data[1] * 256) + (RXbufer1.Data[2] * 65536) + (RXbufer1.Data[3] * 16777216);
			rpm = MotorCBL.POSITION*PULSE_TO_RPM;;
			//BJLB_x = BJLB_x / 512 / 10000 * 1875;
			MotorCBL.POSITION = rpm;
			MotorCBL.update++;
			MotorCBL.alarm = RXbufer1.Data[4] + (RXbufer1.Data[5] * 256);
			break;
		case 0x280 + BR_Clip:
			MotorCBR.POSITION = RXbufer1.Data[0] + (RXbufer1.Data[1] * 256) + (RXbufer1.Data[2] * 65536) + (RXbufer1.Data[3] * 16777216);
			rpm = MotorCBR.POSITION*PULSE_TO_RPM;;
			//BJRB_x = BJRB_x / 512 / 10000 * 1875;
			MotorCBR.POSITION = rpm;
			MotorCBR.update++;
			MotorCBR.alarm = RXbufer1.Data[4] + (RXbufer1.Data[5] * 256);
			break;
		}
	}
}

uint8_t Can_Send_Msg_Ext(uint8_t* msg, uint8_t len, uint32_t t_Stdid) //扩展桢发送
{
	uint16_t i = 0;
	CAN_TxHeaderTypeDef TxMessage;
	uint8_t  FreeTxNum = 0;
	FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
	TxMessage.StdId = t_Stdid; // 标准标识符 
	TxMessage.ExtId = 0x118;		// 设置扩展标示符 
	TxMessage.RTR = CAN_RTR_DATA;		// 数据帧						  
	TxMessage.IDE = CAN_ID_EXT;     // 扩展帧                 	
	if (len > 8)	len = 8;
	TxMessage.DLC = len;
	for (i = 0; i < len; i++)	TXbufer1.Data[i] = msg[i];
	while (0 == FreeTxNum)
	{
		FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
	}
	if (HAL_CAN_AddTxMessage(&hcan1, &TxMessage, TXbufer1.Data, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

uint8_t Can_Send_Msg(uint8_t* msg, uint8_t len, uint32_t t_Stdid) //标准桢发送
{
	uint16_t i = 0;
	CAN_TxHeaderTypeDef TxMessage;
	uint8_t  FreeTxNum = 0;
	FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
	TxMessage.StdId = t_Stdid + 0x600; // 标准标识	
	TxMessage.RTR = CAN_RTR_DATA;		// 数据帧						  
	TxMessage.IDE = CAN_ID_STD;     //标准桢                	
	if (len > 8)	len = 8;
	TxMessage.DLC = len;
	for (i = 0; i < len; i++)	TXbufer2.Data[i] = msg[i];
	while (0 == FreeTxNum)
	{
		FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
	}
	if (HAL_CAN_AddTxMessage(&hcan1, &TxMessage, TXbufer2.Data, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

uint8_t Can_Send_Msg_RPDO(uint8_t* msg, uint8_t len, uint32_t t_Stdid) //标准桢发送
{
	uint16_t i = 0;
	CAN_TxHeaderTypeDef TxMessage;
	uint8_t  FreeTxNum = 0;
	FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
	TxMessage.StdId = t_Stdid + 0x200; // 标准标识	
	TxMessage.RTR = CAN_RTR_DATA;		// 数据帧						  
	TxMessage.IDE = CAN_ID_STD;     //标准桢                	
	if (len > 8)	len = 8;
	TxMessage.DLC = len;
	for (i = 0; i < len; i++)	TXbufer2.Data[i] = msg[i];
	while (0 == FreeTxNum) {
		FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
	}
	if (HAL_CAN_AddTxMessage(&hcan1, &TxMessage, TXbufer2.Data, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

/* USER CODE END 1 */
