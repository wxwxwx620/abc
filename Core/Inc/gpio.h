/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.h
  * @brief   This file contains all the function prototypes for
  *          the gpio.c file
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
#ifndef __GPIO_H__
#define __GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "parameter.h"
/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
//yao START
#define LF_LXD PBout(8)
#define RF_LXD PEout(9)
#define LR_LXD PBout(10)
#define RR_LXD PAout(0)	 
	 
//#define D8 PBout(5)
//#define D9 PBout(4)
//#define D10 PBout(3)
//#define D11 PAout(15)
     
#define BRAKE_EN PEout(5)			

#define MOTORPOWER PBout(14)
#define HOIST_DOWN PCout(3)

#define DIR3			 PBout(15)
#define DIR4			 PDout(3)

#define ES         PEin(2)

#define LR_AK PBout(5)

////���ߵ������,����ĳ�SPI���߶�ȡ
//#define Motor_Alarm_FL PAin(3)
//#define Motor_Alarm_FR PAin(4)
//#define Motor_Alarm_BR PAin(5)
//#define Motor_Alarm_BL PAin(6)
//#define Motor_Alarm_Clip_FL PBin(0)
//#define Motor_Alarm_Clip_FR PBin(1)
//#define Motor_Alarm_Clip_BR PCin(0)
//#define Motor_Alarm_Clip_BL PCin(2)
//��������
#define Motor_Alarm9 PCin(6)
#define Motor_Alarm10 PCin(7)
#define Motor_Alarm11 PDin(12)
#define Motor_Alarm12 PDin(13)

////���п�����λ 4��
//#define OF  PEin(7)  //���ϣ�ǰ���� ����λ   0=��⵽  1=δ��⵽
//#define CF	PEin(8)  //���£�ǰ���� ����λ   0=��⵽  1=δ��⵽
//#define OB	PEin(10) //���£��󱧼� ����λ   0=��⵽  1=δ��⵽
//#define CB  PEin(11) //���ϣ��󱧼� ����λ   0=��⵽  1=δ��⵽

//#define EmergencyStopCar PEin(12) //�����ϵĽ���ֹͣ��ť

//���п�����λ 8��
#define FLO  PEin(7)   //��ǰ���� ����λ   0=��⵽  1=δ��⵽
#define FLC	 PEin(8)   //��ǰ���� ����λ   0=��⵽  1=δ��⵽
#define FRO  PEin(12)  //��ǰ���� ����λ   0=��⵽  1=δ��⵽
#define FRC	 PEin(13)  //��ǰ���� ����λ   0=��⵽  1=δ��⵽
#define BRO	 PEin(10)  //�Һ󱧼� ����λ   0=��⵽  1=δ��⵽
#define BRC  PEin(11)  //�Һ󱧼� ����λ   0=��⵽  1=δ��⵽
#define BLO	 PEin(14)  //��󱧼� ����λ   0=��⵽  1=δ��⵽
#define BLC  PEin(15)  //��󱧼� ����λ   0=��⵽  1=δ��⵽

//��̽�̹�紫����
#define UPF_LED PEin(1)//0�� 1��
#define UPB_LED PEin(3)
#define ResetBout PBout(10)

/* USER CODE END Private defines */

void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ GPIO_H__ */

