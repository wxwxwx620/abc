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

////行走电机报警,后面改成SPI总线读取
//#define Motor_Alarm_FL PAin(3)
//#define Motor_Alarm_FR PAin(4)
//#define Motor_Alarm_BR PAin(5)
//#define Motor_Alarm_BL PAin(6)
//#define Motor_Alarm_Clip_FL PBin(0)
//#define Motor_Alarm_Clip_FR PBin(1)
//#define Motor_Alarm_Clip_BR PCin(0)
//#define Motor_Alarm_Clip_BL PCin(2)
//举升报警
#define Motor_Alarm9 PCin(6)
#define Motor_Alarm10 PCin(7)
#define Motor_Alarm11 PDin(12)
#define Motor_Alarm12 PDin(13)

////抱夹开关限位 4个
//#define OF  PEin(7)  //左上：前抱夹 开限位   0=检测到  1=未检测到
//#define CF	PEin(8)  //左下：前抱夹 关限位   0=检测到  1=未检测到
//#define OB	PEin(10) //右下：后抱夹 开限位   0=检测到  1=未检测到
//#define CB  PEin(11) //右上：后抱夹 关限位   0=检测到  1=未检测到

//#define EmergencyStopCar PEin(12) //车子上的紧急停止按钮

//抱夹开关限位 8个
#define FLO  PEin(7)   //左前抱夹 开限位   0=检测到  1=未检测到
#define FLC	 PEin(8)   //左前抱夹 关限位   0=检测到  1=未检测到
#define FRO  PEin(12)  //右前抱夹 开限位   0=检测到  1=未检测到
#define FRC	 PEin(13)  //右前抱夹 关限位   0=检测到  1=未检测到
#define BRO	 PEin(10)  //右后抱夹 开限位   0=检测到  1=未检测到
#define BRC  PEin(11)  //右后抱夹 关限位   0=检测到  1=未检测到
#define BLO	 PEin(14)  //左后抱夹 开限位   0=检测到  1=未检测到
#define BLC  PEin(15)  //左后抱夹 关限位   0=检测到  1=未检测到

//顶探盘光电传感器
#define UPF_LED PEin(1)//0无 1有
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

