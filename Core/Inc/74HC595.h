/**
  ******************************************************************************
  * @file			74HC595.h
  * @brief		74HC595扩展输出接口
	*	@author		肖智中
	*	@ver			2.0
	*	@date			2021.12.15
  ******************************************************************************
  * @attention
  *	v1.1 驱动方式由GPIO变更为SPI
  *	v2.0 增加初始化过程，fix上电误输出
  ******************************************************************************
  */

#ifndef _74HC595_H
#define _74HC595_H

#include "stm32f4xx_hal.h"
#include "main.h"


#define HC595_EN HAL_GPIO_WritePin(HC595_EN_GPIO_Port, HC595_EN_Pin, GPIO_PIN_RESET);       //PD7
#define HC595_DE HAL_GPIO_WritePin(HC595_EN_GPIO_Port, HC595_EN_Pin, GPIO_PIN_SET);
#define HC595_R1 HAL_GPIO_WritePin(HC595_RK_GPIO_Port, HC595_RK_Pin, GPIO_PIN_SET);          //PE0
#define HC595_R0 HAL_GPIO_WritePin(HC595_RK_GPIO_Port, HC595_RK_Pin, GPIO_PIN_RESET);

#define OUT_ALL	0XFFFFFF
#define OUT_1 	0X010000                 //右前灯
#define OUT_2 	0X020000                 //左前灯
#define OUT_3 	0X040000                 //左后灯
#define OUT_4 	0X080000                 //右后灯
#define OUT_5 	0X100000                 //固态继电器
#define OUT_6 	0X200000                 //
#define OUT_7 	0X400000
#define OUT_8 	0X800000
#define OUT_9 	0X000100
#define OUT_10 	0X000200
#define OUT_11 	0X000400
#define OUT_12 	0X000800
#define OUT_13 	0X001000
#define OUT_14 	0X002000
#define OUT_15 	0X004000
#define OUT_16 	0X008000
#define OUT_17 	0X000001
#define OUT_18 	0X000002
#define OUT_19 	0X000004
#define OUT_20 	0X000008
#define OUT_21 	0X000010
#define OUT_22 	0X000020
#define OUT_23 	0X000040
#define OUT_24 	0X000080

extern void HC595_init(void);
extern void HC595_RCK(void);
extern void HC595_add(uint32_t data);
extern void HC595_del(uint32_t data);
extern void HC595_reset(void);
extern void HC595_updata(void);


#endif
