/**
  ******************************************************************************
  * @file			74HC597.h
  * @brief		74HC597扩展输入接口
	*	@author		肖智中
	*	@ver			1.1
	*	@date			2021.12.16
  ******************************************************************************
  * @attention
  *	v1.1 剪枝优化性能
  *
  ******************************************************************************
  */
#ifndef _74HC597_H
#define _74HC597_H

#include "stm32f4xx_hal.h"
#include "main.h"


#define HC597_R1 HAL_GPIO_WritePin(HC597_STB_GPIO_Port, HC597_STB_Pin, GPIO_PIN_SET);				//PD10
#define HC597_R0 HAL_GPIO_WritePin(HC597_STB_GPIO_Port, HC597_STB_Pin, GPIO_PIN_RESET);
#define HC597_L1 HAL_GPIO_WritePin(HC597_LOAD_GPIO_Port, HC597_LOAD_Pin, GPIO_PIN_SET);			//PD11
#define HC597_L0 HAL_GPIO_WritePin(HC597_LOAD_GPIO_Port, HC597_LOAD_Pin, GPIO_PIN_RESET);
#define HC597_EN HAL_GPIO_WritePin(HC597_CS_GPIO_Port, HC597_CS_Pin, GPIO_PIN_RESET);				//PA10
#define HC597_DE HAL_GPIO_WritePin(HC597_CS_GPIO_Port, HC597_CS_Pin, GPIO_PIN_SET);

#define IN_9		8		//0X0100
#define IN_10 	9		//0X0200
#define IN_11 	10	//0X0400
#define IN_12 	11	//0X0800
#define IN_13 	12	//0X1000
#define IN_14 	13	//0X2000
#define IN_15 	14	//0X4000
#define IN_16 	15	//0X8000
#define IN_17 	0		//0X0001
#define IN_18 	1		//0X0002
#define IN_19 	2		//0X0004
#define IN_20 	3		//0X0008
#define IN_21 	4		//0X0010
#define IN_22 	5		//0X0020
#define IN_23 	6		//0X0040
#define IN_24 	7		//0X0080

extern void exi_read(void);
extern uint8_t get_exi(uint16_t num);

#endif
