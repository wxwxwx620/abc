/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOSBUS_HOST_H
#define __MOSBUS_HOST_H
#ifdef __cplusplus
 extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"	 
	 
/* USER CODE BEGIN Includes */
#include "datacheck.h"
#include "usart.h"	 
/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

#define SlaveAddr		0x01			
#define HBAUD485		UART3_BAUD

#define BmsDatRedAdd_Sta		0x00B0
#define BmsDatRed_Num				0x0013

#define BmsDatRedAdd_Sta1		0x0100
#define BmsDatRed_Num1			0x0002
	 
#define BmsDatWriteAdd_Sta		0x00BD
#define BmsDatWrite_Num				0x0001

#define g_buf_number0 8
#define g_buf_number1 8
extern uint8_t buf0[g_buf_number0];
extern uint8_t buf1[g_buf_number1];
//extern osThreadId BMSTaskHandle;
//extern osThreadId HydraulicLifTasHandle;

						
/* 01H 读强制单线圈 */
/* 05H 写强制单线圈 */
#define REG_D01		0x0101
#define REG_D02		0x0102
#define REG_D03		0x0103
#define REG_D04		0x0104
#define REG_DXX 	REG_D04

/* 02H 读取输入状态 */
#define REG_T01		0x0201
#define REG_T02		0x0202
#define REG_T03		0x0203
#define REG_TXX		REG_T03

/* 03H 读保持寄存器 */
/* 06H 写保持寄存器 */
/* 10H 写多个保存寄存器 */
#define REG_P01		0x0301		
#define REG_P02		0x0302	

/* 04H 读取输入寄存器(模拟信号) */
#define REG_A01		0x0401
#define REG_AXX		REG_A01

/* RTU 应答代码 */
#define RSP_OK				0		/* 成功 */
#define RSP_ERR_CMD			0x01	/* 不支持的功能码 */
#define RSP_ERR_REG_ADDR	0x02	/* 寄存器地址错误 */
#define RSP_ERR_VALUE		0x03	/* 数据值域错误 */
#define RSP_ERR_WRITE		0x04	/* 写入失败 */

#define H_RX_BUF_SIZE		64
#define H_TX_BUF_SIZE      	128
/* USER CODE END Private defines */


/* USER CODE BEGIN Prototypes */

typedef struct
{
	/* 03H 06H 读写保持寄存器 */
	uint16_t P01;
	uint16_t P02;
	
	/* 02H 读写离散输入寄存器 */
	uint16_t T01;
	uint16_t T02;
	uint16_t T03;
	
	/* 04H 读取模拟量寄存器 */
	uint16_t A01;
	
	/* 01H 05H 读写单个强制线圈 */
	uint16_t D01;
	uint16_t D02;
	uint16_t D03;
	uint16_t D04;
	
}VAR_T;

typedef struct
{	
		uint16_t voltage;//电压
		int16_t current;//电流	
		uint16_t capacity_per;//容量百分比
		uint16_t Total_hours;//总运行时间
		uint16_t cycle_count;//循环次数
		uint16_t Max_unit_voltage;//最高单体电压
		uint16_t MaxV_Location2_1;//位置2 位置1
		uint16_t Min_unit_voltage;//最低单体电压
		uint16_t MinV_Location2_1;//位置2 位置1
		int16_t Max_unit_Temperature;//最高单体温度
		uint16_t MaxT_location2_1;//位置2 位置1 
		int16_t Min_unit_Temperature;//最低单体温度
		uint16_t MinT_Location2_1;//位置2 位置1
		uint16_t Ta_Det;//充电控制
		uint16_t Ta_Present;//充电请求
		uint16_t Cha_Discha_Pro_Sta;//充电保护状态 放点保护状态
		uint8_t Cha_Relay_Sta;//充电继电器状态 
		uint8_t Discha_Relay_Sta;//放点继电器状态
		uint16_t ALRN;//告警信息数量
		uint16_t Alarm_Type;//告警类型	
		uint8_t PC_Cha_Command;//告警类型	
		uint8_t PC_Cha_Flag;//告警类型	
	
} BMS_STA;
extern BMS_STA BMS1;
typedef union _INT_2_CHAR_
{
	struct _HIGH_LOW_
	{
		int8_t Low8;
		int8_t High8;
	}HIGH_LOW;
	int16_t Data16;
}INT_CHAR;
extern INT_CHAR data_poor;
extern INT_CHAR data_poor1;
extern INT_CHAR data_poor2;
extern INT_CHAR data_poor3;

extern uint8_t MODH_ReadParam_01H(uint16_t _reg, uint16_t _num);
extern uint8_t MODH_ReadParam_02H(uint16_t _reg, uint16_t _num);
extern uint8_t MODH_ReadParam_03H(uint16_t _reg, uint16_t _num);
extern uint8_t MODH_ReadParam_04H(uint16_t _reg, uint16_t _num);
extern uint8_t MODH_WriteParam_05H(uint16_t _reg, uint16_t _value);
extern uint8_t MODH_WriteParam_06H(uint16_t _reg, uint16_t _value);
extern uint8_t MODH_WriteParam_10H(uint16_t _reg, uint8_t _num, uint8_t *_buf);


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
