#ifndef BMS_H
#define BMS_H

#include "datacheck.h"


typedef __packed struct
{
	uint16_t	voltage;//电压  
	int16_t current;//电流:电流为正表示放电，电流为负表示充电
    uint16_t capacity_per;//容量百分比 SOC
	uint16_t Max_unit_voltage;//最高单体电压
	uint16_t Min_unit_voltage;//最低单体电压
	int16_t Max_unit_Temperature;//最高单体温度
	int16_t Min_unit_Temperature;//最低单体温度
	uint16_t Alarm;//电池报警:电池高温报警，电池SOC低报警
				
}battery_value_t;

typedef __packed struct
{
	uint8_t SOC;//剩余电量百分比
	uint8_t SOH;//电池健康状态百分比
	uint8_t sysVol;//总电压
	uint8_t sysCurt;//总电流
	uint32_t TIME1;//剩余放电时间
	uint32_t TIME2;//剩余充电时间
	uint32_t curtState;//电流状态判断充放电
	uint32_t chrgPlugIn;//充电器接入状态
	uint32_t state0;//放电开关状态
	uint32_t state1;//充电开关状态
	uint32_t state2;//充电器接入状态
	uint32_t alarm1;//总电压过压警告
	uint32_t alarm2;//总电压欠压警告
	uint32_t alarm3;//soc低警告
	uint32_t alarm4;//通信状态警告
	  //	uint8_t alarm4;//
  //	uint8_t alarm6;//放电过流警告
  //	uint8_t alarm7;//充电过流警告
  //	uint8_t alarm12;//MOS回路温度过高警报
  //	uint8_t CC1;//放电回路状态0：断开1：吸合
  //	uint8_t CC2;//预充回路状态
  //	uint8_t CC3;//充电回路状态




}battery_value_t1;

//		uint16_t voltage;//电压
//		int16_t current;//电流	
//		uint16_t capacity_per;//容量百分比
//		uint16_t Total_hours;//总运行时间
//		uint16_t cycle_count;//循环次数
//		uint16_t Max_unit_voltage;//最高单体电压
//		uint16_t MaxV_Location2_1;//位置2 位置1
//		uint16_t Min_unit_voltage;//最低单体电压
//		uint16_t MinV_Location2_1;//位置2 位置1
//		int16_t Max_unit_Temperature;//最高单体温度
//		uint16_t MaxT_location2_1;//位置2 位置1 
//		int16_t Min_unit_Temperature;//最低单体温度
//		uint16_t MinT_Location2_1;//位置2 位置1
//		uint16_t Ta_Det;//充电控制
//		uint16_t Ta_Present;//充电请求
//		uint16_t Cha_Discha_Pro_Sta;//充电保护状态 放点保护状态
//		uint8_t Cha_Relay_Sta;//充电继电器状态 
//		uint8_t Discha_Relay_Sta;//放点继电器状态
//		uint16_t ALRN;//告警信息数量
//		uint16_t Alarm_Type;//告警类型	
//		uint8_t PC_Cha_Command;//告警类型	
//		uint8_t PC_Cha_Flag;//告警类型	


extern battery_value_t battery_t;
 
void UART4_Send(uint8_t c);
void BMSSendWithCRC(void);
void Usart4Receive_BMS_IDLE(void);
long Hex2Dec_32(uint32_t temp);
int Read_parameter1(void);
int Read_parameter2(void);
int Read_parameter3(void);

#endif

