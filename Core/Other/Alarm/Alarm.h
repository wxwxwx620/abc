/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ALARM_H
#define __ALARM_H

#include "main.h"

typedef __packed struct    //can反馈的电机告警
{
	uint16_t Motor_Alarm_FL;
	uint16_t Motor_Alarm_FR;
	uint16_t Motor_Alarm_BR;
	uint16_t Motor_Alarm_BL;
	uint16_t Motor_Alarm_Clip_FL;
	uint16_t Motor_Alarm_Clip_FR;
	uint16_t Motor_Alarm_Clip_BR;
	uint16_t Motor_Alarm_Clip_BL;
}AGV_Motor_Alarm; //驱动器告警，用于电机电源重启 20221007


extern AGV_Motor_Alarm  AGV_Motor_status;  //20221007
extern int8_t Motor_Alarm_Flag;//电机标志位 0正常 其他数值为告警
extern uint8_t Motor_Alarm_cnt;//电机告警计数 20221008

void Motor_Power_Off(void);//电机驱动器电源关闭
	
void Restart_Motor_Power(void);//电机驱动器电源重启
	
//void Motor_Alarm_Processing(void);//电机驱动器告警处理
#endif 
