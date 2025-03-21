/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ALARM_H
#define __ALARM_H

#include "main.h"

typedef __packed struct    //can�����ĵ���澯
{
	uint16_t Motor_Alarm_FL;
	uint16_t Motor_Alarm_FR;
	uint16_t Motor_Alarm_BR;
	uint16_t Motor_Alarm_BL;
	uint16_t Motor_Alarm_Clip_FL;
	uint16_t Motor_Alarm_Clip_FR;
	uint16_t Motor_Alarm_Clip_BR;
	uint16_t Motor_Alarm_Clip_BL;
}AGV_Motor_Alarm; //�������澯�����ڵ����Դ���� 20221007


extern AGV_Motor_Alarm  AGV_Motor_status;  //20221007
extern int8_t Motor_Alarm_Flag;//�����־λ 0���� ������ֵΪ�澯
extern uint8_t Motor_Alarm_cnt;//����澯���� 20221008

void Motor_Power_Off(void);//�����������Դ�ر�
	
void Restart_Motor_Power(void);//�����������Դ����
	
//void Motor_Alarm_Processing(void);//����������澯����
#endif 
