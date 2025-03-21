#include "main.h"
#include "Alarm.h"
#include "74HC595.h"
#include "motor.h"

AGV_Motor_Alarm  AGV_Motor_status;

int8_t Motor_Alarm_Flag=0;//�����־λ 0���� ������ֵΪ�澯
uint8_t Motor_Alarm_cnt=0;//����澯���� 20221008

void Motor_Power_Off(void)//�����������Դ�ر�
{ 
		HC595_del(OUT_5);
        HC595_del(OUT_6);    //��̬�̵����Ͽ�

		HAL_Delay(2000);
}

void Restart_Motor_Power(void)//�����������Դ����
{
		HC595_add(OUT_6);
		HAL_Delay(5000); 
	
		HC595_add(OUT_5);
		HAL_Delay(1000);
	
        HC595_del(OUT_6);     //��̬�̵����Ͽ�
		HAL_Delay(2000);
	
        SetMotorEnable();
	    Motor_Alarm_Flag=0;//�����־λ���� 0����
	    Motor_Alarm_cnt++;
}

//void Motor_Alarm_Processing(void)//����������澯����
//{
//	if((MotorFL.alarm==0x7380)||(MotorFR.alarm==0x7380)
//	 ||(MotorBR.alarm==0x7380)||(MotorBL.alarm==0x7380)
//	 ||(MotorCFL.alarm==0x7380)||(MotorCFR.alarm==0x7380)
//	 ||(MotorCBR.alarm==0x7380)||(MotorCBL.alarm==0x7380))	
//	{
//	  Motor_Alarm_Flag=(1<<0);//���ABZ����	
//	}
//	if((MotorFL.alarm==0x7381)||(MotorFR.alarm==0x7381)
//	 ||(MotorBR.alarm==0x7381)||(MotorBL.alarm==0x7381)
//	 ||(MotorCFL.alarm==0x7381)||(MotorCFR.alarm==0x7381)
//	 ||(MotorCBR.alarm==0x7381)||(MotorCBL.alarm==0x7381))	
//	{
//	  Motor_Alarm_Flag=(1<<1);//���UVW����		
//	}
//	if((MotorFL.alarm==0x6321)||(MotorFR.alarm==0x6321)
//	 ||(MotorBR.alarm==0x6321)||(MotorBL.alarm==0x6321)
//	 ||(MotorCFL.alarm==0x6321)||(MotorCFR.alarm==0x6321)
//	 ||(MotorCBR.alarm==0x6321)||(MotorCBL.alarm==0x6321))	
//	{
//	  Motor_Alarm_Flag=(1<<2);//���ȱ��		
//	}
//	if((MotorFL.alarm==0x7331)||(MotorFR.alarm==0x7331)
//	 ||(MotorBR.alarm==0x7331)||(MotorBL.alarm==0x7331)
//	 ||(MotorCFL.alarm==0x7331)||(MotorCFR.alarm==0x7331)
//	 ||(MotorCBR.alarm==0x7331)||(MotorCBL.alarm==0x7331))	
//	{
//	  Motor_Alarm_Flag=(1<<3);//ͨ�ű�����û������
//	}
//	if((MotorFL.alarm==0x8100)||(MotorFR.alarm==0x8100)
//	 ||(MotorBR.alarm==0x8100)||(MotorBL.alarm==0x8100)
//	 ||(MotorCFL.alarm==0x8100)||(MotorCFR.alarm==0x8100)
//	 ||(MotorCBR.alarm==0x8100)||(MotorCBL.alarm==0x8100))	
//	{
//	  Motor_Alarm_Flag=(1<<4);//����ͨ�ų���
//	}
//	if((MotorFL.alarm==0x81FF)||(MotorFR.alarm==0x81FF)
//	 ||(MotorBR.alarm==0x81FF)||(MotorBL.alarm==0x81FF)
//	 ||(MotorCFL.alarm==0x81FF)||(MotorCFR.alarm==0x81FF)
//	 ||(MotorCBR.alarm==0x81FF)||(MotorCBL.alarm==0x81FF))	
//	{
//	  Motor_Alarm_Flag=(1<<5);//����ͨ�ų�ʱ
//	}
//	if((MotorFL.alarm==0x2350)||(MotorFR.alarm==0x2350)
//	 ||(MotorBR.alarm==0x2350)||(MotorBL.alarm==0x2350)
//	 ||(MotorCFL.alarm==0x2350)||(MotorCFR.alarm==0x2350)
//	 ||(MotorCBR.alarm==0x2350)||(MotorCBL.alarm==0x2350))	
//	{
//	  Motor_Alarm_Flag=(1<<6);//�������
//	}
//	if((MotorFL.alarm==0x2320)||(MotorFR.alarm==0x2320)
//	 ||(MotorBR.alarm==0x2320)||(MotorBL.alarm==0x2320)
//	 ||(MotorCFL.alarm==0x2320)||(MotorCFR.alarm==0x2320)
//	 ||(MotorCBR.alarm==0x2320)||(MotorCBL.alarm==0x2320))	
//	{
//	  Motor_Alarm_Flag=(1<<7);//�����·
//	}
//	if((MotorFL.alarm==0x8611)||(MotorFR.alarm==0x8611)
//	 ||(MotorBR.alarm==0x8611)||(MotorBL.alarm==0x8611)
//	 ||(MotorCFL.alarm==0x8611)||(MotorCFR.alarm==0x8611)
//	 ||(MotorCBR.alarm==0x8611)||(MotorCBL.alarm==0x8611))	
//	{
//	  Motor_Alarm_Flag=(1<<8);//����������
//	}
//	if((MotorFL.alarm==0x5112)||(MotorFR.alarm==0x5112)
//	 ||(MotorBR.alarm==0x5112)||(MotorBL.alarm==0x5112)
//	 ||(MotorCFL.alarm==0x5112)||(MotorCFR.alarm==0x5112)
//	 ||(MotorCBR.alarm==0x5112)||(MotorCBL.alarm==0x5112))	
//	{
//	  Motor_Alarm_Flag=(1<<9);//�߼���ѹ����
//	}
//	if((MotorFL.alarm==0x3210)||(MotorFR.alarm==0x3210)
//	 ||(MotorBR.alarm==0x3210)||(MotorBL.alarm==0x3210)
//	 ||(MotorCFL.alarm==0x3210)||(MotorCFR.alarm==0x3210)
//	 ||(MotorCBR.alarm==0x3210)||(MotorCBL.alarm==0x3210))	
//	{
//	  Motor_Alarm_Flag=(1<<10);//���ߵ�ѹ����
//	}
//	if((MotorFL.alarm==0x3220)||(MotorFR.alarm==0x3220)
//	 ||(MotorBR.alarm==0x3220)||(MotorBL.alarm==0x3220)
//	 ||(MotorCFL.alarm==0x3220)||(MotorCFR.alarm==0x3220)
//	 ||(MotorCBR.alarm==0x3220)||(MotorCBL.alarm==0x3220))	
//	{
//	  Motor_Alarm_Flag=(1<<11);//���ߵ�ѹ����
//	}
//}
