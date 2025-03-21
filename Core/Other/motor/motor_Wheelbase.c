/****************************************************************************
// 2022.09.20		������

*****************************************************************************/
#include "motor.h"
#include "arm_math.h"
#include "can.h"
#include "remote_control.h"
#include "voice.h"
#include "encoder.h"
#include "74HC595.h"
#include "usart.h"
#include "74HC597.h"

#define Adjustment_Value_Max 200 //�������������ٶ� 20221009
#define Adjustment_Value_Mid 100 //�������������ٶ� 20221009
#define Adjustment_Value_Min 60 //�������������ٶ� 20221009
#define ECAxleToMM     0.0521289//��������ֵת��ΪMM
#define WheelBaseZero  2800      //������ԭ�����ֵ

#define D_Value 10 //�������������ٶȲ�ֵ 20221005
#define MaxAjustOffset  200
#define MaxAjustSpdDis 200  //������������ÿ���
#define OVERDIS 20  //���ޱ������
float Wheelbase_Cur = 0;//��⵽���ӣ���һ��ȡ���ֵ
float Wheel_Dis = 0;//����־�
float Wheelbase_Back_Dis = 0;//ֹͣ�����ǰ����ֹͣ��������

void Wheelbase_Calculation(void)//���������
{
	Wheelbase_Data = WheelBaseZero + (Encoder_Axle.val - EncoderZero)*ECAxleToMM;

	agvStatus.wheelBaseLength = Wheelbase_Data;
}

//
//int FindCarWheel()        //��������̥
//{
//	//�ﵽ���ޣ�ͣ����
//	if (Wheelbase_Data <= WheelbaseLowerLimit- OVERDIS || Wheelbase_Data >= WheelbaseUpperLimit+ OVERDIS)                      //����������2400
//	{
//		Stop_Moving();
//		return 2;
//	}
//	//�˶����̽Ƕ�ƫ���������ֹͣ
//	if (CheckECOverLimit( agvStatus.ecKeepAngle - Encoder_Front.val, agvStatus.ecKeepAngle - Encoder_Back.val, MaxAjustOffset))
//	{
//		return 3;
//	}
//
//	int16_t rpm = 0;
//
//	if (Sensor_Flag == 0)//��ʼ��־λ=0  δ��⵽����
//	{
//		if ((get_exi(IN_9) == 0) && (get_exi(IN_10) == 0))//�������ֵĳ���������ͬʱ��⵽����				
//		{
//			rpm = -Adjustment_Value_Mid;
//			Sensor_Flag = 1;
//			Wheelbase_Cur = Wheelbase_Data;//ȡ��ǰ���ֵ
//		}
//		else
//		{
//			rpm = -Adjustment_Value_Max;
//		}
//	}
//	else if (Sensor_Flag == 1)//��һ�μ�⵽���ֺ�
//	{		
//		//����100��ʾ�ҵ�һ�˺����һ�ξ���ֱ�Ӱ��У�С��100��ʾҪǰ����ȡ���ĵ�
//		if (agvTask.wheelBaseLength > 100)
//		{
//			if (Wheelbase_Cur+ agvTask.wheelBaseLength<Wheelbase_Data)
//			{
//				Stop_Moving();
//				return 1;
//			}
//			else
//			{
//				rpm = -Adjustment_Value_Mid;
//			}
//		}
//		else
//		{
//			if ((get_exi(IN_9) == 1) && (get_exi(IN_10) == 1))//�������ֵĳ���������ͬʱ��ⲻ������			
//			{
//				Sensor_Flag = 2;
//				Stop_Moving();
//				Wheel_Dis = (Wheelbase_Data - Wheelbase_Cur) *0.5f;
//				Wheelbase_Back_Dis = Wheelbase_Data - Wheel_Dis;
//				rpm = Adjustment_Value_Mid;
//			}
//			else
//			{
//				rpm = -Adjustment_Value_Mid;
//			}
//		}
//
//		
//	}
//	else if (Sensor_Flag == 2)//���˹�����ֹͣ����ʼǰ������־�
//	{
//		if (Wheelbase_Data > Wheelbase_Back_Dis)
//		{	
//			rpm = Adjustment_Value_Mid;
//		}
//		else
//		{
//			Stop_Moving();
//			return 1;
//		}
//	}
//	agvStatus.outputRpm = RpmUpdateWithDec(4,10, rpm, agvStatus.outputRpm);
//	BackStraightMove(agvStatus.outputRpm);
//	return 0;
//}

//������������̥
int FindCarWheelReverse()
{

	//�˶����̽Ƕ�ƫ���������ֹͣ
	if (CheckECOverLimit(agvStatus.ecKeepAngle - Encoder_Front.val, agvStatus.ecKeepAngle - Encoder_Back.val, MaxAjustOffset))
	{
		return 3;
	}

	int16_t rpm = 0;

	if (Sensor_Flag == 10)//��ʼ��־λ=0  δ��⵽����
	{
		if ((get_exi(IN_9) == 0) && (get_exi(IN_10) == 0))//�������ֵĳ���������ͬʱ��⵽����				
		{
			rpm = Adjustment_Value_Mid;
			Sensor_Flag = 11;
			Wheelbase_Cur = Wheelbase_Data;//ȡ��ǰ���ֵ
		}
		else
		{
			rpm = Adjustment_Value_Max;
		}
	}
	else if (Sensor_Flag == 11)//��һ�μ�⵽���ֺ�
	{
		//����100��ʾ�ҵ�һ�˺����һ�ξ���ֱ�Ӱ��У�С��100��ʾҪǰ����ȡ���ĵ�
		if (agvTask.wheelBaseLength > 100)
		{
			if (Wheelbase_Cur - agvTask.wheelBaseLength > Wheelbase_Data)
			{
				Stop_Moving();
				return 1;
			}
			else
			{
				rpm = Adjustment_Value_Mid;
			}
		}
		else
		{
			if ((get_exi(IN_9) == 1) && (get_exi(IN_10) == 1))//�������ֵĳ���������ͬʱ��ⲻ������			
			{
				Sensor_Flag = 12;
				Stop_Moving();
				Wheel_Dis = (Wheelbase_Cur - Wheelbase_Data) *0.5f;
				Wheelbase_Back_Dis = Wheelbase_Data + Wheel_Dis;
				rpm = -Adjustment_Value_Mid;
			}
			else
			{
				rpm = Adjustment_Value_Mid;
			}
		}


	}
	else if (Sensor_Flag == 12)//���˹�����ֹͣ����ʼǰ������־�
	{
		if (Wheelbase_Data < Wheelbase_Back_Dis)
		{
			rpm = -Adjustment_Value_Mid;
		}
		else
		{
			Stop_Moving();
			return 1;
		}
	}

	if (rpm > 0) //������С
	{
		//�ﵽ��С���ޣ�ͣ����
		if (Wheelbase_Data <= WheelbaseLowerLimit)                      //����������2400
		{
			Stop_Moving();
			return 2;
		}
	}
	else  //��������
	{
		//�ﵽ����ޣ�ͣ����
		if (Wheelbase_Data >= WheelbaseUpperLimit)                      //����������2400
		{
			Stop_Moving();
			return 2;
		}
	}

	agvStatus.outputRpm = RpmUpdateWithDec(4, 10, rpm, agvStatus.outputRpm);
	BackStraightMove(agvStatus.outputRpm);
	return 0;
}


//��������̥
int FindCarWheel()        
{
	if (Sensor_Flag > 9)
	{
		int ret = FindCarWheelReverse();
		return ret;
	}

	//�˶����̽Ƕ�ƫ���������ֹͣ
	if (CheckECOverLimit(agvStatus.ecKeepAngle - Encoder_Front.val, agvStatus.ecKeepAngle - Encoder_Back.val, MaxAjustOffset))
	{
		return 3;
	}

	int16_t rpm = 0;

	if (Sensor_Flag == 0)//��ʼ��־λ=0  δ��⵽����
	{
		if ((get_exi(IN_9) == 0) && (get_exi(IN_10) == 0))//�������ֵĳ���������ͬʱ��⵽����				
		{
			rpm = -Adjustment_Value_Mid;
			Sensor_Flag = 1;
			Wheelbase_Cur = Wheelbase_Data;//ȡ��ǰ���ֵ
		}
		else
		{
			rpm = -Adjustment_Value_Max;
		}
	}
	else if (Sensor_Flag == 1)//��һ�μ�⵽���ֺ�
	{
		//����100��ʾ�ҵ�һ�˺����һ�ξ���ֱ�Ӱ��У�С��100��ʾҪǰ����ȡ���ĵ�
		if (agvTask.wheelBaseLength > 100)
		{
			if (Wheelbase_Cur + agvTask.wheelBaseLength < Wheelbase_Data)
			{
				Stop_Moving();
				return 1;
			}
			else
			{
				rpm = -Adjustment_Value_Mid;
			}
		}
		else
		{
			if ((get_exi(IN_9) == 1) && (get_exi(IN_10) == 1))//�������ֵĳ���������ͬʱ��ⲻ������			
			{
				Sensor_Flag = 2;
				Stop_Moving();
				Wheel_Dis = (Wheelbase_Data - Wheelbase_Cur) *0.5f;
				Wheelbase_Back_Dis = Wheelbase_Data - Wheel_Dis;
				rpm = Adjustment_Value_Mid;
			}
			else
			{
				rpm = -Adjustment_Value_Mid;
			}
		}


	}
	else if (Sensor_Flag == 2)//���˹�����ֹͣ����ʼǰ������־�
	{
		if (Wheelbase_Data > Wheelbase_Back_Dis)
		{
			rpm = Adjustment_Value_Mid;
		}
		else
		{
			Stop_Moving();
			return 1;
		}
	}
	if (rpm > 0) //������С
	{
		//�ﵽ��С���ޣ�ͣ����
		if (Wheelbase_Data <= WheelbaseLowerLimit )                      //����������2400
		{
			Stop_Moving();
			return 2;
		}
	}
	else  //��������
	{
		//�ﵽ����ޣ�ͣ����
		if (Wheelbase_Data >= WheelbaseUpperLimit )                      //����������2400
		{
			Stop_Moving();
			return 2;
		}
	}

	agvStatus.outputRpm = RpmUpdateWithDec(4, 10, rpm, agvStatus.outputRpm);
	BackStraightMove(agvStatus.outputRpm);
	return 0;
}



int AjustCarWheelDis()   //��������ǰ���ּ���
{	
	//�˶����̽Ƕ�ƫ���������ֹͣ
	if (CheckECOverLimit( agvStatus.ecKeepAngle - Encoder_Front.val, agvStatus.ecKeepAngle - Encoder_Back.val, MaxAjustOffset))
	{
		return 3;
	}
	int16_t rpm = 0;
	//�趨ֵС�ڵ�ǰֵ,����
	if ((agvTask.wheelBaseLength + 5.0) < Wheelbase_Data)
	{
		if (Wheelbase_Data <= WheelbaseLowerLimit)                      //����������2400
		{
			Stop_Moving();
			return 2;
		}

		if ((agvTask.wheelBaseLength + MaxAjustSpdDis) < Wheelbase_Data)                //������Ŀ��ֵ����20mm
		{
			rpm = -Adjustment_Value_Max;
		}
		else
		{
			rpm = -Adjustment_Value_Min;
		}
	}
	//�趨ֵ���ڵ�ǰֵ,����
	else if ((agvTask.wheelBaseLength - 5.0) > Wheelbase_Data)
	{

		if (Wheelbase_Data >= WheelbaseUpperLimit)                      //���������3200
		{
			Stop_Moving();
			return 2;
		}
		//������Ŀ��ֵ����20mm
		if ((agvTask.wheelBaseLength - MaxAjustSpdDis) > Wheelbase_Data)
		{
			rpm = Adjustment_Value_Max;	
		}
		else
		{
			rpm = Adjustment_Value_Min;		
		}
	}
	else//�趨ֵ-5<��ǰֵ<�趨ֵ+5 ��˵���������
	{
		Stop_Moving();
		return 1;
	}

	agvStatus.outputRpm = RpmUpdateWithDec(4,10,rpm, agvStatus.outputRpm);
	FrontStraightMove(agvStatus.outputRpm);
	BackStraightMove(-agvStatus.outputRpm);

	return 0;
}


int AjustDisReady(int readyMode)      //�������׼��
{
	if (readyMode==READY_START)
	{
		if (MoveReady())
		{
			HC595_add(OUT_7);                             //�̵���7��	
			osDelay(500);
			return 1;
		}
	
	}
	else
	{
		Stop_Moving();
		if (IsMoveStoped())
		{
			HC595_del(OUT_7);                             //�̵���7����
			osDelay(200);
			return 1;
		}		
	}
	return 0;
}

//ǰ��ֱ���ƶ�  ��Ϊǰ�� ��Ϊ����
void FrontStraightMove(int16_t rpm)
{
	float Accelerate_Front = (agvStatus.ecKeepAngle - Encoder_Front.val)*Encoder_To_Angle*30;//��������Ҫ�ﵽ�ĽǶ�-��ǰ�Ƕ�
	Accelerate_Front = LimitMax(Accelerate_Front,10);
	Set_Can_RPM_RPDO(FRONT_LEFT, rpm + Accelerate_Front, SetCount);    //��ǰ��
	Set_Can_RPM_RPDO(FRONT_RIGHT, -(rpm - Accelerate_Front), SetCount);//��ǰ��
}

//����ֱ���ƶ� ��Ϊǰ�� ��Ϊ����
void BackStraightMove(int16_t rpm)
{
	float Accelerate_Back = (agvStatus.ecKeepAngle - Encoder_Back.val)*Encoder_To_Angle * 30;//��������Ҫ�ﵽ�ĽǶ�-��ǰ�Ƕ�
	Accelerate_Back = LimitMax(Accelerate_Back,10);
	Set_Can_RPM_RPDO(BACK_LEFT, rpm + Accelerate_Back, SetCount);     //�����
	Set_Can_RPM_RPDO(BACK_RIGHT, -(rpm - Accelerate_Back), SetCount); //�Һ���
	
}


int CheckWheel()
{
	if ((get_exi(IN_9) == 1) && (get_exi(IN_10) == 1))
	{
		return 0;
	}
	if ((get_exi(IN_11) == 1) && (get_exi(IN_12) == 1))
	{
		return 0;
	}
	return 1;
}