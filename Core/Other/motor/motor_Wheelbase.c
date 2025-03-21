/****************************************************************************
// 2022.09.20		轴距调整

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

#define Adjustment_Value_Max 200 //差速轮轴距调整速度 20221009
#define Adjustment_Value_Mid 100 //差速轮轴距调整速度 20221009
#define Adjustment_Value_Min 60 //差速轮轴距调整速度 20221009
#define ECAxleToMM     0.0521289//轴距编码器值转换为MM
#define WheelBaseZero  2800      //编码器原点轴距值

#define D_Value 10 //差速轮轴距调整速度差值 20221005
#define MaxAjustOffset  200
#define MaxAjustSpdDis 200  //超过这个距离用快速
#define OVERDIS 20  //超限报错距离
float Wheelbase_Cur = 0;//检测到轮子，第一次取轴距值
float Wheel_Dis = 0;//半个轮距
float Wheelbase_Back_Dis = 0;//停止后继续前进至停止的轴距距离

void Wheelbase_Calculation(void)//轴距计算程序
{
	Wheelbase_Data = WheelBaseZero + (Encoder_Axle.val - EncoderZero)*ECAxleToMM;

	agvStatus.wheelBaseLength = Wheelbase_Data;
}

//
//int FindCarWheel()        //找汽车轮胎
//{
//	//达到极限，停下来
//	if (Wheelbase_Data <= WheelbaseLowerLimit- OVERDIS || Wheelbase_Data >= WheelbaseUpperLimit+ OVERDIS)                      //轴距的下线是2400
//	{
//		Stop_Moving();
//		return 2;
//	}
//	//运动过程角度偏差过大，立即停止
//	if (CheckECOverLimit( agvStatus.ecKeepAngle - Encoder_Front.val, agvStatus.ecKeepAngle - Encoder_Back.val, MaxAjustOffset))
//	{
//		return 3;
//	}
//
//	int16_t rpm = 0;
//
//	if (Sensor_Flag == 0)//起始标志位=0  未检测到车轮
//	{
//		if ((get_exi(IN_9) == 0) && (get_exi(IN_10) == 0))//两个后轮的超声传感器同时检测到车轮				
//		{
//			rpm = -Adjustment_Value_Mid;
//			Sensor_Flag = 1;
//			Wheelbase_Cur = Wheelbase_Data;//取当前轴距值
//		}
//		else
//		{
//			rpm = -Adjustment_Value_Max;
//		}
//	}
//	else if (Sensor_Flag == 1)//第一次监测到车轮后
//	{		
//		//大于100表示找到一端后后退一段距离直接包夹，小于100表示要前后找取中心点
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
//			if ((get_exi(IN_9) == 1) && (get_exi(IN_10) == 1))//两个后轮的超声传感器同时监测不到车轮			
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
//	else if (Sensor_Flag == 2)//后退过车轮停止，开始前进半个轮距
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

//反向找汽车轮胎
int FindCarWheelReverse()
{

	//运动过程角度偏差过大，立即停止
	if (CheckECOverLimit(agvStatus.ecKeepAngle - Encoder_Front.val, agvStatus.ecKeepAngle - Encoder_Back.val, MaxAjustOffset))
	{
		return 3;
	}

	int16_t rpm = 0;

	if (Sensor_Flag == 10)//起始标志位=0  未检测到车轮
	{
		if ((get_exi(IN_9) == 0) && (get_exi(IN_10) == 0))//两个后轮的超声传感器同时检测到车轮				
		{
			rpm = Adjustment_Value_Mid;
			Sensor_Flag = 11;
			Wheelbase_Cur = Wheelbase_Data;//取当前轴距值
		}
		else
		{
			rpm = Adjustment_Value_Max;
		}
	}
	else if (Sensor_Flag == 11)//第一次监测到车轮后
	{
		//大于100表示找到一端后后退一段距离直接包夹，小于100表示要前后找取中心点
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
			if ((get_exi(IN_9) == 1) && (get_exi(IN_10) == 1))//两个后轮的超声传感器同时监测不到车轮			
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
	else if (Sensor_Flag == 12)//后退过车轮停止，开始前进半个轮距
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

	if (rpm > 0) //后轮缩小
	{
		//达到最小极限，停下来
		if (Wheelbase_Data <= WheelbaseLowerLimit)                      //轴距的下线是2400
		{
			Stop_Moving();
			return 2;
		}
	}
	else  //后轮拉伸
	{
		//达到最大极限，停下来
		if (Wheelbase_Data >= WheelbaseUpperLimit)                      //轴距的下线是2400
		{
			Stop_Moving();
			return 2;
		}
	}

	agvStatus.outputRpm = RpmUpdateWithDec(4, 10, rpm, agvStatus.outputRpm);
	BackStraightMove(agvStatus.outputRpm);
	return 0;
}


//找汽车轮胎
int FindCarWheel()        
{
	if (Sensor_Flag > 9)
	{
		int ret = FindCarWheelReverse();
		return ret;
	}

	//运动过程角度偏差过大，立即停止
	if (CheckECOverLimit(agvStatus.ecKeepAngle - Encoder_Front.val, agvStatus.ecKeepAngle - Encoder_Back.val, MaxAjustOffset))
	{
		return 3;
	}

	int16_t rpm = 0;

	if (Sensor_Flag == 0)//起始标志位=0  未检测到车轮
	{
		if ((get_exi(IN_9) == 0) && (get_exi(IN_10) == 0))//两个后轮的超声传感器同时检测到车轮				
		{
			rpm = -Adjustment_Value_Mid;
			Sensor_Flag = 1;
			Wheelbase_Cur = Wheelbase_Data;//取当前轴距值
		}
		else
		{
			rpm = -Adjustment_Value_Max;
		}
	}
	else if (Sensor_Flag == 1)//第一次监测到车轮后
	{
		//大于100表示找到一端后后退一段距离直接包夹，小于100表示要前后找取中心点
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
			if ((get_exi(IN_9) == 1) && (get_exi(IN_10) == 1))//两个后轮的超声传感器同时监测不到车轮			
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
	else if (Sensor_Flag == 2)//后退过车轮停止，开始前进半个轮距
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
	if (rpm > 0) //后轮缩小
	{
		//达到最小极限，停下来
		if (Wheelbase_Data <= WheelbaseLowerLimit )                      //轴距的下线是2400
		{
			Stop_Moving();
			return 2;
		}
	}
	else  //后轮拉伸
	{
		//达到最大极限，停下来
		if (Wheelbase_Data >= WheelbaseUpperLimit )                      //轴距的下线是2400
		{
			Stop_Moving();
			return 2;
		}
	}

	agvStatus.outputRpm = RpmUpdateWithDec(4, 10, rpm, agvStatus.outputRpm);
	BackStraightMove(agvStatus.outputRpm);
	return 0;
}



int AjustCarWheelDis()   //调整汽车前后轮间距距
{	
	//运动过程角度偏差过大，立即停止
	if (CheckECOverLimit( agvStatus.ecKeepAngle - Encoder_Front.val, agvStatus.ecKeepAngle - Encoder_Back.val, MaxAjustOffset))
	{
		return 3;
	}
	int16_t rpm = 0;
	//设定值小于当前值,缩短
	if ((agvTask.wheelBaseLength + 5.0) < Wheelbase_Data)
	{
		if (Wheelbase_Data <= WheelbaseLowerLimit)                      //轴距的下线是2400
		{
			Stop_Moving();
			return 2;
		}

		if ((agvTask.wheelBaseLength + MaxAjustSpdDis) < Wheelbase_Data)                //轴距距离目标值大于20mm
		{
			rpm = -Adjustment_Value_Max;
		}
		else
		{
			rpm = -Adjustment_Value_Min;
		}
	}
	//设定值大于当前值,拉伸
	else if ((agvTask.wheelBaseLength - 5.0) > Wheelbase_Data)
	{

		if (Wheelbase_Data >= WheelbaseUpperLimit)                      //轴距上线是3200
		{
			Stop_Moving();
			return 2;
		}
		//轴距距离目标值大于20mm
		if ((agvTask.wheelBaseLength - MaxAjustSpdDis) > Wheelbase_Data)
		{
			rpm = Adjustment_Value_Max;	
		}
		else
		{
			rpm = Adjustment_Value_Min;		
		}
	}
	else//设定值-5<当前值<设定值+5 ，说明调整完成
	{
		Stop_Moving();
		return 1;
	}

	agvStatus.outputRpm = RpmUpdateWithDec(4,10,rpm, agvStatus.outputRpm);
	FrontStraightMove(agvStatus.outputRpm);
	BackStraightMove(-agvStatus.outputRpm);

	return 0;
}


int AjustDisReady(int readyMode)      //调整轴距准备
{
	if (readyMode==READY_START)
	{
		if (MoveReady())
		{
			HC595_add(OUT_7);                             //继电器7打开	
			osDelay(500);
			return 1;
		}
	
	}
	else
	{
		Stop_Moving();
		if (IsMoveStoped())
		{
			HC595_del(OUT_7);                             //继电器7锁死
			osDelay(200);
			return 1;
		}		
	}
	return 0;
}

//前轮直线移动  正为前进 负为后退
void FrontStraightMove(int16_t rpm)
{
	float Accelerate_Front = (agvStatus.ecKeepAngle - Encoder_Front.val)*Encoder_To_Angle*30;//编码器需要达到的角度-当前角度
	Accelerate_Front = LimitMax(Accelerate_Front,10);
	Set_Can_RPM_RPDO(FRONT_LEFT, rpm + Accelerate_Front, SetCount);    //左前轮
	Set_Can_RPM_RPDO(FRONT_RIGHT, -(rpm - Accelerate_Front), SetCount);//右前轮
}

//后轮直线移动 正为前进 负为后退
void BackStraightMove(int16_t rpm)
{
	float Accelerate_Back = (agvStatus.ecKeepAngle - Encoder_Back.val)*Encoder_To_Angle * 30;//编码器需要达到的角度-当前角度
	Accelerate_Back = LimitMax(Accelerate_Back,10);
	Set_Can_RPM_RPDO(BACK_LEFT, rpm + Accelerate_Back, SetCount);     //左后轮
	Set_Can_RPM_RPDO(BACK_RIGHT, -(rpm - Accelerate_Back), SetCount); //右后轮
	
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