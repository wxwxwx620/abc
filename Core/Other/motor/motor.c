/****************************************************************************
// 2022.01.22		增加前车速度作为后车速度参考，避免行进中方向跑偏			ds
//
//
//
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
#include <stdlib.h>

CAR_CTRL_STA INNER;
MOTOR_MOVE_CTRL MotorFL;
MOTOR_MOVE_CTRL MotorFR;
MOTOR_MOVE_CTRL MotorBL;
MOTOR_MOVE_CTRL MotorBR;
MOTOR_CLIP_CTRL MotorCFL;
MOTOR_CLIP_CTRL MotorCFR;
MOTOR_CLIP_CTRL MotorCBL;
MOTOR_CLIP_CTRL MotorCBR;


CAR_Odometer_Integral CAR_Odometer;

float Wheelbase_Data;//轴距相关结构体 20221004


float Af, Ar, Vf, Vr;




/*机器人 X 轴方向速度VX = （VL + VR） / 2，机器人 Z 轴方向速度VZ = （VR - VL） / D。
机器人左轮的速度VL = VX - （VZ * D） / 2，机器人右轮的速度VR = VX + （VZ * D） / 2
代码如下：
void Drive_Motor(float vx, float vz)
{
	Target_Left = vx - vz * WIDTH_OF_ROBOT / 2.0f; //计算出左轮的目标速度
	Target_Right = vx + vz * WIDTH_OF_ROBOT / 2.0f; //计算出右轮的目标
}*/
//两轮旋转时差速大小 vz * WIDTH_OF_ROBOT / 2.0f

//float W,V; //W角速度， V=W*(L+B)   V=W*(L-B)
float WF, WR;
float B = 182.0;	//B是两个轮子直接的间距364mm 
int L = 2055;//轴距L=2055mm
float px;//原地旋转时，外轮与内轮的速度系数比
int F;//轴距整数
//float pc_speed_rpm;

//PID   pcMovPID_F;  //前轮PID控制
//PID   pcMovPID_B;  //后轮PID控制
//PID   RpmPID;      //速度控制控制PID

float deflection_angleF, angle3;//偏差角3编码器值，编码器3，偏差角度值
float deflection_angleB, angle1;//偏差角1编码器值，编码器1，偏差角值

float AccelerateSpeedB, AccelerateSpeedF;//3是前差动单元，1是后差动单元


#define RPMSTOP 5

uint16_t Pcount = 0;
int16_t FrontReadyRpm = 0; //前后轮转到对应方向的速度
int16_t BackReadyRpm = 0;

//float testf = 0;
int Front_Differential_Unit_Adjustment(int maxRpm)//前差动单元调整(任意角度)
{
	int16_t rpm = 0;
	int32_t ecOffset = agvStatus.ecKeepAngle-Encoder_Front.val;
	if (ecOffset > ZeroAbout || ecOffset < -ZeroAbout)
	{
		rpm = ecOffset / 2;
		rpm = LimitMax(rpm, maxRpm);
		FrontReadyRpm = RpmUpdateWithDec(8, 16, rpm, FrontReadyRpm);
	}
	else
	{
		Set_Can_RPM_RPDO(FRONT_LEFT, 0, SetCount);
		Set_Can_RPM_RPDO(FRONT_RIGHT, 0, SetCount);
		FrontReadyRpm = 0;
		return 0;
	}

	//testf = FrontReadyRpm;
	Set_Can_RPM_RPDO(FRONT_LEFT, FrontReadyRpm, SetCount);
	Set_Can_RPM_RPDO(FRONT_RIGHT, FrontReadyRpm, SetCount);
	return 1;

}

//float testrpm = 0;

int Back_Differential_Unit_Adjustment(int maxRpm)//后差动单元调整(任意角度)
{
	int16_t rpm = 0;
	int32_t ecOffset = agvStatus.ecKeepAngle-Encoder_Back.val;
	if (ecOffset > ZeroAbout|| ecOffset < -ZeroAbout)  //右偏 //右偏 逆时针调整
	{
		rpm = ecOffset / 2;
		rpm = LimitMax(rpm, maxRpm);
		BackReadyRpm = RpmUpdateWithDec(8,16,rpm, BackReadyRpm);
	}
	else
	{
		Set_Can_RPM_RPDO(BACK_LEFT, 0, SetCount);
		Set_Can_RPM_RPDO(BACK_RIGHT, 0, SetCount);
		BackReadyRpm = 0;
		return 0;
	}	
	//testrpm = BackReadyRpm;
	Set_Can_RPM_RPDO(BACK_LEFT, BackReadyRpm, SetCount);
	Set_Can_RPM_RPDO(BACK_RIGHT, BackReadyRpm, SetCount);
	return 1;
}


//顺时针为正，逆时针为负
void Front_Unit_Rotation(int16_t rpm)        //前差动单元旋转
{
	Set_Can_RPM_RPDO(FRONT_LEFT, rpm, SetCount);
	//HAL_Delay(5);
	Set_Can_RPM_RPDO(FRONT_RIGHT, rpm, SetCount);
	//HAL_Delay(5);
}

//后差动单元旋转 顺时针为正，逆时针为负
void Behind_Unit_Rotation(int16_t rpm)      
{
	Set_Can_RPM_RPDO(BACK_LEFT, rpm, SetCount);
	//HAL_Delay(5);
	Set_Can_RPM_RPDO(BACK_RIGHT, rpm, SetCount);
	//HAL_Delay(5);
}

void Stop_Back_Wheel()//后差动单元停止
{
	Set_Can_RPM_RPDO(BACK_LEFT, 0, SetCount);
	//HAL_Delay(5);
	Set_Can_RPM_RPDO(BACK_RIGHT, 0, SetCount);
	//HAL_Delay(5);
}

void Stop_Front_Wheel(void)//前差动单元停止
{
	Set_Can_RPM_RPDO(FRONT_LEFT, 0, SetCount);
	//HAL_Delay(5);
	Set_Can_RPM_RPDO(FRONT_RIGHT, 0, SetCount);
	//HAL_Delay(5);
}

void Stop_Moving(void)//电机停止移动
{

	agvStatus.outputRpm = 0;
	FrontReadyRpm = 0;
	BackReadyRpm = 0;
	FClipRpm = 0;
	BClipRpm = 0;
	Set_Can_RPM_RPDO(FRONT_LEFT, 0, SetCount);
	//HAL_Delay(5);
	Set_Can_RPM_RPDO(FRONT_RIGHT, 0, SetCount);
	//HAL_Delay(5);
	Set_Can_RPM_RPDO(BACK_RIGHT, 0, SetCount);
	//HAL_Delay(5);
	Set_Can_RPM_RPDO(BACK_LEFT, 0, SetCount);
	//HAL_Delay(5);
}


float RpmUpdate(float acc, float  targetSpd, float curSpd)//加减速程序					//需修改为PID控制！！！！！
{	
	if ((targetSpd - curSpd) > acc)
	{
		return curSpd + acc;
	}
	else if ((targetSpd - curSpd) < -acc)
	{
		return curSpd - acc;
	}
	
	return targetSpd;
}


float RpmUpdateWithDec(float acc,float dec, float  targetSpd, float curSpd)//加减速程序					//需修改为PID控制！！！！！
{
	if (curSpd<0)
	{
		if ((targetSpd - curSpd) > acc)
		{
			return curSpd + dec;
		}
		else if ((targetSpd - curSpd) < -acc)
		{
			return curSpd - acc;
		}
	}
	else
	{
		if ((targetSpd - curSpd) > acc)
		{
			return curSpd + acc;
		}
		else if ((targetSpd - curSpd) < -acc)
		{
			return curSpd - dec;
		}
	}
	
	return targetSpd;
}


void Set_Can_Motor_Enable(uint8_t can_channel, int8_t count)
{
	int i;
	uint8_t can_buf[8];

	can_buf[0] = 0x2B;
	can_buf[1] = 0x40;
	can_buf[2] = 0x60;
	can_buf[3] = 0x00;
	can_buf[4] = 0x0F;
	can_buf[5] = 0x00;
	can_buf[6] = 0x00;
	can_buf[7] = 0x00;
	for (i = 0; i < count; i++)
	{
		Can_Send_Msg(can_buf, 8, can_channel);
		HAL_Delay(10);
	}
}
void SetMotorEnable(void)
{
	//防止出现发送命令不成功，发送3次

	for (int i = 0; i < 3; i++)
	{
		Set_Can_Motor_Enable(FRONT_LEFT, 1);
		osDelay(5);

		Set_Can_Motor_Enable(FRONT_RIGHT, 1);
		osDelay(5);

		Set_Can_Motor_Enable(BACK_LEFT, 1);
		osDelay(5);

		Set_Can_Motor_Enable(BACK_RIGHT, 1);
		osDelay(5);

		Set_Can_Motor_Enable(FL_Clip, 1);
		osDelay(5);

		Set_Can_Motor_Enable(BL_Clip, 1);
		osDelay(5);

		Set_Can_Motor_Enable(FR_Clip, 1);
		osDelay(5);

		Set_Can_Motor_Enable(BR_Clip, 1);

		HAL_Delay(100);
	}

	
}
void SetMotorDisable(void)//抱夹时，行走电机关使能
{
	for (int i = 0; i < 3; ++i)
	{
		//防止出现发送命令不成功，发送3次
		Set_Can_Motor_Disable(FRONT_LEFT, 1);
		//HAL_Delay(100);
		osDelay(5);

		Set_Can_Motor_Disable(FRONT_RIGHT, 1);
		//HAL_Delay(100);
		osDelay(5);

		Set_Can_Motor_Disable(BACK_LEFT, 1);
		//HAL_Delay(100);
		osDelay(5);

		Set_Can_Motor_Disable(BACK_RIGHT, 1);
		HAL_Delay(100);
		
	}
	
}
void Set_Can_Motor_Disable(uint8_t can_channel, int8_t count)//抱夹时，行走电机关使能
{
	int i;
	uint8_t can_buf[8];
	can_buf[0] = 0x2B;
	can_buf[1] = 0x40;
	can_buf[2] = 0x60;
	can_buf[3] = 0x00;
	can_buf[4] = 0x06;
	can_buf[5] = 0x00;
	can_buf[6] = 0x00;
	can_buf[7] = 0x00;
	for (i = 0; i < count; i++)
	{
		Can_Send_Msg(can_buf, 8, can_channel);
		HAL_Delay(10);
	}
}
//int32_t rpm_32;
void Set_Can_RPM(uint8_t can_channel, int16_t rpm, int8_t count)
{
	int i;
	int32_t rpm_32;
	uint8_t can_buf[8];

	float	fRpm = LimitMax(rpm, RPM_LIMIT);
	
	//转速转换为脉冲值公式
	//fRpm = fRpm * 512 * 10000 / 1875;
	fRpm = fRpm * RPM_TO_PULSE;

	rpm_32 = fRpm;

	can_buf[0] = 0x23;
	can_buf[1] = 0xFF;
	can_buf[2] = 0x60;
	can_buf[3] = 0x00;
	can_buf[4] = rpm_32;
	can_buf[5] = rpm_32 >> 8;
	can_buf[6] = rpm_32 >> 16;
	can_buf[7] = rpm_32 >> 24;

	for (i = 0; i < count; i++)
	{
		Can_Send_Msg(can_buf, 8, can_channel);
		//HAL_Delay(10);
		if (can_channel == 8)CanDatalLossRate++;
	}
}

void Set_Can_RPM_RPDO(uint8_t can_channel, int16_t rpm, int8_t count)
{
	int i;
	int32_t rpm_32;
	uint8_t can_buf[8];

	float	fRpm = LimitMax(rpm, RPM_LIMIT);
	
	//转速转换为脉冲值公式
	//fRpm = fRpm * 512 * 10000 / 1875;
	fRpm = fRpm * RPM_TO_PULSE;

	rpm_32 = fRpm;


	can_buf[0] = rpm_32;
	can_buf[1] = rpm_32 >> 8;
	can_buf[2] = rpm_32 >> 16;
	can_buf[3] = rpm_32 >> 24;
	can_buf[4] = 0x00;
	can_buf[5] = 0x00;
	can_buf[6] = 0x00;
	can_buf[7] = 0x00;

	for (i = 0; i < count; i++)
	{
		Can_Send_Msg_RPDO(can_buf, 8, can_channel);
		//HAL_Delay(10);
		if (can_channel == 8)CanDatalLossRate++;
	}
}


//里程计算
void Odometer_Solution(void)//里程计解算
{
	//	float Vf,Vr;//Vf=前面左右两个轮子的运动速度之和 (单位mm/s)  ;   Vr=后面左右两个轮子的运动速度之和  (单位mm/s)
		//float COEFFICIENT;//RPM系数
	float Vm, Wm, Vmx, Vmy;//中间点的线速度和角速度
//	float Af,Ar,Am;//Af=前差动单元偏角 (单位：度) ;Ar=后差动单元偏角 (单位：度) ;Am=中间点的角度 (单位:度)
	float Am;
//	float Vfcos;//正负系数
	float L;//轴距

	if (EncoderMiddleValue <= Encoder_Front.val)
		Af = abs((EncoderMiddleValue - Encoder_Front.val))*0.0251116*DEG_TO_RAD;//中间初始值-当前角度  360/14336=0.0251116 
	else
		Af = abs((EncoderMiddleValue - Encoder_Front.val))*-1 * 0.0251116*DEG_TO_RAD;//中间初始值-当前角度  360/14336=0.0251116 

	if (EncoderMiddleValue <= Encoder_Back.val)
		Ar = abs((EncoderMiddleValue - Encoder_Back.val))*0.0251116* DEG_TO_RAD  ;//中间初始值-当前角度  360/14336=0.0251116 
	else
		Ar = abs((EncoderMiddleValue - Encoder_Back.val))*-1 * 0.0251116*DEG_TO_RAD;//中间初始值-当前角度  360/14336=0.0251116 

	Vf = (MotorFL.ACTUAL_RPM + MotorFR.ACTUAL_RPM*-1) *0.5 * RPM_TO_VMM;//前面两个轮子的实际线速度,注：右轮前进时为负速度
	Vr = (MotorBL.ACTUAL_RPM + MotorBR.ACTUAL_RPM*-1) *0.5 * RPM_TO_VMM;//后面两个轮子的实际线速度


	Wheelbase_Calculation();//轴距计算程序
	L = Wheelbase_Data + 970;//轴距：单位毫米

	
	Vmy = Vf * cos(Af);
	Vmx = -(Vf*sin(Af) + Vr * sin(Ar)) / 2.0;//98.1497269


	Wm = (Vf*sin(Af) - Vr * sin(Ar)) / L;//中间点角速度
	Vm = sqrt(Vmx*Vmx + Vmy * Vmy);
	Am = atan(Vmx / Vmy);//
	CAR_Odometer.RT_VY = Vmy;//(Vf*sin(Af)+Vr*sin(Ar))/2;//Vf*cos(Af);//Vm*sin(Am);
	CAR_Odometer.RT_VX = Vmx;//Vf*cos(Af);//Vm*cos(Am);


	CAR_Odometer.RT_RPS = Wm;

	CAR_Odometer.ODOMETER_Y += CAR_Odometer.RT_VY*ODOMETER_Y_COEFFICIENT;
	CAR_Odometer.ODOMETER_X += CAR_Odometer.RT_VX*ODOMETER_X_COEFFICIENT*(-1);
	CAR_Odometer.ODOMETER_Z += CAR_Odometer.RT_RPS*ODOMETER_Z_COEFFICIENT;
	CAR_Odometer.PC_ODOMETER_Z = CAR_Odometer.ODOMETER_Z * 1800 / 3.1415926f;

	if (CAR_Odometer.ODOMETER_X > 32767)
	{
		CAR_Odometer.ODOMETER_X -= 65536;
	}
	else if (CAR_Odometer.ODOMETER_X < -32768)
	{
		CAR_Odometer.ODOMETER_X += 65536;
	}

	if (CAR_Odometer.ODOMETER_Y > 32767)
	{
		CAR_Odometer.ODOMETER_Y -= 65536;
	}
	else if (CAR_Odometer.ODOMETER_Y < -32768)
	{
		CAR_Odometer.ODOMETER_Y += 65536;
	}

	if (CAR_Odometer.PC_ODOMETER_Z > 32767)
	{
		CAR_Odometer.PC_ODOMETER_Z -= 65536;
	}
	else if (CAR_Odometer.PC_ODOMETER_Z < -32768)
	{
		CAR_Odometer.PC_ODOMETER_Z += 65536;
	}

	agvStatus.odoX= CAR_Odometer.ODOMETER_X;               //里程计X mm
	agvStatus.odoY= CAR_Odometer.ODOMETER_Y;               //里程计Y mm
	agvStatus.odoZ= CAR_Odometer.PC_ODOMETER_Z;               //里程计角度 0.1
	
}


void  PID_Init(PID* pid, float kp, float ki, float kd,float limit)
{
	pid->Kp = kp;
	pid->Ki = ki;
	pid->Kd = kd;
	pid->target_val=0;           //目标值
	pid->actual_val=0;        		//实际值
	pid->err=0;             			//定义偏差值
	pid->err_last=0;          		//定义上一个偏差值
	pid->err_last_last=0;        //上上次偏差值
	pid->integral=0;          		//定义积分值
	pid->Limit = limit;
}

//位置式输出
float PID_Pos_Output(PID* pid, float temp_val)
{
	/*计算目标值与实际值的误差*/
	pid->err = pid->target_val - temp_val;
	/*误差累积*/
	pid->integral += pid->err;
	/*PID算法实现*/
	pid->actual_val = pid->Kp*pid->err + pid->Ki*pid->integral + pid->Kd*(pid->err - pid->err_last);
	/*误差传递*/
	pid->err_last = pid->err;
	/*返回当前实际值*/
	return pid->actual_val;
}

//增量型输出
float PID_Inc_Output(PID* pid,float setVal)
{
	/*传入目标值*/
	pid->target_val = setVal;
	/*计算目标值与实际值的误差*/
	pid->err = pid->target_val - pid->actual_val;
	/*PID算法实现*/
	float increment_val = pid->Kp*(pid->err - pid->err_last) + pid->Ki*pid->err + pid->Kd*(pid->err - 2 * pid->err_last + pid->err_last_last);
	/*累加*/
	//限幅
	if (increment_val > pid->Limit)
	{
		pid->actual_val += pid->Limit;
	}
	else if (increment_val < -pid->Limit)
	{
		pid->actual_val -= pid->Limit;
	}
	else
	{
		pid->actual_val += increment_val;
	}
		
	/*传递误差*/
	pid->err_last_last = pid->err_last;
	pid->err_last = pid->err;
	/*返回当前实际值*/
	return pid->actual_val;
}

//增量型输出
float PID_Inc_ErrOutput(PID* pid, float err)
{
	pid->err = err;
	/*PID算法实现*/
	float increment_val = pid->Kp*(pid->err - pid->err_last) + pid->Ki*pid->err + pid->Kd*(pid->err - 2 * pid->err_last + pid->err_last_last);
	/*累加*/
	pid->actual_val += increment_val;
	/*传递误差*/
	pid->err_last_last = pid->err_last;
	pid->err_last = pid->err;
	/*返回当前实际值*/
	return pid->actual_val;
}


//前轮是否静止
int IsFWheelStoped(void)
{
	if(abs(MotorFL.ACTUAL_RPM) < RPMSTOP && abs(MotorFR.ACTUAL_RPM) < RPMSTOP)
	{
		return 1;
	}
	return 0;
}

//后轮是否静止
int IsBWheelStoped(void)
{
	if(abs(MotorBL.ACTUAL_RPM) < RPMSTOP && abs(MotorBR.ACTUAL_RPM) < RPMSTOP)
	{
		return 1;
	}
	return 0;
}

//是否移动静止
int IsMoveStoped(void)
{
	if (IsFWheelStoped() + IsBWheelStoped()>1)
	{
		return 1;
	}
	return 0;
}


int MoveReady(void)
{

	int stop = Back_Differential_Unit_Adjustment(600)+ Front_Differential_Unit_Adjustment(600);
	//if ((Encoder_Back.val > (agvStatus.ecKeepAngle + ZeroAbout)) || (Encoder_Back.val < (agvStatus.ecKeepAngle - ZeroAbout)))
	//{
	//	Back_Differential_Unit_Adjustment();//后轮没到位，调整
	//}
	//else
	//{
	//	stop++;
	//	Stop_Back_Wheel();//后差动单元停止
	//}
	//if ((Encoder_Front.val > (agvStatus.ecKeepAngle + ZeroAbout)) || (Encoder_Front.val < (agvStatus.ecKeepAngle - ZeroAbout)))
	//{
	//	Front_Differential_Unit_Adjustment();//前轮没到位，调整
	//}
	//else
	//{
	//	stop++;
	//	Stop_Front_Wheel();//前差动单元停止
	//}
	if (stop == 0)
	{
		agvStatus.outputRpm = 0;
		Pcount = 0;
		FrontReadyRpm = 0;
		BackReadyRpm = 0;
		return IsMoveStoped();
	
	}
	return 0;
}

int pc_moving_speed = 0;
float testoutput1 = 0;
float testoutput3 = 0;

void AutoMove(int16_t speed ,float fAngleDelta,float bAngleDelta,uint8_t slowdown)
{
	deflection_angleF = agvStatus.ecKeepAngle - Encoder_Front.val;//编码器需要达到的角度-当前角度
	deflection_angleB = agvStatus.ecKeepAngle - Encoder_Back.val;//编码器需要达到的角度-当前角度

	//运动过程角度偏差过大，立即停止
	if (CheckECOverLimit(deflection_angleF, deflection_angleB, MoveEcOffsetLimit))
	{
		return;
	}
	deflection_angleF *= Encoder_To_Angle;
	deflection_angleB *= Encoder_To_Angle;
	if (slowdown>0)
	{
		//RpmPID.actual_val = agvStatus.outputRpm;
		//agvStatus.outputRpm = PID_Inc_Output(&RpmPID, 0);
		agvStatus.outputRpm = RpmUpdate(8, 0, agvStatus.outputRpm);
		if (fabs(agvStatus.outputRpm) < 80)
		{
			Stop_Moving();
			return;
		}		
	}
	else
	{
		float rpm = speed* VMM_TO_RPM;
		
		if (Pcount < 400)
		{
				Pcount++;
			 if (Pcount < 200)
			{
				agvStatus.outputRpm = RpmUpdate(5, rpm, agvStatus.outputRpm);
			}
			else
			{
				agvStatus.outputRpm = RpmUpdate(10, rpm, agvStatus.outputRpm);
			}
		}
		else
		{
			agvStatus.outputRpm = rpm;
		}
	}
	deflection_angleF -= fAngleDelta;
	deflection_angleB -= bAngleDelta;
	//逆时针为负顺时针为正
	KeepMove(agvStatus.outputRpm, deflection_angleF, deflection_angleB);

}


void AutoRotation(float Rot_spd, uint8_t slowdown)
{


	deflection_angleF = agvStatus.ecKeepAngle - Encoder_Front.val;//编码器需要达到的角度-当前角度
	deflection_angleB = agvStatus.ecKeepAngle - Encoder_Back.val;//编码器需要达到的角度-当前角度
	

	//运动过程角度偏差过大，立即停止
	if (CheckECOverLimit(deflection_angleF, deflection_angleB, TurnEcOffsetLimit))
	{
		return;
	}

	F = Wheelbase_Data *0.5f;
	//两轮之间的速度比例
	px = (F + 485.0 + B) / (F + 485.0 - B);         //速度系数计算，轴距加485mm
	

	if (slowdown>0)
	{
		agvStatus.outputRpm = RpmUpdate(20, 0, agvStatus.outputRpm);   //加减速程序，单位是转每分钟
		if (fabs(agvStatus.outputRpm) < 100)
		{		
			Stop_Moving();
			return;
		}
							
	}
	else
	{
		//角速度转换为轮子转速  角速度 x 二分之一轮间距
		float rpm = Rot_spd * (F + 485.0 - B) * VMM_TO_RPM;
		//rpm = LimitMin(rpm, 2);
		
		if (Pcount < 300)
		{
			Pcount++;
			agvStatus.outputRpm = RpmUpdate(5, rpm, agvStatus.outputRpm);
			
		}
		else
		{
			agvStatus.outputRpm = RpmUpdate(10, rpm, agvStatus.outputRpm);   //加减速程序，单位是转每分钟
		}
		
	}

	KeepRotation(agvStatus.outputRpm, px, deflection_angleF*Encoder_To_Angle, deflection_angleB*Encoder_To_Angle);

}

float LimitMax(float val, float limit)
{
	if (val > limit) return limit;
	else if (val < -limit) return -limit;
	return val;
}

float LimitMin(float val, float limit)
{
	if ((val < limit) && (val > -limit))
	{
		if (val < 0)
		{
			return -limit;
		}
		return limit;
	}
	return val;
}


//float testfl,testfr,testbl,testbr;
//float rpmtest;
//float fpeed;
//保持旋转
void KeepRotation(float rpm ,float out_in_rpmScale, float fAngleDelta, float bAngleDelta)
{
	float rbm_ab = fabs(rpm);
	int scale = 10 + (rbm_ab)*0.05f;

	AccelerateSpeedF = scale * fAngleDelta;
	AccelerateSpeedB = scale * bAngleDelta;
	AccelerateSpeedF = LimitMax(AccelerateSpeedF, 300);
	AccelerateSpeedB = LimitMax(AccelerateSpeedB, 300);

	/*if (rbm_ab < 50)
	{
		AccelerateSpeedF = 0;
		AccelerateSpeedB = 0;
	}*/

	//原地旋转时，前轮右轮是内侧，左轮是外侧；后轮左轮是内侧，右轮是外侧
	float flSpd = -(rpm- AccelerateSpeedF) * out_in_rpmScale;
	float frSpd = rpm+ AccelerateSpeedF;
	float blSpd = rpm+ AccelerateSpeedB;
	float brSpd = -(rpm- AccelerateSpeedB) * out_in_rpmScale;
  /*fpeed=rpm;
	testfl=flSpd;
	testfr=frSpd;
	testbl=blSpd;
	testbr=brSpd;*/
    Set_Can_RPM_RPDO(BACK_LEFT, blSpd, SetCount); //左后轮负速度后退，正速度前进 
   // HAL_Delay(5);
	Set_Can_RPM_RPDO(BACK_RIGHT, brSpd, SetCount); //右后轮子负速度为前进，正速度为后退
	//HAL_Delay(5);
	Set_Can_RPM_RPDO(FRONT_LEFT, flSpd, SetCount); //左前轮负速度后退，正速度前进
	//HAL_Delay(5);
	Set_Can_RPM_RPDO(FRONT_RIGHT, frSpd, SetCount);//右前轮子负速度为前进，正速度为后退
	//HAL_Delay(5);
	

}

void KeepMove(float rpm,float fAngleOffset_Deg, float bAngleOffset_Deg)
{

	float rbm_ab = fabs(rpm);
	int scale = 10 + (rbm_ab)*0.04f;

	float Accelerate_Front = scale * fAngleOffset_Deg;
	float Accelerate_Back = scale * bAngleOffset_Deg;

	float limit = rbm_ab *0.2f;
    if(limit<50) limit=50;

	Accelerate_Front = LimitMax(Accelerate_Front, limit);
	Accelerate_Back = LimitMax(Accelerate_Back, limit);


	if (fabs(rbm_ab) < 50)
	{
		Accelerate_Front = 0;
		Accelerate_Back = 0;
	}

	Set_Can_RPM_RPDO(BACK_LEFT, rpm + Accelerate_Back, SetCount);     //左后轮
	Set_Can_RPM_RPDO(BACK_RIGHT, -(rpm - Accelerate_Back), SetCount); //右后轮
	Set_Can_RPM_RPDO(FRONT_LEFT, rpm + Accelerate_Front, SetCount);    //左前轮
	Set_Can_RPM_RPDO(FRONT_RIGHT, -(rpm - Accelerate_Front), SetCount);//右前轮
	

}


float testdef;
float testdeb;
//检测前后编码器是否超限
int CheckECOverLimit(float deltF, float deltB, float limit)
{
	//testdef=fabs(deltF);
	//testdeb=fabs(deltB);
	//运动过程角度偏差过大，立即停止
	if (fabs(deltF) > limit || fabs(deltB) > limit)
	{
		Stop_Moving();
		agvTask.state = TaskState_Stop;
		return 1;
	}
	return 0;
}
