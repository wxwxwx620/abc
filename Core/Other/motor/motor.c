/****************************************************************************
// 2022.01.22		����ǰ���ٶ���Ϊ���ٶȲο��������н��з�����ƫ			ds
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

float Wheelbase_Data;//�����ؽṹ�� 20221004


float Af, Ar, Vf, Vr;




/*������ X �᷽���ٶ�VX = ��VL + VR�� / 2�������� Z �᷽���ٶ�VZ = ��VR - VL�� / D��
���������ֵ��ٶ�VL = VX - ��VZ * D�� / 2�����������ֵ��ٶ�VR = VX + ��VZ * D�� / 2
�������£�
void Drive_Motor(float vx, float vz)
{
	Target_Left = vx - vz * WIDTH_OF_ROBOT / 2.0f; //��������ֵ�Ŀ���ٶ�
	Target_Right = vx + vz * WIDTH_OF_ROBOT / 2.0f; //��������ֵ�Ŀ��
}*/
//������תʱ���ٴ�С vz * WIDTH_OF_ROBOT / 2.0f

//float W,V; //W���ٶȣ� V=W*(L+B)   V=W*(L-B)
float WF, WR;
float B = 182.0;	//B����������ֱ�ӵļ��364mm 
int L = 2055;//���L=2055mm
float px;//ԭ����תʱ�����������ֵ��ٶ�ϵ����
int F;//�������
//float pc_speed_rpm;

//PID   pcMovPID_F;  //ǰ��PID����
//PID   pcMovPID_B;  //����PID����
//PID   RpmPID;      //�ٶȿ��ƿ���PID

float deflection_angleF, angle3;//ƫ���3������ֵ��������3��ƫ��Ƕ�ֵ
float deflection_angleB, angle1;//ƫ���1������ֵ��������1��ƫ���ֵ

float AccelerateSpeedB, AccelerateSpeedF;//3��ǰ���Ԫ��1�Ǻ���Ԫ


#define RPMSTOP 5

uint16_t Pcount = 0;
int16_t FrontReadyRpm = 0; //ǰ����ת����Ӧ������ٶ�
int16_t BackReadyRpm = 0;

//float testf = 0;
int Front_Differential_Unit_Adjustment(int maxRpm)//ǰ���Ԫ����(����Ƕ�)
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

int Back_Differential_Unit_Adjustment(int maxRpm)//����Ԫ����(����Ƕ�)
{
	int16_t rpm = 0;
	int32_t ecOffset = agvStatus.ecKeepAngle-Encoder_Back.val;
	if (ecOffset > ZeroAbout|| ecOffset < -ZeroAbout)  //��ƫ //��ƫ ��ʱ�����
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


//˳ʱ��Ϊ������ʱ��Ϊ��
void Front_Unit_Rotation(int16_t rpm)        //ǰ���Ԫ��ת
{
	Set_Can_RPM_RPDO(FRONT_LEFT, rpm, SetCount);
	//HAL_Delay(5);
	Set_Can_RPM_RPDO(FRONT_RIGHT, rpm, SetCount);
	//HAL_Delay(5);
}

//����Ԫ��ת ˳ʱ��Ϊ������ʱ��Ϊ��
void Behind_Unit_Rotation(int16_t rpm)      
{
	Set_Can_RPM_RPDO(BACK_LEFT, rpm, SetCount);
	//HAL_Delay(5);
	Set_Can_RPM_RPDO(BACK_RIGHT, rpm, SetCount);
	//HAL_Delay(5);
}

void Stop_Back_Wheel()//����Ԫֹͣ
{
	Set_Can_RPM_RPDO(BACK_LEFT, 0, SetCount);
	//HAL_Delay(5);
	Set_Can_RPM_RPDO(BACK_RIGHT, 0, SetCount);
	//HAL_Delay(5);
}

void Stop_Front_Wheel(void)//ǰ���Ԫֹͣ
{
	Set_Can_RPM_RPDO(FRONT_LEFT, 0, SetCount);
	//HAL_Delay(5);
	Set_Can_RPM_RPDO(FRONT_RIGHT, 0, SetCount);
	//HAL_Delay(5);
}

void Stop_Moving(void)//���ֹͣ�ƶ�
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


float RpmUpdate(float acc, float  targetSpd, float curSpd)//�Ӽ��ٳ���					//���޸�ΪPID���ƣ���������
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


float RpmUpdateWithDec(float acc,float dec, float  targetSpd, float curSpd)//�Ӽ��ٳ���					//���޸�ΪPID���ƣ���������
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
	//��ֹ���ַ�������ɹ�������3��

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
void SetMotorDisable(void)//����ʱ�����ߵ����ʹ��
{
	for (int i = 0; i < 3; ++i)
	{
		//��ֹ���ַ�������ɹ�������3��
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
void Set_Can_Motor_Disable(uint8_t can_channel, int8_t count)//����ʱ�����ߵ����ʹ��
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
	
	//ת��ת��Ϊ����ֵ��ʽ
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
	
	//ת��ת��Ϊ����ֵ��ʽ
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


//��̼���
void Odometer_Solution(void)//��̼ƽ���
{
	//	float Vf,Vr;//Vf=ǰ�������������ӵ��˶��ٶ�֮�� (��λmm/s)  ;   Vr=���������������ӵ��˶��ٶ�֮��  (��λmm/s)
		//float COEFFICIENT;//RPMϵ��
	float Vm, Wm, Vmx, Vmy;//�м������ٶȺͽ��ٶ�
//	float Af,Ar,Am;//Af=ǰ���Ԫƫ�� (��λ����) ;Ar=����Ԫƫ�� (��λ����) ;Am=�м��ĽǶ� (��λ:��)
	float Am;
//	float Vfcos;//����ϵ��
	float L;//���

	if (EncoderMiddleValue <= Encoder_Front.val)
		Af = abs((EncoderMiddleValue - Encoder_Front.val))*0.0251116*DEG_TO_RAD;//�м��ʼֵ-��ǰ�Ƕ�  360/14336=0.0251116 
	else
		Af = abs((EncoderMiddleValue - Encoder_Front.val))*-1 * 0.0251116*DEG_TO_RAD;//�м��ʼֵ-��ǰ�Ƕ�  360/14336=0.0251116 

	if (EncoderMiddleValue <= Encoder_Back.val)
		Ar = abs((EncoderMiddleValue - Encoder_Back.val))*0.0251116* DEG_TO_RAD  ;//�м��ʼֵ-��ǰ�Ƕ�  360/14336=0.0251116 
	else
		Ar = abs((EncoderMiddleValue - Encoder_Back.val))*-1 * 0.0251116*DEG_TO_RAD;//�м��ʼֵ-��ǰ�Ƕ�  360/14336=0.0251116 

	Vf = (MotorFL.ACTUAL_RPM + MotorFR.ACTUAL_RPM*-1) *0.5 * RPM_TO_VMM;//ǰ���������ӵ�ʵ�����ٶ�,ע������ǰ��ʱΪ���ٶ�
	Vr = (MotorBL.ACTUAL_RPM + MotorBR.ACTUAL_RPM*-1) *0.5 * RPM_TO_VMM;//�����������ӵ�ʵ�����ٶ�


	Wheelbase_Calculation();//���������
	L = Wheelbase_Data + 970;//��ࣺ��λ����

	
	Vmy = Vf * cos(Af);
	Vmx = -(Vf*sin(Af) + Vr * sin(Ar)) / 2.0;//98.1497269


	Wm = (Vf*sin(Af) - Vr * sin(Ar)) / L;//�м����ٶ�
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

	agvStatus.odoX= CAR_Odometer.ODOMETER_X;               //��̼�X mm
	agvStatus.odoY= CAR_Odometer.ODOMETER_Y;               //��̼�Y mm
	agvStatus.odoZ= CAR_Odometer.PC_ODOMETER_Z;               //��̼ƽǶ� 0.1
	
}


void  PID_Init(PID* pid, float kp, float ki, float kd,float limit)
{
	pid->Kp = kp;
	pid->Ki = ki;
	pid->Kd = kd;
	pid->target_val=0;           //Ŀ��ֵ
	pid->actual_val=0;        		//ʵ��ֵ
	pid->err=0;             			//����ƫ��ֵ
	pid->err_last=0;          		//������һ��ƫ��ֵ
	pid->err_last_last=0;        //���ϴ�ƫ��ֵ
	pid->integral=0;          		//�������ֵ
	pid->Limit = limit;
}

//λ��ʽ���
float PID_Pos_Output(PID* pid, float temp_val)
{
	/*����Ŀ��ֵ��ʵ��ֵ�����*/
	pid->err = pid->target_val - temp_val;
	/*����ۻ�*/
	pid->integral += pid->err;
	/*PID�㷨ʵ��*/
	pid->actual_val = pid->Kp*pid->err + pid->Ki*pid->integral + pid->Kd*(pid->err - pid->err_last);
	/*����*/
	pid->err_last = pid->err;
	/*���ص�ǰʵ��ֵ*/
	return pid->actual_val;
}

//���������
float PID_Inc_Output(PID* pid,float setVal)
{
	/*����Ŀ��ֵ*/
	pid->target_val = setVal;
	/*����Ŀ��ֵ��ʵ��ֵ�����*/
	pid->err = pid->target_val - pid->actual_val;
	/*PID�㷨ʵ��*/
	float increment_val = pid->Kp*(pid->err - pid->err_last) + pid->Ki*pid->err + pid->Kd*(pid->err - 2 * pid->err_last + pid->err_last_last);
	/*�ۼ�*/
	//�޷�
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
		
	/*�������*/
	pid->err_last_last = pid->err_last;
	pid->err_last = pid->err;
	/*���ص�ǰʵ��ֵ*/
	return pid->actual_val;
}

//���������
float PID_Inc_ErrOutput(PID* pid, float err)
{
	pid->err = err;
	/*PID�㷨ʵ��*/
	float increment_val = pid->Kp*(pid->err - pid->err_last) + pid->Ki*pid->err + pid->Kd*(pid->err - 2 * pid->err_last + pid->err_last_last);
	/*�ۼ�*/
	pid->actual_val += increment_val;
	/*�������*/
	pid->err_last_last = pid->err_last;
	pid->err_last = pid->err;
	/*���ص�ǰʵ��ֵ*/
	return pid->actual_val;
}


//ǰ���Ƿ�ֹ
int IsFWheelStoped(void)
{
	if(abs(MotorFL.ACTUAL_RPM) < RPMSTOP && abs(MotorFR.ACTUAL_RPM) < RPMSTOP)
	{
		return 1;
	}
	return 0;
}

//�����Ƿ�ֹ
int IsBWheelStoped(void)
{
	if(abs(MotorBL.ACTUAL_RPM) < RPMSTOP && abs(MotorBR.ACTUAL_RPM) < RPMSTOP)
	{
		return 1;
	}
	return 0;
}

//�Ƿ��ƶ���ֹ
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
	//	Back_Differential_Unit_Adjustment();//����û��λ������
	//}
	//else
	//{
	//	stop++;
	//	Stop_Back_Wheel();//����Ԫֹͣ
	//}
	//if ((Encoder_Front.val > (agvStatus.ecKeepAngle + ZeroAbout)) || (Encoder_Front.val < (agvStatus.ecKeepAngle - ZeroAbout)))
	//{
	//	Front_Differential_Unit_Adjustment();//ǰ��û��λ������
	//}
	//else
	//{
	//	stop++;
	//	Stop_Front_Wheel();//ǰ���Ԫֹͣ
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
	deflection_angleF = agvStatus.ecKeepAngle - Encoder_Front.val;//��������Ҫ�ﵽ�ĽǶ�-��ǰ�Ƕ�
	deflection_angleB = agvStatus.ecKeepAngle - Encoder_Back.val;//��������Ҫ�ﵽ�ĽǶ�-��ǰ�Ƕ�

	//�˶����̽Ƕ�ƫ���������ֹͣ
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
	//��ʱ��Ϊ��˳ʱ��Ϊ��
	KeepMove(agvStatus.outputRpm, deflection_angleF, deflection_angleB);

}


void AutoRotation(float Rot_spd, uint8_t slowdown)
{


	deflection_angleF = agvStatus.ecKeepAngle - Encoder_Front.val;//��������Ҫ�ﵽ�ĽǶ�-��ǰ�Ƕ�
	deflection_angleB = agvStatus.ecKeepAngle - Encoder_Back.val;//��������Ҫ�ﵽ�ĽǶ�-��ǰ�Ƕ�
	

	//�˶����̽Ƕ�ƫ���������ֹͣ
	if (CheckECOverLimit(deflection_angleF, deflection_angleB, TurnEcOffsetLimit))
	{
		return;
	}

	F = Wheelbase_Data *0.5f;
	//����֮����ٶȱ���
	px = (F + 485.0 + B) / (F + 485.0 - B);         //�ٶ�ϵ�����㣬����485mm
	

	if (slowdown>0)
	{
		agvStatus.outputRpm = RpmUpdate(20, 0, agvStatus.outputRpm);   //�Ӽ��ٳ��򣬵�λ��תÿ����
		if (fabs(agvStatus.outputRpm) < 100)
		{		
			Stop_Moving();
			return;
		}
							
	}
	else
	{
		//���ٶ�ת��Ϊ����ת��  ���ٶ� x ����֮һ�ּ��
		float rpm = Rot_spd * (F + 485.0 - B) * VMM_TO_RPM;
		//rpm = LimitMin(rpm, 2);
		
		if (Pcount < 300)
		{
			Pcount++;
			agvStatus.outputRpm = RpmUpdate(5, rpm, agvStatus.outputRpm);
			
		}
		else
		{
			agvStatus.outputRpm = RpmUpdate(10, rpm, agvStatus.outputRpm);   //�Ӽ��ٳ��򣬵�λ��תÿ����
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
//������ת
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

	//ԭ����תʱ��ǰ���������ڲ࣬��������ࣻ�����������ڲ࣬���������
	float flSpd = -(rpm- AccelerateSpeedF) * out_in_rpmScale;
	float frSpd = rpm+ AccelerateSpeedF;
	float blSpd = rpm+ AccelerateSpeedB;
	float brSpd = -(rpm- AccelerateSpeedB) * out_in_rpmScale;
  /*fpeed=rpm;
	testfl=flSpd;
	testfr=frSpd;
	testbl=blSpd;
	testbr=brSpd;*/
    Set_Can_RPM_RPDO(BACK_LEFT, blSpd, SetCount); //����ָ��ٶȺ��ˣ����ٶ�ǰ�� 
   // HAL_Delay(5);
	Set_Can_RPM_RPDO(BACK_RIGHT, brSpd, SetCount); //�Һ����Ӹ��ٶ�Ϊǰ�������ٶ�Ϊ����
	//HAL_Delay(5);
	Set_Can_RPM_RPDO(FRONT_LEFT, flSpd, SetCount); //��ǰ�ָ��ٶȺ��ˣ����ٶ�ǰ��
	//HAL_Delay(5);
	Set_Can_RPM_RPDO(FRONT_RIGHT, frSpd, SetCount);//��ǰ���Ӹ��ٶ�Ϊǰ�������ٶ�Ϊ����
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

	Set_Can_RPM_RPDO(BACK_LEFT, rpm + Accelerate_Back, SetCount);     //�����
	Set_Can_RPM_RPDO(BACK_RIGHT, -(rpm - Accelerate_Back), SetCount); //�Һ���
	Set_Can_RPM_RPDO(FRONT_LEFT, rpm + Accelerate_Front, SetCount);    //��ǰ��
	Set_Can_RPM_RPDO(FRONT_RIGHT, -(rpm - Accelerate_Front), SetCount);//��ǰ��
	

}


float testdef;
float testdeb;
//���ǰ��������Ƿ���
int CheckECOverLimit(float deltF, float deltB, float limit)
{
	//testdef=fabs(deltF);
	//testdeb=fabs(deltB);
	//�˶����̽Ƕ�ƫ���������ֹͣ
	if (fabs(deltF) > limit || fabs(deltB) > limit)
	{
		Stop_Moving();
		agvTask.state = TaskState_Stop;
		return 1;
	}
	return 0;
}
