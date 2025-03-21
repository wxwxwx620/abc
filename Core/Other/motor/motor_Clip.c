/****************************************************************************
// 2022.08.12		抱夹车抱夹时改为位置控制模式
*****************************************************************************/
#include "main.h"
#include "motor.h"
#include "74HC595.h"
#include "arm_math.h"
#include "remote_control.h"
#include <stdlib.h>
#include "cmsis_os.h"

#define Trans_Ratio 10     //减速机的传动比
#define Gear_Ratio  13/17  //减速机与丝杠之间的齿轮比
#define Lead_Screw  10     //丝杠导程,单位：mm

#define Max_Lead_LF  270//264      //左前丝杠可以行走的最大导程,单位：mm
#define Max_Lead_RF  273//264      //右前丝杠可以行走的最大导程,单位：mm
#define Max_Lead_LB  266//264      //左后丝杠可以行走的最大导程,单位：mm
#define Max_Lead_RB  272//264      //左后丝杠可以行走的最大导程,单位：mm

#define Encoder_Resolution 10000 //编码器分辨率,电机旋转一周对应的值

#define CLIP_SENSOR_DETECT_YES 0  //包夹到位传感器信号到位
#define CLIP_SENSOR_DETECT_NO  1  //包夹到位传感器信号未到位

ClipPosion ClipPos;

int16_t FClipRpm = 0;   //前包夹速度
int16_t BClipRpm = 0;   //后包夹速度

int16_t MaxClipRpm = 1200;  //当前包夹最大运动速度

//编码器最大行程 0x500  1280

int ClipOpen()            //抱夹打开
{
	uint8_t limit = FLO | FLC << 1 | BRO << 2 | BRC << 3 | FRO << 4 | FRC << 5 | BLO << 6 | BLC << 7;
	if (limit == 0xAA)
	{
		ClipStop();

		MotorCFL.clipPos_open = MotorCFL.POSITION;//-1300
		MotorCFR.clipPos_open = MotorCFR.POSITION;//-1300	
		MotorCBL.clipPos_open = MotorCBL.POSITION;//-1300	
		MotorCBR.clipPos_open = MotorCBR.POSITION;//-1300	

		return 1;
	}
	else //分段抱夹，开
	{
		
		int16_t FCRPM = -Clip_SPEED_HIGH;
		int16_t BCRPM = -Clip_SPEED_HIGH;

		//左前距离
		MotorCFL.clip_distance = abs(MotorCFL.POSITION - MotorCFL.clipPos_close);
		//右前距离
		MotorCFR.clip_distance = abs(MotorCFR.POSITION - MotorCFR.clipPos_close);
		//左后距离
		MotorCBL.clip_distance = abs(MotorCBL.POSITION - MotorCBL.clipPos_close);
		//右后前离
		MotorCBR.clip_distance = abs(MotorCBR.POSITION - MotorCBR.clipPos_close);

		//前左右两夹臂距离差
		float O_distance_F_LR = abs(MotorCFL.clip_distance - MotorCFR.clip_distance);

		//后左右两夹臂距离差
		float O_distance_B_LR = abs(MotorCBL.clip_distance - MotorCBR.clip_distance);

		if (agvStatus.owner == AgvOwner_PC)
		{
			if (O_distance_F_LR > Clip_DISTANCE_ERR || O_distance_B_LR > Clip_DISTANCE_ERR)
			{
				ClipStop();
				return 0;
			}
		}
		


		//左前距离大于900或者右前距离大于900 ，快到终点总距离大概1280，用慢速档
		if ((MotorCFL.clip_distance > 900) || (MotorCFR.clip_distance > 900))
		{
			FCRPM = -Clip_SPEED_LOW;
		}

		//	//左后距离大于900或者右后距离大于900 ，用慢速档
		if ((MotorCBL.clip_distance > 900) | (MotorCBR.clip_distance > 900))
		{
			BCRPM = -Clip_SPEED_LOW;
		}
		//限制最大速度
		FCRPM = LimitMax(FCRPM, MaxClipRpm);
		BCRPM = LimitMax(BCRPM, MaxClipRpm);
		FClipRpm = RpmUpdate(4, FCRPM, FClipRpm);  //加速度控制
		BClipRpm = RpmUpdate(4, BCRPM, BClipRpm);   //加速度控制

		int16_t clipFL_rpm = FClipRpm;
		int16_t clipFR_rpm = FClipRpm;
		int16_t clipBL_rpm = BClipRpm;
		int16_t clipBR_rpm = BClipRpm;

		//判断前包夹
		if (O_distance_F_LR > Clip_DISTANCE)
		{
			//左边比右边距离大,左边速度快 左边减速
			if (MotorCFL.clip_distance > MotorCFR.clip_distance)
			{
				clipFL_rpm += Clip_SPEED_OFFSET;
			}
			//右边快，右边减速
			else
			{
				clipFR_rpm += Clip_SPEED_OFFSET;
			}
		}

		if (O_distance_B_LR > Clip_DISTANCE)
		{
			//左边比右边距离大  左边速度快，左边慢速
			if (MotorCBL.clip_distance > MotorCBR.clip_distance)
			{
				clipBL_rpm += Clip_SPEED_OFFSET;
			}
			//右边速度快，右边慢速
			else
			{
				clipBR_rpm += Clip_SPEED_OFFSET;
			}
		}

		if (CLIP_SENSOR_DETECT_YES == FLO)
		{
			clipFL_rpm = 0;
		}

		if (CLIP_SENSOR_DETECT_YES == FRO)  //如果前右传感器没有检测到
		{
			clipFR_rpm = 0;
		}

		if (CLIP_SENSOR_DETECT_YES == BLO)  //
		{
			clipBL_rpm = 0;
		}
		if (CLIP_SENSOR_DETECT_YES == BRO)   //
		{
			clipBR_rpm = 0;
		}
		Set_Can_RPM_RPDO(FL_Clip, clipFL_rpm, 1);
		Set_Can_RPM_RPDO(FR_Clip, clipFR_rpm, 1);
		Set_Can_RPM_RPDO(BL_Clip, clipBL_rpm, 1);
		Set_Can_RPM_RPDO(BR_Clip, clipBR_rpm, 1);
	}

	return 0;
}
int ClipClose()          //包夹闭合，开始夹车
{
	uint8_t limit = FLO | FLC << 1 | BRO << 2 | BRC << 3 | FRO << 4 | FRC << 5 | BLO << 6 | BLC << 7;

	if (limit == 0x55)
	{
		ClipStop();

		MotorCFL.clipPos_close = MotorCFL.POSITION;//0
		MotorCFR.clipPos_close = MotorCFR.POSITION;//0	
		MotorCBL.clipPos_close = MotorCBL.POSITION;//0
		MotorCBR.clipPos_close = MotorCBR.POSITION;//0	

		return 1;
	}
	else
	{
		int16_t FCRPM = Clip_SPEED_HIGH;
		int16_t BCRPM = Clip_SPEED_HIGH;
		//计算出四个电机对应的位置
		MotorCFL.clip_distance = abs(MotorCFL.POSITION - MotorCFL.clipPos_open);
		MotorCFR.clip_distance = abs(MotorCFR.POSITION - MotorCFR.clipPos_open);
		MotorCBL.clip_distance = abs(MotorCBL.POSITION - MotorCBL.clipPos_open);
		MotorCBR.clip_distance = abs(MotorCBR.POSITION - MotorCBR.clipPos_open);

		//前左右两个的距离差
		float O_distance_F_LR = abs(MotorCFL.clip_distance - MotorCFR.clip_distance);
		//后左右两个距离 差
		float O_distance_B_LR = abs(MotorCBL.clip_distance - MotorCBR.clip_distance);

		if (agvStatus.owner == AgvOwner_PC)
		{
			if (O_distance_F_LR > Clip_DISTANCE_ERR || O_distance_B_LR > Clip_DISTANCE_ERR)
			{
				ClipStop();
				return 0;
			}
		}

		//左前距离大于900或者右前距离大于900 ，快到终点总距离大概1280，用慢速档
		if ((MotorCFL.clip_distance > 900) || (MotorCFR.clip_distance > 900))
		{
			FCRPM = Clip_SPEED_LOW;
		}

		//	//左后距离大于900或者右后距离大于900 ，用慢速档
		if ((MotorCBL.clip_distance > 900) | (MotorCBR.clip_distance > 900))
		{
			BCRPM = Clip_SPEED_LOW;
		}

		//限制最大速度
		FCRPM = LimitMax(FCRPM, MaxClipRpm);
		BCRPM = LimitMax(BCRPM, MaxClipRpm);

		FClipRpm = RpmUpdate(5, FCRPM, FClipRpm);  //加速度控制
		BClipRpm = RpmUpdate(5, BCRPM, BClipRpm);   //加速度控制

		int16_t clipFL_rpm = FClipRpm;
		int16_t clipFR_rpm = FClipRpm;
		int16_t clipBL_rpm = BClipRpm;
		int16_t clipBR_rpm = BClipRpm;

		//判断前包夹
		if (O_distance_F_LR > Clip_DISTANCE)
		{
			//左边比右边距离大,左边速度快 左边减速
			if (MotorCFL.clip_distance > MotorCFR.clip_distance)
			{
				clipFL_rpm -= Clip_SPEED_OFFSET;
			}
			//右边快，右边减速
			else
			{
				clipFR_rpm -= Clip_SPEED_OFFSET;
			}
		}

		if (O_distance_B_LR > Clip_DISTANCE)
		{
			//左边比右边距离大  左边速度快，左边慢速
			if (MotorCBL.clip_distance > MotorCBR.clip_distance)
			{
				clipBL_rpm -= Clip_SPEED_OFFSET;
			}
			//右边速度快，右边慢速
			else
			{
				clipBR_rpm -= Clip_SPEED_OFFSET;
			}
		}

		if (CLIP_SENSOR_DETECT_YES == FLC)
		{
			clipFL_rpm = 0;
		}

		if (CLIP_SENSOR_DETECT_YES == FRC)  //如果前右传感器没有检测到
		{
			clipFR_rpm = 0;
		}

		if (CLIP_SENSOR_DETECT_YES == BLC)  //
		{
			clipBL_rpm = 0;
		}
		if (CLIP_SENSOR_DETECT_YES == BRC)   //
		{
			clipBR_rpm = 0;
		}
		Set_Can_RPM_RPDO(FL_Clip, clipFL_rpm, 1);
		Set_Can_RPM_RPDO(FR_Clip, clipFR_rpm, 1);
		Set_Can_RPM_RPDO(BL_Clip, clipBL_rpm, 1);
		Set_Can_RPM_RPDO(BR_Clip, clipBR_rpm, 1);
	}

	return 0;

}
void ClipStop()           //包夹停止
{
	FClipRpm = 0;
	BClipRpm = 0;
	//防止出现发送命令不成功，发送3次
	Set_Can_RPM_RPDO(BL_Clip, 0, 1);
	//HAL_Delay(2);
	Set_Can_RPM_RPDO(FL_Clip, 0, 1);
	//HAL_Delay(2);				
	Set_Can_RPM_RPDO(FR_Clip, 0, 1);
	//HAL_Delay(2);
	Set_Can_RPM_RPDO(BR_Clip, 0, 1);
}


int ClipReady(int readyMode)   //包夹准备
{
	if (readyMode==READY_START)   //开始动作
	{
		HC595_add(OUT_7);//继电器7打开，防止轴距不对，可以拉伸
		SetMotorDisable();//行走电机断使能，防止轴距不对，可以拉伸	
		osDelay(500);
		return 1;
	}
	else  //结束动作
	{
		ClipStop();
		if (IsClipStop())
		{
			HC595_del(OUT_7);//继电器7锁死，停止轴距自动调整
			SetMotorEnable();//开使能，重新使能运动
			osDelay(500);
			return 1;
		}	
	}

	return 0;
}


int IsClipStop()            //包夹运动是否停止
{
	if (ClipPos.clip_pos_LF == MotorCFL.POSITION
		&&ClipPos.clip_pos_RF == MotorCFR.POSITION
		&&ClipPos.clip_pos_LB == MotorCBL.POSITION
		&&ClipPos.clip_pos_RB == MotorCBR.POSITION)
	{
		ClipPos.stopCount++;
	}
	else
	{
		ClipPos.clip_pos_LF = MotorCFL.POSITION;
		ClipPos.clip_pos_RF = MotorCFR.POSITION;
		ClipPos.clip_pos_LB = MotorCBL.POSITION;
		ClipPos.clip_pos_RB = MotorCBR.POSITION;
		ClipPos.stopCount = 0;
	}

	if (ClipPos.stopCount > 3)
	{
		return 1;
	}

	return 0;
}

int GetClipState()
{
	uint8_t clip = FLO | FLC << 1 | BRO << 2 | BRC << 3 | FRO << 4 | FRC << 5 | BLO << 6 | BLC << 7;

	if (clip == 0x55) //关到位
	{
		return ClipState_Closed;          //包夹状态 
	}
	else if (clip == 0xAA)//开到位
	{
		return ClipState_Opened;          //包夹状态 
	}
	return  ClipState_Stoped;          //包夹状态 
	
}
