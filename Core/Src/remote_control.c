#include "main.h"
#include "motor.h"
#include "remote_control.h"
#include "arm_math.h"
#include "usart.h"
#include "74HC595.h"
#include "voice.h"
#include "Alarm.h"
#include "encoder.h"
#include <stdlib.h>
#include "Protocol.h"

RCCtrl rc_ctrl;
PCCtrl pc_ctrl;
ctrl_mode_t ctrl_mode;
AGVTASK agvTask;       //当前任务
AGVSTATUS agvStatus;   //当前状态
ui_mode_t ui_t;
uint8_t  Sensor_Flag = 0;     //超声波传感器标志位：  =1第一次检测到轮子 //20221005
uint8_t  PowerReady = 0;      //上电成功
uint8_t  MotorPowerOff = 0;          //电机电源关闭

TimeoutCheck ECBackCheck;     //编码器1超时判断
TimeoutCheck ECAxleCheck;     //编码器2超时判断
TimeoutCheck ECFrontCheck;    //编码器3超时判断
TimeoutCheck PCCheck;              //上位机超时判断
TimeoutCheck RCCheck;              //遥控器超时判断
TimeoutCheck BMSCheck;             //BMS超时判断
TimeoutCheck MotorFLCheck;         //前左电机运动超时判断
TimeoutCheck MotorFRCheck;         //前右电机运动超时判断
TimeoutCheck MotorBLCheck;         //后左电机运动超时判断
TimeoutCheck MotorBRCheck;         //后右电机运动超时判断
TimeoutCheck MotorCFLCheck;        //前左包夹电机运动超时判断
TimeoutCheck MotorCFRCheck;        //前右包夹电机运动超时判断
TimeoutCheck MotorCBLCheck;        //后左包夹电机运动超时判断
TimeoutCheck MotorCBRCheck;        //后右包夹电机运动超时判断

uint32_t TaskKeepCount = 0;   //任务保持计数，防止在在较短时间内相同任务同时做。

AGVTASK pcTask;   //上位机任务
AGVTASK rcTask;   //遥控器任务
uint8_t InitOK = 0;

void RemoteInit(void)
{
	InitTimeout(&ECBackCheck, 20);
	InitTimeout(&ECAxleCheck, 200);
	InitTimeout(&ECFrontCheck, 20);
	InitTimeout(&PCCheck, 30);
	InitTimeout(&RCCheck, 40);

	InitTimeout(&BMSCheck, 200);
	InitTimeout(&MotorFLCheck, 20);
	InitTimeout(&MotorFRCheck, 20);
	InitTimeout(&MotorBLCheck, 20);
	InitTimeout(&MotorBRCheck, 20);
	InitTimeout(&MotorCFLCheck, 20);
	InitTimeout(&MotorCFRCheck, 20);
	InitTimeout(&MotorCBLCheck, 20);
	InitTimeout(&MotorCBRCheck, 20);

	//PID_Init(&pcMovPID_F, 0.2, 0.5, 0, 200);
	//PID_Init(&pcMovPID_B, 0.2, 0.5, 0, 200);
	//PID_Init(&RpmPID, 0.01, 0.015, 0, 50);
	memset(&agvTask, 0, sizeof(AGVTASK));
	memset(&agvStatus, 0, sizeof(AGVSTATUS));
	agvTask.cmd = Task_None;
	agvTask.state = TaskState_Finish;
	agvStatus.ecKeepAngle = EncoderZero;
	agvStatus.outputRpm = 0;
	FrontReadyRpm = 0; //前后轮转到对应方向的速度
	BackReadyRpm = 0;
	//等待上电成功
	/*while (PowerReady == 0)
	{
		osDelay(200);
	}*/
	Stop_Moving();
	osDelay(2000);
	//开机初始化，轮子转到0度
	Stop_Moving();
	/*while (1)
	{
		
		if (HandleError(1) == 0)
		{			
			int stop = Back_Differential_Unit_Adjustment(300)+ Front_Differential_Unit_Adjustment(300);
			if(stop==0)
			{
				break;
			}
			
		}
		osDelay(100);
	}*/

}

//判断需要停止的检查
int HandleError(int isInit)
{
	uint8_t errCode = 0;
	uint16_t errInfo = 0;

//	uint32_t encoderMin = EncoderRunMin;
//	uint32_t encoderMax = EncoderRunMax;
	//if (isInit == 1)
//	{
	//	encoderMin = EncoderReadMin;
//		encoderMax = EncoderReadMax;
//	}

	//===========行走电机错误=====================
	if (MotorFL.alarm > 0)
	{
		errCode = Err_MotorFL_Alarm;
		errInfo = MotorFL.alarm;
	}


	else if (MotorFR.alarm > 0)
	{
		errCode = Err_MotorFR_Alarm;
		errInfo = MotorFR.alarm;
	}

	else  if (MotorBL.alarm > 0)
	{
		errCode = Err_MotorBL_Alarm;
		errInfo = MotorBL.alarm;
	}


	else if (MotorBR.alarm > 0)
	{
		errCode = Err_MotorBR_Alarm;
		errInfo = MotorBR.alarm;
	}

	//==============包夹电机错误==============
	else if (MotorCFL.alarm > 0)
	{
		errCode = Err_MotorCFL_Alarm;
		errInfo = MotorCFL.alarm;
	}


	else if (MotorCFR.alarm > 0)
	{
		errCode = Err_MotorFR_Alarm;
		errInfo = MotorFR.alarm;
	}


	else if (MotorCBL.alarm > 0)
	{
		errCode = Err_MotorBL_Alarm;
		errInfo = MotorBL.alarm;
	}


	else if (MotorCBR.alarm > 0)
	{
		errCode = Err_MotorBR_Alarm;
		errInfo = MotorBR.alarm;
	}

	//编码通信超时
	else if (CheckTimeout(&ECBackCheck, Encoder_Back.update))
	{
		errCode = Err_BEC_Timeout;
	}

	else if (CheckTimeout(&ECAxleCheck, Encoder_Axle.update))
	{
		errCode = Err_DisEC_Timeout;

	}

	else if (CheckTimeout(&ECFrontCheck, Encoder_Front.update))
	{
		errCode = Err_FEC_Timeout;

	}
	else if (CheckTimeout(&MotorFLCheck, MotorFL.update))
	{
		errCode = Err_MotorFL_Timeout;
	}
	else if (CheckTimeout(&MotorFRCheck, MotorFR.update))
	{
		errCode = Err_MotorFR_Timeout;
	}
	else if (CheckTimeout(&MotorBLCheck, MotorBL.update))
	{
		errCode = Err_MotorBL_Timeout;
	}
	else if (CheckTimeout(&MotorBRCheck, MotorBR.update))
	{
		errCode = Err_MotorBR_Timeout;
	}
	else if (CheckTimeout(&MotorCFLCheck, MotorCFL.update))
	{
		errCode = Err_MotorCFL_Timeout;
	}


	else if (CheckTimeout(&MotorCFRCheck, MotorCFR.update))
	{
		errCode = Err_MotorCFR_Timeout;
	}
	else if (CheckTimeout(&MotorCBLCheck, MotorCBL.update))
	{
		errCode = Err_MotorCBL_Timeout;
	}
	else if (CheckTimeout(&MotorCBRCheck, MotorCBR.update))
	{
		errCode = Err_MotorCBR_Timeout;
	}
	//	else if (CheckTimeout(&BMSCheck, .update))
	//	{
	//		errCode = Err_MBS_Timeout;
	//	}
		//编码器超限
	else if (Encoder_Back.val < EncoderRunMin || Encoder_Back.val > EncoderRunMax)
	{
		if (Encoder_Back.val < EncoderReadMin || Encoder_Back.val > EncoderReadMax)
		{
			errCode = Err_BEC_OverLimit;
		}
		else if (isInit == 0)
		{
			Stop_Moving();
		}
	}

	else if (Encoder_Front.val < EncoderRunMin || Encoder_Front.val > EncoderRunMax)
	{
		if (Encoder_Back.val < EncoderReadMin || Encoder_Back.val > EncoderReadMax)
		{
			errCode = Err_FEC_OverLimit;
		}
		else if (isInit == 0)
		{
			Stop_Moving();
		}
	}


	/*else if ()
	{
		errCode = Err_MotorFL_Alarm;
		errInfo = MotorFL.alarm;
	}


	else if ()
	{
		errCode = Err_MotorFL_Alarm;
		errInfo = MotorFL.alarm;
	}*/


	/* else if (MotorFR.alarm > 0)
	 {
		 errCode = Err_MotorFL_Alarm;
		 errInfo = MotorFL.alarm;
	 }


	 else if (MotorFR.alarm > 0)
	 {
		 errCode = Err_MotorFL_Alarm;
		 errInfo = MotorFL.alarm;
	 }*/

	 //保存电机未关闭之前的错误
	if (MotorPowerOff == 0)
	{
		agvStatus.errorCode = errCode;
		agvStatus.errorInfo = errInfo;
	}


	if (errCode > 0)
	{
		Stop_Moving();
		if (MotorPowerOff == 0)
		{
			Motor_Power_Off();
			MotorPowerOff = 1;
		}	
		return 1;
	}

	return 0;
}

void srcdata_to_rc(volatile uint8_t *srcdata)
{
	if (srcdata == NULL)
	{
		return;
	}

	rc_ctrl.EmergencyStop = srcdata[4];        //   急停按扭
	rc_ctrl.Clip = srcdata[5];        //  抱夹开关              抱夹关01    抱夹停止02    抱夹开04
	rc_ctrl.SpeedGear = srcdata[6];        //   速度挡位
	rc_ctrl.OperationMode = srcdata[7];        //   操作模式
	rc_ctrl.OnlineOperation = srcdata[8];        //  上线操作
	rc_ctrl.WheelBaseAdjust = srcdata[9];        //  轴距调整   轴距前进：04   轴距停止：02    轴距后退：01

	rc_ctrl.MovingDirection = (srcdata[10] + (srcdata[11] << 8));     // 行走方向 0-360°
	rc_ctrl.MovingSpeed = (srcdata[12] + (srcdata[13] << 8));     // 行走速度 0-600mm/s
	rc_ctrl.RotationSpeed = (srcdata[14] + (srcdata[15] << 8));     // 旋转速度 0-20°/s

	rc_ctrl.RADv = rc_ctrl.RotationSpeed*0.1* 3.1415926 / 180.0;	//角速度 °/s   

}

void srcdata_to_pc(volatile uint8_t *srcdata)			//协议有调整1.24，需调整代码
{
	if (srcdata == NULL)
	{
		return;
	}
	pc_ctrl.cmd = srcdata[1];
	pc_ctrl.speed = (srcdata[2] << 8) + srcdata[3];
	pc_ctrl.deg = (srcdata[4] << 8) + srcdata[5];
	pc_ctrl.frontAngle = (srcdata[6] << 8) + srcdata[7];
	pc_ctrl.backAngle = (srcdata[8] << 8) + srcdata[9];
	pc_ctrl.em_stop = srcdata[10];
	pc_ctrl.numb = srcdata[11];
	pc_ctrl.voc_sta = srcdata[12];
	pc_ctrl.wheelBaseLength = (srcdata[13] << 8) + srcdata[14];//轴距需要调整到的距离 2500-3400mm

	if (pc_ctrl.cmd == Task_Turn)
	{
		pc_ctrl.rot_spd = pc_ctrl.deg;
	}
	else
	{
		pc_ctrl.rot_spd = 0;
	}

}


//更新控制数据
int  UpdateCtrlData(void)
{
	int8_t owner = agvStatus.owner;
	int pcTimeout = CheckTimeout(&PCCheck, pc_updateCount);
	int rcTimeout = CheckTimeout(&RCCheck, rc_updateCount);
	ui_t.NetworkingStatus = pcTimeout > 0 ? 0 : 1;
	ui_t.CommunicationAlarm = rcTimeout;

	//如果遥控器没有超时
	if (rcTimeout == 0)
	{
		//获取遥控器数据
		uint8_t rcData[18] = { 0 };
		rc_get_srcdata(rcData);
		srcdata_to_rc(rcData);

		//遥控器上线与下线与上位机相反
		agvStatus.onLineState = (rc_ctrl.OnlineOperation == RC_Offline) ? PC_Offline : PC_OnLine;
		//如果是自动按钮
		if (rc_ctrl.OperationMode == RC_Auto)
		{
			//如果当前是遥控器控制，释放遥控器控制权,如果是自动模式，保持不变
			if (agvStatus.owner == AgvOwner_RC)//mannual -> auto, stop; auto->auto, no action
			{
				Voice_Update(8);
				agvStatus.owner = AgvOwner_None;//释放控制权   
			}
			agvStatus.ctrlMode = PC_Auto;//自动
		}
		else  //如果是手动，收回遥控器控制权
		{
			if (agvStatus.owner != AgvOwner_RC)
			{
				Voice_Update(7);
			}
			agvStatus.owner = AgvOwner_RC;//抢控制权	
			agvStatus.ctrlMode = PC_Manual;//手动
			ctrl_mode.move.ACC_COEFFICIENT = 100;

		}

		//如果遥控器急停，收回遥控器控制权
		if ((rc_ctrl.EmergencyStop & 0x01) == 0x01) //wireless control,E-stop press down
		{
			agvStatus.owner = AgvOwner_RC;//抢控制权	
			ctrl_mode.state.scram_alarm_filter |= 0x01; //set e-stop flag, bit0

		}
		else	//E-stop press up
		{
			ctrl_mode.state.scram_alarm_filter &= 0xfe;	//清除急停									
			if (agvStatus.ctrlMode == PC_Manual)//手动
			{
				agvStatus.owner = AgvOwner_RC;//抢控制权
				ctrl_mode.move.ACC_COEFFICIENT = 100;
			}
			else
			{
				if (agvStatus.owner == AgvOwner_RC)
				{
					agvStatus.owner = AgvOwner_None;
				}

			}
		}
	}
	else
	{
		if (agvStatus.owner == AgvOwner_RC)
		{
			agvStatus.owner = AgvOwner_None;
			Voice_Update(2);//语音播遥控器已断开		
			rc_clear_srcdata();
			rc_ctrl.MovingDirection = 0;
			rc_ctrl.MovingSpeed = 0;
			rc_ctrl.RotationSpeed = 0;
		}
	}


	//如果上位机没有超时
	if (pcTimeout == 0)
	{
		//获取上位机数据
		uint8_t pcData[18] = { 0 };
		pc_get_srcdata(pcData);
		srcdata_to_pc(pcData);
		
		if (agvStatus.owner == AgvOwner_None)
		{
			if (pc_ctrl.cmd != Task_None)
			{
				agvStatus.owner = AgvOwner_PC;
			}
		}

	}
	else  //如果上位机超时
	{
		if (agvStatus.owner == AgvOwner_PC)
		{
			agvStatus.owner = AgvOwner_None;
			Voice_Update(4); //语音播上位机通讯失败
		}
		pc_clear_srcdata();
		pc_ctrl.speed = 0;
		pc_ctrl.rot_spd = 0;

	}

	if (rcTimeout > 0 && pcTimeout > 0)
	{
		agvStatus.owner = AgvOwner_None;//抢控制权
	}

	//切换控制模式
	if (owner != agvStatus.owner)
	{
		UpdateAgvStatus();
		if (agvStatus.owner == AgvOwner_PC)
		{
			//rc_clear_srcdata();
			rc_ctrl.MovingDirection = 0;
			rc_ctrl.MovingSpeed = 0;
			rc_ctrl.RotationSpeed = 0;
		}
		else if (agvStatus.owner == AgvOwner_RC)
		{
			pc_clear_srcdata();
			pc_ctrl.speed = 0;
			pc_ctrl.deg = 0;
		}
	}

	if (agvStatus.owner == AgvOwner_PC)
	{
		//根据上位机数据获取上位机任务
		pcTask.cmd = pc_ctrl.cmd;
		pcTask.speed = pc_ctrl.speed;			            //目标方向设定速度,mm/s
		pcTask.deg = pc_ctrl.deg;				            //目标方向与当前车体夹角 0.1度
		pcTask.frontAngle = pc_ctrl.frontAngle;			//前进方向前轮调整角度 0.1度
		pcTask.backAngle = pc_ctrl.backAngle;			//前进方向后轮调整角度 0.1度
		pcTask.rotSpeed = pc_ctrl.rot_spd;
		pcTask.wheelBaseLength = pc_ctrl.wheelBaseLength;   //轴距值 mm
		rcTask.cmd = Task_None;
	}
	else if (agvStatus.owner == AgvOwner_RC)
	{
		//根据遥控器数据获取遥控器的任务
		rcTask.cmd = Task_None;
		rcTask.speed = rc_ctrl.MovingSpeed;
		rcTask.rotSpeed = rc_ctrl.RotationSpeed;
		//遥控器限速

		if (rcTask.speed > 0)
		{
			if (rc_ctrl.SpeedGear == RC_SPEED_HIGH)
			{
				if (rcTask.speed > 1000)
				{
					rcTask.speed = 1000;
				}

				rcTask.rotSpeed = LimitMax(rcTask.rotSpeed, 200);

			}
			else if (rc_ctrl.SpeedGear == RC_SPEED_MID)
			{
				if (rcTask.speed > 500)
				{
					rcTask.speed = 500;
				}

				rcTask.rotSpeed = LimitMax(rcTask.rotSpeed, 100);
			}
		}


		int16_t deg = rc_ctrl.MovingDirection - 900;
		if (deg < 0)deg += 3600;
		rcTask.deg = deg;
		rcTask.wheelBaseLength = Wheelbase_Data;
		rcTask.frontAngle = 0;
		rcTask.backAngle = 0;

		if (rcTask.speed > 0)
		{
			rcTask.cmd = Task_Move;
		}
		else if (rcTask.rotSpeed != 0)
		{
			rcTask.cmd = Task_Turn;
		}
		//抱夹开关     抱夹夹车01    抱夹停止02    抱夹打开04
		else if (rc_ctrl.Clip == RC_CLAMP_CLOSE || rc_ctrl.Clip == RC_CLAMP_OPEN)
		{
			if (rc_ctrl.Clip == RC_CLAMP_CLOSE)
			{
				rcTask.cmd = Task_ClampClose;
			}
			else if (rc_ctrl.Clip == RC_CLAMP_OPEN)
			{
				rcTask.cmd = Task_ClampOpen;
			}
		}
		//  轴距调整   轴距前进：04   轴距停止：02    轴距后退：01  
		else if (rc_ctrl.WheelBaseAdjust == RC_AJUST_CLOSE || rc_ctrl.WheelBaseAdjust == RC_AJUST_OPEN)
		{
			if (rc_ctrl.WheelBaseAdjust == RC_AJUST_CLOSE)
			{
				rcTask.cmd = Task_AjustDis;
				rcTask.wheelBaseLength = WheelbaseLowerLimit;
			}
			else if (rc_ctrl.WheelBaseAdjust == RC_AJUST_OPEN)
			{
				rcTask.cmd = Task_AjustDis;
				rcTask.wheelBaseLength = WheelbaseUpperLimit;
			}
		}

		pcTask.cmd = Task_None;
	}
	else
	{
		pcTask.cmd = Task_None;
		rcTask.cmd = Task_None;
	}

	return 0;
}

void RC_CtrlAgv(void)
{
	

}

void PC_CtrlAgv(void)
{
	
}

//AGV 控制程序
void DoAgvCtrl(void)
{
	UpdateCtrlData();

	if (InitOK < 1)
	{
		if (HandleError(1) == 0)
		{
			int stop = Back_Differential_Unit_Adjustment(100) + Front_Differential_Unit_Adjustment(100);
			if (stop == 0)
			{
				InitOK=1;
			}
		}
		return;
	}

	int stop = 0;
	if ((ctrl_mode.state.scram_alarm_filter & 0x01) == 0x01 || agvStatus.stopButton > 0)
	{
		stop = 1;
	}
	//切换任务,如果是上位机控制，查看是否有优先级较高的任务
	if (agvStatus.owner == AgvOwner_PC)
	{
		if (Task_Reset == pcTask.cmd)
		{
			HandleReset();
			return;
		}
		else if (Task_MotorOff == pcTask.cmd)
		{
			HandleMotorPowerOff();
			return;
		}
		else if (Task_RestartMotor == pcTask.cmd)
		{
			HandleRestartMotor();
			return;
		}

		SwitchTask(&pcTask);
		if (pc_ctrl.em_stop > 0)
		{
			stop = 1;
		}
		Voice_Update(pc_ctrl.voc_sta);
	}
	else if (agvStatus.owner == AgvOwner_RC)
	{
		SwitchTask(&rcTask);
	}
	else
	{
		stop = 2;
	}


	if (HandleError(0) != 0) return;

	
	if (stop >0)
	{
		agvStatus.stopFlag=1;
		Stop_Moving();
		agvStatus.dirReady = MOVEREADY_NO;
		agvTask.state = TaskState_Stop;
		if (stop == 2)
		{
			/*	rc_clear_srcdata();
		rc_ctrl.MovingDirection = 0;*/
			rc_ctrl.MovingSpeed = 0;
			rc_ctrl.RotationSpeed = 0;
			pc_clear_srcdata();
			pc_ctrl.speed = 0;
			pc_ctrl.deg = 0;
		}
	}
	else
	{
		agvStatus.stopFlag=0;
		HandleTask();
	}

}

void SwitchTask(AGVTASK *pTask)   //切换任务
{
	//如果当前任务正在停止
	if (agvTask.state == TaskState_Stop) return;
	int bNeedStop = 0;
	//如果当前任务没有完成
	if (agvTask.state != TaskState_Finish)
	{
		TaskKeepCount = 0;
		//如果新任务与当前任务不同,停止当前任务
		if (agvTask.cmd != pTask->cmd)
		{
			bNeedStop = 1;
		}
		else   //两个任务相同
		{
			//移动方向改变需要停下来
			if (agvTask.cmd == Task_Move && agvTask.deg != pTask->deg)
			{
				bNeedStop = 1;
			}
			//旋转角度改变需要停下来
			else if (agvTask.cmd == Task_Turn )
			{
				if ((agvTask.rotSpeed > 0 && pTask->rotSpeed < 0) || (agvTask.rotSpeed < 0 && pTask->rotSpeed > 0))
				{
					bNeedStop = 1;
				}		
			}
		}
	}
	else  //任务完成，开始计数，防止相同时间内做相同任务
	{
		TaskKeepCount++;
		if (agvTask.cmd != pTask->cmd)
		{
			TaskKeepCount = 0;
			agvStatus.outputRpm = 0;
			agvStatus.clipState = GetClipState();
		}
		//防止较短时间内做相同的任务
		else if ((agvTask.cmd >Task_Turn && TaskKeepCount < 100))
		{
			return;
		}
		

		//if (agvTask.cmd != Task_AjustDis && agvTask.cmd != Task_FindDis && agvTask.cmd != Task_None)
		//{
		//	agvStatus.ajustDisState = AjustState_None;
		//	//Sensor_Flag = 0;
		//}
		if (pTask->cmd != Task_None)
		{
			agvTask.state = TaskState_Init;
			agvStatus.ajustDisState = AjustState_None;
		}
		else
		{
			agvTask.state = TaskState_Finish;
			if (TaskKeepCount > 60)
			{
				agvStatus.ajustDisState = AjustState_None;
			}
		}				
		
	}

	if (bNeedStop)
	{
		agvTask.state = TaskState_Stop;
		agvTask.speed = 0;
		agvTask.rotSpeed = 0;
		return;
	}

	agvTask.cmd = pTask->cmd;
	agvTask.speed = pTask->speed;			            //目标方向设定速度,mm/s
	agvTask.deg = pTask->deg;				            //目标方向与当前车体夹角 0.1度
	agvTask.frontAngle = pTask->frontAngle;			//前进方向前轮调整角度 0.1度
	agvTask.backAngle = pTask->backAngle;			//前进方向后轮调整角度 0.1度
	agvTask.rotSpeed = pTask->rotSpeed;
	agvTask.wheelBaseLength = pTask->wheelBaseLength;   //轴距值 mm

	//限速
	if (agvTask.speed > SPEED_LIMIT_MM) agvTask.speed = SPEED_LIMIT_MM;
	agvTask.rotSpeed = LimitMax(agvTask.rotSpeed, ROTSPEED_LIMIT_DEG * 10);


	//	if (agvTask.backAngle > rotLimit)
	//	{
	//		agvTask.backAngle = rotLimit;
	//	}
	//	else if (agvTask.backAngle < -rotLimit)
	//	{
	//		agvTask.backAngle = -rotLimit;
	//	}

	//	if (agvTask.frontAngle > rotLimit)
	//	{
	//		agvTask.frontAngle = rotLimit;
	//	}
	//	else if (agvTask.frontAngle < -rotLimit)
	//	{
	//		agvTask.frontAngle = -rotLimit;
	//	}


		//TODO 判断参数正确性


}


void HandleMove(void)    //处理移动
{
	if (agvTask.speed == 0)
	{
		agvTask.state = TaskState_Stop;
	}

	if (agvTask.state == TaskState_Init)
	{
		Stop_Moving();
		if (IsMoveStoped())
		{
			agvTask.state = TaskState_Start;
		}
	}
	else if (agvTask.state == TaskState_Start)
	{
		//0 度或者-90度
		if (agvTask.deg == 900 || agvTask.deg == 2700)
		{
			agvStatus.ecKeepAngle = EncoderAngle90;
		}
		else
		{
			agvStatus.ecKeepAngle = EncoderZero;
		}

		if (MoveReady())
		{
	
//			PID_Init(&pcMovPID_F, 0.2, 0.5, 0, 200);
	//		PID_Init(&pcMovPID_B, 0.2, 0.5, 0, 200);
//			PID_Init(&RpmPID, 0.01, 0.015, 0, 50);
			agvTask.state = TaskState_Run;
		}
	}
	else if (agvTask.state == TaskState_Run)
	{
		if (agvTask.deg == 0 || agvTask.deg == 2700)
		{
			AutoMove(agvTask.speed, agvTask.frontAngle*0.1, agvTask.backAngle*0.1, 0);
		}
		else if (agvTask.deg == 1800 || agvTask.deg == 900)
		{
			AutoMove(-agvTask.speed, agvTask.frontAngle*0.1, agvTask.backAngle*0.1, 0);
		}

	}
	else if (agvTask.state == TaskState_Stop)
	{
		//Stop_Moving();
		AutoMove(agvTask.speed, 0, 0, 1);
		if (IsMoveStoped())
		{
			agvTask.state = TaskState_Finish;
		}
	}
	else
	{
		Stop_Moving();
	}
}


void HandleTurn(void)    //处理旋转
{
	if (agvTask.rotSpeed == 0)
	{
		agvTask.state = TaskState_Stop;
	}
	if (agvTask.state == TaskState_Init)
	{
		Stop_Moving();
		if (IsMoveStoped())
		{
			agvTask.state = TaskState_Start;
		}
	}
	else if (agvTask.state == TaskState_Start)
	{
		agvStatus.ecKeepAngle = EncoderAngle90;
		if (MoveReady())
		{
			agvTask.state = TaskState_Run;
		}
	}
	else if (agvTask.state == TaskState_Run)
	{
		float rot_par = LimitMax(agvTask.rotSpeed*0.1*DEG_TO_RAD, 0.4);
		AutoRotation(rot_par, 0);

	}
	else if (agvTask.state == TaskState_Stop)
	{
		AutoRotation(0, 1);
		//Stop_Moving();
		if (IsMoveStoped())
		{
			agvTask.state = TaskState_Finish;
		}
	}
	else
	{
		Stop_Moving();
	}
}

//处理松夹
void HandleClampOpen(void)
{

	if (agvTask.state == TaskState_Init)
	{
		Stop_Moving();
		if (IsMoveStoped())
		{
			agvTask.state = TaskState_Start;
		}
	}
	else if (agvTask.state == TaskState_Start)
	{
		int clipState = GetClipState();	
		if (clipState == ClipState_Opened)
		{			
			agvStatus.clipState = clipState;
			agvTask.state = TaskState_Finish;
			return;
		}
		else if (clipState == ClipState_Closed)
		{
			MaxClipRpm = Clip_SPEED_HIGH;
		}
		else
		{
			MaxClipRpm = Clip_SPEED_LOW;
		}
		agvStatus.clipState = ClipState_Running;
		//停止运动,松夹不需要其他操作
		Stop_Moving();
		if (IsMoveStoped())
		{
			agvTask.state = TaskState_Run;
		}
	}
	else if (agvTask.state == TaskState_Run)
	{
		ClipPos.stopCount = 0;
		if (ClipOpen())
		{
			agvTask.state = TaskState_Stop;
		}

	}
	else if (agvTask.state == TaskState_Stop)
	{
		ClipStop();
		if (IsClipStop())
		{
			//TODO 判断夹闭状态，反馈上位机
			agvStatus.clipState = ClipState_Opened;
			agvTask.state = TaskState_Finish;

		}
	}
}

//处理夹车
void HandleClampClose(void)
{
	//包夹前准备
	if (agvTask.state == TaskState_Init)
	{
		Stop_Moving();
		int clipState = GetClipState();
		if (clipState == ClipState_Closed)
		{
			agvStatus.clipState = ClipState_Closed;
			agvTask.state = TaskState_Finish;
			return;
		}
		else if (clipState == ClipState_Opened)
		{
			MaxClipRpm = Clip_SPEED_HIGH;
		}
		else
		{
			MaxClipRpm = Clip_SPEED_LOW;
		}

		if (IsMoveStoped())
		{
			//如果是自动，判断是否有轮胎，如果有轮胎，才可以包夹
			/*if (agvStatus.owner == AgvOwner_PC)
			{
				if (CheckWheel()==0)
				{
					agvStatus.errorCode = Err_Wheel_NotFind;
					agvTask.state = TaskState_Finish;
					return 0;
				}
			}*/
			agvTask.state = TaskState_Start;
		}
	}
	else if (agvTask.state == TaskState_Start)
	{

		agvStatus.clipState = ClipState_Running;

		if (ClipReady(READY_START))
		{
			agvTask.state = TaskState_Run;
		}
	}
	else if (agvTask.state == TaskState_Run)
	{
		ClipPos.stopCount = 0;
		if (ClipClose())
		{
			agvTask.state = TaskState_Stop;
		}

	}
	else if (agvTask.state == TaskState_Stop)
	{
		//包夹结束准备
			//TODO 判断夹闭状态，反馈上位机
		if (ClipReady(READY_STOP))
		{
			agvStatus.clipState = ClipState_Closed;
			agvTask.state = TaskState_Finish;
		}

	}
}
void HandleCharge(void)
{
	Stop_Moving();
}
void HandleStopCharge(void)
{
	Stop_Moving();
}
void HandleAjustDis(void)
{
	if (agvTask.state == TaskState_Init)
	{
		agvStatus.ajustDisState = AjustState_Running;

		if (abs(agvStatus.wheelBaseLength - agvTask.wheelBaseLength) < 10)
		{
			agvStatus.ajustDisState = AjustState_OK;
			agvTask.state = TaskState_Finish;
			return;
		}
		else if ((agvTask.wheelBaseLength + 5.0) < Wheelbase_Data)
		{
			if (Wheelbase_Data <= WheelbaseLowerLimit)                      //轴距的下线是2400
			{
				agvStatus.ajustDisState = AjustState_Error;
				agvTask.state = TaskState_Finish;
				return;
			}
		}
		else if ((agvTask.wheelBaseLength - 5.0) > Wheelbase_Data)
		{
			if (Wheelbase_Data >= WheelbaseUpperLimit)                      //轴距上线是3200
			{
				agvStatus.ajustDisState = AjustState_Error;
				agvTask.state = TaskState_Finish;
				return;
			}
		}

		Stop_Moving();
		if (IsMoveStoped())
		{
			//自动状态下，判断包夹是否收回，没有收回，不能调整轴距，报错
			/*if (agvStatus.owner == AgvOwner_PC)
			{
				int clipState = GetClipState();
	            if (clipState != ClipState_Opened)
				{
					agvStatus.errorCode = Err_Clip_NotOpened;
					agvTask.state = TaskState_Finish;
					return 0;
				}
			}*/
			agvTask.state = TaskState_Start;
		}
	}
	else if (agvTask.state == TaskState_Start)
	{
		agvStatus.ecKeepAngle = EncoderZero;
		if (AjustDisReady(READY_START))
		{
			agvTask.state = TaskState_Run;
		}
	}
	else if (agvTask.state == TaskState_Run)
	{
		//TODO判断是否有错误
		int ret = AjustCarWheelDis();
		if (ret > 0)
		{
			if (ret == 1)
			{
				agvStatus.ajustDisState = AjustState_OK;
			}
			else
			{
				agvStatus.ajustDisState = AjustState_Error;
			}
			agvTask.state = TaskState_Stop;
		}
	}
	else if (agvTask.state == TaskState_Stop)
	{
		/*if (agvStatus.ajustDisState != AjustState_Error)
		{
			agvStatus.ajustDisState = AjustState_OK;
		}*/
		if (AjustDisReady(READY_STOP))
		{
			agvTask.state = TaskState_Finish;
		}
	}
	else
	{
		Stop_Moving();
	}
}
void HandleFindDis(void)
{
	if (agvTask.state == TaskState_Init)
	{
		//达到极限，停下来
		if (Wheelbase_Data <= WheelbaseLowerLimit - 30 || Wheelbase_Data >= WheelbaseUpperLimit+30)                      //轴距的下线是2400
		{
			agvStatus.ajustDisState = AjustState_Error;
			agvTask.state = TaskState_Finish;
			return;
		}

		agvStatus.ajustDisState = AjustState_Running;
		Stop_Moving();
		if (IsMoveStoped())
		{
			//自动状态下，判断包夹是否收回，没有收回，不能调整轴距，报错
			/*if (agvStatus.owner == AgvOwner_PC)
			{
				int clipState = GetClipState();
				if (clipState != ClipState_Opened)
				{
					agvStatus.errorCode = Err_Clip_NotOpened;
					agvTask.state = TaskState_Finish;
					return 0;
				}
			}*/
			agvTask.state = TaskState_Start;
		}
	}
	else if (agvTask.state == TaskState_Start)
	{
		agvStatus.ajustDisState = AjustState_Running;
		agvStatus.ecKeepAngle = EncoderZero;
		if (AjustDisReady(READY_START))
		{
			if (Wheelbase_Data >= WheelbaseUpperLimit-10)
			{
				Sensor_Flag = 10;
			}
			else
			{
				Sensor_Flag = 0;
			}
			
			agvTask.state = TaskState_Run;
		}
	}
	else if (agvTask.state == TaskState_Run)
	{
		//TODO判断是否有错误
		int ret = FindCarWheel();
		if (ret > 0)
		{
			if (ret == 1)
			{
				agvStatus.ajustDisState = AjustState_OK;
			}
			else if (ret == 2)
			{
				agvStatus.ajustDisState = AjustState_Error;

			}
			agvTask.state = TaskState_Stop;
		}
				
	}
	else if (agvTask.state == TaskState_Stop)
	{
		/*if (agvStatus.ajustDisState != AjustState_Error)
		{
			agvStatus.ajustDisState = AjustState_OK;
		}*/
		if (AjustDisReady(READY_STOP))
		{
			agvTask.state = TaskState_Finish;
		}
	}
	else
	{
		Stop_Moving();
	}
}
void HandleNone(void)
{
	Stop_Moving();
	agvTask.state = TaskState_Finish;

}


void UpdateAgvStatus(void)   //更新agv状态
{

	agvStatus.clipState = GetClipState();          //包夹状态 
	agvStatus.chargeState=0;        //充电状态
	agvStatus.ajustDisState = AjustState_OK;      //轴距调整状态 	            
	agvStatus.wheelBaseLength = Wheelbase_Data;    //轴距值 mm
}

void HandleTask(void)                 //处理任务
{
	

	if (Task_Move == agvTask.cmd)
	{
		HandleMove();
	}
	else if (Task_Turn == agvTask.cmd)
	{
		HandleTurn();
	}
	else if (Task_ClampOpen == agvTask.cmd)
	{
		HandleClampOpen();
	}
	else if (Task_ClampClose == agvTask.cmd)
	{
		HandleClampClose();
	}
	else if (Task_AjustDis == agvTask.cmd)
	{
		HandleAjustDis();
	}
	else if (Task_FindDis == agvTask.cmd)
	{
		HandleFindDis();
	}
	else if (Task_Charge == agvTask.cmd)
	{
		HandleCharge();
	}
	else if (Task_StopCharge == agvTask.cmd)
	{
		HandleStopCharge();
	}
	/*else if (== agvTask.cmd)
	{

	}*/
	else
	{
		HandleNone();
	}


	if (agvTask.state == TaskState_Run)
	{
		agvStatus.dirReady = MOVEREADY_OK;
	}
	else
	{
		agvStatus.dirReady = MOVEREADY_NO;
	}
}


void HandleReset(void)                //处理复位
{
	agvStatus.errorCode = 0;
	agvStatus.ajustDisState = AjustState_None;
}
void HandleRestartMotor(void)         //处理重启电机
{
	if (MotorPowerOff == 1)
	{
		Restart_Motor_Power();
		MotorPowerOff = 0;
		osDelay(2000);
	}
	HandleReset();
}
void HandleMotorPowerOff(void)        //处理电机断电
{
	if (MotorPowerOff == 0)
	{
		Stop_Moving();
		Motor_Power_Off();
		MotorPowerOff = 1;
	}
}


