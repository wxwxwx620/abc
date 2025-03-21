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
AGVTASK agvTask;       //��ǰ����
AGVSTATUS agvStatus;   //��ǰ״̬
ui_mode_t ui_t;
uint8_t  Sensor_Flag = 0;     //��������������־λ��  =1��һ�μ�⵽���� //20221005
uint8_t  PowerReady = 0;      //�ϵ�ɹ�
uint8_t  MotorPowerOff = 0;          //�����Դ�ر�

TimeoutCheck ECBackCheck;     //������1��ʱ�ж�
TimeoutCheck ECAxleCheck;     //������2��ʱ�ж�
TimeoutCheck ECFrontCheck;    //������3��ʱ�ж�
TimeoutCheck PCCheck;              //��λ����ʱ�ж�
TimeoutCheck RCCheck;              //ң������ʱ�ж�
TimeoutCheck BMSCheck;             //BMS��ʱ�ж�
TimeoutCheck MotorFLCheck;         //ǰ�����˶���ʱ�ж�
TimeoutCheck MotorFRCheck;         //ǰ�ҵ���˶���ʱ�ж�
TimeoutCheck MotorBLCheck;         //�������˶���ʱ�ж�
TimeoutCheck MotorBRCheck;         //���ҵ���˶���ʱ�ж�
TimeoutCheck MotorCFLCheck;        //ǰ����е���˶���ʱ�ж�
TimeoutCheck MotorCFRCheck;        //ǰ�Ұ��е���˶���ʱ�ж�
TimeoutCheck MotorCBLCheck;        //������е���˶���ʱ�ж�
TimeoutCheck MotorCBRCheck;        //���Ұ��е���˶���ʱ�ж�

uint32_t TaskKeepCount = 0;   //���񱣳ּ�������ֹ���ڽ϶�ʱ������ͬ����ͬʱ����

AGVTASK pcTask;   //��λ������
AGVTASK rcTask;   //ң��������
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
	FrontReadyRpm = 0; //ǰ����ת����Ӧ������ٶ�
	BackReadyRpm = 0;
	//�ȴ��ϵ�ɹ�
	/*while (PowerReady == 0)
	{
		osDelay(200);
	}*/
	Stop_Moving();
	osDelay(2000);
	//������ʼ��������ת��0��
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

//�ж���Ҫֹͣ�ļ��
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

	//===========���ߵ������=====================
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

	//==============���е������==============
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

	//����ͨ�ų�ʱ
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
		//����������
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

	 //������δ�ر�֮ǰ�Ĵ���
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

	rc_ctrl.EmergencyStop = srcdata[4];        //   ��ͣ��Ť
	rc_ctrl.Clip = srcdata[5];        //  ���п���              ���й�01    ����ֹͣ02    ���п�04
	rc_ctrl.SpeedGear = srcdata[6];        //   �ٶȵ�λ
	rc_ctrl.OperationMode = srcdata[7];        //   ����ģʽ
	rc_ctrl.OnlineOperation = srcdata[8];        //  ���߲���
	rc_ctrl.WheelBaseAdjust = srcdata[9];        //  ������   ���ǰ����04   ���ֹͣ��02    �����ˣ�01

	rc_ctrl.MovingDirection = (srcdata[10] + (srcdata[11] << 8));     // ���߷��� 0-360��
	rc_ctrl.MovingSpeed = (srcdata[12] + (srcdata[13] << 8));     // �����ٶ� 0-600mm/s
	rc_ctrl.RotationSpeed = (srcdata[14] + (srcdata[15] << 8));     // ��ת�ٶ� 0-20��/s

	rc_ctrl.RADv = rc_ctrl.RotationSpeed*0.1* 3.1415926 / 180.0;	//���ٶ� ��/s   

}

void srcdata_to_pc(volatile uint8_t *srcdata)			//Э���е���1.24�����������
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
	pc_ctrl.wheelBaseLength = (srcdata[13] << 8) + srcdata[14];//�����Ҫ�������ľ��� 2500-3400mm

	if (pc_ctrl.cmd == Task_Turn)
	{
		pc_ctrl.rot_spd = pc_ctrl.deg;
	}
	else
	{
		pc_ctrl.rot_spd = 0;
	}

}


//���¿�������
int  UpdateCtrlData(void)
{
	int8_t owner = agvStatus.owner;
	int pcTimeout = CheckTimeout(&PCCheck, pc_updateCount);
	int rcTimeout = CheckTimeout(&RCCheck, rc_updateCount);
	ui_t.NetworkingStatus = pcTimeout > 0 ? 0 : 1;
	ui_t.CommunicationAlarm = rcTimeout;

	//���ң����û�г�ʱ
	if (rcTimeout == 0)
	{
		//��ȡң��������
		uint8_t rcData[18] = { 0 };
		rc_get_srcdata(rcData);
		srcdata_to_rc(rcData);

		//ң������������������λ���෴
		agvStatus.onLineState = (rc_ctrl.OnlineOperation == RC_Offline) ? PC_Offline : PC_OnLine;
		//������Զ���ť
		if (rc_ctrl.OperationMode == RC_Auto)
		{
			//�����ǰ��ң�������ƣ��ͷ�ң��������Ȩ,������Զ�ģʽ�����ֲ���
			if (agvStatus.owner == AgvOwner_RC)//mannual -> auto, stop; auto->auto, no action
			{
				Voice_Update(8);
				agvStatus.owner = AgvOwner_None;//�ͷſ���Ȩ   
			}
			agvStatus.ctrlMode = PC_Auto;//�Զ�
		}
		else  //������ֶ����ջ�ң��������Ȩ
		{
			if (agvStatus.owner != AgvOwner_RC)
			{
				Voice_Update(7);
			}
			agvStatus.owner = AgvOwner_RC;//������Ȩ	
			agvStatus.ctrlMode = PC_Manual;//�ֶ�
			ctrl_mode.move.ACC_COEFFICIENT = 100;

		}

		//���ң������ͣ���ջ�ң��������Ȩ
		if ((rc_ctrl.EmergencyStop & 0x01) == 0x01) //wireless control,E-stop press down
		{
			agvStatus.owner = AgvOwner_RC;//������Ȩ	
			ctrl_mode.state.scram_alarm_filter |= 0x01; //set e-stop flag, bit0

		}
		else	//E-stop press up
		{
			ctrl_mode.state.scram_alarm_filter &= 0xfe;	//�����ͣ									
			if (agvStatus.ctrlMode == PC_Manual)//�ֶ�
			{
				agvStatus.owner = AgvOwner_RC;//������Ȩ
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
			Voice_Update(2);//������ң�����ѶϿ�		
			rc_clear_srcdata();
			rc_ctrl.MovingDirection = 0;
			rc_ctrl.MovingSpeed = 0;
			rc_ctrl.RotationSpeed = 0;
		}
	}


	//�����λ��û�г�ʱ
	if (pcTimeout == 0)
	{
		//��ȡ��λ������
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
	else  //�����λ����ʱ
	{
		if (agvStatus.owner == AgvOwner_PC)
		{
			agvStatus.owner = AgvOwner_None;
			Voice_Update(4); //��������λ��ͨѶʧ��
		}
		pc_clear_srcdata();
		pc_ctrl.speed = 0;
		pc_ctrl.rot_spd = 0;

	}

	if (rcTimeout > 0 && pcTimeout > 0)
	{
		agvStatus.owner = AgvOwner_None;//������Ȩ
	}

	//�л�����ģʽ
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
		//������λ�����ݻ�ȡ��λ������
		pcTask.cmd = pc_ctrl.cmd;
		pcTask.speed = pc_ctrl.speed;			            //Ŀ�귽���趨�ٶ�,mm/s
		pcTask.deg = pc_ctrl.deg;				            //Ŀ�귽���뵱ǰ����н� 0.1��
		pcTask.frontAngle = pc_ctrl.frontAngle;			//ǰ������ǰ�ֵ����Ƕ� 0.1��
		pcTask.backAngle = pc_ctrl.backAngle;			//ǰ��������ֵ����Ƕ� 0.1��
		pcTask.rotSpeed = pc_ctrl.rot_spd;
		pcTask.wheelBaseLength = pc_ctrl.wheelBaseLength;   //���ֵ mm
		rcTask.cmd = Task_None;
	}
	else if (agvStatus.owner == AgvOwner_RC)
	{
		//����ң�������ݻ�ȡң����������
		rcTask.cmd = Task_None;
		rcTask.speed = rc_ctrl.MovingSpeed;
		rcTask.rotSpeed = rc_ctrl.RotationSpeed;
		//ң��������

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
		//���п���     ���мг�01    ����ֹͣ02    ���д�04
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
		//  ������   ���ǰ����04   ���ֹͣ��02    �����ˣ�01  
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

//AGV ���Ƴ���
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
	//�л�����,�������λ�����ƣ��鿴�Ƿ������ȼ��ϸߵ�����
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

void SwitchTask(AGVTASK *pTask)   //�л�����
{
	//�����ǰ��������ֹͣ
	if (agvTask.state == TaskState_Stop) return;
	int bNeedStop = 0;
	//�����ǰ����û�����
	if (agvTask.state != TaskState_Finish)
	{
		TaskKeepCount = 0;
		//����������뵱ǰ����ͬ,ֹͣ��ǰ����
		if (agvTask.cmd != pTask->cmd)
		{
			bNeedStop = 1;
		}
		else   //����������ͬ
		{
			//�ƶ�����ı���Ҫͣ����
			if (agvTask.cmd == Task_Move && agvTask.deg != pTask->deg)
			{
				bNeedStop = 1;
			}
			//��ת�Ƕȸı���Ҫͣ����
			else if (agvTask.cmd == Task_Turn )
			{
				if ((agvTask.rotSpeed > 0 && pTask->rotSpeed < 0) || (agvTask.rotSpeed < 0 && pTask->rotSpeed > 0))
				{
					bNeedStop = 1;
				}		
			}
		}
	}
	else  //������ɣ���ʼ��������ֹ��ͬʱ��������ͬ����
	{
		TaskKeepCount++;
		if (agvTask.cmd != pTask->cmd)
		{
			TaskKeepCount = 0;
			agvStatus.outputRpm = 0;
			agvStatus.clipState = GetClipState();
		}
		//��ֹ�϶�ʱ��������ͬ������
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
	agvTask.speed = pTask->speed;			            //Ŀ�귽���趨�ٶ�,mm/s
	agvTask.deg = pTask->deg;				            //Ŀ�귽���뵱ǰ����н� 0.1��
	agvTask.frontAngle = pTask->frontAngle;			//ǰ������ǰ�ֵ����Ƕ� 0.1��
	agvTask.backAngle = pTask->backAngle;			//ǰ��������ֵ����Ƕ� 0.1��
	agvTask.rotSpeed = pTask->rotSpeed;
	agvTask.wheelBaseLength = pTask->wheelBaseLength;   //���ֵ mm

	//����
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


		//TODO �жϲ�����ȷ��


}


void HandleMove(void)    //�����ƶ�
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
		//0 �Ȼ���-90��
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


void HandleTurn(void)    //������ת
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

//�����ɼ�
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
		//ֹͣ�˶�,�ɼв���Ҫ��������
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
			//TODO �жϼб�״̬��������λ��
			agvStatus.clipState = ClipState_Opened;
			agvTask.state = TaskState_Finish;

		}
	}
}

//����г�
void HandleClampClose(void)
{
	//����ǰ׼��
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
			//������Զ����ж��Ƿ�����̥���������̥���ſ��԰���
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
		//���н���׼��
			//TODO �жϼб�״̬��������λ��
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
			if (Wheelbase_Data <= WheelbaseLowerLimit)                      //����������2400
			{
				agvStatus.ajustDisState = AjustState_Error;
				agvTask.state = TaskState_Finish;
				return;
			}
		}
		else if ((agvTask.wheelBaseLength - 5.0) > Wheelbase_Data)
		{
			if (Wheelbase_Data >= WheelbaseUpperLimit)                      //���������3200
			{
				agvStatus.ajustDisState = AjustState_Error;
				agvTask.state = TaskState_Finish;
				return;
			}
		}

		Stop_Moving();
		if (IsMoveStoped())
		{
			//�Զ�״̬�£��жϰ����Ƿ��ջأ�û���ջأ����ܵ�����࣬����
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
		//TODO�ж��Ƿ��д���
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
		//�ﵽ���ޣ�ͣ����
		if (Wheelbase_Data <= WheelbaseLowerLimit - 30 || Wheelbase_Data >= WheelbaseUpperLimit+30)                      //����������2400
		{
			agvStatus.ajustDisState = AjustState_Error;
			agvTask.state = TaskState_Finish;
			return;
		}

		agvStatus.ajustDisState = AjustState_Running;
		Stop_Moving();
		if (IsMoveStoped())
		{
			//�Զ�״̬�£��жϰ����Ƿ��ջأ�û���ջأ����ܵ�����࣬����
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
		//TODO�ж��Ƿ��д���
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


void UpdateAgvStatus(void)   //����agv״̬
{

	agvStatus.clipState = GetClipState();          //����״̬ 
	agvStatus.chargeState=0;        //���״̬
	agvStatus.ajustDisState = AjustState_OK;      //������״̬ 	            
	agvStatus.wheelBaseLength = Wheelbase_Data;    //���ֵ mm
}

void HandleTask(void)                 //��������
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


void HandleReset(void)                //����λ
{
	agvStatus.errorCode = 0;
	agvStatus.ajustDisState = AjustState_None;
}
void HandleRestartMotor(void)         //�����������
{
	if (MotorPowerOff == 1)
	{
		Restart_Motor_Power();
		MotorPowerOff = 0;
		osDelay(2000);
	}
	HandleReset();
}
void HandleMotorPowerOff(void)        //�������ϵ�
{
	if (MotorPowerOff == 0)
	{
		Stop_Moving();
		Motor_Power_Off();
		MotorPowerOff = 1;
	}
}


