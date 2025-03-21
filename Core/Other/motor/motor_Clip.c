/****************************************************************************
// 2022.08.12		���г�����ʱ��Ϊλ�ÿ���ģʽ
*****************************************************************************/
#include "main.h"
#include "motor.h"
#include "74HC595.h"
#include "arm_math.h"
#include "remote_control.h"
#include <stdlib.h>
#include "cmsis_os.h"

#define Trans_Ratio 10     //���ٻ��Ĵ�����
#define Gear_Ratio  13/17  //���ٻ���˿��֮��ĳ��ֱ�
#define Lead_Screw  10     //˿�ܵ���,��λ��mm

#define Max_Lead_LF  270//264      //��ǰ˿�ܿ������ߵ���󵼳�,��λ��mm
#define Max_Lead_RF  273//264      //��ǰ˿�ܿ������ߵ���󵼳�,��λ��mm
#define Max_Lead_LB  266//264      //���˿�ܿ������ߵ���󵼳�,��λ��mm
#define Max_Lead_RB  272//264      //���˿�ܿ������ߵ���󵼳�,��λ��mm

#define Encoder_Resolution 10000 //�������ֱ���,�����תһ�ܶ�Ӧ��ֵ

#define CLIP_SENSOR_DETECT_YES 0  //���е�λ�������źŵ�λ
#define CLIP_SENSOR_DETECT_NO  1  //���е�λ�������ź�δ��λ

ClipPosion ClipPos;

int16_t FClipRpm = 0;   //ǰ�����ٶ�
int16_t BClipRpm = 0;   //������ٶ�

int16_t MaxClipRpm = 1200;  //��ǰ��������˶��ٶ�

//����������г� 0x500  1280

int ClipOpen()            //���д�
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
	else //�ֶα��У���
	{
		
		int16_t FCRPM = -Clip_SPEED_HIGH;
		int16_t BCRPM = -Clip_SPEED_HIGH;

		//��ǰ����
		MotorCFL.clip_distance = abs(MotorCFL.POSITION - MotorCFL.clipPos_close);
		//��ǰ����
		MotorCFR.clip_distance = abs(MotorCFR.POSITION - MotorCFR.clipPos_close);
		//������
		MotorCBL.clip_distance = abs(MotorCBL.POSITION - MotorCBL.clipPos_close);
		//�Һ�ǰ��
		MotorCBR.clip_distance = abs(MotorCBR.POSITION - MotorCBR.clipPos_close);

		//ǰ�������б۾����
		float O_distance_F_LR = abs(MotorCFL.clip_distance - MotorCFR.clip_distance);

		//���������б۾����
		float O_distance_B_LR = abs(MotorCBL.clip_distance - MotorCBR.clip_distance);

		if (agvStatus.owner == AgvOwner_PC)
		{
			if (O_distance_F_LR > Clip_DISTANCE_ERR || O_distance_B_LR > Clip_DISTANCE_ERR)
			{
				ClipStop();
				return 0;
			}
		}
		


		//��ǰ�������900������ǰ�������900 ���쵽�յ��ܾ�����1280�������ٵ�
		if ((MotorCFL.clip_distance > 900) || (MotorCFR.clip_distance > 900))
		{
			FCRPM = -Clip_SPEED_LOW;
		}

		//	//���������900�����Һ�������900 �������ٵ�
		if ((MotorCBL.clip_distance > 900) | (MotorCBR.clip_distance > 900))
		{
			BCRPM = -Clip_SPEED_LOW;
		}
		//��������ٶ�
		FCRPM = LimitMax(FCRPM, MaxClipRpm);
		BCRPM = LimitMax(BCRPM, MaxClipRpm);
		FClipRpm = RpmUpdate(4, FCRPM, FClipRpm);  //���ٶȿ���
		BClipRpm = RpmUpdate(4, BCRPM, BClipRpm);   //���ٶȿ���

		int16_t clipFL_rpm = FClipRpm;
		int16_t clipFR_rpm = FClipRpm;
		int16_t clipBL_rpm = BClipRpm;
		int16_t clipBR_rpm = BClipRpm;

		//�ж�ǰ����
		if (O_distance_F_LR > Clip_DISTANCE)
		{
			//��߱��ұ߾����,����ٶȿ� ��߼���
			if (MotorCFL.clip_distance > MotorCFR.clip_distance)
			{
				clipFL_rpm += Clip_SPEED_OFFSET;
			}
			//�ұ߿죬�ұ߼���
			else
			{
				clipFR_rpm += Clip_SPEED_OFFSET;
			}
		}

		if (O_distance_B_LR > Clip_DISTANCE)
		{
			//��߱��ұ߾����  ����ٶȿ죬�������
			if (MotorCBL.clip_distance > MotorCBR.clip_distance)
			{
				clipBL_rpm += Clip_SPEED_OFFSET;
			}
			//�ұ��ٶȿ죬�ұ�����
			else
			{
				clipBR_rpm += Clip_SPEED_OFFSET;
			}
		}

		if (CLIP_SENSOR_DETECT_YES == FLO)
		{
			clipFL_rpm = 0;
		}

		if (CLIP_SENSOR_DETECT_YES == FRO)  //���ǰ�Ҵ�����û�м�⵽
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
int ClipClose()          //���бպϣ���ʼ�г�
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
		//������ĸ������Ӧ��λ��
		MotorCFL.clip_distance = abs(MotorCFL.POSITION - MotorCFL.clipPos_open);
		MotorCFR.clip_distance = abs(MotorCFR.POSITION - MotorCFR.clipPos_open);
		MotorCBL.clip_distance = abs(MotorCBL.POSITION - MotorCBL.clipPos_open);
		MotorCBR.clip_distance = abs(MotorCBR.POSITION - MotorCBR.clipPos_open);

		//ǰ���������ľ����
		float O_distance_F_LR = abs(MotorCFL.clip_distance - MotorCFR.clip_distance);
		//�������������� ��
		float O_distance_B_LR = abs(MotorCBL.clip_distance - MotorCBR.clip_distance);

		if (agvStatus.owner == AgvOwner_PC)
		{
			if (O_distance_F_LR > Clip_DISTANCE_ERR || O_distance_B_LR > Clip_DISTANCE_ERR)
			{
				ClipStop();
				return 0;
			}
		}

		//��ǰ�������900������ǰ�������900 ���쵽�յ��ܾ�����1280�������ٵ�
		if ((MotorCFL.clip_distance > 900) || (MotorCFR.clip_distance > 900))
		{
			FCRPM = Clip_SPEED_LOW;
		}

		//	//���������900�����Һ�������900 �������ٵ�
		if ((MotorCBL.clip_distance > 900) | (MotorCBR.clip_distance > 900))
		{
			BCRPM = Clip_SPEED_LOW;
		}

		//��������ٶ�
		FCRPM = LimitMax(FCRPM, MaxClipRpm);
		BCRPM = LimitMax(BCRPM, MaxClipRpm);

		FClipRpm = RpmUpdate(5, FCRPM, FClipRpm);  //���ٶȿ���
		BClipRpm = RpmUpdate(5, BCRPM, BClipRpm);   //���ٶȿ���

		int16_t clipFL_rpm = FClipRpm;
		int16_t clipFR_rpm = FClipRpm;
		int16_t clipBL_rpm = BClipRpm;
		int16_t clipBR_rpm = BClipRpm;

		//�ж�ǰ����
		if (O_distance_F_LR > Clip_DISTANCE)
		{
			//��߱��ұ߾����,����ٶȿ� ��߼���
			if (MotorCFL.clip_distance > MotorCFR.clip_distance)
			{
				clipFL_rpm -= Clip_SPEED_OFFSET;
			}
			//�ұ߿죬�ұ߼���
			else
			{
				clipFR_rpm -= Clip_SPEED_OFFSET;
			}
		}

		if (O_distance_B_LR > Clip_DISTANCE)
		{
			//��߱��ұ߾����  ����ٶȿ죬�������
			if (MotorCBL.clip_distance > MotorCBR.clip_distance)
			{
				clipBL_rpm -= Clip_SPEED_OFFSET;
			}
			//�ұ��ٶȿ죬�ұ�����
			else
			{
				clipBR_rpm -= Clip_SPEED_OFFSET;
			}
		}

		if (CLIP_SENSOR_DETECT_YES == FLC)
		{
			clipFL_rpm = 0;
		}

		if (CLIP_SENSOR_DETECT_YES == FRC)  //���ǰ�Ҵ�����û�м�⵽
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
void ClipStop()           //����ֹͣ
{
	FClipRpm = 0;
	BClipRpm = 0;
	//��ֹ���ַ�������ɹ�������3��
	Set_Can_RPM_RPDO(BL_Clip, 0, 1);
	//HAL_Delay(2);
	Set_Can_RPM_RPDO(FL_Clip, 0, 1);
	//HAL_Delay(2);				
	Set_Can_RPM_RPDO(FR_Clip, 0, 1);
	//HAL_Delay(2);
	Set_Can_RPM_RPDO(BR_Clip, 0, 1);
}


int ClipReady(int readyMode)   //����׼��
{
	if (readyMode==READY_START)   //��ʼ����
	{
		HC595_add(OUT_7);//�̵���7�򿪣���ֹ��಻�ԣ���������
		SetMotorDisable();//���ߵ����ʹ�ܣ���ֹ��಻�ԣ���������	
		osDelay(500);
		return 1;
	}
	else  //��������
	{
		ClipStop();
		if (IsClipStop())
		{
			HC595_del(OUT_7);//�̵���7������ֹͣ����Զ�����
			SetMotorEnable();//��ʹ�ܣ�����ʹ���˶�
			osDelay(500);
			return 1;
		}	
	}

	return 0;
}


int IsClipStop()            //�����˶��Ƿ�ֹͣ
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

	if (clip == 0x55) //�ص�λ
	{
		return ClipState_Closed;          //����״̬ 
	}
	else if (clip == 0xAA)//����λ
	{
		return ClipState_Opened;          //����״̬ 
	}
	return  ClipState_Stoped;          //����״̬ 
	
}
