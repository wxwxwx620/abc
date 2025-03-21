#include "main.h"
#include "voice.h"
#include "remote_control.h"
#include "BMS.h"

uint8_t Voice_XX;
uint8_t Voice_TXAdd;
uint8_t Voice_CrC_flag;


/*
*********************************************************************************************************
*	�� �� ��: MODH_SendAckWithCRC
*	����˵��: ����Ӧ��,�Զ���CRC.
*	��    ��: �ޡ����������� g_tModH.TxBuf[], [g_tModH.TxCount
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void MODH_SendAckWithCRC(void)
{
	uint16_t crcfifo;
	crcfifo = CRC16(&UART4_STA.TX_BUF[0], 6);                  //����5  0��ʼ��6����CRCУ����
	UART4_STA.TX_BUF[6] = crcfifo;
	UART4_STA.TX_BUF[7] = crcfifo >> 8;
	UsartSendData_DMA(4, UART4_STA.TX_BUF, 8);
}

void Voice_MODH_SendH(uint16_t _num)
{
	UART4_STA.TX_BUF[0] = 0x0B;
	UART4_STA.TX_BUF[1] = 0x06;
	UART4_STA.TX_BUF[2] = 0x00;
	UART4_STA.TX_BUF[3] = 0x01;
	UART4_STA.TX_BUF[4] = 0x00;
	UART4_STA.TX_BUF[5] = _num;
	MODH_SendAckWithCRC();		/* �������ݣ��Զ���CRC */
}

/*
*********************************************************************************************************
*	�� �� ��: MODH_Poll
*	����˵��: ���տ�����ָ��. 1ms ��Ӧʱ�䡣
*	��    ��: ��
*	�� �� ֵ: 0 ��ʾ������ 1��ʾ�յ���ȷ����
*********************************************************************************************************
*/
void MODH_Poll(void)
{
	//	uint16_t crc1;
	//	uint32_t temp;
	//	if(__HAL_UART_GET_FLAG(&huart4,UART_FLAG_IDLE)!= RESET)
	//	{		
	//			__HAL_UART_CLEAR_IDLEFLAG(&huart4);
	//			HAL_UART_DMAStop(&huart4);
	//			temp = huart4.hdmarx->Instance->NDTR;
	//			UART4_STA.RX_LEN = RX_URB_Size - temp;
	//			HAL_UART_Receive_DMA(&huart4,UART4_STA.RX_BUF,RX_URB_Size);	
	//			crc1 = usMBCRC16(UART4_STA.RX_BUF, UART4_STA.RX_LEN);
	//			
	//			if (crc1 == 0)//У��ɹ�
	//			{			
	//				if(UART4_STA.RX_LEN==8)	Voice_CrC_flag=1;
	//				if(UART4_STA.RX_BUF[5]==0x02)	Voice_CrC_flag=0;
	//				UART4_STA.RX_LEN = 0;	/* ��������������������´�֡ͬ�� */
	//			}
	//			else
	//			{
	//				//У��ʧ��
	//			}
	//	}
}





void UI_date1(void)//UI���ݴ���
{
//	if (agvStatus.ctrlMode == 1)
//	{
//		ui_t.ControlModel = 0;//����ģʽ0�Զ�
//	}
//	if (agvStatus.ctrlMode == 0)
//	{
//		ui_t.ControlModel = 1;//����ģʽ1����(���ֶ�)
//	}
//	if (rc_updateCount >= 0)
//	{
//		if (rc_updateCount < 3)
//		{
//			ui_t.CommunicationAlarm = 1;//ͨ�ű��� ң�������ݶϿ�
//		}
//		else
//		{
//			ui_t.CommunicationAlarm = 0;//ͨ�ű��� ң������������
//		}
//	}
//	if (pc_updateCount >= 0)
//	{
//		if (pc_updateCount < 3)
//		{
//			ui_t.NetworkingStatus = 0;//����״̬ PC�Ͽ�
//		}
//		else
//		{
//			ui_t.NetworkingStatus = 1;//����״̬ PC��������
//		}
//	}
}
void UI_date2(void)//UI���ݴ���
{

//	if ((rc_ctrl.EmergencyStop + pc_ctrl.em_stop) > 0)
//	{
//		ui_t.RunningState = 2;//��ͣ״̬
//	}
//	else if (MotorBL.alarm>0
//		|| MotorBR.alarm>0
//		|| MotorFL.alarm > 0
//		|| MotorFR.alarm > 0
//		|| MotorCBR.alarm > 0
//		|| MotorCBL.alarm > 0
//		|| MotorCFR.alarm > 0
//		|| MotorCFL.alarm > 0
//		)
//	{
//		ui_t.RunningState = 1;//����״̬
//	}
//	else
//	{
//		ui_t.RunningState = 0;//����
//	}
//	
}


void UI_Initialize()//ui��ʼ������
{
	UART4_STA.TX_BUF[0] = 0x5a;
	UART4_STA.TX_BUF[1] = 0xa5;
	UART4_STA.TX_BUF[2] = 0x0e;
	UART4_STA.TX_BUF[3] = 0x80;
	UART4_STA.TX_BUF[4] = 0x56;
	UART4_STA.TX_BUF[5] = 0x5A;
	UART4_STA.TX_BUF[6] = 0xa0;
	UART4_STA.TX_BUF[7] = 0x04;
	UART4_STA.TX_BUF[8] = 0x80;
	UART4_STA.TX_BUF[9] = 0x41;
	UART4_STA.TX_BUF[10] = 0x00;
	UART4_STA.TX_BUF[11] = 0x41;
	UART4_STA.TX_BUF[12] = 0x00;
	UART4_STA.TX_BUF[13] = 0x00;
	UART4_STA.TX_BUF[14] = 0x0b;
	UART4_STA.TX_BUF[15] = 0x78;
	UART4_STA.TX_BUF[16] = 0x42;
	UsartSendData_DMA(4, UART4_STA.TX_BUF, 17);
}


/*void UART5_ui_voice()//ui�������ϲ�����
{
	if (Voice_XX > 0)
	{
		if (Message1_Flag1 != Message1_Flag)
		{
			UART5_STA.TX_BUF[0] = 0x0B;
			UART5_STA.TX_BUF[1] = 0x06;
			UART5_STA.TX_BUF[2] = 0x00;
			UART5_STA.TX_BUF[3] = 0x01;
			UART5_STA.TX_BUF[4] = 0x00;
			UART5_STA.TX_BUF[5] = Message1_Flag;

			uint16_t crcfifo;
			crcfifo = CRC16(&UART5_STA.TX_BUF[0], 6);                  //����5  0��ʼ��6����CRCУ����
			UART5_STA.TX_BUF[6] = crcfifo;
			UART5_STA.TX_BUF[7] = crcfifo >> 8;
			UsartSendData_DMA(5, UART5_STA.TX_BUF, 8);
			osDelay(20);                  //һ��Ҫ����ʱ����Ȼ��Voice_XX++���ٶȱ�--�죬���²�����������
			Message1_Flag1 = Message1_Flag;
		}
		if (Voice_XX > 0) Voice_XX--;
	}

	//  osDelay(50);	
	HAL_Delay(20);
	UART5_STA.TX_BUF[0] = 0x5a;
	UART5_STA.TX_BUF[1] = 0xa5;
	UART5_STA.TX_BUF[2] = 0x5f;          //�ұ�95������			
	UART5_STA.TX_BUF[3] = 0x82;
	UART5_STA.TX_BUF[4] = 0x30;
	UART5_STA.TX_BUF[5] = 0x00;

	UART5_STA.TX_BUF[6] = BMS1.current >> 8;								UART5_STA.TX_BUF[7] = BMS1.current;	//����
	UART5_STA.TX_BUF[8] = battery_t.voltage >> 8;						UART5_STA.TX_BUF[9] = battery_t.voltage;	//��ѹ
	UART5_STA.TX_BUF[10] = BMS1.capacity_per >> 8;					UART5_STA.TX_BUF[11] = BMS1.capacity_per;	//��ǰʣ������
	UART5_STA.TX_BUF[12] = 0x0073 >> 8;											UART5_STA.TX_BUF[13] = 0x0073;	//��ǰ���������
	UART5_STA.TX_BUF[14] = 0x0073 >> 8;											UART5_STA.TX_BUF[15] = 0x0073;	//�������
	UART5_STA.TX_BUF[16] = BMS1.capacity_per >> 8;					UART5_STA.TX_BUF[17] = BMS1.capacity_per;	//�����ٷֱ�
	UART5_STA.TX_BUF[18] = BMS1.Max_unit_voltage >> 8;			UART5_STA.TX_BUF[19] = BMS1.Max_unit_voltage;	//��ߵ����ѹ
	UART5_STA.TX_BUF[20] = BMS1.Min_unit_voltage >> 8; 			UART5_STA.TX_BUF[21] = BMS1.Min_unit_voltage;	//��͵����ѹ
	UART5_STA.TX_BUF[22] = BMS1.Max_unit_Temperature >> 8; 	UART5_STA.TX_BUF[23] = BMS1.Max_unit_Temperature;	//����¶ȵ�
	UART5_STA.TX_BUF[24] = BMS1.Min_unit_Temperature >> 8; 	UART5_STA.TX_BUF[25] = BMS1.Min_unit_Temperature;	//����¶ȵ�
	UART5_STA.TX_BUF[26] = BMS1.cycle_count >> 8; 					UART5_STA.TX_BUF[27] = BMS1.cycle_count;	//ѭ������
	//״̬��Ϣ
	UART5_STA.TX_BUF[28] = (BMS1.Alarm_Type & 0x2000) >> 13; 			UART5_STA.TX_BUF[29] = (BMS1.Alarm_Type & 0x2000);	//���״̬	1����硣0���ŵ�
	UART5_STA.TX_BUF[30] = (BMS1.Alarm_Type & 0x0800) >> 11; 			UART5_STA.TX_BUF[31] = (BMS1.Alarm_Type & 0x0800);	//����������30%
	UART5_STA.TX_BUF[32] = (BMS1.Alarm_Type & 0x0400) >> 10; 			UART5_STA.TX_BUF[33] = (BMS1.Alarm_Type & 0x0400);	//��ز����� 1
	UART5_STA.TX_BUF[34] = (BMS1.Alarm_Type & 0x0200) >> 9; 			UART5_STA.TX_BUF[35] = (BMS1.Alarm_Type & 0x0200);	//�ŵ��·���� 1
	UART5_STA.TX_BUF[36] = (BMS1.Alarm_Type & 0x0100) >> 8; 			UART5_STA.TX_BUF[37] = (BMS1.Alarm_Type & 0x0100);	//����·���� 1
	UART5_STA.TX_BUF[38] = (BMS1.Alarm_Type & 0x0080) >> 7; 			UART5_STA.TX_BUF[39] = (BMS1.Alarm_Type & 0x0080);	//�ŵ���������� 1
	UART5_STA.TX_BUF[40] = (BMS1.Alarm_Type & 0x0040) >> 6; 			UART5_STA.TX_BUF[41] = (BMS1.Alarm_Type & 0x0040);	//������������ 1
	UART5_STA.TX_BUF[42] = (BMS1.Alarm_Type & 0x0020) >> 5; 			UART5_STA.TX_BUF[43] = (BMS1.Alarm_Type & 0x0020);	//���±��� 1
	UART5_STA.TX_BUF[44] = (BMS1.Alarm_Type & 0x0010) >> 4; 			UART5_STA.TX_BUF[45] = (BMS1.Alarm_Type & 0x0010);	//���±��� 1
	UART5_STA.TX_BUF[46] = (BMS1.Alarm_Type & 0x0008) >> 3; 			UART5_STA.TX_BUF[47] = (BMS1.Alarm_Type & 0x0008);	//���±��� 1
	UART5_STA.TX_BUF[48] = (BMS1.Alarm_Type & 0x0004) >> 2; 			UART5_STA.TX_BUF[49] = (BMS1.Alarm_Type & 0x0004);	//���±��� 1
	UART5_STA.TX_BUF[50] = (BMS1.Alarm_Type & 0x0002) >> 1; 			UART5_STA.TX_BUF[51] = (BMS1.Alarm_Type & 0x0002);	//����Ƿѹ 1
	UART5_STA.TX_BUF[52] = (BMS1.Alarm_Type & 0x0001) >> 0; 			UART5_STA.TX_BUF[53] = (BMS1.Alarm_Type & 0x0001);	//���峬ѹ 1
	//����״̬
	UI_date1();//UI���ݴ���
	UI_date2();//UI���ݴ���
	UART5_STA.TX_BUF[54] = 0x0000 >> 8; 			  UART5_STA.TX_BUF[55] = ui_t.NetworkingStatus;	//����״̬ 1���� 0δ����
	UART5_STA.TX_BUF[56] = 0x0000 >> 8; 				UART5_STA.TX_BUF[57] = ui_t.RunningState;	//����״̬ 0���� 1���� 2��ͣ
	UART5_STA.TX_BUF[58] = 0x0000 >> 8; 				UART5_STA.TX_BUF[59] = ui_t.ControlModel;	//����ģʽ 0�Զ� 1���� 2���ߣ�
	UART5_STA.TX_BUF[60] = 0x0000 >> 8; 				UART5_STA.TX_BUF[61] = ui_t.CommunicationAlarm;	//ͨ�ű���(0���� 1����)
	UART5_STA.TX_BUF[62] = 0x0000 >> 8; 				UART5_STA.TX_BUF[63] = MotorFL.alarm>0;	//���1����
	UART5_STA.TX_BUF[64] = 0x0000 >> 8; 				UART5_STA.TX_BUF[65] = MotorFR.alarm > 0;	//���2����
	UART5_STA.TX_BUF[66] = 0x0000 >> 8; 				UART5_STA.TX_BUF[67] = MotorBR.alarm > 0;	//���3����
	UART5_STA.TX_BUF[68] = 0x0000 >> 8; 				UART5_STA.TX_BUF[69] = MotorBL.alarm > 0;	//���4����	
	UART5_STA.TX_BUF[70] = 0x0000 >> 8; 				UART5_STA.TX_BUF[71] = MotorCFL.alarm > 0;	//���5����	
	UART5_STA.TX_BUF[72] = 0x0000 >> 8; 				UART5_STA.TX_BUF[73] = MotorCFR.alarm > 0;	//���6����	
	UART5_STA.TX_BUF[74] = 0x0000 >> 8; 				UART5_STA.TX_BUF[75] = MotorCBR.alarm > 0;	//���7����	
	UART5_STA.TX_BUF[76] = 0x0000 >> 8; 				UART5_STA.TX_BUF[77] = MotorCBL.alarm > 0;	//���8����	
	UART5_STA.TX_BUF[78] = 0x0000 >> 8; 				UART5_STA.TX_BUF[79] = 0;
	UART5_STA.TX_BUF[80] = 0x0000 >> 8; 				UART5_STA.TX_BUF[81] = 0;
	UART5_STA.TX_BUF[82] = 0x0000 >> 8; 				UART5_STA.TX_BUF[83] = 0;
	UART5_STA.TX_BUF[84] = 0x0000 >> 8; 				UART5_STA.TX_BUF[85] = 0;
	UART5_STA.TX_BUF[86] = 0x0000 >> 8; 				UART5_STA.TX_BUF[87] = 0x0001;	//�����
	UART5_STA.TX_BUF[88] = 0x0002 >> 8; 				UART5_STA.TX_BUF[89] = 0x0001; //�汾��

	UART5_STA.TX_BUF[90] = INNER.TARGET_SPEED >> 8; 				UART5_STA.TX_BUF[91] = INNER.TARGET_SPEED; //���ٶ�
	if (INNER.TARGET_RPS > 0) INNER.TARGET_RPS = INNER.TARGET_RPS*(-1);//���ĳ�����
	INNER.TARGET_RPS = INNER.TARGET_RPS*60.3;
	UART5_STA.TX_BUF[92] = (int16_t)INNER.TARGET_RPS >> 8; 				UART5_STA.TX_BUF[93] = (int16_t)INNER.TARGET_RPS; //���ٶ�
	UART5_STA.TX_BUF[94] = INNER.TARGET_ANGLE >> 8; 				      UART5_STA.TX_BUF[95] = INNER.TARGET_ANGLE; //��ʻ����			

	uint16_t crcfifo1;
	crcfifo1 = CRC16(&UART5_STA.TX_BUF[3], 93);                  //����5  ����λ��ʼ��93����CRCУ��
	UART5_STA.TX_BUF[96] = crcfifo1;
	UART5_STA.TX_BUF[97] = crcfifo1 >> 8;
	UsartSendData_DMA(5, UART5_STA.TX_BUF, 98);	 //����5

//  osDelay(50);
	HAL_Delay(20);


}*/

#define VoicePackLen 8
uint8_t Voice_ID = 0;      //��ǰID
uint8_t Voice_Sended = 0;  //�ϴη��͵�ID
uint8_t sendBuf[VoicePackLen] = { 0x0B,0x06,0x00,0x01,0x00 ,0x00,0x00,0x00 };
void Voice_Init()
{
	sendBuf[0] = 0x0B;
	sendBuf[1] = 0x06;
	sendBuf[2] = 0x00;
	sendBuf[3] = 0x01;
	sendBuf[4] = 0x00;
	sendBuf[5] = Voice_ID;
}
void Voice_Update(uint8_t voiceId)
{
	Voice_ID = voiceId;

}

void Voice_Process()
{
	if (Voice_ID != Voice_Sended)
	{
		DIR4=1;
		//Voice_MODH_SendH(Voice_ID);
		Voice_Sended = Voice_ID;
		//return;
		sendBuf[5] = Voice_ID;
		uint16_t crcfifo = CRC16(sendBuf, 6);                  //����4  0��ʼ��6����CRCУ����
		sendBuf[6] = crcfifo;
		sendBuf[7] = crcfifo >> 8;
		UsartSendData_DMA(4, sendBuf, 8);
		osDelay(10);

		DIR4=0;
		
	}
}

void Voice_Send(uint8_t voiceId)
{
	sendBuf[5] = voiceId;
	uint16_t crcfifo = CRC16(sendBuf, 6);                  //����4  0��ʼ��6����CRCУ����
	sendBuf[6] = crcfifo;
	sendBuf[7] = crcfifo >> 8;
	UsartSendData_DMA(4, sendBuf, 8);

}








