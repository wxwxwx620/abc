#include "main.h"
#include "FIFO.h"
#include "usart.h"
#include "stdio.h"
#include "BMS.h"
#include "voice.h"
#include "remote_control.h"
#include "math.h"
battery_value_t battery_t;
battery_value_t1 BMS2;
uint8_t BMS_Time_out;
//uint8_t BMS_Relay;
void BMSSendWithCRC(void)
{
	uint16_t crcfifo;
	//crcfifo = CRC16(&UART4_STA.TX_BUF[0],6);                  //����4  0��ʼ��6����CRCУ����
	crcfifo = MODBUS_CalCRC(&UART4_STA.TX_BUF[0], 6);
	UART4_STA.TX_BUF[6] = crcfifo;
	UART4_STA.TX_BUF[7] = crcfifo >> 8;
	UsartSendData_DMA(4, UART4_STA.TX_BUF, 8);

	osDelay(9);

	DIR4 = 0;
	osDelay(50);
}

void UART4_Send(uint8_t c)
{
	switch (c)
	{
	case 1:	//
	{
		DIR4 = 1;
		UART4_STA.TX_BUF[0] = 0x01;
		UART4_STA.TX_BUF[1] = 0x03;
		UART4_STA.TX_BUF[2] = 0x00;
		UART4_STA.TX_BUF[3] = 0x78;
		UART4_STA.TX_BUF[4] = 0x00;
		UART4_STA.TX_BUF[5] = 0x05;
		BMSSendWithCRC();		/* �������ݣ��Զ���CRC */
		//DIR4 = 1;
		//UART4_STA.TX_BUF[0] = 0x01;
		//UART4_STA.TX_BUF[1] = 0x03;
		//UART4_STA.TX_BUF[2] = 0x00;
		//UART4_STA.TX_BUF[3] = 0x98;
		//UART4_STA.TX_BUF[4] = 0x00;
		//UART4_STA.TX_BUF[5] = 0x06;
		//BMSSendWithCRC();		/* �������ݣ��Զ���CRC */
		//DIR4 = 1;
		////		DIR3=1;
		//UART4_STA.TX_BUF[0] = 0x01;
		//UART4_STA.TX_BUF[1] = 0x03;
		//UART4_STA.TX_BUF[2] = 0x00;
		//UART4_STA.TX_BUF[3] = 0x4C;
		//UART4_STA.TX_BUF[4] = 0x00;
		//UART4_STA.TX_BUF[5] = 0x04;
		//BMSSendWithCRC();		/* �������ݣ��Զ���CRC */
	}break;
	case 2:	//
	{
		DIR4 = 1;
		if (Voice_XX > 0)
		{
			if (Message1_Flag1 != Message1_Flag)
			{
				UART4_STA.TX_BUF[0] = 0x0B;
				UART4_STA.TX_BUF[1] = 0x06;
				UART4_STA.TX_BUF[2] = 0x01;
				UART4_STA.TX_BUF[3] = 0x01;
				UART4_STA.TX_BUF[4] = 0x00;
				UART4_STA.TX_BUF[5] = Message1_Flag;

				uint16_t crcfifo;
				crcfifo = CRC16(&UART4_STA.TX_BUF[0], 6);                  //����4  0��ʼ��6����CRCУ����
				UART4_STA.TX_BUF[6] = crcfifo;
				UART4_STA.TX_BUF[7] = crcfifo >> 8;
				UsartSendData_DMA(4, UART4_STA.TX_BUF, 8);
				osDelay(20);                  //һ��Ҫ����ʱ����Ȼ��Voice_XX++���ٶȱ�--�죬���²�����������
				Message1_Flag1 = Message1_Flag;
			}
			if (Voice_XX > 0) Voice_XX--;
		}
	}break;
	case 3:
	{
	/*	DIR4 = 1;
		UART4_STA.TX_BUF[0] = 0x5a;
		UART4_STA.TX_BUF[1] = 0xa5;
		UART4_STA.TX_BUF[2] = 0x5f;          //�ұ�95������			
		UART4_STA.TX_BUF[3] = 0x82;
		UART4_STA.TX_BUF[4] = 0x30;
		UART4_STA.TX_BUF[5] = 0x00;

		UART4_STA.TX_BUF[6] = BMS1.current >> 8;								UART4_STA.TX_BUF[7] = BMS1.current;	//����
		UART4_STA.TX_BUF[8] = battery_t.voltage >> 8;						UART4_STA.TX_BUF[9] = battery_t.voltage;	//��ѹ
		UART4_STA.TX_BUF[10] = BMS1.capacity_per >> 8;					UART4_STA.TX_BUF[11] = BMS1.capacity_per;	//��ǰʣ������
		UART4_STA.TX_BUF[12] = 0x0073 >> 8;											UART4_STA.TX_BUF[13] = 0x0073;	//��ǰ���������
		UART4_STA.TX_BUF[14] = 0x0073 >> 8;											UART4_STA.TX_BUF[15] = 0x0073;	//�������
		UART4_STA.TX_BUF[16] = BMS1.capacity_per >> 8;					UART4_STA.TX_BUF[17] = BMS1.capacity_per;	//�����ٷֱ�
		UART4_STA.TX_BUF[18] = BMS1.Max_unit_voltage >> 8;			UART4_STA.TX_BUF[19] = BMS1.Max_unit_voltage;	//��ߵ����ѹ
		UART4_STA.TX_BUF[20] = BMS1.Min_unit_voltage >> 8; 			UART4_STA.TX_BUF[21] = BMS1.Min_unit_voltage;	//��͵����ѹ
		UART4_STA.TX_BUF[22] = BMS1.Max_unit_Temperature >> 8; 	UART4_STA.TX_BUF[23] = BMS1.Max_unit_Temperature;	//����¶ȵ�
		UART4_STA.TX_BUF[24] = BMS1.Min_unit_Temperature >> 8; 	UART4_STA.TX_BUF[25] = BMS1.Min_unit_Temperature;	//����¶ȵ�
		UART4_STA.TX_BUF[26] = BMS1.cycle_count >> 8; 					UART4_STA.TX_BUF[27] = BMS1.cycle_count;	//ѭ������
		//״̬��Ϣ
		UART4_STA.TX_BUF[28] = (BMS1.Alarm_Type & 0x2000) >> 13; 			UART4_STA.TX_BUF[29] = (BMS1.Alarm_Type & 0x2000);	//���״̬	1����硣0���ŵ�
		UART4_STA.TX_BUF[30] = (BMS1.Alarm_Type & 0x0800) >> 11; 			UART4_STA.TX_BUF[31] = (BMS1.Alarm_Type & 0x0800);	//����������30%
		UART4_STA.TX_BUF[32] = (BMS1.Alarm_Type & 0x0400) >> 10; 			UART4_STA.TX_BUF[33] = (BMS1.Alarm_Type & 0x0400);	//��ز����� 1
		UART4_STA.TX_BUF[34] = (BMS1.Alarm_Type & 0x0200) >> 9; 			UART4_STA.TX_BUF[35] = (BMS1.Alarm_Type & 0x0200);	//�ŵ��·���� 1
		UART4_STA.TX_BUF[36] = (BMS1.Alarm_Type & 0x0100) >> 8; 			UART4_STA.TX_BUF[37] = (BMS1.Alarm_Type & 0x0100);	//����·���� 1
		UART4_STA.TX_BUF[38] = (BMS1.Alarm_Type & 0x0080) >> 7; 			UART4_STA.TX_BUF[39] = (BMS1.Alarm_Type & 0x0080);	//�ŵ���������� 1
		UART4_STA.TX_BUF[40] = (BMS1.Alarm_Type & 0x0040) >> 6; 			UART4_STA.TX_BUF[41] = (BMS1.Alarm_Type & 0x0040);	//������������ 1
		UART4_STA.TX_BUF[42] = (BMS1.Alarm_Type & 0x0020) >> 5; 			UART4_STA.TX_BUF[43] = (BMS1.Alarm_Type & 0x0020);	//���±��� 1
		UART4_STA.TX_BUF[44] = (BMS1.Alarm_Type & 0x0010) >> 4; 			UART4_STA.TX_BUF[45] = (BMS1.Alarm_Type & 0x0010);	//���±��� 1
		UART4_STA.TX_BUF[46] = (BMS1.Alarm_Type & 0x0008) >> 3; 			UART4_STA.TX_BUF[47] = (BMS1.Alarm_Type & 0x0008);	//���±��� 1
		UART4_STA.TX_BUF[48] = (BMS1.Alarm_Type & 0x0004) >> 2; 			UART4_STA.TX_BUF[49] = (BMS1.Alarm_Type & 0x0004);	//���±��� 1
		UART4_STA.TX_BUF[50] = (BMS1.Alarm_Type & 0x0002) >> 1; 			UART4_STA.TX_BUF[51] = (BMS1.Alarm_Type & 0x0002);	//����Ƿѹ 1
		UART4_STA.TX_BUF[52] = (BMS1.Alarm_Type & 0x0001) >> 0; 			UART4_STA.TX_BUF[53] = (BMS1.Alarm_Type & 0x0001);	//���峬ѹ 1
		//����״̬
		UI_date1();//UI���ݴ���
		UI_date2();//UI���ݴ���
		UART4_STA.TX_BUF[54] = 0x0000 >> 8; 			  UART4_STA.TX_BUF[55] = ui_t.NetworkingStatus;	//����״̬ 1���� 0δ����
		UART4_STA.TX_BUF[56] = 0x0000 >> 8; 				UART4_STA.TX_BUF[57] = ui_t.RunningState;	//����״̬ 0���� 1���� 2��ͣ
		UART4_STA.TX_BUF[58] = 0x0000 >> 8; 				UART4_STA.TX_BUF[59] = ui_t.ControlModel;	//����ģʽ 0�Զ� 1���� 2���ߣ�
		UART4_STA.TX_BUF[60] = 0x0000 >> 8; 				UART4_STA.TX_BUF[61] = ui_t.CommunicationAlarm;	//ͨ�ű���(0���� 1����)
		UART4_STA.TX_BUF[62] = 0x0000 >> 8; 				UART4_STA.TX_BUF[63] = can_status.Motor_Alarm1;	//���1����
		UART4_STA.TX_BUF[64] = 0x0000 >> 8; 				UART4_STA.TX_BUF[65] = can_status.Motor_Alarm2;	//���2����
		UART4_STA.TX_BUF[66] = 0x0000 >> 8; 				UART4_STA.TX_BUF[67] = can_status.Motor_Alarm3;	//���3����
		UART4_STA.TX_BUF[68] = 0x0000 >> 8; 				UART4_STA.TX_BUF[69] = can_status.Motor_Alarm4;	//���4����	
		UART4_STA.TX_BUF[70] = 0x0000 >> 8; 				UART4_STA.TX_BUF[71] = can_status.Motor_Alarm5;	//���5����	
		UART4_STA.TX_BUF[72] = 0x0000 >> 8; 				UART4_STA.TX_BUF[73] = can_status.Motor_Alarm6;	//���6����	
		UART4_STA.TX_BUF[74] = 0x0000 >> 8; 				UART4_STA.TX_BUF[75] = can_status.Motor_Alarm7;	//���7����	
		UART4_STA.TX_BUF[76] = 0x0000 >> 8; 				UART4_STA.TX_BUF[77] = can_status.Motor_Alarm8;	//���8����	
		UART4_STA.TX_BUF[78] = 0x0000 >> 8; 				UART4_STA.TX_BUF[79] = can_status.Motor_Alarm8;
		UART4_STA.TX_BUF[80] = 0x0000 >> 8; 				UART4_STA.TX_BUF[81] = can_status.Motor_Alarm8;
		UART4_STA.TX_BUF[82] = 0x0000 >> 8; 				UART4_STA.TX_BUF[83] = can_status.Motor_Alarm8;
		UART4_STA.TX_BUF[84] = 0x0000 >> 8; 				UART4_STA.TX_BUF[85] = can_status.Motor_Alarm8;
		UART4_STA.TX_BUF[86] = 0x0000 >> 8; 				UART4_STA.TX_BUF[87] = 0x0001;	//�����
		UART4_STA.TX_BUF[88] = 0x0002 >> 8; 				UART4_STA.TX_BUF[89] = 0x0001; //�汾��

		UART4_STA.TX_BUF[90] = INNER.TARGET_SPEED >> 8; 				UART4_STA.TX_BUF[91] = INNER.TARGET_SPEED; //���ٶ�
		if (INNER.TARGET_RPS > 0) INNER.TARGET_RPS = INNER.TARGET_RPS*(-1);//���ĳ�����
		INNER.TARGET_RPS = INNER.TARGET_RPS*60.3;
		UART4_STA.TX_BUF[92] = (int16_t)INNER.TARGET_RPS >> 8; 				UART4_STA.TX_BUF[93] = (int16_t)INNER.TARGET_RPS; //���ٶ�
		UART4_STA.TX_BUF[94] = INNER.TARGET_ANGLE >> 8; 				      UART4_STA.TX_BUF[95] = INNER.TARGET_ANGLE; //��ʻ����			

		uint16_t crcfifo1;
		crcfifo1 = CRC16(&UART4_STA.TX_BUF[3], 93);                  //����4  ����λ��ʼ��93����CRCУ��
		UART4_STA.TX_BUF[96] = crcfifo1;
		UART4_STA.TX_BUF[97] = crcfifo1 >> 8;
		UsartSendData_DMA(4, UART4_STA.TX_BUF, 98);	 //����5*/

	}break;
	}
}
void Read_To_Charge()
{
	DIR4 = 1;
	UART4_STA.TX_BUF[0] = 0x01;
	UART4_STA.TX_BUF[1] = 0x11;
	UART4_STA.TX_BUF[2] = 0x04;
	UART4_STA.TX_BUF[3] = 0x25;
	UART4_STA.TX_BUF[4] = 0x00;
	UART4_STA.TX_BUF[5] = 0x02;
	UART4_STA.TX_BUF[6] = 0x01;
	UART4_STA.TX_BUF[7] = 0x00;
	UART4_STA.TX_BUF[8] = 0x6C;
	UART4_STA.TX_BUF[9] = 0xD5;
	UsartSendData_DMA(4, UART4_STA.TX_BUF, 10);

	osDelay(20);

	DIR4 = 0;

	osDelay(19);
	osDelay(100);

}

void Read_To_DisCharge()
{
	DIR4 = 1;
	UART4_STA.TX_BUF[0] = 0x01;
	UART4_STA.TX_BUF[1] = 0x11;
	UART4_STA.TX_BUF[2] = 0x04;
	UART4_STA.TX_BUF[3] = 0x25;
	UART4_STA.TX_BUF[4] = 0x00;
	UART4_STA.TX_BUF[5] = 0x02;
	UART4_STA.TX_BUF[6] = 0x00;
	UART4_STA.TX_BUF[7] = 0x00;
	UART4_STA.TX_BUF[8] = 0x6D;
	UART4_STA.TX_BUF[9] = 0x45;
	UsartSendData_DMA(4, UART4_STA.TX_BUF, 10);

	osDelay(20);//osDelay(9); //!!!!!!!when delay time is too short, datas in TX_BUF array will not be send. DIR3=0,

	DIR4 = 0;

	osDelay(19);
	osDelay(100);

}


void Usart4Receive_BMS_IDLE()
{
	uint32_t temp;
	uint16_t crc1;
	uint8_t byte;
	if (__HAL_UART_GET_FLAG(&huart4, UART_FLAG_IDLE) != RESET)
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart4);
		HAL_UART_DMAStop(&huart4);
		temp = huart4.hdmarx->Instance->NDTR;
		UART4_STA.RX_LEN = RX_URB_Size - temp;
		UART4_STA.RX_FLAG = 1;
		HAL_UART_Receive_DMA(&huart4, UART4_STA.RX_BUF, RX_URB_Size);
		// printf(	"UART4_STA.RX_LEN= %d\r\n",UART4_STA.RX_LEN);	
		crc1 = MODBUS_CalCRC(UART4_STA.RX_BUF, UART4_STA.RX_LEN);
		// crc1 = CRC16(UART4_STA.RX_BUF, UART4_STA.RX_LEN);
		if (crc1 == 0)//У��ɹ�
		{
			//        battery_t.capacity_per=UART4_STA.RX_BUF[3];
			//				battery_t.Alarm=UART4_STA.RX_BUF[39];
			byte = UART4_STA.RX_BUF[2];


			switch (byte)
			{
			case 0x0A:
			{
				Read_parameter1();
			}break;
			case 0x0C:
			{
				Read_parameter2();
			}
			case 0x08:
			{
				Read_parameter3();
			}
			}
			BMS_Time_out = 250;
		}
	}
}
int Read_parameter1(void)
{
	BMS2.SOC = UART4_STA.RX_BUF[3];
	agvStatus.power=BMS2.SOC;
	BMS2.SOH = UART4_STA.RX_BUF[4];
	BMS2.TIME1 = UART4_STA.RX_BUF[11] + (UART4_STA.RX_BUF[12] << 8);//ʣ��ŵ�ʱ��
	BMS2.TIME2 = UART4_STA.RX_BUF[9] + (UART4_STA.RX_BUF[10] << 8);//ʣ����ʱ��
	return 0;
}


int Read_parameter2(void)
{
	uint32_t temp1, temp2;


	temp1 = UART4_STA.RX_BUF[3] + (UART4_STA.RX_BUF[4] << 8)
		+ (UART4_STA.RX_BUF[5] << 16) + (UART4_STA.RX_BUF[6] << 24);
	BMS2.state0 = (temp1&(1 << 0));
	BMS2.state1 = (temp1&(1 << 1)) >> 1;
	BMS2.curtState = (temp1&((1 << 26) | (1 << 27))) >> 26;
	BMS2.chrgPlugIn = (temp1&((1 << 5) | (1 << 6))) >> 5;
	if (((temp1&(1 << 20)) >> 20) == 1)
		BMS2.alarm4 = 0;
	else
		BMS2.alarm4 = 1;
	temp2 = (UART4_STA.RX_BUF[11]) + (UART4_STA.RX_BUF[12] << 8)
		+ (UART4_STA.RX_BUF[13] << 16) + (UART4_STA.RX_BUF[14] << 24);
	BMS2.alarm1 = (temp2&(1 << 3)) >> 3;
	BMS2.alarm2 = (temp2&(1 << 4)) >> 4;
	BMS2.alarm3 = (temp2&(1 << 14)) >> 14;
	return 0;
}

int Read_parameter3(void)
{
	uint32_t VOL;
	int32_t CUR;
	VOL = UART4_STA.RX_BUF[3] + (UART4_STA.RX_BUF[4] << 8) + (UART4_STA.RX_BUF[5] << 16) + (UART4_STA.RX_BUF[6] << 24);
	BMS2.sysVol = (uint8_t)(Hex2Dec_32(VOL) / 1000);
	//BMS2.sysVol= (uint8_t)(VOL/1000);
	CUR = UART4_STA.RX_BUF[7] + (UART4_STA.RX_BUF[8] << 8) + (UART4_STA.RX_BUF[9] << 16) + (UART4_STA.RX_BUF[10] << 24);
	BMS2.sysCurt = (uint8_t)(Hex2Dec_32(CUR) / 100);
	//BMS2.sysCurt=(uint8_t)(CUR/100);
	return 0;
}

long Hex2Dec_32(uint32_t temp)
{
	uint32_t Hex = temp;
	uint32_t Dec = 0;
	for (int j = 0; j < 32; j++)
	{
		if (Hex & 0x01U)
		{
			Dec += pow(2, j);
			Hex >>= 1;

		}
		else
		{
			Dec += 0;
			Hex >>= 1;
		}
	}
	return Dec;
}



