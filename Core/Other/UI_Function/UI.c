#include "UI.h"


//void UI_task_Initialize(USART_STA *usart_struct)
//{
//		usart_struct->TxCount=0;
//		osDelay(1000);
//		taskENTER_CRITICAL();	
//		usart_struct->TX_BUF[0]=0x5a;
//		usart_struct->TX_BUF[1]=0xa5;	
//		usart_struct->TX_BUF[2]=0x0e;			
//		usart_struct->TX_BUF[3]=0x80;	
//		usart_struct->TX_BUF[4]=0x56;							
//		usart_struct->TX_BUF[5]=0x5A;
//		usart_struct->TX_BUF[6]=0xa0;
//		usart_struct->TX_BUF[7]=0x04;
//		usart_struct->TX_BUF[8]=0x80;
//		usart_struct->TX_BUF[9]=0x41;
//		usart_struct->TX_BUF[10]=0x00;
//		usart_struct->TX_BUF[11]=0x41;
//		usart_struct->TX_BUF[12]=0x00;	
//		usart_struct->TX_BUF[13]=0x00;	
//		usart_struct->TX_BUF[14]=0x0b;	
//		usart_struct->TX_BUF[15]=0x78;	
//		usart_struct->TX_BUF[16]=0x42;
//		usart_struct->TX_LEN=17;	
//		if(usart_struct->TX_LEN != 0)
//		{
//			UsartSendData_DMA(5,usart_struct->TX_BUF,usart_struct->TX_LEN);               //����5
//			usart_struct->TX_LEN=0;
//		}																	
//		if(UIInitTaskHandle != NULL)
//		{
//				vTaskDelete(UIInitTaskHandle);
//				UIInitTaskHandle = NULL;
//		}
//		taskEXIT_CRITICAL();
//}
//void UI_task_Function(USART_STA *usart_struct)
//{
//		uint16_t crcfifo;
//		usart_struct->TxCount=0;
//		osDelay(3000);
//		taskENTER_CRITICAL();			
//		usart_struct->TX_BUF[usart_struct->TxCount++]=0x5a;
//		usart_struct->TX_BUF[usart_struct->TxCount++]=0xa5;	
//		usart_struct->TX_BUF[usart_struct->TxCount++]=0x5f;          //�ұ�95������			
//		usart_struct->TX_BUF[usart_struct->TxCount++]=0x82;	
//		usart_struct->TX_BUF[usart_struct->TxCount++]=0x30;							
//		usart_struct->TX_BUF[usart_struct->TxCount++]=0x00;
//	
//		usart_struct->TX_BUF[usart_struct->TxCount++]=BMS1.current>>8;								usart_struct->TX_BUF[usart_struct->TxCount++]=BMS1.current;	//����
//		usart_struct->TX_BUF[usart_struct->TxCount++]=BMS1.voltage>>8;								usart_struct->TX_BUF[usart_struct->TxCount++]=BMS1.voltage;	//��ѹ
//		usart_struct->TX_BUF[usart_struct->TxCount++]=BMS1.capacity_per>>8;						usart_struct->TX_BUF[usart_struct->TxCount++]=BMS1.capacity_per;	//��ǰʣ������
//		usart_struct->TX_BUF[usart_struct->TxCount++]=0x0073>>8;											usart_struct->TX_BUF[usart_struct->TxCount++]=0x0073;	//��ǰ���������
//		usart_struct->TX_BUF[usart_struct->TxCount++]=0x0073>>8;											usart_struct->TX_BUF[usart_struct->TxCount++]=0x0073;	//�������
//		usart_struct->TX_BUF[usart_struct->TxCount++]=BMS1.capacity_per>>8;						usart_struct->TX_BUF[usart_struct->TxCount++]=BMS1.capacity_per;	//�����ٷֱ�
//		usart_struct->TX_BUF[usart_struct->TxCount++]=BMS1.Max_unit_voltage>>8;				usart_struct->TX_BUF[usart_struct->TxCount++]=BMS1.Max_unit_voltage;	//��ߵ����ѹ
//		usart_struct->TX_BUF[usart_struct->TxCount++]=BMS1.Min_unit_voltage>>8; 			usart_struct->TX_BUF[usart_struct->TxCount++]=BMS1.Min_unit_voltage;	//��͵����ѹ
//		usart_struct->TX_BUF[usart_struct->TxCount++]=BMS1.Max_unit_Temperature>>8; 	usart_struct->TX_BUF[usart_struct->TxCount++]=BMS1.Max_unit_Temperature;	//����¶ȵ�
//		usart_struct->TX_BUF[usart_struct->TxCount++]=BMS1.Min_unit_Temperature>>8; 	usart_struct->TX_BUF[usart_struct->TxCount++]=BMS1.Min_unit_Temperature;	//����¶ȵ�
//		usart_struct->TX_BUF[usart_struct->TxCount++]=BMS1.cycle_count>>8; 						usart_struct->TX_BUF[usart_struct->TxCount++]=BMS1.cycle_count;	//ѭ������
//		//״̬��Ϣ
//		usart_struct->TX_BUF[usart_struct->TxCount++]=(BMS1.Alarm_Type&0x2000)>>13; 			usart_struct->TX_BUF[usart_struct->TxCount++]=(BMS1.Alarm_Type&0x2000);	//���״̬	1����硣0���ŵ�
//		usart_struct->TX_BUF[usart_struct->TxCount++]=(BMS1.Alarm_Type&0x0800)>>11; 			usart_struct->TX_BUF[usart_struct->TxCount++]=(BMS1.Alarm_Type&0x0800);	//����������30%
//		usart_struct->TX_BUF[usart_struct->TxCount++]=(BMS1.Alarm_Type&0x0400)>>10; 			usart_struct->TX_BUF[usart_struct->TxCount++]=(BMS1.Alarm_Type&0x0400);	//��ز����� 1
//		usart_struct->TX_BUF[usart_struct->TxCount++]=(BMS1.Alarm_Type&0x0200)>>9; 				usart_struct->TX_BUF[usart_struct->TxCount++]=(BMS1.Alarm_Type&0x0200);	//�ŵ��·���� 1
//		usart_struct->TX_BUF[usart_struct->TxCount++]=(BMS1.Alarm_Type&0x0100)>>8; 				usart_struct->TX_BUF[usart_struct->TxCount++]=(BMS1.Alarm_Type&0x0100);	//����·���� 1
//		usart_struct->TX_BUF[usart_struct->TxCount++]=(BMS1.Alarm_Type&0x0080)>>7; 				usart_struct->TX_BUF[usart_struct->TxCount++]=(BMS1.Alarm_Type&0x0080);	//�ŵ���������� 1
//		usart_struct->TX_BUF[usart_struct->TxCount++]=(BMS1.Alarm_Type&0x0040)>>6; 				usart_struct->TX_BUF[usart_struct->TxCount++]=(BMS1.Alarm_Type&0x0040);	//������������ 1
//		usart_struct->TX_BUF[usart_struct->TxCount++]=(BMS1.Alarm_Type&0x0020)>>5; 				usart_struct->TX_BUF[usart_struct->TxCount++]=(BMS1.Alarm_Type&0x0020);	//���±��� 1
//		usart_struct->TX_BUF[usart_struct->TxCount++]=(BMS1.Alarm_Type&0x0010)>>4; 				usart_struct->TX_BUF[usart_struct->TxCount++]=(BMS1.Alarm_Type&0x0010);	//���±��� 1
//		usart_struct->TX_BUF[usart_struct->TxCount++]=(BMS1.Alarm_Type&0x0008)>>3; 				usart_struct->TX_BUF[usart_struct->TxCount++]=(BMS1.Alarm_Type&0x0008);	//���±��� 1
//		usart_struct->TX_BUF[usart_struct->TxCount++]=(BMS1.Alarm_Type&0x0004)>>2; 				usart_struct->TX_BUF[usart_struct->TxCount++]=(BMS1.Alarm_Type&0x0004);	//���±��� 1
//		usart_struct->TX_BUF[usart_struct->TxCount++]=(BMS1.Alarm_Type&0x0002)>>1; 				usart_struct->TX_BUF[usart_struct->TxCount++]=(BMS1.Alarm_Type&0x0002);	//����Ƿѹ 1
//		usart_struct->TX_BUF[usart_struct->TxCount++]=(BMS1.Alarm_Type&0x0001)>>0; 				usart_struct->TX_BUF[usart_struct->TxCount++]=(BMS1.Alarm_Type&0x0001);	//���峬ѹ 1
//		//����״̬
//		usart_struct->TX_BUF[usart_struct->TxCount++]=0x0001>>8; 				usart_struct->TX_BUF[usart_struct->TxCount++]=0x0001;	//����״̬ 1���� 0δ����
//		usart_struct->TX_BUF[usart_struct->TxCount++]=0x0001>>8; 				usart_struct->TX_BUF[usart_struct->TxCount++]=0x0001;	//����״̬ 0���� 1���� 2��ͣ
//		usart_struct->TX_BUF[usart_struct->TxCount++]=0x0001>>8; 				usart_struct->TX_BUF[usart_struct->TxCount++]=0x0001;	//����ģʽ 0�Զ� 1���� 2����
//		usart_struct->TX_BUF[usart_struct->TxCount++]=0x0001>>8; 				usart_struct->TX_BUF[usart_struct->TxCount++]=0x0001;	//ͨ�ű���
//		usart_struct->TX_BUF[usart_struct->TxCount++]=0x0000>>8; 				usart_struct->TX_BUF[usart_struct->TxCount++]=Motor_alarm&0x01;	//���1����
//		usart_struct->TX_BUF[usart_struct->TxCount++]=0x0000>>8; 				usart_struct->TX_BUF[usart_struct->TxCount++]=(Motor_alarm&0x02)>>1;	//���2����
//		usart_struct->TX_BUF[usart_struct->TxCount++]=0x0000>>8; 				usart_struct->TX_BUF[usart_struct->TxCount++]=(Motor_alarm&0x04)>>2;	//���3����
//		usart_struct->TX_BUF[usart_struct->TxCount++]=0x0000>>8; 				usart_struct->TX_BUF[usart_struct->TxCount++]=(Motor_alarm&0x08)>>3;	//���4����	
//		usart_struct->TX_BUF[usart_struct->TxCount++]=0x0000>>8; 				usart_struct->TX_BUF[usart_struct->TxCount++]=(Motor_alarm&0x10)>>4;	//���5����	
//		usart_struct->TX_BUF[usart_struct->TxCount++]=0x0000>>8; 				usart_struct->TX_BUF[usart_struct->TxCount++]=(Motor_alarm&0x20)>>5;	//���6����	
//		usart_struct->TX_BUF[usart_struct->TxCount++]=0x0000>>8; 				usart_struct->TX_BUF[usart_struct->TxCount++]=(Motor_alarm&0x40)>>6;	//���7����	
//		usart_struct->TX_BUF[usart_struct->TxCount++]=0x0000>>8; 				usart_struct->TX_BUF[usart_struct->TxCount++]=(Motor_alarm&0x80)>>7;	//���8����	
//		usart_struct->TX_BUF[usart_struct->TxCount++]=0x0000>>8; 				usart_struct->TX_BUF[usart_struct->TxCount++]=Motor_Alarm9;	//���9����	
//		usart_struct->TX_BUF[usart_struct->TxCount++]=0x0000>>8; 				usart_struct->TX_BUF[usart_struct->TxCount++]=Motor_Alarm10;	//���10����	
//		usart_struct->TX_BUF[usart_struct->TxCount++]=0x0000>>8; 				usart_struct->TX_BUF[usart_struct->TxCount++]=Motor_Alarm11;	//���11����	
//		usart_struct->TX_BUF[usart_struct->TxCount++]=0x0000>>8; 				usart_struct->TX_BUF[usart_struct->TxCount++]=Motor_Alarm12;	//���12����
//		usart_struct->TX_BUF[usart_struct->TxCount++]=0x04d2>>8; 				usart_struct->TX_BUF[usart_struct->TxCount++]=0x0001;	//�����
//		usart_struct->TX_BUF[usart_struct->TxCount++]=0x04d2>>8; 				usart_struct->TX_BUF[usart_struct->TxCount++]=0x0001; //�汾��
//		usart_struct->TX_BUF[usart_struct->TxCount++]=INNER.TARGET_SPEED>>8; 				usart_struct->TX_BUF[usart_struct->TxCount++]=INNER.TARGET_SPEED; //���ٶ�
//		usart_struct->TX_BUF[usart_struct->TxCount++]=(int16_t)INNER.TARGET_RPS>>8; 				usart_struct->TX_BUF[usart_struct->TxCount++]=(int16_t)INNER.TARGET_RPS; //���ٶ�
//		usart_struct->TX_BUF[usart_struct->TxCount++]=INNER.TARGET_ANGLE>>8; 				usart_struct->TX_BUF[usart_struct->TxCount++]=INNER.TARGET_ANGLE; //��ʻ����						
//		crcfifo = CRC16(&UART5_STA.TX_BUF[3],93);                  //����5  ����λ��ʼ��93����CRCУ��
//		usart_struct->TX_BUF[usart_struct->TxCount++]=crcfifo;
//		usart_struct->TX_BUF[usart_struct->TxCount++]=crcfifo>>8;


//		if(usart_struct->TxCount != 0)
//		{
//			UsartSendData_DMA(5,usart_struct->TX_BUF,usart_struct->TxCount);               //����5
//			usart_struct->TxCount=0;
//		}	
//		taskEXIT_CRITICAL();
//}
