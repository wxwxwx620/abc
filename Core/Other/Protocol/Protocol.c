/* Includes ------------------------------------------------------------------*/
#include "Protocol.h"
#include "usart.h"
#include "stdio.h"
#include "FIFO.h"
#include "remote_control.h"
#include "motor.h"

uint8_t BMS_Relay;
uint8_t rc_srcdata[18]={0};
uint8_t pc_srcdata[LEN_PC_2_AGV]={0};
//USART_STA UART5_STA;

typedef __packed struct
{
	uint8_t     head;               //��ͷ
	uint8_t     cmd;                //��ǰָ��
	uint8_t     clipState:2;        //����״̬      0 ����λ  1����λ  2������ 3 �м�λ
	uint8_t     ajustDisState:2;    //������״̬  0δ����  1������� 2 ������  3��������
	uint8_t     chargeState:1;      //���״̬      0 δ��� 1�����
	uint8_t     carState:1;         //����״̬      0 �޳��� 1�г���
	uint8_t     onLineState:1;      //����״̬      0  ����  1����
	uint8_t     ctrlMode:1;         //����ģʽ      0  �ֶ�  1�Զ�
	uint8_t     stop:1;             //��ͣ״̬      0δ��ͣ   1��ͣ
	uint8_t     dirReady:1;         //���ӷ����������  0δ����  1����
	uint8_t     remark : 6;         //����
	uint8_t     errorCode;          //������
	uint16_t    errorInfo;          //���󸽼���Ϣ
	int16_t     odoX;               //��̼�X mm
	int16_t     odoY;               //��̼�Y mm
	int16_t     odoZ;               //��̼ƽǶ� 0.1
	uint8_t     power;              //����
	uint16_t    wheelBaseLength;    //���ֵ mm

}SendPCData;

SendPCData sendData;

void rc_get_srcdata(uint8_t* temp)
{
	memcpy(temp,rc_srcdata,18);
}
void rc_clear_srcdata()
{
	memset(rc_srcdata,0,18);
}
void pc_get_srcdata(uint8_t* temp)
{
	memcpy(temp,pc_srcdata,LEN_PC_2_AGV);
}
void pc_clear_srcdata()
{
	memset(pc_srcdata,0,LEN_PC_2_AGV);
}

////DMA�ж�ѭ��������ȡң��������
//void RC_Receive_handler(UART_HandleTypeDef *huart)
//{
//	uint8_t data[36]={0};
//	float RADv;
//	char t_buf[128];
//  //�崮��״̬�Ĵ���
//  data[0] = USART1->SR;
//  data[0] = USART1->DR;
//	
//	RC_STA.RX_LEN = RX_RC_Size-__HAL_DMA_GET_COUNTER(huart->hdmarx);
//	
//	HAL_UART_DMAStop(huart);
//	memcpy(data,RC_STA.RX_BUF,RC_STA.RX_LEN);
//	HAL_UART_Receive_DMA(huart,RC_STA.RX_BUF,RX_RC_Size);	
//	if(write_ringbuff(&RC_buff,data,RC_STA.RX_LEN))
//	{

//		while(RC_buff.lenght>=18)
//		{
//			
//			Read_RingBuff(&RC_buff,data,1);

//			if(RC_buff.ring_buff[RC_buff.head] == 0xFF)
//			{

//				if(usMBCRC16(&RC_buff.ring_buff[RC_buff.head], 16)
//						==(RC_buff.ring_buff[RC_buff.head+16]+(RC_buff.ring_buff[RC_buff.head+17]<<8)))
//				{
//					memcpy(rc_srcdata,&RC_buff.ring_buff[RC_buff.head],18);
//					Read_RingBuff(&RC_buff,data,17);
//					rc_updateCount=600;		// ң��������ֹͣ
//					Usart1Send_RC();
//				}
//			}
//		}	
//	}
//}



//DMA�ж�ѭ��������ȡң��������
void RC_Receive_handler(UART_HandleTypeDef *huart)
{
	uint8_t data[36]={0};
	//float RADv;
	//char t_buf[128];
  //�崮��״̬�Ĵ���
  data[0] = USART1->SR;
  data[0] = USART1->DR;
	
	RC_STA.RX_LEN = RX_RC_Size-__HAL_DMA_GET_COUNTER(huart->hdmarx);
	
	HAL_UART_DMAStop(huart);
	
	memcpy(data,RC_STA.RX_BUF,RC_STA.RX_LEN);
	
	HAL_UART_Receive_DMA(huart,RC_STA.RX_BUF,RX_RC_Size);
	if(write_ringbuff(&RC_buff,data,RC_STA.RX_LEN))
	{
		while(RC_buff.lenght>=18)
		{
			if(RC_buff.ring_buff[RC_buff.head] == 0xFF)
			{
				Read_RingBuff(&RC_buff,data,18);
				if(usMBCRC16(data, 16)
						==(data[16]+(data[17]<<8)))
				{
					memcpy(rc_srcdata,data,18);
					move_ringbuff_head(&RC_buff,17);
					rc_updateCount++;		// ң��������ֹͣ
//					Usart1Send_RC();
				}
			}
			move_ringbuff_head(&RC_buff,1);
		}
	}
}


////DMA�ж�ѭ��������ȡ��λ������
//void PC_Receive_handler(UART_HandleTypeDef *huart)
//{
//	uint8_t data[LEN_PC_2_AGV*2]={0};
//	float RADv;
//	char t_buf[128];
//  //�崮��״̬�Ĵ���
//  data[0] = USART2->SR;
//  data[0] = USART2->DR;
//	
//	PC_STA.RX_LEN = RX_PC_Size-__HAL_DMA_GET_COUNTER(huart->hdmarx);
//	
//	HAL_UART_DMAStop(huart);
//	for(uint8_t i=0;i<PC_STA.RX_LEN;i++)
//	{
//		data[i]=PC_STA.RX_BUF[i];
//	}
//	//memcpy(data,PC_STA.RX_BUF,PC_STA.RX_LEN);

//	HAL_UART_Receive_DMA(huart,PC_STA.RX_BUF,RX_PC_Size);	
//	if(write_ringbuff(&PC_buff,data,PC_STA.RX_LEN))
//	{

//		while(PC_buff.lenght>=LEN_PC_2_AGV)
//		{
//			
//			Read_RingBuff(&PC_buff,data,1);

//			if(PC_buff.ring_buff[PC_buff.head] == 0xF1)
//			{

//				if(getchecksum(&PC_buff.ring_buff[PC_buff.head], LEN_PC_2_AGV-2)
//						== (PC_buff.ring_buff[PC_buff.head+LEN_PC_2_AGV-1]+(PC_buff.ring_buff[PC_buff.head+LEN_PC_2_AGV-2]<<8)))
//				{
//					memcpy(pc_srcdata,&PC_buff.ring_buff[PC_buff.head],LEN_PC_2_AGV);
//					//Read_RingBuff(&PC_buff,data,12);
//					Read_RingBuff(&PC_buff,data,LEN_PC_2_AGV-1);
//					pc_updateCount = 10;//6;//1s��ʱ	
//					Usart2Send_PC();
//					//to pc
//				}
//			}
//		}
//	}
//}



//DMA�ж�ѭ��������ȡ��λ������
void PC_Receive_handler(UART_HandleTypeDef *huart)
{
	uint8_t data[LEN_PC_2_AGV*2]={0};
//	float RADv;
//	char t_buf[128];
  //�崮��״̬�Ĵ���
  data[0] = USART2->SR;
  data[0] = USART2->DR;
	
	PC_STA.RX_LEN = RX_PC_Size-__HAL_DMA_GET_COUNTER(huart->hdmarx);
	
	HAL_UART_DMAStop(huart);
//	for(uint8_t i=0;i<PC_STA.RX_LEN;i++)
//	{
//		data[i]=PC_STA.RX_BUF[i];
//	}
  memcpy(data,PC_STA.RX_BUF,PC_STA.RX_LEN);

	HAL_UART_Receive_DMA(huart,PC_STA.RX_BUF,RX_PC_Size);	
	if(write_ringbuff(&PC_buff,data,PC_STA.RX_LEN))
	{

		while(PC_buff.lenght>=LEN_PC_2_AGV)
		{
			if(PC_buff.ring_buff[PC_buff.head] == 0xF1)
			{
			  Read_RingBuff(&PC_buff,data,LEN_PC_2_AGV);
				if(getchecksum(data, LEN_PC_2_AGV-2)
						== (data[LEN_PC_2_AGV-1]+(data[LEN_PC_2_AGV-2]<<8)))
				{
					memcpy(pc_srcdata,data,LEN_PC_2_AGV);
					//Read_RingBuff(&PC_buff,data,12);
					move_ringbuff_head(&PC_buff,LEN_PC_2_AGV-1);
					pc_updateCount ++;//6;//1s��ʱ	
					//Usart2Send_PC();
					SendToPC();
					//to pc
				}
			}
			move_ringbuff_head(&PC_buff,1);
		}
	}
}

void Usart2Send_PC(void)
{
//		memset(USART2_STA.RX_BUF,0,128);
	    uint16_t checksumFIFO;
//			float RADv;		
	
			USART2_STA.TX_BUF[0] =	0xF2;
			USART2_STA.TX_BUF[1] =	0x70;//USART2_STA.RX_BUF[1];//����
			USART2_STA.TX_BUF[4]=0;//����
			USART2_STA.TX_BUF[6]=0;
			USART2_STA.TX_BUF[7]=0;//Ԥ��
			if(agvStatus.onLineState==1) //��ǰΪ����״̬
			{
				USART2_STA.TX_BUF[3] |=0x10;//����
				USART2_STA.TX_BUF[6] |=0x02;//
			}//
			else if(agvStatus.onLineState==0)
			{		
				USART2_STA.TX_BUF[3] &=0xef;//����
				USART2_STA.TX_BUF[6] &=0x01;//
			}
			
			if((BMS1.Cha_Relay_Sta&0x01)==0x01) USART2_STA.TX_BUF[3] |=0x04;//�������
			else USART2_STA.TX_BUF[3] &=0xfb;
			
			if(BMS1.current>=0)	USART2_STA.TX_BUF[3] &=0x7f;//�ŵ�
			else								USART2_STA.TX_BUF[3] |=0x80;//���

			USART2_STA.TX_BUF[3]&=0xfc;  //����״̬λ�����㣬�ٸ��ݵ�ǰ״̬����
			if(agvStatus.clipState==ClipState_Closed)  USART2_STA.TX_BUF[3] |=0x01;//�ص�λ //20220911
			else if(agvStatus.clipState == ClipState_Running)	USART2_STA.TX_BUF[3] |=0x02;//������ //20220911
			else if(agvStatus.clipState == ClipState_Stoped)	USART2_STA.TX_BUF[3] |=0x03;//ֹͣ	 //20220911				
			
			if((UPB_LED==0)&&(UPF_LED==0)) USART2_STA.TX_BUF[3]|=0x08;//������
			else USART2_STA.TX_BUF[3]&=0xf7;//������
			
			if (agvStatus.ajustDisState == AjustState_OK)
			{
				USART2_STA.TX_BUF[3] |= 0x80;//���������
				ctrl_mode.state.WheelbaseAdjust_mode_pc = 0;//���������
			}
			else
			{
				USART2_STA.TX_BUF[3] &= 0x7F;//��������
			}
			//if(ctrl_mode.state.WheelbaseAdjust_mode_pc)
			//{					
			//	USART2_STA.TX_BUF[3]&=0x7F;//��������
			//}
			//if(!WheelbaseAdjust_mode_pc_Flag)
			//{
			//	USART2_STA.TX_BUF[3]|=0x80;//���������
			//	ctrl_mode.state.WheelbaseAdjust_mode_pc=0;//���������
			//}
			//if((Wheelbase_Data>=WheelbaseUpperLimit)||(Wheelbase_Data<=WheelbaseLowerLimit))
			//{
			//	USART2_STA.TX_BUF[3]|=0x80;//���������
			//	ctrl_mode.state.WheelbaseAdjust_mode_pc=0;//���������
			//}
			
						
			if((ctrl_mode.state.scram_alarm_filter&0x01)==0x01) USART2_STA.TX_BUF[2] |=0x01;//���弱ͣ
			else	USART2_STA.TX_BUF[2] &=0xfe;	
				
//			if((Motor_alarm>=1)||(Lif_Motor_alarm>=1)) USART2_STA.TX_BUF[2] |=00x02;//�������
//			else USART2_STA.TX_BUF[2]&=0xfd;//�ޱ���	
      USART2_STA.TX_BUF[2]&=0xfd;//�ޱ���	
			
//			if(1==BMS_time_out_Flag)	USART2_STA.TX_BUF[2]|=0x10;//���ͨ���쳣
//			else 											USART2_STA.TX_BUF[2]&=0xef;//���ͨ������
			USART2_STA.TX_BUF[2]&=0xef;//���ͨ������			
			
//			USART2_STA.TX_BUF[5]=BMS1.capacity_per;	//����
//			if(BMS1.capacity_per<=20)	USART2_STA.TX_BUF[2] |=0x04;//������
//			else USART2_STA.TX_BUF[2]&=0xfb;				
			USART2_STA.TX_BUF[5]=80;	//����
			USART2_STA.TX_BUF[2]&=0xfb;						
					
			if(agvStatus.ctrlMode !=PC_Auto)		//ң�ط�ʽ
			{	
				USART2_STA.TX_BUF[2] |=0x08;//�޿���Ȩ
//						scram_alarm_filter &=0xfb;
			}
			else
			{						

					USART2_STA.Voice_RX=USART2_STA.RX_BUF[8];	//����end
						
					USART2_STA.TX_BUF[2] &=0xf7;
					//if((USART2_STA.RX_BUF[9]&0x08) == 0x08) //����	
					//{
					//	ctrl_mode.state.scram_alarm_filter |=0x04;//ֹͣ����λ������ֹͣ
					//}
					//else
					{
							ctrl_mode.state.scram_alarm_filter &=0xfb;
							//if((USART2_STA.RX_BUF[10]&0x80)==0x80)		BRAKE_EN=0;//���ʹ�ܷ�
							//else																			BRAKE_EN=1;//���ʹ�ܺ�
							//if((USART2_STA.RX_BUF[9]&0x04)==0x04)//���̵���
							//{
							//	BMS_Relay=1;
							//}	
							//else if((USART2_STA.RX_BUF[9]&0x04)==0x00)
							//{
							//	BMS_Relay=0;
							//}	
							//if((USART2_STA.RX_BUF[9]&0x80)==0x80)   //���߿���
							//{

							//}           
							//else 
							//{
							//		ctrl_mode.state.scram_alarm_filter &=0xeF;	//��ͣ��־λ
							//}
						}
					}
							
							USART2_STA.TX_BUF[4]=0x00;//0xff&(LN1_IP1|LN1_IP2<<1|Laser_Edge_F<<2|LN2_IP1<<3|LN2_IP2<<4|Laser_Edge_F<<5|UPF_LED<<6|UPB_LED<<7);//���ⷴ����̽�ﷴ��	 
		//new start  ���ߵ������
							USART2_STA.TX_BUF[7]=0x00;//0xff&(Motor_Alarm_FL|Motor_Alarm_FR<<1|Motor_Alarm_BR<<2|Motor_Alarm_BL<<3|Motor_Alarm_Clip_FL<<4|Motor_Alarm_Clip_FR<<5|Motor_Alarm_Clip_BR<<6|Motor_Alarm_Clip_BL<<7);
		//  				 �����������
							USART2_STA.TX_BUF[8]=0x00;//0xff&(Motor_Alarm9|Motor_Alarm10<<1|Motor_Alarm11<<2|Motor_Alarm12<<3);
		//  				 ��ص���
							USART2_STA.TX_BUF[9]=30;//BMS1.current/10;	
		//  				 ��ص�ѹ	
							USART2_STA.TX_BUF[10]=80;//BMS1.voltage/10;
		//  				 ��ŵ�ѭ������
							USART2_STA.TX_BUF[11]=50;//BMS1.cycle_count;		
							USART2_STA.TX_BUF[12]=50;//BMS1.cycle_count>>8;	
		//  				 ��ع���
							USART2_STA.TX_BUF[13]=0x00;//BMS1.Alarm_Type;
		//					 �������
							USART2_STA.TX_BUF[14]=0x00;//0xff&(LN1_OP1|LN1_OP2<<1|LN1_OP3<<2|LN1_2_OP4<<3|LN1_2_OP5<<4);
	//					
////							USART2_STA.TX_BUF[15]=0x00;//LF_LXD|RF_LXD<<1|LR_LXD<<2|RR_LXD<<3|LR_AK<<4|BRAKE_EN<<5|MOTORPOWER<<6;
////							USART2_STA.TX_BUF[16]=0x00;
////							USART2_STA.TX_BUF[17]=0x00;
////							USART2_STA.TX_BUF[18]=0x00;

//////							USART2_STA.TX_BUF[19]=UL9|DL9<<1|UL10<<2|DL10<<3|UL11<<4|DL11<<5|UL12<<6|DL12<<7;
////							USART2_STA.TX_BUF[20]=0x00;//ES;//LF_JT|RF_JT<<1|LR_JT<<2|RR_JT<<3;
////							USART2_STA.TX_BUF[21]=0x00;
////							USART2_STA.TX_BUF[22]=0x00;
							
							USART2_STA.TX_BUF[15]=((int16_t)CAR_Odometer.RT_VX)>>8;//LF_LXD|RF_LXD<<1|LR_LXD<<2|RR_LXD<<3|LR_AK<<4|BRAKE_EN<<5|MOTORPOWER<<6;
							USART2_STA.TX_BUF[16]=((int16_t)CAR_Odometer.RT_VX);
							USART2_STA.TX_BUF[17]=((int16_t)CAR_Odometer.RT_VY)>>8;
							USART2_STA.TX_BUF[18]=((int16_t)CAR_Odometer.RT_VY);

							USART2_STA.TX_BUF[19]=((int16_t)CAR_Odometer.RT_RPS)>>8;
							USART2_STA.TX_BUF[20]=((int16_t)CAR_Odometer.RT_RPS);//ES;//LF_JT|RF_JT<<1|LR_JT<<2|RR_JT<<3;
							
							
							USART2_STA.TX_BUF[21]=((int16_t)Wheelbase_Data)>>8;//ʵʱ����λ
							USART2_STA.TX_BUF[22]=(int16_t)Wheelbase_Data;//ʵʱ����λ


							USART2_STA.TX_BUF[23] = ((int16_t)CAR_Odometer.ODOMETER_X)>>8;
							USART2_STA.TX_BUF[24] = (int16_t)CAR_Odometer.ODOMETER_X;
							USART2_STA.TX_BUF[25] = ((int16_t)CAR_Odometer.ODOMETER_Y)>>8;	
							USART2_STA.TX_BUF[26] = (int16_t)CAR_Odometer.ODOMETER_Y;	
							USART2_STA.TX_BUF[27] = ((int16_t)CAR_Odometer.PC_ODOMETER_Z)>>8;	
							USART2_STA.TX_BUF[28] = (int16_t)CAR_Odometer.PC_ODOMETER_Z;						
	//new end  	 
	/********************************����ֵ*************************************/

						checksumFIFO = getchecksum(USART2_STA.TX_BUF, 29);															
						USART2_STA.TX_BUF[29] =	checksumFIFO>>8;
						USART2_STA.TX_BUF[30] =	checksumFIFO;	
						USART2_STA.TX_LEN = 31;		
					
	
	
				if(USART2_STA.TX_LEN != 0)
				{
					USART2_STA.TX_FLAG =0;
					UsartSendData_DMA(2,USART2_STA.TX_BUF,USART2_STA.TX_LEN);
					USART2_STA.TX_LEN=0;
		//			memset(USART2_STA.RX_BUF,0,sizeof(USART2_STA.RX_BUF));
				}	

}//����������

int testlen=0;
uint8_t buf[18]={0};
//���͸���λ������
void SendToPC(void)
{
	 
	sendData.head= 0xAA;
	sendData.cmd= agvTask.cmd;
	sendData.clipState = agvStatus.clipState ;
	sendData.ajustDisState = agvStatus.ajustDisState ;
	sendData.chargeState = agvStatus.chargeState ;
	sendData.carState = agvStatus.carState ;
	sendData.onLineState = agvStatus.onLineState ;
	sendData.ctrlMode = agvStatus.ctrlMode ;
	sendData.stop = agvStatus.stopFlag ;
	sendData.dirReady = agvStatus.dirReady ;
	//sendData.remark = agvStatus.remark ;
	sendData.errorCode= agvStatus.errorCode;
	sendData.errorInfo= agvStatus.errorInfo;
	sendData.odoX= agvStatus.odoX;
	sendData.odoY= agvStatus.odoY;
	sendData.odoZ= agvStatus.odoZ;
	sendData.power= agvStatus.power;
	sendData.wheelBaseLength= agvStatus.wheelBaseLength;
	
//	testlen=sizeof(sendData);
	
	
	memcpy(USART2_STA.TX_BUF,&sendData,16);	
	uint16_t	checksumFIFO = getchecksum(USART2_STA.TX_BUF, 16);
	USART2_STA.TX_BUF[16] =	checksumFIFO>>8;
	USART2_STA.TX_BUF[17] =	checksumFIFO;
	USART2_STA.TX_FLAG =0;
	UsartSendData_DMA(2, USART2_STA.TX_BUF, 18);	
	USART2_STA.TX_LEN=0;
}




void Usart1Send_RC(void)
{
//	    uint16_t checksumFIFO;
			uint16_t crcfifo;
	    int16_t g;
	    g=Wheelbase_Data;
	
			USART1_STA.TX_BUF[0] =	0xFF;
			USART1_STA.TX_BUF[1] =	0xAF;
			
	    USART1_STA.TX_BUF[2] =	g;                 //���Ͱ�λ
	    USART1_STA.TX_BUF[3] =	g>>8;              //���߰�λ
			USART1_STA.TX_BUF[4] =	rc_srcdata[11];	   // ���߷��� 0-360��      ң��������ֵ������ʵ����ֵ
			USART1_STA.TX_BUF[5] =	rc_srcdata[12];
			USART1_STA.TX_BUF[6] =	rc_srcdata[13];		 // �����ٶ� 0-600mm/s   ң��������ֵ������ʵ����ֵ
			USART1_STA.TX_BUF[7] =	rc_srcdata[14];
			USART1_STA.TX_BUF[8] =	rc_srcdata[15];    // ��ת�ٶ� 0-20��/s    ң��������ֵ������ʵ����ֵ
			USART1_STA.TX_BUF[9] =  rc_srcdata[7];	   //�ֶ��Զ�
	    	    
	    USART1_STA.TX_BUF[10] = 0;
	    USART1_STA.TX_BUF[11] = 0;	
		  USART1_STA.TX_BUF[12] = 0;	
	    USART1_STA.TX_BUF[13] = 0;	
		  USART1_STA.TX_BUF[14] = 0;	
	    USART1_STA.TX_BUF[15] = 0;	

	
//		  USART1_STA.TX_BUF[2] =	rc_srcdata[9];    // ���  ���ǰ����04   ���ֹͣ��02    �����ˣ�01  
//	    USART1_STA.TX_BUF[3] =	rc_srcdata[10];
//			USART1_STA.TX_BUF[4] =	rc_srcdata[11];	  // ���߷��� 0-360��      ң��������ֵ������ʵ����ֵ
//			USART1_STA.TX_BUF[5] =	rc_srcdata[12];
//			USART1_STA.TX_BUF[6] =	rc_srcdata[13];		// �����ٶ� 0-600mm/s   ң��������ֵ������ʵ����ֵ
//			USART1_STA.TX_BUF[7] =	rc_srcdata[14];
//			USART1_STA.TX_BUF[8] =	rc_srcdata[15];    // ��ת�ٶ� 0-20��/s    ң��������ֵ������ʵ����ֵ
	

//		rc_ctrl.MovingDirection = (srcdata[10] + (srcdata[11] << 8));     // ���߷��� 0-360��
//		rc_ctrl.MovingSpeed = (srcdata[12] + (srcdata[13] << 8));     // �����ٶ� 0-600mm/s
//		rc_ctrl.RotationSpeed = (srcdata[14] + (srcdata[15] << 8));     // ��ת�ٶ� 0-20��/s
//	  RADv = rc_ctrl.RotationSpeed*0.1*3.1415926/180.0;	
	

//new end  	 
/********************************����ֵ*************************************/
			crcfifo = CRC16(&USART1_STA.TX_BUF[0],16);                  //����5  0��ʼ��9����CRCУ����
			USART1_STA.TX_BUF[16] = crcfifo;
			USART1_STA.TX_BUF[17] = crcfifo>>8;	
      USART1_STA.TX_LEN = 18;		
//			checksumFIFO = getchecksum(USART1_STA.TX_BUF, 9);				//�ۼӺ�У����				 							
//			USART1_STA.TX_BUF[9] =	checksumFIFO>>8;
//			USART1_STA.TX_BUF[10] =	checksumFIFO;	
//			USART1_STA.TX_LEN = 11;		

			if(USART1_STA.TX_LEN != 0)
			{
				USART1_STA.TX_FLAG =0;
				UsartSendData_DMA(1,USART1_STA.TX_BUF,USART1_STA.TX_LEN);
				USART1_STA.TX_LEN=0;
	//			memset(USART2_STA.RX_BUF,0,sizeof(USART2_STA.RX_BUF));
			}	
}











/***********����5  ui��Ϊң����*************/
void Usart5Send_RC(void)
{
////	    uint16_t checksumFIFO;
//			uint16_t crcfifo;
//	    int16_t g;
//	    g=f;
//	
//			UART5_STA.TX_BUF[0] =	0xFF;
//			UART5_STA.TX_BUF[1] =	0xAF;
//			
//	    UART5_STA.TX_BUF[2] =	g;                 //���Ͱ�λ
//	    UART5_STA.TX_BUF[3] =	g>>8;              //���߰�λ
//			UART5_STA.TX_BUF[4] =	rc_srcdata[11];	   // ���߷��� 0-360��      ң��������ֵ������ʵ����ֵ
//			UART5_STA.TX_BUF[5] =	rc_srcdata[12];
//			UART5_STA.TX_BUF[6] =	rc_srcdata[13];		 // �����ٶ� 0-600mm/s   ң��������ֵ������ʵ����ֵ
//			UART5_STA.TX_BUF[7] =	rc_srcdata[14];
//			UART5_STA.TX_BUF[8] =	rc_srcdata[15];    // ��ת�ٶ� 0-20��/s    ң��������ֵ������ʵ����ֵ
//			UART5_STA.TX_BUF[9] = rc_srcdata[7];	   //�ֶ��Զ�
//	    	    
//	    UART5_STA.TX_BUF[10] = 0;
//	    UART5_STA.TX_BUF[11] = 0;	
//		  UART5_STA.TX_BUF[12] = 0;	
//	    UART5_STA.TX_BUF[13] = 0;	
//		  UART5_STA.TX_BUF[14] = 0;	
//	    UART5_STA.TX_BUF[15] = 0;	

//	
////		  UART5_STA.TX_BUF[2] =	rc_srcdata[9];    // ���  ���ǰ����04   ���ֹͣ��02    �����ˣ�01  
////	    UART5_STA.TX_BUF[3] =	rc_srcdata[10];
////			UART5_STA.TX_BUF[4] =	rc_srcdata[11];	  // ���߷��� 0-360��      ң��������ֵ������ʵ����ֵ
////			UART5_STA.TX_BUF[5] =	rc_srcdata[12];
////			UART5_STA.TX_BUF[6] =	rc_srcdata[13];		// �����ٶ� 0-600mm/s   ң��������ֵ������ʵ����ֵ
////			UART5_STA.TX_BUF[7] =	rc_srcdata[14];
////			UART5_STA.TX_BUF[8] =	rc_srcdata[15];    // ��ת�ٶ� 0-20��/s    ң��������ֵ������ʵ����ֵ
//	

////		rc_ctrl.MovingDirection = (srcdata[10] + (srcdata[11] << 8));     // ���߷��� 0-360��
////		rc_ctrl.MovingSpeed = (srcdata[12] + (srcdata[13] << 8));     // �����ٶ� 0-600mm/s
////		rc_ctrl.RotationSpeed = (srcdata[14] + (srcdata[15] << 8));     // ��ת�ٶ� 0-20��/s
////	  RADv = rc_ctrl.RotationSpeed*0.1*3.1415926/180.0;	
//	

////new end  	 
///********************************����ֵ*************************************/
//			crcfifo = CRC16(&UART5_STA.TX_BUF[0],16);                  //����5  0��ʼ��9����CRCУ����
//			UART5_STA.TX_BUF[16] = crcfifo;
//			UART5_STA.TX_BUF[17] = crcfifo>>8;	
//      UART5_STA.TX_LEN = 18;		
////			checksumFIFO = getchecksum(USART1_STA.TX_BUF, 9);				//�ۼӺ�У����				 							
////			USART1_STA.TX_BUF[9] =	checksumFIFO>>8;
////			USART1_STA.TX_BUF[10] =	checksumFIFO;	
////			USART1_STA.TX_LEN = 11;		

//			if(UART5_STA.TX_LEN != 0)
//			{
//				UART5_STA.TX_FLAG =0;
//				UsartSendData_DMA(5,UART5_STA.TX_BUF,UART5_STA.TX_LEN);
//				UART5_STA.TX_LEN=0;
//	//			memset(USART2_STA.RX_BUF,0,sizeof(USART2_STA.RX_BUF));
//			}		
}


void Usart5Receive_IDLE(void)//ui��Ϊң����
{
//	uint32_t temp;
//	uint16_t checksumFIFO;
//	float RADv;
//	
//	uint16_t crc1;
//	
//	
//	if(__HAL_UART_GET_FLAG(&huart5,UART_FLAG_IDLE)!= RESET)
//	{		
//		__HAL_UART_CLEAR_IDLEFLAG(&huart5);
//		HAL_UART_DMAStop(&huart5);
//		temp = huart5.hdmarx->Instance->NDTR;
//		UART5_STA.RX_LEN = RX_URB_Size - temp;
//		UART5_STA.RX_FLAG = 1;	
//		HAL_UART_Receive_DMA(&huart5,UART5_STA.RX_BUF,RX_URB_Size);	
//		  
//		if(UART5_STA.RX_BUF[0] == 0xFF)
//		{
//			if(UART5_STA.RX_BUF[1] == 0xAF)
//			{
//				if(usMBCRC16(UART5_STA.RX_BUF, 16)
//						==(UART5_STA.RX_BUF[16]+(UART5_STA.RX_BUF[17]<<8)))
//				{
//					memcpy(rc_srcdata,UART5_STA.RX_BUF,18);
//					rc_updateCount=600;		// ң��������ֹͣ
//					Usart5Send_RC();
//				}
//			}
//		}
//	}
}








const uint32_t TEXT_Buffer[]={0x10};
void Debug_Communication(USART_STA *usart_struct)
{
		
}

