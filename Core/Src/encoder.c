#include "main.h"
#include "encoder.h"
#include "FIFO.h"
#include "usart.h"
#include "stdio.h"
#include "motor.h"
#include "remote_control.h"

#define EncoderB_Zero_Offset 30    //���ֱ�����ֵ1���� ��ƫ��С����  ��ƫ��������    20220921
#define EncoderF_Zero_Offset -210  //ǰ��ֱ�����ֵ3���� ��ƫ��С����  ��ƫ��������     20220921
#define EncoderAxle_Zero_Offset -4109 //�ֱ�����ֵ2���� ��ֵ�����ֵ��ƫ           20221005
uint32_t encoder_t_totalvalue02 = 0; //��������ֵ   20221005
uint32_t encoder_t_totalvalue01 = 0;
uint32_t encoder_t_totalvalue03 = 0;       //��������ֵ   20220921

Encoder Encoder_Front;   //ǰ�ֱ�����
Encoder Encoder_Back;    //���ֱ�����
Encoder Encoder_Axle;    //��������

//��ȡ��ֵ
//void GetMidVal(Encoder *pEc)
//{
//	int32_t a = pEc->vals[0];
//	int32_t b = pEc->vals[1];
//	int32_t c = pEc->val;
//	int32_t tmp;
//	if (a < b)//���a��bС������a��b��ֵ��ʹa��Ŵ������
//	{
//		tmp = a;
//		a = b;
//		b = tmp;
//	}
//
//	if (a < c)//���a��cС������a��c��ֵ��ʹa��Ŵ������
//	{
//		tmp = a;
//		a = c;
//		c = tmp;
//
//	}
//
//	if (b < c)//���b��cС������b��c��ֵ��ʹb��Ŵ������
//	{
//		tmp = b;
//		b = c;
//		c = tmp;
//
//	}
//	pEc->midVal = b;
//	if(pEc->midVal==0)pEc->midVal= pEc->val;
//	pEc->vals[1] = pEc->vals[0];
//	pEc->vals[0] = pEc->val;
//}


void EncoderSendWithCRC(void)
{
	uint16_t crcfifo;
	crcfifo = CRC16(&USART3_STA.TX_BUF[0], 6);                  //����3  0��ʼ��6����CRCУ����
	USART3_STA.TX_BUF[6] = crcfifo;
	USART3_STA.TX_BUF[7] = crcfifo >> 8;
	UsartSendData_DMA(3, USART3_STA.TX_BUF, 8);

	//	osDelay(9);  
	//	DIR3=0;
	////	Usart3Receive_encoder_IDLE();
	osDelay(10);
}
uint32_t M;
void Encoder_Send(void)
{
	//DIR3=1;   
	if (M == 4)
	{

		USART3_STA.TX_BUF[0] = 0x02;
		USART3_STA.TX_BUF[1] = 0x03;
		USART3_STA.TX_BUF[2] = 0x00;
		USART3_STA.TX_BUF[3] = 0x09;
		USART3_STA.TX_BUF[4] = 0x00;
		USART3_STA.TX_BUF[5] = 0x04;
		EncoderSendWithCRC();		/* �������ݣ��Զ���CRC */
		M = 0;

	}
	else
	{
		USART3_STA.TX_BUF[0] = 0x03;
		USART3_STA.TX_BUF[1] = 0x03;
		USART3_STA.TX_BUF[2] = 0x00;
		USART3_STA.TX_BUF[3] = 0x09;
		USART3_STA.TX_BUF[4] = 0x00;
		USART3_STA.TX_BUF[5] = 0x04;
		EncoderSendWithCRC();		/* �������ݣ��Զ���CRC */
		M++;

		USART3_STA.TX_BUF[0] = 0x01;
		USART3_STA.TX_BUF[1] = 0x03;
		USART3_STA.TX_BUF[2] = 0x00;
		USART3_STA.TX_BUF[3] = 0x09;
		USART3_STA.TX_BUF[4] = 0x00;
		USART3_STA.TX_BUF[5] = 0x04;
		EncoderSendWithCRC();		/* �������ݣ��Զ���CRC */

	}
}

void Usart3Receive_encoder_IDLE(void)
//void EC_Receive_handler(UART_HandleTypeDef *huart)
{
	uint32_t temp;
	uint16_t crc1;

	if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE) != RESET)
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart3);
		HAL_UART_DMAStop(&huart3);
		temp = huart3.hdmarx->Instance->NDTR;
		USART3_STA.RX_LEN = RX_URB_Size - temp;
		USART3_STA.RX_FLAG = 1;
		HAL_UART_Receive_DMA(&huart3, USART3_STA.RX_BUF, RX_URB_Size);
		// printf(	"USART3_STA.RX_LEN= %d\r\n",USART3_STA.RX_LEN);	

		crc1 = CRC16(USART3_STA.RX_BUF, USART3_STA.RX_LEN);
		if (crc1 == 0)//У��ɹ�
		{

			if (USART3_STA.RX_BUF[0] == 0x01)
			{

				encoder_t_totalvalue01 = USART3_STA.RX_BUF[6] + (USART3_STA.RX_BUF[5] << 8) + (USART3_STA.RX_BUF[4] << 16) + (USART3_STA.RX_BUF[3] << 24);
				if (encoder_t_totalvalue01 > EncoderReadMin&&encoder_t_totalvalue01 < EncoderReadMax)
				{
					Encoder_Back.update++;
					Encoder_Back.val = encoder_t_totalvalue01 + EncoderB_Zero_Offset;  //������ֵ���� 20220921
					Encoder_Back.dir = USART3_STA.RX_BUF[8];
					Encoder_Back.rpm = USART3_STA.RX_BUF[10] + (USART3_STA.RX_BUF[9] << 8);
					//GetMidVal(&Encoder_Back);
				}


				//					if((Encoder_Back.val<agvStatus.ecKeepAngle)&&(Rotation_FLAG1==2))//��ʱ����ת 20220913
				//					{
				//            Stop_Back_Wheel();//����Ԫֹͣ
				//						Rotation_FLAG1=0;
				//					}	
				//          if((Encoder_Back.val>agvStatus.ecKeepAngle)&&(Rotation_FLAG1==1))//˳ʱ����ת 20220913
				//					{
				//            Stop_Back_Wheel();//����Ԫֹͣ
				//						Rotation_FLAG1=0;
				//					}					


									//sprintf(&temp_str[0],"ϵʱ:%d,������1:%d\r\n", HAL_GetTick(), Encoder_Back.val);
							  //UsartPrintf(temp_str);
			}
			else if (USART3_STA.RX_BUF[0] == 0x02)
			{

				encoder_t_totalvalue02 = USART3_STA.RX_BUF[6] + (USART3_STA.RX_BUF[5] << 8) + (USART3_STA.RX_BUF[4] << 16) + (USART3_STA.RX_BUF[3] << 24);
				encoder_t_totalvalue02 += EncoderZero;
				if (encoder_t_totalvalue02 > 0x400000)
					encoder_t_totalvalue02 -= 0x400000;

				Encoder_Axle.update++;
				Encoder_Axle.val = encoder_t_totalvalue02 + EncoderAxle_Zero_Offset;  //������ֵ���� 20221005
				Encoder_Axle.dir= USART3_STA.RX_BUF[8];
				Encoder_Axle.rpm = USART3_STA.RX_BUF[10] + (USART3_STA.RX_BUF[9] << 8);
				//sprintf(&temp_str[0],"ϵʱ:%d,������2:%d\r\n", HAL_GetTick(), Encoder_Axle.val);
	  //UsartPrintf(temp_str);
			}
			else if (USART3_STA.RX_BUF[0] == 0x03)
			{

				encoder_t_totalvalue03 = USART3_STA.RX_BUF[6] + (USART3_STA.RX_BUF[5] << 8) + (USART3_STA.RX_BUF[4] << 16) + (USART3_STA.RX_BUF[3] << 24);

				if (encoder_t_totalvalue03 > EncoderReadMin&&encoder_t_totalvalue03 < EncoderReadMax)
				{
					Encoder_Front.val = encoder_t_totalvalue03 + EncoderF_Zero_Offset;  //������ֵ���� 20220921
					Encoder_Front.update++;
					Encoder_Front.dir = USART3_STA.RX_BUF[8];
					Encoder_Front.rpm = USART3_STA.RX_BUF[10] + (USART3_STA.RX_BUF[9] << 8);
					//GetMidVal(&Encoder_Front);
				}




				//					if((Encoder_Front.val<agvStatus.ecKeepAngle)&&(Rotation_FLAG3==2))//��ʱ����ת 20220913
				//					{
				//            Stop_Front_Wheel();//ǰ���Ԫֹͣ
				//						Rotation_FLAG3=0;
				//					}	
				//          if((Encoder_Front.val>agvStatus.ecKeepAngle)&&(Rotation_FLAG3==1))//˳ʱ����ת 20220913
				//					{
				//            Stop_Front_Wheel();//ǰ���Ԫֹͣ
				//						Rotation_FLAG3=0;
				//					}

										//sprintf(&temp_str[0],"ϵʱ:%d,������3:%d\r\n", HAL_GetTick(), Encoder_Axle.val);
							  //UsartPrintf(temp_str);
			}
		}
	}
}

void InitEncoder(Encoder * ec, int16_t zeroDelt)
{
	ec->dir = 0;
	ec->rpm = 0;
	ec->val = 0;
	ec->update = 0;
	ec->zeroDelt = zeroDelt;	
}
