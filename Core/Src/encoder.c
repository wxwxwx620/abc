#include "main.h"
#include "encoder.h"
#include "FIFO.h"
#include "usart.h"
#include "stdio.h"
#include "motor.h"
#include "remote_control.h"

#define EncoderB_Zero_Offset 30    //后差动轮编码器值1修正 左偏减小修正  右偏增大修正    20220921
#define EncoderF_Zero_Offset -210  //前差动轮编码器值3修正 左偏减小修正  右偏增大修正     20220921
#define EncoderAxle_Zero_Offset -4109 //轮编码器值2修正 负值左边正值右偏           20221005
uint32_t encoder_t_totalvalue02 = 0; //编码器总值   20221005
uint32_t encoder_t_totalvalue01 = 0;
uint32_t encoder_t_totalvalue03 = 0;       //编码器总值   20220921

Encoder Encoder_Front;   //前轮编码器
Encoder Encoder_Back;    //后轮编码器
Encoder Encoder_Axle;    //轴距编码器

//获取中值
//void GetMidVal(Encoder *pEc)
//{
//	int32_t a = pEc->vals[0];
//	int32_t b = pEc->vals[1];
//	int32_t c = pEc->val;
//	int32_t tmp;
//	if (a < b)//如果a比b小，交换a和b的值，使a存放大的数字
//	{
//		tmp = a;
//		a = b;
//		b = tmp;
//	}
//
//	if (a < c)//如果a比c小，交换a和c的值，使a存放大的数字
//	{
//		tmp = a;
//		a = c;
//		c = tmp;
//
//	}
//
//	if (b < c)//如果b比c小，交换b和c的值，使b存放大的数字
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
	crcfifo = CRC16(&USART3_STA.TX_BUF[0], 6);                  //串口3  0开始，6个做CRC校验码
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
		EncoderSendWithCRC();		/* 发送数据，自动加CRC */
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
		EncoderSendWithCRC();		/* 发送数据，自动加CRC */
		M++;

		USART3_STA.TX_BUF[0] = 0x01;
		USART3_STA.TX_BUF[1] = 0x03;
		USART3_STA.TX_BUF[2] = 0x00;
		USART3_STA.TX_BUF[3] = 0x09;
		USART3_STA.TX_BUF[4] = 0x00;
		USART3_STA.TX_BUF[5] = 0x04;
		EncoderSendWithCRC();		/* 发送数据，自动加CRC */

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
		if (crc1 == 0)//校验成功
		{

			if (USART3_STA.RX_BUF[0] == 0x01)
			{

				encoder_t_totalvalue01 = USART3_STA.RX_BUF[6] + (USART3_STA.RX_BUF[5] << 8) + (USART3_STA.RX_BUF[4] << 16) + (USART3_STA.RX_BUF[3] << 24);
				if (encoder_t_totalvalue01 > EncoderReadMin&&encoder_t_totalvalue01 < EncoderReadMax)
				{
					Encoder_Back.update++;
					Encoder_Back.val = encoder_t_totalvalue01 + EncoderB_Zero_Offset;  //编码器值修正 20220921
					Encoder_Back.dir = USART3_STA.RX_BUF[8];
					Encoder_Back.rpm = USART3_STA.RX_BUF[10] + (USART3_STA.RX_BUF[9] << 8);
					//GetMidVal(&Encoder_Back);
				}


				//					if((Encoder_Back.val<agvStatus.ecKeepAngle)&&(Rotation_FLAG1==2))//逆时针旋转 20220913
				//					{
				//            Stop_Back_Wheel();//后差动单元停止
				//						Rotation_FLAG1=0;
				//					}	
				//          if((Encoder_Back.val>agvStatus.ecKeepAngle)&&(Rotation_FLAG1==1))//顺时针旋转 20220913
				//					{
				//            Stop_Back_Wheel();//后差动单元停止
				//						Rotation_FLAG1=0;
				//					}					


									//sprintf(&temp_str[0],"系时:%d,编码器1:%d\r\n", HAL_GetTick(), Encoder_Back.val);
							  //UsartPrintf(temp_str);
			}
			else if (USART3_STA.RX_BUF[0] == 0x02)
			{

				encoder_t_totalvalue02 = USART3_STA.RX_BUF[6] + (USART3_STA.RX_BUF[5] << 8) + (USART3_STA.RX_BUF[4] << 16) + (USART3_STA.RX_BUF[3] << 24);
				encoder_t_totalvalue02 += EncoderZero;
				if (encoder_t_totalvalue02 > 0x400000)
					encoder_t_totalvalue02 -= 0x400000;

				Encoder_Axle.update++;
				Encoder_Axle.val = encoder_t_totalvalue02 + EncoderAxle_Zero_Offset;  //编码器值修正 20221005
				Encoder_Axle.dir= USART3_STA.RX_BUF[8];
				Encoder_Axle.rpm = USART3_STA.RX_BUF[10] + (USART3_STA.RX_BUF[9] << 8);
				//sprintf(&temp_str[0],"系时:%d,编码器2:%d\r\n", HAL_GetTick(), Encoder_Axle.val);
	  //UsartPrintf(temp_str);
			}
			else if (USART3_STA.RX_BUF[0] == 0x03)
			{

				encoder_t_totalvalue03 = USART3_STA.RX_BUF[6] + (USART3_STA.RX_BUF[5] << 8) + (USART3_STA.RX_BUF[4] << 16) + (USART3_STA.RX_BUF[3] << 24);

				if (encoder_t_totalvalue03 > EncoderReadMin&&encoder_t_totalvalue03 < EncoderReadMax)
				{
					Encoder_Front.val = encoder_t_totalvalue03 + EncoderF_Zero_Offset;  //编码器值修正 20220921
					Encoder_Front.update++;
					Encoder_Front.dir = USART3_STA.RX_BUF[8];
					Encoder_Front.rpm = USART3_STA.RX_BUF[10] + (USART3_STA.RX_BUF[9] << 8);
					//GetMidVal(&Encoder_Front);
				}




				//					if((Encoder_Front.val<agvStatus.ecKeepAngle)&&(Rotation_FLAG3==2))//逆时针旋转 20220913
				//					{
				//            Stop_Front_Wheel();//前差动单元停止
				//						Rotation_FLAG3=0;
				//					}	
				//          if((Encoder_Front.val>agvStatus.ecKeepAngle)&&(Rotation_FLAG3==1))//顺时针旋转 20220913
				//					{
				//            Stop_Front_Wheel();//前差动单元停止
				//						Rotation_FLAG3=0;
				//					}

										//sprintf(&temp_str[0],"系时:%d,编码器3:%d\r\n", HAL_GetTick(), Encoder_Axle.val);
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
