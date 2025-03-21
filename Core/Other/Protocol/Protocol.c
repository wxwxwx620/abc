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
	uint8_t     head;               //包头
	uint8_t     cmd;                //当前指令
	uint8_t     clipState:2;        //包夹状态      0 降到位  1升到位  2运行中 3 中间位
	uint8_t     ajustDisState:2;    //轴距调整状态  0未调整  1调整完成 2 调整中  3调整出错
	uint8_t     chargeState:1;      //充电状态      0 未充电 1充电中
	uint8_t     carState:1;         //车辆状态      0 无车辆 1有车辆
	uint8_t     onLineState:1;      //上线状态      0  下线  1上线
	uint8_t     ctrlMode:1;         //控制模式      0  手动  1自动
	uint8_t     stop:1;             //急停状态      0未急停   1急停
	uint8_t     dirReady:1;         //轮子方向调整就绪  0未就绪  1就绪
	uint8_t     remark : 6;         //保留
	uint8_t     errorCode;          //错误码
	uint16_t    errorInfo;          //错误附加信息
	int16_t     odoX;               //里程计X mm
	int16_t     odoY;               //里程计Y mm
	int16_t     odoZ;               //里程计角度 0.1
	uint8_t     power;              //电量
	uint16_t    wheelBaseLength;    //轴距值 mm

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

////DMA中断循环缓冲提取遥控器数据
//void RC_Receive_handler(UART_HandleTypeDef *huart)
//{
//	uint8_t data[36]={0};
//	float RADv;
//	char t_buf[128];
//  //清串口状态寄存器
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
//					rc_updateCount=600;		// 遥控器断了停止
//					Usart1Send_RC();
//				}
//			}
//		}	
//	}
//}



//DMA中断循环缓冲提取遥控器数据
void RC_Receive_handler(UART_HandleTypeDef *huart)
{
	uint8_t data[36]={0};
	//float RADv;
	//char t_buf[128];
  //清串口状态寄存器
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
					rc_updateCount++;		// 遥控器断了停止
//					Usart1Send_RC();
				}
			}
			move_ringbuff_head(&RC_buff,1);
		}
	}
}


////DMA中断循环缓冲提取上位机数据
//void PC_Receive_handler(UART_HandleTypeDef *huart)
//{
//	uint8_t data[LEN_PC_2_AGV*2]={0};
//	float RADv;
//	char t_buf[128];
//  //清串口状态寄存器
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
//					pc_updateCount = 10;//6;//1s记时	
//					Usart2Send_PC();
//					//to pc
//				}
//			}
//		}
//	}
//}



//DMA中断循环缓冲提取上位机数据
void PC_Receive_handler(UART_HandleTypeDef *huart)
{
	uint8_t data[LEN_PC_2_AGV*2]={0};
//	float RADv;
//	char t_buf[128];
  //清串口状态寄存器
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
					pc_updateCount ++;//6;//1s记时	
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
			USART2_STA.TX_BUF[1] =	0x70;//USART2_STA.RX_BUF[1];//命令
			USART2_STA.TX_BUF[4]=0;//避障
			USART2_STA.TX_BUF[6]=0;
			USART2_STA.TX_BUF[7]=0;//预留
			if(agvStatus.onLineState==1) //当前为下线状态
			{
				USART2_STA.TX_BUF[3] |=0x10;//下线
				USART2_STA.TX_BUF[6] |=0x02;//
			}//
			else if(agvStatus.onLineState==0)
			{		
				USART2_STA.TX_BUF[3] &=0xef;//上线
				USART2_STA.TX_BUF[6] &=0x01;//
			}
			
			if((BMS1.Cha_Relay_Sta&0x01)==0x01) USART2_STA.TX_BUF[3] |=0x04;//充电连接
			else USART2_STA.TX_BUF[3] &=0xfb;
			
			if(BMS1.current>=0)	USART2_STA.TX_BUF[3] &=0x7f;//放电
			else								USART2_STA.TX_BUF[3] |=0x80;//充电

			USART2_STA.TX_BUF[3]&=0xfc;  //举升状态位先清零，再根据当前状态设置
			if(agvStatus.clipState==ClipState_Closed)  USART2_STA.TX_BUF[3] |=0x01;//关到位 //20220911
			else if(agvStatus.clipState == ClipState_Running)	USART2_STA.TX_BUF[3] |=0x02;//运行中 //20220911
			else if(agvStatus.clipState == ClipState_Stoped)	USART2_STA.TX_BUF[3] |=0x03;//停止	 //20220911				
			
			if((UPB_LED==0)&&(UPF_LED==0)) USART2_STA.TX_BUF[3]|=0x08;//有托盘
			else USART2_STA.TX_BUF[3]&=0xf7;//无托盘
			
			if (agvStatus.ajustDisState == AjustState_OK)
			{
				USART2_STA.TX_BUF[3] |= 0x80;//轴距调整完成
				ctrl_mode.state.WheelbaseAdjust_mode_pc = 0;//轴距调整完成
			}
			else
			{
				USART2_STA.TX_BUF[3] &= 0x7F;//轴距调整中
			}
			//if(ctrl_mode.state.WheelbaseAdjust_mode_pc)
			//{					
			//	USART2_STA.TX_BUF[3]&=0x7F;//轴距调整中
			//}
			//if(!WheelbaseAdjust_mode_pc_Flag)
			//{
			//	USART2_STA.TX_BUF[3]|=0x80;//轴距调整完成
			//	ctrl_mode.state.WheelbaseAdjust_mode_pc=0;//轴距调整完成
			//}
			//if((Wheelbase_Data>=WheelbaseUpperLimit)||(Wheelbase_Data<=WheelbaseLowerLimit))
			//{
			//	USART2_STA.TX_BUF[3]|=0x80;//轴距调整完成
			//	ctrl_mode.state.WheelbaseAdjust_mode_pc=0;//轴距调整完成
			//}
			
						
			if((ctrl_mode.state.scram_alarm_filter&0x01)==0x01) USART2_STA.TX_BUF[2] |=0x01;//车体急停
			else	USART2_STA.TX_BUF[2] &=0xfe;	
				
//			if((Motor_alarm>=1)||(Lif_Motor_alarm>=1)) USART2_STA.TX_BUF[2] |=00x02;//电机报警
//			else USART2_STA.TX_BUF[2]&=0xfd;//无报警	
      USART2_STA.TX_BUF[2]&=0xfd;//无报警	
			
//			if(1==BMS_time_out_Flag)	USART2_STA.TX_BUF[2]|=0x10;//电池通信异常
//			else 											USART2_STA.TX_BUF[2]&=0xef;//电池通信正常
			USART2_STA.TX_BUF[2]&=0xef;//电池通信正常			
			
//			USART2_STA.TX_BUF[5]=BMS1.capacity_per;	//电量
//			if(BMS1.capacity_per<=20)	USART2_STA.TX_BUF[2] |=0x04;//电量低
//			else USART2_STA.TX_BUF[2]&=0xfb;				
			USART2_STA.TX_BUF[5]=80;	//电量
			USART2_STA.TX_BUF[2]&=0xfb;						
					
			if(agvStatus.ctrlMode !=PC_Auto)		//遥控方式
			{	
				USART2_STA.TX_BUF[2] |=0x08;//无控制权
//						scram_alarm_filter &=0xfb;
			}
			else
			{						

					USART2_STA.Voice_RX=USART2_STA.RX_BUF[8];	//语音end
						
					USART2_STA.TX_BUF[2] &=0xf7;
					//if((USART2_STA.RX_BUF[9]&0x08) == 0x08) //报警	
					//{
					//	ctrl_mode.state.scram_alarm_filter |=0x04;//停止，上位机报警停止
					//}
					//else
					{
							ctrl_mode.state.scram_alarm_filter &=0xfb;
							//if((USART2_STA.RX_BUF[10]&0x80)==0x80)		BRAKE_EN=0;//电机使能分
							//else																			BRAKE_EN=1;//电机使能合
							//if((USART2_STA.RX_BUF[9]&0x04)==0x04)//充电继电器
							//{
							//	BMS_Relay=1;
							//}	
							//else if((USART2_STA.RX_BUF[9]&0x04)==0x00)
							//{
							//	BMS_Relay=0;
							//}	
							//if((USART2_STA.RX_BUF[9]&0x80)==0x80)   //休眠控制
							//{

							//}           
							//else 
							//{
							//		ctrl_mode.state.scram_alarm_filter &=0xeF;	//急停标志位
							//}
						}
					}
							
							USART2_STA.TX_BUF[4]=0x00;//0xff&(LN1_IP1|LN1_IP2<<1|Laser_Edge_F<<2|LN2_IP1<<3|LN2_IP2<<4|Laser_Edge_F<<5|UPF_LED<<6|UPB_LED<<7);//激光反馈及探物反馈	 
		//new start  行走电机报警
							USART2_STA.TX_BUF[7]=0x00;//0xff&(Motor_Alarm_FL|Motor_Alarm_FR<<1|Motor_Alarm_BR<<2|Motor_Alarm_BL<<3|Motor_Alarm_Clip_FL<<4|Motor_Alarm_Clip_FR<<5|Motor_Alarm_Clip_BR<<6|Motor_Alarm_Clip_BL<<7);
		//  				 举升电机报警
							USART2_STA.TX_BUF[8]=0x00;//0xff&(Motor_Alarm9|Motor_Alarm10<<1|Motor_Alarm11<<2|Motor_Alarm12<<3);
		//  				 电池电流
							USART2_STA.TX_BUF[9]=30;//BMS1.current/10;	
		//  				 电池电压	
							USART2_STA.TX_BUF[10]=80;//BMS1.voltage/10;
		//  				 充放电循环次数
							USART2_STA.TX_BUF[11]=50;//BMS1.cycle_count;		
							USART2_STA.TX_BUF[12]=50;//BMS1.cycle_count>>8;	
		//  				 电池故障
							USART2_STA.TX_BUF[13]=0x00;//BMS1.Alarm_Type;
		//					 激光策略
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
							
							
							USART2_STA.TX_BUF[21]=((int16_t)Wheelbase_Data)>>8;//实时轴距高位
							USART2_STA.TX_BUF[22]=(int16_t)Wheelbase_Data;//实时轴距低位


							USART2_STA.TX_BUF[23] = ((int16_t)CAR_Odometer.ODOMETER_X)>>8;
							USART2_STA.TX_BUF[24] = (int16_t)CAR_Odometer.ODOMETER_X;
							USART2_STA.TX_BUF[25] = ((int16_t)CAR_Odometer.ODOMETER_Y)>>8;	
							USART2_STA.TX_BUF[26] = (int16_t)CAR_Odometer.ODOMETER_Y;	
							USART2_STA.TX_BUF[27] = ((int16_t)CAR_Odometer.PC_ODOMETER_Z)>>8;	
							USART2_STA.TX_BUF[28] = (int16_t)CAR_Odometer.PC_ODOMETER_Z;						
	//new end  	 
	/********************************返回值*************************************/

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

}//主函数结束

int testlen=0;
uint8_t buf[18]={0};
//发送给上位机数据
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
			
	    USART1_STA.TX_BUF[2] =	g;                 //轴距低八位
	    USART1_STA.TX_BUF[3] =	g>>8;              //轴距高八位
			USART1_STA.TX_BUF[4] =	rc_srcdata[11];	   // 行走方向 0-360°      遥控器的数值，不是实际数值
			USART1_STA.TX_BUF[5] =	rc_srcdata[12];
			USART1_STA.TX_BUF[6] =	rc_srcdata[13];		 // 行走速度 0-600mm/s   遥控器的数值，不是实际数值
			USART1_STA.TX_BUF[7] =	rc_srcdata[14];
			USART1_STA.TX_BUF[8] =	rc_srcdata[15];    // 旋转速度 0-20°/s    遥控器的数值，不是实际数值
			USART1_STA.TX_BUF[9] =  rc_srcdata[7];	   //手动自动
	    	    
	    USART1_STA.TX_BUF[10] = 0;
	    USART1_STA.TX_BUF[11] = 0;	
		  USART1_STA.TX_BUF[12] = 0;	
	    USART1_STA.TX_BUF[13] = 0;	
		  USART1_STA.TX_BUF[14] = 0;	
	    USART1_STA.TX_BUF[15] = 0;	

	
//		  USART1_STA.TX_BUF[2] =	rc_srcdata[9];    // 轴距  轴距前进：04   轴距停止：02    轴距后退：01  
//	    USART1_STA.TX_BUF[3] =	rc_srcdata[10];
//			USART1_STA.TX_BUF[4] =	rc_srcdata[11];	  // 行走方向 0-360°      遥控器的数值，不是实际数值
//			USART1_STA.TX_BUF[5] =	rc_srcdata[12];
//			USART1_STA.TX_BUF[6] =	rc_srcdata[13];		// 行走速度 0-600mm/s   遥控器的数值，不是实际数值
//			USART1_STA.TX_BUF[7] =	rc_srcdata[14];
//			USART1_STA.TX_BUF[8] =	rc_srcdata[15];    // 旋转速度 0-20°/s    遥控器的数值，不是实际数值
	

//		rc_ctrl.MovingDirection = (srcdata[10] + (srcdata[11] << 8));     // 行走方向 0-360°
//		rc_ctrl.MovingSpeed = (srcdata[12] + (srcdata[13] << 8));     // 行走速度 0-600mm/s
//		rc_ctrl.RotationSpeed = (srcdata[14] + (srcdata[15] << 8));     // 旋转速度 0-20°/s
//	  RADv = rc_ctrl.RotationSpeed*0.1*3.1415926/180.0;	
	

//new end  	 
/********************************返回值*************************************/
			crcfifo = CRC16(&USART1_STA.TX_BUF[0],16);                  //串口5  0开始，9个做CRC校验码
			USART1_STA.TX_BUF[16] = crcfifo;
			USART1_STA.TX_BUF[17] = crcfifo>>8;	
      USART1_STA.TX_LEN = 18;		
//			checksumFIFO = getchecksum(USART1_STA.TX_BUF, 9);				//累加和校验码				 							
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











/***********串口5  ui改为遥控器*************/
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
//	    UART5_STA.TX_BUF[2] =	g;                 //轴距低八位
//	    UART5_STA.TX_BUF[3] =	g>>8;              //轴距高八位
//			UART5_STA.TX_BUF[4] =	rc_srcdata[11];	   // 行走方向 0-360°      遥控器的数值，不是实际数值
//			UART5_STA.TX_BUF[5] =	rc_srcdata[12];
//			UART5_STA.TX_BUF[6] =	rc_srcdata[13];		 // 行走速度 0-600mm/s   遥控器的数值，不是实际数值
//			UART5_STA.TX_BUF[7] =	rc_srcdata[14];
//			UART5_STA.TX_BUF[8] =	rc_srcdata[15];    // 旋转速度 0-20°/s    遥控器的数值，不是实际数值
//			UART5_STA.TX_BUF[9] = rc_srcdata[7];	   //手动自动
//	    	    
//	    UART5_STA.TX_BUF[10] = 0;
//	    UART5_STA.TX_BUF[11] = 0;	
//		  UART5_STA.TX_BUF[12] = 0;	
//	    UART5_STA.TX_BUF[13] = 0;	
//		  UART5_STA.TX_BUF[14] = 0;	
//	    UART5_STA.TX_BUF[15] = 0;	

//	
////		  UART5_STA.TX_BUF[2] =	rc_srcdata[9];    // 轴距  轴距前进：04   轴距停止：02    轴距后退：01  
////	    UART5_STA.TX_BUF[3] =	rc_srcdata[10];
////			UART5_STA.TX_BUF[4] =	rc_srcdata[11];	  // 行走方向 0-360°      遥控器的数值，不是实际数值
////			UART5_STA.TX_BUF[5] =	rc_srcdata[12];
////			UART5_STA.TX_BUF[6] =	rc_srcdata[13];		// 行走速度 0-600mm/s   遥控器的数值，不是实际数值
////			UART5_STA.TX_BUF[7] =	rc_srcdata[14];
////			UART5_STA.TX_BUF[8] =	rc_srcdata[15];    // 旋转速度 0-20°/s    遥控器的数值，不是实际数值
//	

////		rc_ctrl.MovingDirection = (srcdata[10] + (srcdata[11] << 8));     // 行走方向 0-360°
////		rc_ctrl.MovingSpeed = (srcdata[12] + (srcdata[13] << 8));     // 行走速度 0-600mm/s
////		rc_ctrl.RotationSpeed = (srcdata[14] + (srcdata[15] << 8));     // 旋转速度 0-20°/s
////	  RADv = rc_ctrl.RotationSpeed*0.1*3.1415926/180.0;	
//	

////new end  	 
///********************************返回值*************************************/
//			crcfifo = CRC16(&UART5_STA.TX_BUF[0],16);                  //串口5  0开始，9个做CRC校验码
//			UART5_STA.TX_BUF[16] = crcfifo;
//			UART5_STA.TX_BUF[17] = crcfifo>>8;	
//      UART5_STA.TX_LEN = 18;		
////			checksumFIFO = getchecksum(USART1_STA.TX_BUF, 9);				//累加和校验码				 							
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


void Usart5Receive_IDLE(void)//ui改为遥控器
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
//					rc_updateCount=600;		// 遥控器断了停止
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

