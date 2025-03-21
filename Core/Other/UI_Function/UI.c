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
//			UsartSendData_DMA(5,usart_struct->TX_BUF,usart_struct->TX_LEN);               //串口5
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
//		usart_struct->TX_BUF[usart_struct->TxCount++]=0x5f;          //右边95个数据			
//		usart_struct->TX_BUF[usart_struct->TxCount++]=0x82;	
//		usart_struct->TX_BUF[usart_struct->TxCount++]=0x30;							
//		usart_struct->TX_BUF[usart_struct->TxCount++]=0x00;
//	
//		usart_struct->TX_BUF[usart_struct->TxCount++]=BMS1.current>>8;								usart_struct->TX_BUF[usart_struct->TxCount++]=BMS1.current;	//电流
//		usart_struct->TX_BUF[usart_struct->TxCount++]=BMS1.voltage>>8;								usart_struct->TX_BUF[usart_struct->TxCount++]=BMS1.voltage;	//电压
//		usart_struct->TX_BUF[usart_struct->TxCount++]=BMS1.capacity_per>>8;						usart_struct->TX_BUF[usart_struct->TxCount++]=BMS1.capacity_per;	//当前剩余容量
//		usart_struct->TX_BUF[usart_struct->TxCount++]=0x0073>>8;											usart_struct->TX_BUF[usart_struct->TxCount++]=0x0073;	//当前电池组容量
//		usart_struct->TX_BUF[usart_struct->TxCount++]=0x0073>>8;											usart_struct->TX_BUF[usart_struct->TxCount++]=0x0073;	//设计容量
//		usart_struct->TX_BUF[usart_struct->TxCount++]=BMS1.capacity_per>>8;						usart_struct->TX_BUF[usart_struct->TxCount++]=BMS1.capacity_per;	//容量百分比
//		usart_struct->TX_BUF[usart_struct->TxCount++]=BMS1.Max_unit_voltage>>8;				usart_struct->TX_BUF[usart_struct->TxCount++]=BMS1.Max_unit_voltage;	//最高单体电压
//		usart_struct->TX_BUF[usart_struct->TxCount++]=BMS1.Min_unit_voltage>>8; 			usart_struct->TX_BUF[usart_struct->TxCount++]=BMS1.Min_unit_voltage;	//最低单体电压
//		usart_struct->TX_BUF[usart_struct->TxCount++]=BMS1.Max_unit_Temperature>>8; 	usart_struct->TX_BUF[usart_struct->TxCount++]=BMS1.Max_unit_Temperature;	//最高温度点
//		usart_struct->TX_BUF[usart_struct->TxCount++]=BMS1.Min_unit_Temperature>>8; 	usart_struct->TX_BUF[usart_struct->TxCount++]=BMS1.Min_unit_Temperature;	//最低温度点
//		usart_struct->TX_BUF[usart_struct->TxCount++]=BMS1.cycle_count>>8; 						usart_struct->TX_BUF[usart_struct->TxCount++]=BMS1.cycle_count;	//循环次数
//		//状态信息
//		usart_struct->TX_BUF[usart_struct->TxCount++]=(BMS1.Alarm_Type&0x2000)>>13; 			usart_struct->TX_BUF[usart_struct->TxCount++]=(BMS1.Alarm_Type&0x2000);	//充放状态	1：充电。0：放电
//		usart_struct->TX_BUF[usart_struct->TxCount++]=(BMS1.Alarm_Type&0x0800)>>11; 			usart_struct->TX_BUF[usart_struct->TxCount++]=(BMS1.Alarm_Type&0x0800);	//低容量报警30%
//		usart_struct->TX_BUF[usart_struct->TxCount++]=(BMS1.Alarm_Type&0x0400)>>10; 			usart_struct->TX_BUF[usart_struct->TxCount++]=(BMS1.Alarm_Type&0x0400);	//电池不均匀 1
//		usart_struct->TX_BUF[usart_struct->TxCount++]=(BMS1.Alarm_Type&0x0200)>>9; 				usart_struct->TX_BUF[usart_struct->TxCount++]=(BMS1.Alarm_Type&0x0200);	//放电短路保护 1
//		usart_struct->TX_BUF[usart_struct->TxCount++]=(BMS1.Alarm_Type&0x0100)>>8; 				usart_struct->TX_BUF[usart_struct->TxCount++]=(BMS1.Alarm_Type&0x0100);	//充电短路保护 1
//		usart_struct->TX_BUF[usart_struct->TxCount++]=(BMS1.Alarm_Type&0x0080)>>7; 				usart_struct->TX_BUF[usart_struct->TxCount++]=(BMS1.Alarm_Type&0x0080);	//放电过电流保护 1
//		usart_struct->TX_BUF[usart_struct->TxCount++]=(BMS1.Alarm_Type&0x0040)>>6; 				usart_struct->TX_BUF[usart_struct->TxCount++]=(BMS1.Alarm_Type&0x0040);	//充电过电流保护 1
//		usart_struct->TX_BUF[usart_struct->TxCount++]=(BMS1.Alarm_Type&0x0020)>>5; 				usart_struct->TX_BUF[usart_struct->TxCount++]=(BMS1.Alarm_Type&0x0020);	//低温保护 1
//		usart_struct->TX_BUF[usart_struct->TxCount++]=(BMS1.Alarm_Type&0x0010)>>4; 				usart_struct->TX_BUF[usart_struct->TxCount++]=(BMS1.Alarm_Type&0x0010);	//低温报警 1
//		usart_struct->TX_BUF[usart_struct->TxCount++]=(BMS1.Alarm_Type&0x0008)>>3; 				usart_struct->TX_BUF[usart_struct->TxCount++]=(BMS1.Alarm_Type&0x0008);	//高温保护 1
//		usart_struct->TX_BUF[usart_struct->TxCount++]=(BMS1.Alarm_Type&0x0004)>>2; 				usart_struct->TX_BUF[usart_struct->TxCount++]=(BMS1.Alarm_Type&0x0004);	//高温报警 1
//		usart_struct->TX_BUF[usart_struct->TxCount++]=(BMS1.Alarm_Type&0x0002)>>1; 				usart_struct->TX_BUF[usart_struct->TxCount++]=(BMS1.Alarm_Type&0x0002);	//单体欠压 1
//		usart_struct->TX_BUF[usart_struct->TxCount++]=(BMS1.Alarm_Type&0x0001)>>0; 				usart_struct->TX_BUF[usart_struct->TxCount++]=(BMS1.Alarm_Type&0x0001);	//单体超压 1
//		//车体状态
//		usart_struct->TX_BUF[usart_struct->TxCount++]=0x0001>>8; 				usart_struct->TX_BUF[usart_struct->TxCount++]=0x0001;	//联网状态 1正常 0未连接
//		usart_struct->TX_BUF[usart_struct->TxCount++]=0x0001>>8; 				usart_struct->TX_BUF[usart_struct->TxCount++]=0x0001;	//运行状态 0正常 1故障 2急停
//		usart_struct->TX_BUF[usart_struct->TxCount++]=0x0001>>8; 				usart_struct->TX_BUF[usart_struct->TxCount++]=0x0001;	//控制模式 0自动 1无线 2有线
//		usart_struct->TX_BUF[usart_struct->TxCount++]=0x0001>>8; 				usart_struct->TX_BUF[usart_struct->TxCount++]=0x0001;	//通信报警
//		usart_struct->TX_BUF[usart_struct->TxCount++]=0x0000>>8; 				usart_struct->TX_BUF[usart_struct->TxCount++]=Motor_alarm&0x01;	//电机1报警
//		usart_struct->TX_BUF[usart_struct->TxCount++]=0x0000>>8; 				usart_struct->TX_BUF[usart_struct->TxCount++]=(Motor_alarm&0x02)>>1;	//电机2报警
//		usart_struct->TX_BUF[usart_struct->TxCount++]=0x0000>>8; 				usart_struct->TX_BUF[usart_struct->TxCount++]=(Motor_alarm&0x04)>>2;	//电机3报警
//		usart_struct->TX_BUF[usart_struct->TxCount++]=0x0000>>8; 				usart_struct->TX_BUF[usart_struct->TxCount++]=(Motor_alarm&0x08)>>3;	//电机4报警	
//		usart_struct->TX_BUF[usart_struct->TxCount++]=0x0000>>8; 				usart_struct->TX_BUF[usart_struct->TxCount++]=(Motor_alarm&0x10)>>4;	//电机5报警	
//		usart_struct->TX_BUF[usart_struct->TxCount++]=0x0000>>8; 				usart_struct->TX_BUF[usart_struct->TxCount++]=(Motor_alarm&0x20)>>5;	//电机6报警	
//		usart_struct->TX_BUF[usart_struct->TxCount++]=0x0000>>8; 				usart_struct->TX_BUF[usart_struct->TxCount++]=(Motor_alarm&0x40)>>6;	//电机7报警	
//		usart_struct->TX_BUF[usart_struct->TxCount++]=0x0000>>8; 				usart_struct->TX_BUF[usart_struct->TxCount++]=(Motor_alarm&0x80)>>7;	//电机8报警	
//		usart_struct->TX_BUF[usart_struct->TxCount++]=0x0000>>8; 				usart_struct->TX_BUF[usart_struct->TxCount++]=Motor_Alarm9;	//电机9报警	
//		usart_struct->TX_BUF[usart_struct->TxCount++]=0x0000>>8; 				usart_struct->TX_BUF[usart_struct->TxCount++]=Motor_Alarm10;	//电机10报警	
//		usart_struct->TX_BUF[usart_struct->TxCount++]=0x0000>>8; 				usart_struct->TX_BUF[usart_struct->TxCount++]=Motor_Alarm11;	//电机11报警	
//		usart_struct->TX_BUF[usart_struct->TxCount++]=0x0000>>8; 				usart_struct->TX_BUF[usart_struct->TxCount++]=Motor_Alarm12;	//电机12报警
//		usart_struct->TX_BUF[usart_struct->TxCount++]=0x04d2>>8; 				usart_struct->TX_BUF[usart_struct->TxCount++]=0x0001;	//车编号
//		usart_struct->TX_BUF[usart_struct->TxCount++]=0x04d2>>8; 				usart_struct->TX_BUF[usart_struct->TxCount++]=0x0001; //版本号
//		usart_struct->TX_BUF[usart_struct->TxCount++]=INNER.TARGET_SPEED>>8; 				usart_struct->TX_BUF[usart_struct->TxCount++]=INNER.TARGET_SPEED; //线速度
//		usart_struct->TX_BUF[usart_struct->TxCount++]=(int16_t)INNER.TARGET_RPS>>8; 				usart_struct->TX_BUF[usart_struct->TxCount++]=(int16_t)INNER.TARGET_RPS; //角速度
//		usart_struct->TX_BUF[usart_struct->TxCount++]=INNER.TARGET_ANGLE>>8; 				usart_struct->TX_BUF[usart_struct->TxCount++]=INNER.TARGET_ANGLE; //行驶方向						
//		crcfifo = CRC16(&UART5_STA.TX_BUF[3],93);                  //串口5  第三位开始，93个做CRC校验
//		usart_struct->TX_BUF[usart_struct->TxCount++]=crcfifo;
//		usart_struct->TX_BUF[usart_struct->TxCount++]=crcfifo>>8;


//		if(usart_struct->TxCount != 0)
//		{
//			UsartSendData_DMA(5,usart_struct->TX_BUF,usart_struct->TxCount);               //串口5
//			usart_struct->TxCount=0;
//		}	
//		taskEXIT_CRITICAL();
//}
