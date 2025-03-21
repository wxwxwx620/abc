#include "main.h"
#include "modbus_host.h"
#include "voice.h"


#define TIMEOUT		100		/* 接收命令超时时间, 单位ms */
#define NUM			1		/* 循环发送次数 */

/* 保存每个从机的计数器值 */

//MODH_T g_tModH;
uint8_t g_modh_timeout = 0;

//static void MODH_AnalyzeApp(void);

//static void MODH_Read_04H(void);
//static void MODH_Read_10H(void);

uint8_t buf0[g_buf_number0] ={0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00};//充电合
uint8_t buf1[g_buf_number1] ={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};//充电分
						
VAR_T g_tVar;
BMS_STA BMS1;
INT_CHAR data_poor;
INT_CHAR data_poor1;
INT_CHAR data_poor2;
INT_CHAR data_poor3;
/*
*********************************************************************************************************
*	函 数 名: BEBufToUint16
*	功能说明: 将2字节数组(大端Big Endian次序，高字节在前)转换为16位整数
*	形    参: _pBuf : 数组
*	返 回 值: 16位整数值
*
*   大端(Big Endian)与小端(Little Endian)
*********************************************************************************************************
*/
uint16_t BEBufToUint16(uint8_t *_pBuf)
{
    return (((uint16_t)_pBuf[0] << 8) | _pBuf[1]);
}
/*
*********************************************************************************************************
*	函 数 名: MODH_SendAckWithCRC
*	功能说明: 发送应答,自动加CRC.  
*	形    参: 无。发送数据在 g_tModH.TxBuf[], [g_tModH.TxCount
*	返 回 值: 无
*********************************************************************************************************
*/
//static void MODH_SendAckWithCRC(void)
//{
//	uint16_t crc;
//	
////	crc = usMBCRC16(UART4_STA.TX_BUF, UART4_STA.TxCount);
////	UART4_STA.TX_BUF[UART4_STA.TxCount++] = crc;	
////	UART4_STA.TX_BUF[UART4_STA.TxCount++] = crc >> 8;	
////	UsartSendData_DMA(4,UART4_STA.TX_BUF,UART4_STA.TxCount);	
//	
//	crc = usMBCRC16(UART4_STA.TX_BUF, 6);
//	UART4_STA.TX_BUF[6] = crc;	
//	UART4_STA.TX_BUF[7] = crc >> 8;	
//	UART4_STA.TX_LEN = 8;	
//	if(UART4_STA.TX_LEN != 0)
//	{	
//			while((UART4->SR&0x40) == 0);
//			HAL_UART_Transmit(&huart4,(uint8_t*)UART4_STA.TX_BUF,UART4_STA.TX_LEN,1000);
//			UART4_STA.TX_LEN=0;		
//	}
//}



///*                                         //从这里往下
//*********************************************************************************************************
//*	函 数 名: MODH_AnalyzeApp
//*	功能说明: 分析应用层协议。处理应答。
//*	形    参: 无
//*	返 回 值: 无
//*********************************************************************************************************
//*/
//static void MODH_AnalyzeApp(void)
//{	
//	switch (UART4_STA.RX_BUF[1])			/* 第2个字节 功能码 */
//	{

//		case 0x04:	/* 读取输入寄存器 */
//			//UsartSendData_DMA(4,UART4_STA.RX_BUF,UART4_STA.RX_LEN);//测试
//			MODH_Read_04H();
//			break;		

//		case 0x10:	/* 写多个寄存器 */
//			MODH_Read_10H();
//			break;
//		
//		default:
//			break;
//	}
//}

///*
//*********************************************************************************************************
//*	函 数 名: MODH_Send01H
//*	功能说明: 发送01H指令，查询1个或多个保持寄存器
//*	形    参: _addr : 从站地址
//*			  _reg : 寄存器编号
//*			  _num : 寄存器个数
//*	返 回 值: 无
//*********************************************************************************************************
//*/
//void MODH_Send01H(uint8_t _addr, uint16_t _reg, uint16_t _num)
//{
//	UART4_STA.TxCount = 0;
//	UART4_STA.TX_BUF[UART4_STA.TxCount++] = _addr;		/* 从站地址 */
//	UART4_STA.TX_BUF[UART4_STA.TxCount++] = 0x01;		/* 功能码 */	
//	UART4_STA.TX_BUF[UART4_STA.TxCount++] = _reg >> 8;	/* 寄存器编号 高字节 */
//	UART4_STA.TX_BUF[UART4_STA.TxCount++] = _reg;		/* 寄存器编号 低字节 */
//	UART4_STA.TX_BUF[UART4_STA.TxCount++] = _num >> 8;	/* 寄存器个数 高字节 */
//	UART4_STA.TX_BUF[UART4_STA.TxCount++] = _num;		/* 寄存器个数 低字节 */
//	
//	MODH_SendAckWithCRC();		/* 发送数据，自动加CRC */
//	UART4_STA.fAck01H = 0;		/* 清接收标志 */
//	UART4_STA.RegNum = _num;		/* 寄存器个数 */
//	UART4_STA.Reg01H = _reg;		/* 保存03H指令中的寄存器地址，方便对应答数据进行分类 */	
//}

///*
//*********************************************************************************************************
//*	函 数 名: MODH_Send02H
//*	功能说明: 发送02H指令，读离散输入寄存器
//*	形    参: _addr : 从站地址
//*			  _reg : 寄存器编号
//*			  _num : 寄存器个数
//*	返 回 值: 无
//*********************************************************************************************************
//*/
//void MODH_Send02H(uint8_t _addr, uint16_t _reg, uint16_t _num)
//{
//	UART4_STA.TxCount = 0;
//	UART4_STA.TX_BUF[UART4_STA.TxCount++] = _addr;		/* 从站地址 */
//	UART4_STA.TX_BUF[UART4_STA.TxCount++] = 0x02;		/* 功能码 */	
//	UART4_STA.TX_BUF[UART4_STA.TxCount++] = _reg >> 8;	/* 寄存器编号 高字节 */
//	UART4_STA.TX_BUF[UART4_STA.TxCount++] = _reg;		/* 寄存器编号 低字节 */
//	UART4_STA.TX_BUF[UART4_STA.TxCount++] = _num >> 8;	/* 寄存器个数 高字节 */
//	UART4_STA.TX_BUF[UART4_STA.TxCount++] = _num;		/* 寄存器个数 低字节 */
//	
//	MODH_SendAckWithCRC();		/* 发送数据，自动加CRC */
//	UART4_STA.fAck02H = 0;		/* 清接收标志 */
//	UART4_STA.RegNum = _num;		/* 寄存器个数 */
//	UART4_STA.Reg02H = _reg;		/* 保存03H指令中的寄存器地址，方便对应答数据进行分类 */	
//}

///*
//*********************************************************************************************************
//*	函 数 名: MODH_Send03H
//*	功能说明: 发送03H指令，查询1个或多个保持寄存器
//*	形    参: _addr : 从站地址
//*			  _reg : 寄存器编号
//*			  _num : 寄存器个数
//*	返 回 值: 无
//*********************************************************************************************************
//*/
//void MODH_Send03H(uint8_t _addr, uint16_t _reg, uint16_t _num)
//{
//	UART4_STA.TxCount = 0;
//	UART4_STA.TX_BUF[UART4_STA.TxCount++] = _addr;		/* 从站地址 */
//	UART4_STA.TX_BUF[UART4_STA.TxCount++] = 0x03;		/* 功能码 */	
//	UART4_STA.TX_BUF[UART4_STA.TxCount++] = _reg >> 8;	/* 寄存器编号 高字节 */
//	UART4_STA.TX_BUF[UART4_STA.TxCount++] = _reg;		/* 寄存器编号 低字节 */
//	UART4_STA.TX_BUF[UART4_STA.TxCount++] = _num >> 8;	/* 寄存器个数 高字节 */
//	UART4_STA.TX_BUF[UART4_STA.TxCount++] = _num;		/* 寄存器个数 低字节 */
//	
//	MODH_SendAckWithCRC();		/* 发送数据，自动加CRC */
//	UART4_STA.fAck03H = 0;		/* 清接收标志 */
//	UART4_STA.RegNum = _num;		/* 寄存器个数 */
//	UART4_STA.Reg03H = _reg;		/* 保存03H指令中的寄存器地址，方便对应答数据进行分类 */	
//}

///*
//*********************************************************************************************************
//*	函 数 名: MODH_Send04H
//*	功能说明: 发送04H指令，读输入寄存器
//*	形    参: _addr : 从站地址
//*			  _reg : 寄存器编号
//*			  _num : 寄存器个数
//*	返 回 值: 无
//*********************************************************************************************************
//*/
//void MODH_Send04H(uint8_t _addr, uint16_t _reg, uint16_t _num)
//{
//	UART4_STA.TxCount = 0;
//	UART4_STA.TX_BUF[UART4_STA.TxCount++] = _addr;		/* 从站地址 */
//	UART4_STA.TX_BUF[UART4_STA.TxCount++] = 0x04;		/* 功能码 */	
//	UART4_STA.TX_BUF[UART4_STA.TxCount++] = _reg >> 8;	/* 寄存器编号 高字节 */
//	UART4_STA.TX_BUF[UART4_STA.TxCount++] = _reg;		/* 寄存器编号 低字节 */
//	UART4_STA.TX_BUF[UART4_STA.TxCount++] = _num >> 8;	/* 寄存器个数 高字节 */
//	UART4_STA.TX_BUF[UART4_STA.TxCount++] = _num;		/* 寄存器个数 低字节 */
//	
//	MODH_SendAckWithCRC();		/* 发送数据，自动加CRC */
//	UART4_STA.fAck04H = 0;		/* 清接收标志 */
//	UART4_STA.RegNum = _num;		/* 寄存器个数 */
//	UART4_STA.Reg04H = _reg;		/* 保存03H指令中的寄存器地址，方便对应答数据进行分类 */	
//}

///*
//*********************************************************************************************************
//*	函 数 名: MODH_Send05H
//*	功能说明: 发送05H指令，写强置单线圈
//*	形    参: _addr : 从站地址
//*			  _reg : 寄存器编号
//*			  _value : 寄存器值,2字节
//*	返 回 值: 无
//*********************************************************************************************************
//*/
//void MODH_Send05H(uint8_t _addr, uint16_t _reg, uint16_t _value)
//{
//	UART4_STA.TxCount = 0;
//	UART4_STA.TX_BUF[UART4_STA.TxCount++] = _addr;			/* 从站地址 */
//	UART4_STA.TX_BUF[UART4_STA.TxCount++] = 0x04;			/* 功能码 */	
//	UART4_STA.TX_BUF[UART4_STA.TxCount++] = _reg >> 8;		/* 寄存器编号 高字节 */
//	UART4_STA.TX_BUF[UART4_STA.TxCount++] = _reg;			/* 寄存器编号 低字节 */
//	UART4_STA.TX_BUF[UART4_STA.TxCount++] = _value >> 8;		/* 寄存器值 高字节 */
//	UART4_STA.TX_BUF[UART4_STA.TxCount++] = _value;			/* 寄存器值 低字节 */
//	
//	MODH_SendAckWithCRC();		/* 发送数据，自动加CRC */

//	UART4_STA.fAck05H = 0;		/* 如果收到从机的应答，则这个标志会设为1 */
//}

///*
//*********************************************************************************************************
//*	函 数 名: MODH_Send06H
//*	功能说明: 发送06H指令，写1个保持寄存器
//*	形    参: _addr : 从站地址
//*			  _reg : 寄存器编号
//*			  _value : 寄存器值,2字节
//*	返 回 值: 无
//*********************************************************************************************************
//*/
//void MODH_Send06H(uint8_t _addr, uint16_t _reg, uint16_t _value)
//{
//	UART4_STA.TxCount = 0;
//	UART4_STA.TX_BUF[UART4_STA.TxCount++] = _addr;			/* 从站地址 */
//	UART4_STA.TX_BUF[UART4_STA.TxCount++] = 0x06;			/* 功能码 */	
//	UART4_STA.TX_BUF[UART4_STA.TxCount++] = _reg >> 8;		/* 寄存器编号 高字节 */
//	UART4_STA.TX_BUF[UART4_STA.TxCount++] = _reg;			/* 寄存器编号 低字节 */
//	UART4_STA.TX_BUF[UART4_STA.TxCount++] = _value >> 8;		/* 寄存器值 高字节 */
//	UART4_STA.TX_BUF[UART4_STA.TxCount++] = _value;			/* 寄存器值 低字节 */
//	
//	MODH_SendAckWithCRC();		/* 发送数据，自动加CRC */
//	
//	UART4_STA.fAck06H = 0;		/* 如果收到从机的应答，则这个标志会设为1 */
//}

///*
//*********************************************************************************************************
//*	函 数 名: MODH_Send10H
//*	功能说明: 发送10H指令，连续写多个保持寄存器. 最多一次支持23个寄存器。
//*	形    参: _addr : 从站地址
//*			  _reg : 寄存器编号
//*			  _num : 寄存器个数n (每个寄存器2个字节) 值域
//*			  _buf : n个寄存器的数据。长度 = 2 * n
//*	返 回 值: 无
//*********************************************************************************************************
//*/
//void MODH_Send10H(uint8_t _addr, uint16_t _reg, uint8_t _num, uint8_t *_buf)
//{
//	uint16_t i;
//	
//	UART4_STA.TxCount = 0;
//	UART4_STA.TX_BUF[UART4_STA.TxCount++] = _addr;		/* 从站地址 */
//	UART4_STA.TX_BUF[UART4_STA.TxCount++] = 0x10;		/* 从站地址 */	
//	UART4_STA.TX_BUF[UART4_STA.TxCount++] = _reg >> 8;	/* 寄存器编号 高字节 */
//	UART4_STA.TX_BUF[UART4_STA.TxCount++] = _reg;		/* 寄存器编号 低字节 */
//	UART4_STA.TX_BUF[UART4_STA.TxCount++] = _num >> 8;	/* 寄存器个数 高字节 */
//	UART4_STA.TX_BUF[UART4_STA.TxCount++] = _num;		/* 寄存器个数 低字节 */
//	UART4_STA.TX_BUF[UART4_STA.TxCount++] = 2 * _num;	/* 数据字节数 */
//	
//	for (i = 0; i < 2 * _num; i++)
//	{
//		if (UART4_STA.TxCount > H_RX_BUF_SIZE - 3)
//		{
//			return;		/* 数据超过缓冲区超度，直接丢弃不发送 */
//		}
//		UART4_STA.TX_BUF[UART4_STA.TxCount++] = _buf[i];		/* 后面的数据长度 */
//	}
//	
//	MODH_SendAckWithCRC();	/* 发送数据，自动加CRC */
//}

///*
//*********************************************************************************************************
//*	函 数 名: MODH_ReciveNew
//*	功能说明: 串口接收中断服务程序会调用本函数。当收到一个字节时，执行一次本函数。
//*	形    参: 
//*	返 回 值: 1 表示有数据
//*********************************************************************************************************
//*/
//void MODH_ReciveNew(uint8_t _data)
//{
//	/*
//		3.5个字符的时间间隔，只是用在RTU模式下面，因为RTU模式没有开始符和结束符，
//		两个数据包之间只能靠时间间隔来区分，Modbus定义在不同的波特率下，间隔时间是不一样的，
//		所以就是3.5个字符的时间，波特率高，这个时间间隔就小，波特率低，这个时间间隔相应就大

//		4800  = 7.297ms
//		9600  = 3.646ms
//		19200  = 1.771ms
//		38400  = 0.885ms
//	*/

//	g_modh_timeout = 0;
//	
////	timeout = 35000000 / HBAUD485;		/* yao计算超时时间，单位us 35000000*/
//	
//	/* 硬件定时中断，定时精度us 硬件定时器2用于MODBUS从机, 定时器3用于MODBUS从机主机*/
//	//bsp_StartHardTimer(3, timeout, (void *)MODH_RxTimeOut);
//	
//	if (UART4_STA.RxCount < H_RX_BUF_SIZE)
//	{
//		UART4_STA.RX_BUF[UART4_STA.RxCount++] = _data;
//	}
//}

///*
//*********************************************************************************************************
//*	函 数 名: MODH_Poll
//*	功能说明: 接收控制器指令. 1ms 响应时间。
//*	形    参: 无
//*	返 回 值: 0 表示无数据 1表示收到正确命令
//*********************************************************************************************************
//*/
////void MODH_Poll(void)
////{	
////	uint16_t crc1;
////	uint32_t temp;
////	if(__HAL_UART_GET_FLAG(&huart4,UART_FLAG_IDLE)!= RESET)
////	{		
////			__HAL_UART_CLEAR_IDLEFLAG(&huart4);
////			HAL_UART_DMAStop(&huart4);
////			temp = huart4.hdmarx->Instance->NDTR;
////			UART4_STA.RX_LEN = RX_URB_Size - temp;
////			HAL_UART_Receive_DMA(&huart4,UART4_STA.RX_BUF,RX_URB_Size);	
////			crc1 = usMBCRC16(UART4_STA.RX_BUF, UART4_STA.RX_LEN);
////			
////			if (crc1 == 0)//校验成功
////			{			
////				if(UART4_STA.RX_LEN==8)	Voice_CrC_flag=1;
////				if(UART4_STA.RX_BUF[5]==0x02)	Voice_CrC_flag=0;
////				UART4_STA.RX_LEN = 0;	/* 必须清零计数器，方便下次帧同步 */
////			}
////			else
////			{
////				//校验失败
////			}
////			
////	}
////}
///*
//*********************************************************************************************************
//*	函 数 名: MODH_Read_04H
//*	功能说明: 分析04H指令的应答数据
//*	形    参: 无
//*	返 回 值: 无
//*********************************************************************************************************
//*/
//static void MODH_Read_04H(void)
//{
//	uint8_t bytes;
//	uint8_t *p;
//	
//	if (UART4_STA.RX_LEN > 0)
//	{
//		bytes = UART4_STA.RX_BUF[2];	/* 数据长度 字节数 */		
//		p = &UART4_STA.RX_BUF[3];	
//		if(38==bytes)
//		{			
//			BMS1.voltage=BEBufToUint16(p)*0.5;	p += 2;//电压0.05
//			BMS1.current=BEBufToUint16(p)*0.5-16000;	p += 2;//电流	0.05-1600
//			BMS1.capacity_per=BEBufToUint16(p)*0.4;	p += 2;//容量百分比
//			BMS1.Total_hours=BEBufToUint16(p);	p += 2;//总运行时间
//			BMS1.cycle_count=BEBufToUint16(p);	p += 2;//循环次数
//			BMS1.Max_unit_voltage=BEBufToUint16(p)*0.1;	p += 2;//最高单体电压0.001
//			BMS1.MaxV_Location2_1=BEBufToUint16(p);	p += 2;//位置2 位置1
//			BMS1.Min_unit_voltage=BEBufToUint16(p)*0.1;	p += 2;//最低单体电压0.001
//			BMS1.MinV_Location2_1=BEBufToUint16(p);	p += 2;//位置2 位置1
//			BMS1.Max_unit_Temperature=BEBufToUint16(p)/3.2-1270;	p += 2;//最高单体温度/32-127
//			BMS1.MaxT_location2_1=BEBufToUint16(p);	p += 2;//位置2 位置1 
//			BMS1.Min_unit_Temperature=BEBufToUint16(p)/3.2-1270;	p += 2;//最低单体温度/32-127
//			BMS1.MinT_Location2_1=BEBufToUint16(p);	p += 2;//位置2 位置1
//			BMS1.Ta_Det=BEBufToUint16(p);	p += 2;//充电控制
//			BMS1.Ta_Present=BEBufToUint16(p);	p += 2;//充电请求
//			BMS1.Cha_Discha_Pro_Sta=BEBufToUint16(p);	p += 2;//充电保护状态 放点保护状态
////			BMS1.Cha_Discha_Relay_Sta=BEBufToUint16(p);	p += 2;//充电继电器状态 放点继电器状态		
//		//	UsartSendData_DMA(4,UART4_STA.RX_BUF,UART4_STA.RX_LEN);//测试
//			
//		}
//		else if(4==bytes)
//		{
//			BMS1.ALRN=BEBufToUint16(p);	p += 2;//告警信息数量
//			BMS1.Alarm_Type=BEBufToUint16(p);	p += 2;//告警类型	
//		}
//	
//			
//	}
//}
///*
//*********************************************************************************************************
//*	函 数 名: MODH_Read_03H
//*	功能说明: 分析03H指令的应答数据
//*	形    参: 无
//*	返 回 值: 无
//*********************************************************************************************************
//*/
////void MODH_Read_03H(void)
////{
////	uint8_t bytes;
////	uint8_t *p;
////	
////	if (UART4_STA.RxCount > 0)
////	{
////		bytes = UART4_STA.RX_BUF[2];	/* 数据长度 字节数 */				
////		switch (UART4_STA.Reg03H)
////		{
////			case REG_P01:
////				if (bytes == 32)
////				{
////					p = &UART4_STA.RX_BUF[3];	
////					
////					g_tVar.P01 = BEBufToUint16(p); p += 2;	/* 寄存器 */	
////					g_tVar.P02 = BEBufToUint16(p); p += 2;	/* 寄存器 */	
////		
////					UART4_STA.fAck03H = 1;
////				}
////				break;
////		}
////	}
////}

///*
//*********************************************************************************************************
//*	函 数 名: MODH_Read_10H
//*	功能说明: 分析10H指令的应答数据
//*	形    参: 无
//*	返 回 值: 无
//*********************************************************************************************************
//*/
//void MODH_Read_10H(void)
//{
//	/*
//		10H指令的应答:
//			从机地址                11
//			功能码                  10
//			寄存器起始地址高字节	00
//			寄存器起始地址低字节    01
//			寄存器数量高字节        00
//			寄存器数量低字节        02
//			CRC校验高字节           12
//			CRC校验低字节           98
//	*/
//	if (UART4_STA.RxCount > 0)
//	{
//		if (UART4_STA.RX_BUF[0] == SlaveAddr)		
//		{
//			UART4_STA.fAck10H = 1;		/* 接收到应答 */
//		}
//	}
//}

///*
//*********************************************************************************************************
//*	函 数 名: MODH_ReadParam_01H
//*	功能说明: 单个参数. 通过发送01H指令实现，发送之后，等待从机应答。
//*	形    参: 无
//*	返 回 值: 1 表示成功。0 表示失败（通信超时或被拒绝）
//*********************************************************************************************************
//*/
//uint8_t MODH_ReadParam_01H(uint16_t _reg, uint16_t _num)
//{
//	uint8_t i;
//	
//	for (i = 0; i < NUM; i++)
//	{
//		MODH_Send01H (SlaveAddr, _reg, _num);		  /* 发送命令 */
//	}
//	
//	if (UART4_STA.fAck01H == 0)
//	{
//		return 0;
//	}
//	else 
//	{
//		return 1;	/* 01H 读成功 */
//	}
//}

///*
//*********************************************************************************************************
//*	函 数 名: MODH_ReadParam_02H
//*	功能说明: 单个参数. 通过发送02H指令实现，发送之后，等待从机应答。
//*	形    参: 无
//*	返 回 值: 1 表示成功。0 表示失败（通信超时或被拒绝）
//*********************************************************************************************************
//*/
//uint8_t MODH_ReadParam_02H(uint16_t _reg, uint16_t _num)
//{
//	uint8_t i;
//	
//	for (i = 0; i < NUM; i++)
//	{
//		MODH_Send02H (SlaveAddr, _reg, _num);
//	}
//	
//	if (UART4_STA.fAck02H == 0)
//	{
//		return 0;
//	}
//	else 
//	{
//		return 1;	/* 02H 读成功 */
//	}
//}
///*
//*********************************************************************************************************
//*	函 数 名: MODH_ReadParam_03H
//*	功能说明: 单个参数. 通过发送03H指令实现，发送之后，等待从机应答。
//*	形    参: 无
//*	返 回 值: 1 表示成功。0 表示失败（通信超时或被拒绝）
//*********************************************************************************************************
//*/
//uint8_t MODH_ReadParam_03H(uint16_t _reg, uint16_t _num)
//{
//	uint8_t i;
//	
//	for (i = 0; i < NUM; i++)
//	{
//		MODH_Send03H (SlaveAddr, _reg, _num);
//	}
//	
//	if (UART4_STA.fAck03H == 0)
//	{
//		return 0;	/* 通信超时了 */
//	}
//	else 
//	{
//		return 1;	/* 写入03H参数成功 */
//	}
//}


///*
//*********************************************************************************************************
//*	函 数 名: MODH_ReadParam_04H
//*	功能说明: 单个参数. 通过发送04H指令实现，发送之后，等待从机应答。
//*	形    参: 无
//*	返 回 值: 1 表示成功。0 表示失败（通信超时或被拒绝）
//*********************************************************************************************************
//*/
//uint8_t MODH_ReadParam_04H(uint16_t _reg, uint16_t _num)
//{
//	uint8_t i;
//	
//	for (i = 0; i < NUM; i++)
//	{
//		MODH_Send04H (SlaveAddr, _reg, _num);
//	}
//	
//	if (UART4_STA.fAck04H == 0)
//	{
//		return 0;	/* 通信超时了 */
//	}
//	else 
//	{
//		return 1;	/* 04H 读成功 */
//	}
//}
///*
//*********************************************************************************************************
//*	函 数 名: MODH_WriteParam_05H
//*	功能说明: 单个参数. 通过发送05H指令实现，发送之后，等待从机应答。
//*	形    参: 无
//*	返 回 值: 1 表示成功。0 表示失败（通信超时或被拒绝）
//*********************************************************************************************************
//*/
//uint8_t MODH_WriteParam_05H(uint16_t _reg, uint16_t _value)
//{
//	uint8_t i;

//	for (i = 0; i < NUM; i++)
//	{
//		MODH_Send05H (SlaveAddr, _reg, _value);
//	}
//	
//	if (UART4_STA.fAck05H == 0)
//	{
//		return 0;	/* 通信超时了 */
//	}
//	else
//	{
//		return 1;	/* 05H 写成功 */
//	}
//}

///*
//*********************************************************************************************************
//*	函 数 名: MODH_WriteParam_06H
//*	功能说明: 单个参数. 通过发送06H指令实现，发送之后，等待从机应答。循环NUM次写命令
//*	形    参: 无
//*	返 回 值: 1 表示成功。0 表示失败（通信超时或被拒绝）
//*********************************************************************************************************
//*/
//uint8_t MODH_WriteParam_06H(uint16_t _reg, uint16_t _value)
//{
//	uint8_t i;
//	
//	for (i = 0; i < NUM; i++)
//	{	
//		MODH_Send06H (SlaveAddr, _reg, _value);
//	}
//	
//	if (UART4_STA.fAck06H == 0)
//	{
//		return 0;	/* 通信超时了 */
//	}
//	else
//	{
//		return 1;	/* 写入06H参数成功 */
//	}
//}

///*
//*********************************************************************************************************
//*	函 数 名: MODH_WriteParam_10H
//*	功能说明: 单个参数. 通过发送10H指令实现，发送之后，等待从机应答。循环NUM次写命令
//*	形    参: 无
//*	返 回 值: 1 表示成功。0 表示失败（通信超时或被拒绝）
//*********************************************************************************************************
//*/
//uint8_t MODH_WriteParam_10H(uint16_t _reg, uint8_t _num, uint8_t *_buf)
//{
//	uint8_t i;
//	
//	for (i = 0; i < NUM; i++)
//	{	
//		MODH_Send10H(SlaveAddr, _reg, _num, _buf);
//	}
//	
//	if (UART4_STA.fAck10H == 0)
//	{
//		return 0;	/* 通信超时了 */
//	}
//	else
//	{
//		return 1;	/* 写入10H参数成功 */
//	}
//}



