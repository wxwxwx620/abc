/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __datacheck_H
#define __datacheck_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"	 
	 
/* USER CODE BEGIN Includes */
	 
/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

typedef uint16_t USHORT;
typedef unsigned char UCHAR;
typedef uint16_t USHORT;

typedef struct
{
	uint32_t IOSTRB_IO;
	
}IOSTRB_IN;

//��ʱ���ṹ��
typedef struct
{
	uint16_t CurVale;         //��ǰֵ
	uint16_t NoUpdateCount;   //δ���¼���
	uint16_t MaxNoUpdate;     //��������δ���¼���	
}TimeoutCheck;
//��ʼ����ʱ�ṹ��
void InitTimeout(TimeoutCheck *check, uint16_t noUpdateLimit);
//����Ƿ�ʱ��0 δ��ʱ 1��ʱ
int CheckTimeout(TimeoutCheck *check, uint16_t newValue);

//��ʼ����ʱ�ṹ��
void ResetTimeout(TimeoutCheck *check, uint16_t newValue);

extern IOSTRB_IN IOSTRB_INx;

#define IOSTRB_STATE_0 0
#define IOSTRB_STATE_1 1
#define IOSTRB_STATE_2 2
 
#define IOSTRB_NONE 0
#define IOSTRB_FILTER_TIME 500 //��ʱʱ��
#define IOSTRB_LONGPRESS_TIME 500000 //�ȶ��ź�ʱ��
	 
#define ABS(x) ((x)>=0?(x):(-(x)))
	 
/* USER CODE END Private defines */


/* USER CODE BEGIN Prototypes */
uint16_t MODBUS_CalCRC(const uint8_t buff[], const uint8_t len);
extern uint16_t getchecksum( uint8_t *ptr, uint8_t len) ;	 
extern USHORT usMBCRC16( UCHAR * pucFrame, USHORT usLen );
extern uint32_t IOSTRB(uint32_t code);
extern unsigned int CRC16(unsigned char *Ptr, unsigned char length);
/* USER CODE END Prototypes */


#ifdef __cplusplus
}
#endif
#endif 

/**
  * @}
  */

/**
  * @}
  */
