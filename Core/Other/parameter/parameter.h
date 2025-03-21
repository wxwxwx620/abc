/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __parameter_H
#define __parameter_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h" 
	 
/* USER CODE BEGIN Includes */
#include "gpio.h"	 
/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

#define ROUND_TO_INT16(x) ((x)>=0?(int16_t)((x)+0.5f):(int16_t)((x)-0.5f))  
#define ROUND_TO_INT32(x) ((x)>=0?(int32_t)((x)+0.5f):(int32_t)((x)-0.5f)) 
	 
//IO�ڲ����궨��
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
//IO�ڵ�ַӳ��
#define GPIOA_ODR_Addr    (GPIOA_BASE+20) //0x40020014
#define GPIOB_ODR_Addr    (GPIOB_BASE+20) //0x40020414 
#define GPIOC_ODR_Addr    (GPIOC_BASE+20) //0x40020814 
#define GPIOD_ODR_Addr    (GPIOD_BASE+20) //0x40020C14 
#define GPIOE_ODR_Addr    (GPIOE_BASE+20) //0x40021014 
#define GPIOF_ODR_Addr    (GPIOF_BASE+20) //0x40021414    
#define GPIOG_ODR_Addr    (GPIOG_BASE+20) //0x40021814   
#define GPIOH_ODR_Addr    (GPIOH_BASE+20) //0x40021C14    
#define GPIOI_ODR_Addr    (GPIOI_BASE+20) //0x40022014     

#define GPIOA_IDR_Addr    (GPIOA_BASE+16) //0x40020010 
#define GPIOB_IDR_Addr    (GPIOB_BASE+16) //0x40020410 
#define GPIOC_IDR_Addr    (GPIOC_BASE+16) //0x40020810 
#define GPIOD_IDR_Addr    (GPIOD_BASE+16) //0x40020C10 
#define GPIOE_IDR_Addr    (GPIOE_BASE+16) //0x40021010 
#define GPIOF_IDR_Addr    (GPIOF_BASE+16) //0x40021410 
#define GPIOG_IDR_Addr    (GPIOG_BASE+16) //0x40021810 
#define GPIOH_IDR_Addr    (GPIOH_BASE+16) //0x40021C10 
#define GPIOI_IDR_Addr    (GPIOI_BASE+16) //0x40022010 
 
//IO�ڲ���,ֻ�Ե�һ��IO��!
//ȷ��n��ֵС��16!
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //��� 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //���� 
#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //��� 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //���� 
#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //��� 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //���� 
#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //��� 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //���� 
#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //��� 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //����
#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //��� 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //����
#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //��� 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //����
#define PHout(n)   BIT_ADDR(GPIOH_ODR_Addr,n)  //��� 
#define PHin(n)    BIT_ADDR(GPIOH_IDR_Addr,n)  //����
#define PIout(n)   BIT_ADDR(GPIOI_ODR_Addr,n)  //��� 
#define PIin(n)    BIT_ADDR(GPIOI_IDR_Addr,n)  //����

//
#define CW 1
#define CCW 0

extern uint8_t INTRGRAL_TIME;
extern uint8_t ACC_COEFFICIENT; 

extern float ODOMETER_Y_COEFFICIENT;
extern float ODOMETER_X_COEFFICIENT;
extern float ODOMETER_Z_COEFFICIENT;	 
	 
extern uint16_t IL_COEFFICIENT;
extern uint16_t OL_COEFFICIENT;

extern float VMM_TO_RPM;      //���ٶ�mm/sת��Ϊת��
extern float RPM_TO_VMM;      //ת��ת��Ϊ���ٶ�mm/s
extern float DEG_TO_RAD;      //�Ƕ�ת��Ϊ����
extern float RAD_TO_DEG;      //����ת�Ƕ�


extern uint8_t AutomatiCaccess_control;//yao//����Ȩ��־ 0���ޣ�12���ɰ�����ң�أ�10������ң�أ�8������ң�أ�6��appң�أ�4����λ����2�Զ���

extern uint8_t alarm__control;//yao 0���޼�ͣ 1�����弱ͣ��2ң������ͣ��4����λ����ͣ
extern uint8_t control_mode;	 
extern uint8_t control_mode_1;
/* USER CODE END Private defines */


/* USER CODE BEGIN Prototypes */
 
void Parameter_Init(void);
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
