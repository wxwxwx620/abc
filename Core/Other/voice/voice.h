/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __VOICE_H
#define __VOICE_H
#ifdef __cplusplus
 extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"	 
	 
/* USER CODE BEGIN Includes */
#include "datacheck.h"
#include "usart.h"	 
/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */


/* USER CODE END Private defines */


/* USER CODE BEGIN Prototypes */
extern void MODH_Poll(void);   
extern void Voice_MODH_SendH(uint16_t _num);
extern void MODH_SendAckWithCRC(void);
	 
	 
extern void UI_date1(void);//UI数据处理
extern void UI_date2(void);//UI数据处理	 
extern void UART5_ui_voice(void);//ui和语音合并发送
extern void UI_Initialize(void);//ui初始化程序


	 
extern uint8_t Voice_XX;
extern uint8_t Voice_TXAdd;
extern uint8_t Voice_CrC_flag;
/* USER CODE END Prototypes */


void Voice_Init(void);
void Voice_Update(uint8_t voiceId);
void Voice_Send(uint8_t voiceId);
void Voice_Process(void);

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
