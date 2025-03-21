/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UI_H
#define __UI_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"	 
	 
/* USER CODE BEGIN Includes */
#include "usart.h"	 
#include "modbus_host.h"
//#include "datacheck.h"

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

extern osThreadId UIInitTaskHandle;
extern osThreadId UITaskHandle;
 
/* USER CODE END Private defines */

/* USER CODE BEGIN Prototypes */

extern void UI_task_Initialize(USART_STA *usart_struct);
extern void UI_task_Function(USART_STA *usart_struct);

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
