/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
  /* USER CODE END Header */

  /* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "chassis.h"
#include "voice.h"
#include "UI.h"
#include "motor.h"
#include "Protocol.h"
#include "remote_control.h"
#include "74HC595.h"
#include "indicatorlight.h"

#include "tim.h"
#include "Alarm.h"

#include "encoder.h"
#include "BMS.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern IWDG_HandleTypeDef hiwdg;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId chassisTaskHandle;
osThreadId comTaskHandle;
osThreadId voiceTaskHandle;
osThreadId UITaskHandle;
osThreadId detectTaskHandle;
osThreadId BMSTaskHandle;
osThreadId userDebugTaskHandle;
osThreadId UIInitTaskHandle;
osThreadId liftCtrlTaskHandle;
osThreadId timeManageTaskHandle;
osThreadId slowlyPowerTaskHandle;
osThreadId AlarmTaskHandle; //20221008
osMessageQId myQueue01Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void chassis_Task(void const * argument);
void com_Task(void const * argument);
void voice_Task(void const * argument);
void UI_Task(void const * argument);
void detect_Task(void const * argument);
void BMS_Task(void const * argument);
void user_debug_task(void const * argument);
void UI_Initialize_Task(void const * argument);
void lift_Ctrl_Task(void const * argument);
void slowly_Power_Task(void const * argument);
void Alarm_Task(void const * argument);//20221008
void InitPower(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize);

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
{
	*ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
	*ppxIdleTaskStackBuffer = &xIdleStack[0];
	*pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
	/* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* Create the queue(s) */
	/* definition and creation of myQueue01 */
	osMessageQDef(myQueue01, 16, uint16_t);
	myQueue01Handle = osMessageCreate(osMessageQ(myQueue01), NULL);

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* definition and creation of defaultTask */
  //  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);  //20220913
  //  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

	/* definition and creation of chassisTask */
  //  osThreadDef(chassisTask, chassis_Task, osPriorityAboveNormal, 0, 256); //20220913
  //  chassisTaskHandle = osThreadCreate(osThread(chassisTask), NULL);

	/* definition and creation of comTask */
	osThreadDef(comTask, com_Task, osPriorityNormal, 0, 256);
	comTaskHandle = osThreadCreate(osThread(comTask), NULL);

	//语音模块线程
	/* definition and creation of voiceTask */
	osThreadDef(voiceTask, voice_Task, osPriorityIdle, 0, 128);
	voiceTaskHandle = osThreadCreate(osThread(voiceTask), NULL);

	/* definition and creation of UITask */
  //  osThreadDef(UITask, UI_Task, osPriorityIdle, 0, 128);   //20220913
  //  UITaskHandle = osThreadCreate(osThread(UITask), NULL);

	/* definition and creation of detectTask */
    osThreadDef(detectTask, detect_Task, osPriorityNormal, 0, 128); //20220913
    detectTaskHandle = osThreadCreate(osThread(detectTask), NULL);

	/* definition and creation of BMSTask */
	//osThreadDef(BMSTask, BMS_Task, osPriorityBelowNormal, 0, 128);
	//BMSTaskHandle = osThreadCreate(osThread(BMSTask), NULL);

	/* definition and creation of userDebugTask */
	osThreadDef(userDebugTask, user_debug_task, osPriorityAboveNormal, 0, 128);
	userDebugTaskHandle = osThreadCreate(osThread(userDebugTask), NULL);

	/* definition and creation of UIInitTask */
  //  osThreadDef(UIInitTask, UI_Initialize_Task, osPriorityIdle, 0, 128); //20220920
  //  UIInitTaskHandle = osThreadCreate(osThread(UIInitTask), NULL);

	/* definition and creation of liftCtrlTask */
	//osThreadDef(liftCtrlTask, lift_Ctrl_Task, osPriorityNormal, 0, 128); //20220920
	//liftCtrlTaskHandle = osThreadCreate(osThread(liftCtrlTask), NULL);

	/* definition and creation of timeManageTask */
	//osThreadDef(timeManageTask, Time_Management_Task, osPriorityIdle, 0, 128);
	//timeManageTaskHandle = osThreadCreate(osThread(timeManageTask), NULL);

	/* definition and creation of slowlyPowerTask */
//	osThreadDef(slowlyPowerTask, slowly_Power_Task, osPriorityIdle, 0, 128);
	//slowlyPowerTaskHandle = osThreadCreate(osThread(slowlyPowerTask), NULL);

	//	osThreadDef(AlarmTask, Alarm_Task, osPriorityAboveNormal, 0, 128);  //20221008
	//  AlarmTaskHandle = osThreadCreate(osThread(AlarmTask), NULL);

	  /* USER CODE BEGIN RTOS_THREADS */
	  /* add threads, ... */
	  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
  /* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
	/* USER CODE BEGIN StartDefaultTask */
	/* Infinite loop */
	for (;;)
	{
		//Usart2Send_PC();
		osDelay(200);
	}
	/* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_chassis_Task */
/**
* @brief Function implementing the chassisTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_chassis_Task */
void chassis_Task(void const * argument)
{
	/* USER CODE BEGIN chassis_Task */
	/* Infinite loop */
	for (;;)
	{
		osDelay(20);
	}
	/* USER CODE END chassis_Task */
}

/* USER CODE BEGIN Header_com_Task */
/**
* @brief Function implementing the comTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_com_Task */
void com_Task(void const * argument)
{
	/* USER CODE BEGIN com_Task */


  //	uint8_t rc_srcdata[18]={0};
  //	uint8_t pc_srcdata[18]={0};
	  //开机初始化
	RemoteInit();
	/* Infinite loop */
	for (;;)
	{
		//如果没有错误，进行处理
	/*	if(HandleError(0)==0)
		{

		rc_get_srcdata(rc_srcdata);
		srcdata_to_rc(rc_srcdata);
		motor_move_rc();
		pc_get_srcdata(pc_srcdata);
		srcdata_to_pc(pc_srcdata);
		motor_move_pc();
		}*/
		DoAgvCtrl();
		//		HAL_IWDG_Refresh(&hiwdg);
		osDelay(10);
	}
	/* USER CODE END com_Task */
}

/* USER CODE BEGIN Header_voice_Task */
/**
* @brief Function implementing the voiceTask thread.
* @param argument: Not used
* @retval None
*/


/* USER CODE END Header_voice_Task */
uint8_t switchFlag = 0;
void voice_Task(void const * argument)
{
	/* USER CODE BEGIN voice_Task */
	//osEvent event;
//	const TickType_t xMaxBlockTime = pdMS_TO_TICKS(10); /* 设置最大等待时间为10ms */
//  uint8_t Message1_Flag1=0;
  /* Infinite loop */

	for (;;)
	{
		switchFlag++;
		//			{
		//				event=osMessageGet(myQueue01Handle,xMaxBlockTime);
		//			}
		if (switchFlag == 10)
		{
			UART4_Send(1);  //BMS
			switchFlag = 0;
		}
		else
		{
			Voice_Process();
		}
		
		osDelay(200);
		/*if (ch == 10)
		{
		
		}
		if (ch > 20)
		{
			UART4_Send(3); //显示屏
			ch = 0;
		}*/
	}
	/* USER CODE END voice_Task */
}

/* USER CODE BEGIN Header_UI_Task */
/**
* @brief Function implementing the UITask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UI_Task */
void UI_Task(void const * argument)
{
	/* USER CODE BEGIN UI_Task */
	/* Infinite loop */
	for (;;)
	{
		//		UI_task_Function(&UART5_STA);
		osDelay(200);
	}
	/* USER CODE END UI_Task */
}

/* USER CODE BEGIN Header_detect_Task */
/**
* @brief Function implementing the detectTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_detect_Task */
void detect_Task(void const * argument)
{
	/* USER CODE BEGIN detect_Task */
	/* Infinite loop */
	for (;;)
	{

		osDelay(200);
	}
	/* USER CODE END detect_Task */
}

/* USER CODE BEGIN Header_BMS_Task */
/**
* @brief Function implementing the BMSTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_BMS_Task */
void BMS_Task(void const * argument)
{
	/* USER CODE BEGIN BMS_Task */
	/* Infinite loop */
	uint8_t ch;
	for (;;)
	{
		//		if(1==BMS_Relay)
		//		{
		//			if(((USART2_STA.RX_BUF[9]&0x04)==0x04)&&((BMS1.Cha_Relay_Sta&0x01)==0x00))
		//			{				
		//				//Can_Send_Msg(buf0,8,0x118);//充电合
		//				osDelay(1000);
		//			}
		//		}
		//		else
		//		{
		//			if(((USART2_STA.RX_BUF[9]&0x04)==0x00)&&(BMS1.Cha_Relay_Sta&0x01)==0x01)
		//			{
		//				//Can_Send_Msg(buf1,8,0x118);//充电分
		//				osDelay(1000);
		//			}						
		//		}


		ch++;
		if (ch == 10)
		{
			UART4_Send(1);
		}
		if (ch > 20)
		{
			UART4_Send(3);
			ch = 0;
		}
		osDelay(200);
	}
	/* USER CODE END BMS_Task */
}

/* USER CODE BEGIN Header_user_debug_task */
/**
* @brief Function implementing the userDebugTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_user_debug_task */
void user_debug_task(void const * argument)
{
	/* USER CODE BEGIN user_debug_task */
	/* Infinite loop */
//	float RADv;
//	uint8_t srcdata[18] = { 0 };
	/* Infinite loop */
	for (;;)
	{
		Encoder_Send();
		osDelay(20);
	}
	/* USER CODE END user_debug_task */
}

/* USER CODE BEGIN Header_UI_Initialize_Task */
/**
* @brief Function implementing the UIInitTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UI_Initialize_Task */
void UI_Initialize_Task(void const * argument)
{
	/* USER CODE BEGIN UI_Initialize_Task */
	/* Infinite loop */
	for (;;)
	{

		//		UI_task_Initialize(&UART5_STA);
		osDelay(50);
	}
	/* USER CODE END UI_Initialize_Task */
}

/* USER CODE BEGIN Header_lift_Ctrl_Task */
/**
* @brief Function implementing the liftCtrlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_lift_Ctrl_Task */
void lift_Ctrl_Task(void const * argument)
{
	/* USER CODE BEGIN lift_Ctrl_Task */
	/* Infinite loop */
	for (;;)
	{
//		Motor_Alarm_Processing();//电机驱动器告警处理
//		if (Motor_Alarm_Flag)//电机发生告警
//		{
//			Motor_Power_Off();//电机驱动器电源关闭	
//			HAL_Delay(2000);
//			Restart_Motor_Power();//电机驱动器电源重启
//			if (Motor_Alarm_cnt > 3)//10分钟内重启3次把任务挂起并且停止电机
//			{
//				HC595_del(OUT_19);//继电器19关闭 //给行走电机抱闸掉电
//				vTaskSuspend(liftCtrlTaskHandle);
//			}
//		}
		osDelay(200);
	}
	/* USER CODE END lift_Ctrl_Task */
}

/* USER CODE BEGIN Header_Time_Management_Task */
/**
* @brief Function implementing the timeManageTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Time_Management_Task */
void Time_Management_Task(void const * argument)
{
	/* USER CODE BEGIN Time_Management_Task */
	/* Infinite loop */
	for (;;)
	{
		//Run_Light();                          //之前的蓝牙换口的程序是放在这里的

		osDelay(200);
	}
	/* USER CODE END Time_Management_Task */
}

/* USER CODE BEGIN Header_slowly_Power_Task */
/**
* @brief Function implementing the slowlyPowerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_slowly_Power_Task */
void slowly_Power_Task(void const * argument)
{
	/* USER CODE BEGIN slowly_Power_Task */
	/* Infinite loop */
	for (;;)
	{
		HC595_add(OUT_6);
		//		HOIST_DOWN=0;//缓上电开
		osDelay(5000);
		HC595_add(OUT_5);
		//		MOTORPOWER=0;//驱动器电源开
		osDelay(1000);
		HC595_del(OUT_6);                //固态继电器断开
	//		HOIST_DOWN=1;//缓上电关
	//		BRAKE_EN=1;//使能/抱闸开
		osDelay(2000);
		SetMotorEnable();
		PowerReady = 1;
		osDelay(2000);
		vTaskSuspend(slowlyPowerTaskHandle);

		//		if(1==HOIST_DOWN)	vTaskSuspend(BufferToElectriHandle);
		//		else		vTaskResume(BufferToElectriHandle);//恢复任务
	}
	/* USER CODE END slowly_Power_Task */
}



void Alarm_Task(void const * argument)//20221008
{
	/* USER CODE BEGIN slowly_Power_Task */
	/* Infinite loop */
	//for (;;)
	{
//		Motor_Alarm_Processing();//电机驱动器告警处理
//		if (Motor_Alarm_Flag)//电机发生告警
//		{
//			Motor_Power_Off();//电机驱动器电源关闭	
//			HAL_Delay(2000);
//			Restart_Motor_Power();//电机驱动器电源重启
//			if (Motor_Alarm_cnt > 3)//10分钟内重启3次把任务挂起并且停止电机
//			{
//				vTaskSuspend(AlarmTaskHandle);
//				HC595_del(OUT_19);//继电器19关闭 //给行走电机抱闸掉电
//			}
//		}
	}
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
