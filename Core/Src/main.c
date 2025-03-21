/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "iwdg.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "modbus_host.h"
#include "indicatorlight.h"
#include "UI.h"
#include "voice.h"
#include "FIFO.h"
#include "chassis.h"
#include "74HC595.h"
#include "encoder.h"

#include "74HC597.h"
#include "spi.h"
#include "remote_control.h"
#include "BMS.h"
#include "exti.h"
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void can_filter_init(CAN_HandleTypeDef *hcan)
{

    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);


    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(hcan, &can_filter_st);
    HAL_CAN_Start(hcan);
    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}
	

void InitPower(void )
{
	/* USER CODE BEGIN slowly_Power_Task */
	/* Infinite loop */
		HC595_del(OUT_5);
        HAL_Delay(100);

		HC595_add(OUT_6);
		//		HOIST_DOWN=0;//缓上电开
		HAL_Delay(5000);
		HC595_add(OUT_5);
		//		MOTORPOWER=0;//驱动器电源开
	
	  HAL_Delay(1000);
	  HC595_add(OUT_19);//继电器19打开 //给行走电机抱闸上电 20220911
	
	
		HAL_Delay(2000);
		HC595_del(OUT_6);                //固态继电器断开
	//		HOIST_DOWN=1;//缓上电关
	//		BRAKE_EN=1;//使能/抱闸开
		HAL_Delay(2000);
		SetMotorEnable();
		//HAL_Delay(5000);
		//PowerReady = 1;
	
		
	
	/* USER CODE END slowly_Power_Task */
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
	EXTI_GPIO_Init();//中断GPIO初始化 20220902
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_UART4_Init();
  //MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM7_Init();
  MX_SPI1_Init();
  MX_TIM10_Init();
  //MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
	HC595_init();
	can_filter_init(&hcan1);
	Parameter_Init();	//参数初始化
	ringbuff_init(&RC_buff);//遥控器循环缓冲初始化
	ringbuff_init(&PC_buff);//遥控器循环缓冲初始化
	
	HAL_TIM_Base_Start_IT(&htim7);//启动定时器7
	HAL_TIM_Base_Start_IT(&htim10);//启动定时器10
//	CAN_User_Init_Can1();
	//__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);//打开串口空闲中断
	//__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);	
	__HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
	__HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);


	HAL_UART_Receive_DMA(&huart1,USART1_STA.RX_BUF,RX_RC_Size);//初始化DMA接收	
	HAL_UART_Receive_DMA(&huart2,USART2_STA.RX_BUF,RX_PC_Size);	
	HAL_UART_Receive_DMA(&huart3,USART3_STA.RX_BUF,RX_URB_Size);	
	HAL_UART_Receive_DMA(&huart4,UART4_STA.RX_BUF,RX_URB_Size);	
	HAL_UART_Receive_DMA(&huart5,UART5_STA.RX_BUF,RX_URB_Size);


 // HC595_add(OUT_19);//继电器19打开 //给行走电机抱闸上电 20220911
	
  DIR4=1;                
  Voice_MODH_SendH(1);//开机欢迎
	HAL_Delay(1000);	            //一定要加hal延时，否则播报不了 ！！  

 // UI_Initialize();//ui初始化
//	HAL_Delay(1000);	           //和语音的数据稍微间隔一点时间	
	
//	HC595_del(OUT_5);
//    HAL_Delay(20);
	
  /* USER CODE END 2 */
  InitPower();
  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();
 
  
 
  /* Start scheduler */
  osKernelStart();
	
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */



  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */



/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
		static unsigned short int CNT7 = 0;
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == htim7.Instance)
	{
//			Motor_alarm=(Motor_Alarm_Clip_BL<<7)+(Motor_Alarm_Clip_BR<<6)+(Motor_Alarm_Clip_FR<<5)+(Motor_Alarm_Clip_FL<<4)+(Motor_Alarm_BL<<3)+(Motor_Alarm_BR<<2)+(Motor_Alarm_FR<<1)+Motor_Alarm_FL;
//			Lif_Motor_alarm=(Motor_Alarm9<<3)+(Motor_Alarm10<<2)+(Motor_Alarm11<<1)+Motor_Alarm12;
			////if((LF_JT==1)||(RF_JT==1)||(LR_JT==1)||(RR_JT==1)) scram_alarm_filter |=0x02;
		
//			Motor_alarm=(Motor_Alarm_BL<<3)+(Motor_Alarm_BR<<2)+(Motor_Alarm_FR<<1)+Motor_Alarm_FL;
		
//			if(EmergencyStopCar==1) ctrl_mode.state.scram_alarm_filter |=0x02;//车体急停
//			else	ctrl_mode.state.scram_alarm_filter &=0xfd;
			if(get_exi(IN_24)==1) agvStatus.stopButton=0;//车体急停按钮拍下
			else	agvStatus.stopButton = 0;
		  
			if(CNT7>500)
			{
				CNT7 = 0;
			}
			
			if((CNT7%(INTRGRAL_TIME))==0)//速度及里程计刷,timer7 2ms中断一次
			{
					if(ctrl_mode.state.scram_alarm_filter != 0x00)	
					{
						LR_AK=0;	
						Stop_Moving();
					}
					else 
					{
						LR_AK=1;
						
					}		
					Odometer_Solution();
			}			
			
			CNT7++;	
	}
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

