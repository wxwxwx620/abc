#include "exti.h"
#include "main.h"
#include "motor.h"

void  EXTI_GPIO_Init(void)//中断GPIO初始化 20220902
{
    GPIO_InitTypeDef GPIO_Initure;
    
    __HAL_RCC_GPIOE_CLK_ENABLE();               //开启GPIOE时钟
    
    GPIO_Initure.Pin = UL9_Pin|DL9_Pin|UL10_Pin|DL10_Pin
	                   |UL11_Pin|DL11_Pin|UL12_Pin|DL12_Pin;
    GPIO_Initure.Mode=GPIO_MODE_IT_FALLING;     //下降沿触发
    GPIO_Initure.Pull=GPIO_PULLUP;
    HAL_GPIO_Init(GPIOE,&GPIO_Initure);
    
    //中断线7-9
    HAL_NVIC_SetPriority(EXTI9_5_IRQn,3,0);       //抢占优先级为2，子优先级为0
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);             //使能中断线7
    
    //中断线10-15
    HAL_NVIC_SetPriority(EXTI15_10_IRQn,3,1);       //抢占优先级为2，子优先级为1
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);             //使能中断线2      	//使能中断线4
}


//中断服务函数
void EXTI9_5_IRQHandler(void)
{
    if(FLO==0)
			HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);		//调用中断处理公用函数
	  if(FLC==0)
			HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8);		//调用中断处理公用函数
}

void EXTI15_10_IRQHandler(void)
{
		if(BRO==0)
			HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);		//调用中断处理公用函数
		if(BRC==0)
			HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_11);		//调用中断处理公用函数
		if(FRO==0)
			HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12);		//调用中断处理公用函数
		if(FRC==0)
			HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);		//调用中断处理公用函数
		if(BLO==0)
			HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_14);		//调用中断处理公用函数
		if(BLC==0)
			HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);		//调用中断处理公用函数
}


//中断服务程序中需要做的事情
//在HAL库中所有的外部中断服务函数都会调用此函数
//GPIO_Pin:中断引脚号
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    HAL_Delay(10);      //消抖
    switch(GPIO_Pin)
    {
        case GPIO_PIN_7:
               Set_Can_RPM_RPDO(FL_Clip,0,1);
        break;
				
				case GPIO_PIN_8:
               Set_Can_RPM_RPDO(FL_Clip,0,1);
        break;
				
        case GPIO_PIN_14:
               Set_Can_RPM_RPDO(BL_Clip,0,1);
        break;
				
				case GPIO_PIN_15:
               Set_Can_RPM_RPDO(BL_Clip,0,1);
        break;
				
        case GPIO_PIN_12:
               Set_Can_RPM_RPDO(FR_Clip,0,1);
        break;
				
				case GPIO_PIN_13:
               Set_Can_RPM_RPDO(FR_Clip,0,1);
        break;
				
        case GPIO_PIN_10:
               Set_Can_RPM_RPDO(BR_Clip,0,1);
        break;
				
				case GPIO_PIN_11:
               Set_Can_RPM_RPDO(BR_Clip,0,1);
        break;
    }
}

