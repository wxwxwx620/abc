#include "exti.h"
#include "main.h"
#include "motor.h"

void  EXTI_GPIO_Init(void)//�ж�GPIO��ʼ�� 20220902
{
    GPIO_InitTypeDef GPIO_Initure;
    
    __HAL_RCC_GPIOE_CLK_ENABLE();               //����GPIOEʱ��
    
    GPIO_Initure.Pin = UL9_Pin|DL9_Pin|UL10_Pin|DL10_Pin
	                   |UL11_Pin|DL11_Pin|UL12_Pin|DL12_Pin;
    GPIO_Initure.Mode=GPIO_MODE_IT_FALLING;     //�½��ش���
    GPIO_Initure.Pull=GPIO_PULLUP;
    HAL_GPIO_Init(GPIOE,&GPIO_Initure);
    
    //�ж���7-9
    HAL_NVIC_SetPriority(EXTI9_5_IRQn,3,0);       //��ռ���ȼ�Ϊ2�������ȼ�Ϊ0
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);             //ʹ���ж���7
    
    //�ж���10-15
    HAL_NVIC_SetPriority(EXTI15_10_IRQn,3,1);       //��ռ���ȼ�Ϊ2�������ȼ�Ϊ1
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);             //ʹ���ж���2      	//ʹ���ж���4
}


//�жϷ�����
void EXTI9_5_IRQHandler(void)
{
    if(FLO==0)
			HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);		//�����жϴ����ú���
	  if(FLC==0)
			HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8);		//�����жϴ����ú���
}

void EXTI15_10_IRQHandler(void)
{
		if(BRO==0)
			HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);		//�����жϴ����ú���
		if(BRC==0)
			HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_11);		//�����жϴ����ú���
		if(FRO==0)
			HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12);		//�����жϴ����ú���
		if(FRC==0)
			HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);		//�����жϴ����ú���
		if(BLO==0)
			HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_14);		//�����жϴ����ú���
		if(BLC==0)
			HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);		//�����жϴ����ú���
}


//�жϷ����������Ҫ��������
//��HAL�������е��ⲿ�жϷ�����������ô˺���
//GPIO_Pin:�ж����ź�
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    HAL_Delay(10);      //����
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

