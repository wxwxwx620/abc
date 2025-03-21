/**
  ******************************************************************************
  * @file			74HC595.c
  * @brief		74HC595��չ����ӿ�
	*	@author		Ф����
	*	@ver			2.0
	*	@date			2021.12.15
  ******************************************************************************
  * @attention
  *	v1.1 ������ʽ��GPIO���ΪSPI
	*	�˴�HC595_updata���ڶ�ʱ��10��50ms��ʱ���ã�ֱ��ʹ��HC595_add��HC595_del����
  *	v2.0 ���ӳ�ʼ�����̣�fix�ϵ������
  ******************************************************************************
  */

#include "74HC595.h"
#include "spi.h"


volatile uint32_t out_buff=0,new_buff=0;



/**
  * @brief	����һ��595����ʱ��
  */

void HC595_RCK(void)
{
	HC595_R1;

	HC595_R0;
}


/**
  *	@brief		�򻺴�������ݣ��������HC595_updata������˿�
	*	@param		data Ҫ��ӵ�����
	*	@example	HC595_add(OUT_1|OUT_2);
  */

void HC595_add(uint32_t data)
{
	new_buff|=data;
}



/**
  * @brief		�򻺴�ɾ�����ݣ��������HC595_updata������˿�
	*	@param		data Ҫɾ��������
	*	@example	HC595_del(OUT_1|OUT_2);
  */

void HC595_del(uint32_t data)
{
	new_buff&=(~data);
}



/**
  *  @brief ��595���㲢�ر����
  */

void HC595_reset(void)
{
	HC595_DE;
	out_buff=0;
	new_buff=0;
}

/**
  *  @brief ��չ�����ʼ��
	*	 @param none
  */
void HC595_init(void)
{
	HC595_reset();
	HAL_SPI_Transmit(&hspi1, ((uint8_t*)&out_buff), 3, 1000);
	HC595_RCK();
	HC595_EN;
}

/**
  *  @brief �����������595
	*	 @param none
  */

void HC595_updata(void)
{
	if(new_buff != out_buff)
	{
		out_buff = new_buff;
		HAL_SPI_Transmit(&hspi1, ((uint8_t*)&out_buff), 3, 1000); //((uint8_t*)&data)+1
		HC595_RCK();
	}
}
