/**
  ******************************************************************************
  * @file			74HC595.c
  * @brief		74HC595扩展输出接口
	*	@author		肖智中
	*	@ver			2.0
	*	@date			2021.12.15
  ******************************************************************************
  * @attention
  *	v1.1 驱动方式由GPIO变更为SPI
	*	此次HC595_updata已在定时器10中50ms定时调用，直接使用HC595_add、HC595_del即可
  *	v2.0 增加初始化过程，fix上电误输出
  ******************************************************************************
  */

#include "74HC595.h"
#include "spi.h"


volatile uint32_t out_buff=0,new_buff=0;



/**
  * @brief	触发一次595锁存时钟
  */

void HC595_RCK(void)
{
	HC595_R1;

	HC595_R0;
}


/**
  *	@brief		向缓存添加数据，后需调用HC595_updata输出到端口
	*	@param		data 要添加的数据
	*	@example	HC595_add(OUT_1|OUT_2);
  */

void HC595_add(uint32_t data)
{
	new_buff|=data;
}



/**
  * @brief		向缓存删除数据，后需调用HC595_updata输出到端口
	*	@param		data 要删除的数据
	*	@example	HC595_del(OUT_1|OUT_2);
  */

void HC595_del(uint32_t data)
{
	new_buff&=(~data);
}



/**
  *  @brief 将595清零并关闭输出
  */

void HC595_reset(void)
{
	HC595_DE;
	out_buff=0;
	new_buff=0;
}

/**
  *  @brief 扩展输出初始化
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
  *  @brief 将数据输出到595
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
