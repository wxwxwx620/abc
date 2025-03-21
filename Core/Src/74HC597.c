/**
  ******************************************************************************
  * @file   74HC597.c
  * @brief  74HC597?????????
 * @author  ?????
 * @ver   1.2
 * @date   2021.12.20
  ******************************************************************************
  * @attention
  * ???exi_read????????10??50ms??????????'??get_exi????
  * v1.1 ????Z?????
 * v1.2 fix????????????????
  ******************************************************************************
  */
#include "74HC597.h"
#include "spi.h"


uint16_t exi_buf=0;

/**                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                
 *@brief  ????h??74HC597???????
 *@param  none
 *@example none
 *@retval none
  */

void HC597_RCK(void)
{
 HC597_R0;

 HC597_R1;

 HC597_L0;

 HC597_L1;
}


/**
  *@brief  ?????????????????
 *@param  none
 *@example none
 *@retval none
  */

void exi_read(void)
{
 HC597_RCK();
 HAL_SPI_Receive(&hspi1, (uint8_t*)&exi_buf, 2, 1000);
}


/**
  *@brief  ?????????????????????????exi_read()
 *@param  num ?????
 *@example get_exi(IN_1);
 *@retval ??j?????? 1:??(???) 0:??(???)
  */

uint8_t get_exi(uint16_t num)
{
 return exi_buf>>num&0x1;
}
