#include "indicatorlight.h"
#include "gpio.h"
#include "74HC595.h"
#include "remote_control.h"

void Run_Light(void)
{	
	if(INNER.TARGET_SPEED==0)	
	{
		if(INNER.TARGET_RPS==0)
		{
			HC595_del(OUT_1|OUT_2|OUT_3|OUT_4);//Í£Ö¹×´Ì¬
		}
		else if(INNER.TARGET_RPS>0)//Ë³Ê±Õë0x60 0110 0000
		{
			HC595_add(OUT_2|OUT_3);
			HC595_del(OUT_1|OUT_4);
		}
		else//ÄæÊ±Õë0x90 1001 0000
		{
			HC595_add(OUT_1|OUT_4);
			HC595_del(OUT_2|OUT_3);
		}
	}
	else
	{
		if((INNER.TARGET_ANGLE>=450)&&(INNER.TARGET_ANGLE<1350)) {HC595_add(OUT_1|OUT_2);HC595_del(OUT_3|OUT_4);}       //Ç°½ø 0000 0011
		else if((INNER.TARGET_ANGLE>=2250)&&(INNER.TARGET_ANGLE<3150)) {HC595_add(OUT_3|OUT_4);HC595_del(OUT_1|OUT_2);} //ºóÍË 0000 1100
		else if((INNER.TARGET_ANGLE>=1350)&&(INNER.TARGET_ANGLE<2250)) {HC595_add(OUT_1|OUT_3);HC595_del(OUT_2|OUT_4);} //×óÒÆ	0x05	0000 0101
		else {HC595_add(OUT_2|OUT_4);HC595_del(OUT_1|OUT_3);}                                                           //ÓÒÒÆ 0x0A	0000 1010				
	}

}







