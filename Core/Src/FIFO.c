#include "FIFO.h"

ringbuff_t RC_buff;
ringbuff_t PC_buff;

void ringbuff_init(ringbuff_t* name)
{
   //初始化相关信息
   name->head = 0;
   name->tail = 0;
   name->lenght = 0;
}

uint8_t write_ringbuff(ringbuff_t* name,uint8_t* data,uint8_t lenght)
{
   if(lenght+name->lenght > RINGBUFF_LEN || data==NULL)
    {
      return 0;
    }
		
		for(uint8_t i=0;i<lenght;i++)
		{
			name->ring_buff[name->tail++]=data[i];
			name->lenght ++;
		}

		return 1;
}

uint8_t Read_RingBuff(ringbuff_t* name,uint8_t* data,uint8_t lenght)
{
   if(lenght > name->lenght || data==NULL)
    {
       return 0;
    }
	 uint8_t head=name->head;
		for(uint8_t i=0;i<lenght;i++)
		{
			data[i]=name->ring_buff[head++];
		}
		
   return 1;
}

uint8_t move_ringbuff_head(ringbuff_t* name,uint8_t lenght)
{
   if(lenght > name->lenght)
    {
       return 0;
    }
	 //memcpy(data,&name->ring_buff[name->head],(unsigned int)lenght);
			name->head+=lenght;
			name->lenght-=lenght;
		
   return 1;
}




