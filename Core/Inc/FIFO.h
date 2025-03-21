#ifndef FIFO_H
#define FIFO_H

#include "main.h"
#define RINGBUFF_LEN 256

typedef struct
{
    uint8_t head;           
    uint8_t tail;
    uint8_t lenght;
    uint8_t ring_buff[RINGBUFF_LEN];
}ringbuff_t;

extern ringbuff_t RC_buff;
extern ringbuff_t PC_buff;

extern void ringbuff_init(ringbuff_t* name);
extern uint8_t write_ringbuff(ringbuff_t* name,uint8_t* data,uint8_t lenght);
extern uint8_t Read_RingBuff(ringbuff_t* name,uint8_t* data,uint8_t lenght);
extern uint8_t move_ringbuff_head(ringbuff_t* name,uint8_t lenght);

#endif
