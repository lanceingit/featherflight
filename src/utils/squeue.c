
#include <string.h>

#include "squeue.h"



void sqInit(squeue_t *q, uint8_t* buf, uint16_t buflen)
{
    q->head = 0;
    q->tail = 0;
    q->data = buf;
    q->buflen = buflen;
}

int sqPut(squeue_t *q, uint8_t p)
{
    //Check if the queue is full
    if (((q->head+1)%q->buflen) == q->tail)
        return SQ_ERROR;
  
    //Add the new item
    memcpy(&q->data[q->head], p, sizeof(CRTPPacket));
    q->head = (q->head+1)%q->buflen;
  
    return SQ_OK;
}

int sqGet(squeue_t *q, uint8_t *p)
{
    //Check if the queue contains at least one element
    if (q->head==q->tail)
        return SQ_ERROR;
  
    //Get one element of the queue
    memcpy(p, &q->data[q->tail], sizeof(CRTPPacket));
    q->tail = (q->tail+1)%SQUEUE_SIZE;

    return SQ_OK;
}
