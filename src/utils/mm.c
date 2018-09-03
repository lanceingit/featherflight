#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "mm.h"
#include "debug.h"


static void* heap;
static uint16_t heap_size;
static uint8_t block_num;
static uint16_t heap_free;
static void* heap_pos;


void mm_init(void* h, uint16_t size)
{
    heap = h;
    heap_size = size;
    block_num = 0;
    heap_free = size;
    heap_pos = h;    
}

void mm_reset(void)
{
    block_num = 0;
    heap_free = heap_size;
    heap_pos = heap;    
}


void *mm_new(size_t s)
{
    void *mm = NULL;

    //DEBUG_PRINTF("s:%d, free:%d\r\n", s, heap_free);
    
    if(s > 0 && s < heap_free)
    {
        uint8_t* p=heap_pos;        
        mm = heap_pos;
        p+=s;
        heap_pos = p;
        heap_free -= s;
        block_num++;
        
//        DEBUG_PRINTF("**********************\r\n");
//        DEBUG_PRINTF("pos:%x\r\n", heap_pos);
//        DEBUG_PRINTF("mm:%x\r\n", mm);
//        DEBUG_PRINTF("heap_free:%d\r\n", heap_free);
//        DEBUG_PRINTF("block_num:%d\r\n", block_num);
//        DEBUG_PRINTF("**********************\r\n");
    }

	return mm;
}


void mm_delete(void* m)
{
}




