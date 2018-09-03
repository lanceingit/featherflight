#pragma once

#ifdef __cplusplus
 extern "C" {
#endif 
     
     
void mm_init(void* h, uint16_t size);     
void mm_reset(void);
void *mm_new(size_t s);
void mm_delete(void* m);

     
     
#ifdef __cplusplus
}
#endif

