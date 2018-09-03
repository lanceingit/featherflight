/**
 *
 * timer.h
 *
 * simple timer, delay and time block function 
 */


#pragma once

#ifdef __cplusplus
 extern "C" {
#endif

void Timer_init();
void Timer_disable();

uint64_t Timer_create(uint64_t us);
bool Timer_isTimeout(uint64_t t);
void Timer_delayUs(uint64_t us);
uint64_t Timer_getTime();
uint64_t Timer_elapsedTime(uint64_t* t);

#ifdef __cplusplus
}
#endif


//class Timer
//{
//public:
//
//    static void init();
//    static void disable();
//
//    static uint64_t create(uint64_t us);
//    static bool isTimeout(uint64_t t);
//    static void delayUs(uint64_t us);
//    static uint64_t getTime() { return _timerCnt*10; }
//    static uint64_t elapsedTime(uint64_t* t) { return _timerCnt*10 - *t; }
//    static void irs();
//
//private:
//    static volatile uint64_t _timerCnt;
//
//    Timer();
//    ~Timer();
//};


     

