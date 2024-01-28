#ifndef _TIMER3_H_
#define _TIMER3_H_
#include "stm32f10x.h"
#include "stm32f10x_tim.h"


/******************************************************************************
							全局变量声明
*******************************************************************************/ 
extern uint32_t Timer3_Count;
extern uint16_t Timer3_Frequency;
extern uint8_t Count_2ms,Count_10ms,Count_40ms,Count_200ms,Count_1000ms;

/******************************************************************************
							全局函数声明
*******************************************************************************/ 
void Timer3_Init(uint16_t Handler_Frequency);

#endif
