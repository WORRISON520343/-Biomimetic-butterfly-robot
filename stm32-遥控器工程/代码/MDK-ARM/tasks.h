#ifndef _TASKS_H_
#define	_TASKS_H_
#include "stm32f10x.h"
#include "struct_all.h"

/******************************************************************************
							全局变量声明
*******************************************************************************/ 
extern uint8_t Bsp_Int_Ok; 

/******************************************************************************
							全局函数声明
*******************************************************************************/ 
void BSP_Int(void);
void Task_500HZ(void);
void Task_100HZ(void);
void Task_25HZ(void);
void Task_5HZ(void);
void Task_1HZ(void);

#endif
