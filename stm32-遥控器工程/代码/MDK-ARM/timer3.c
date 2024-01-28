//***************************************************************************************
#include "struct_all.h"



uint32_t Timer3_Count = 0;//记录Timer3中断次数
uint16_t Timer3_Frequency;//Timer3中断频率

/******************************************************************************
函数原型：	void Timer3_Init(uint16_t Handler_Frequency)
功    能：	初始化定时器3
参    数：   Handler_Frequency为Timer3中断频率
*******************************************************************************/ 
void Timer3_Init(uint16_t Handler_Frequency)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	
	Timer3_Frequency = Handler_Frequency;
	TIM_DeInit(TIM3);
	TIM_TimeBaseStructure.TIM_Period = 1000*1000/Handler_Frequency ;//装载值
	TIM_TimeBaseStructure.TIM_Prescaler = 72-1;	//分频系数
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //不分割时钟
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//向上计数
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);
	TIM_ClearFlag(TIM3,TIM_FLAG_Update);//清除中断标志
	
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM3,ENABLE);//使能定时器3
}
