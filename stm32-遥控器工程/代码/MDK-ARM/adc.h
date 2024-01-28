#ifndef   _ADC_H
#define   _ADC_H
#include "sys.h" 
#include "stm32f10x_adc.h"

/*
void Adc_Init(void); 				//ADC通道初始化
u16  Get_Adc(u8 ch); 				//获得某个通道值 
u16 Get_Adc_Average(u8 ch,u8 times);//得到某个通道给定次数采样的平均值  
*/

//采用DMA的方式，进行

/******************************************************************************
							宏定义
*******************************************************************************/ 
#define M 6 //6个ADC通道，四个遥控器通道，一个电池电压测量通道,一个基准电压通道

/******************************************************************************
							全局变量声明
*******************************************************************************/ 
extern uint16_t Battery_Fly,Battery_Rc;
extern uint16_t ADC_Value[M]; 

/******************************************************************************
							全局函数声明
*******************************************************************************/ 
void ADC1_Init(void);
void Voltage_Printf(void);
	


#endif
