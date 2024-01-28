/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.4.0
  * @date    10/15/2010
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_exti.h"
#include "stm32f10x.h"
#include "tasks.h"

/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/******************************************************************************/
uint8_t Count_2ms,Count_10ms,Count_40ms,Count_200ms;

void TIM3_IRQHandler(void)//Timer3中断
{
	if(TIM3->SR & TIM_IT_Update)
	{     
		TIM3->SR = ~TIM_FLAG_Update;//清除中断标志
		
		if( Bsp_Int_Ok == 0 )	return;//硬件未初始化完成，则返回
		
		Timer3_Count++;
		Count_2ms++;
		Count_10ms++;
		Count_40ms++;
		Count_200ms++;
		//Count_1000ms++;
	}
}

void EXTI15_10_IRQHandler(void)//Nrf2401中断,PC13
{	
	if(EXTI_GetITStatus(EXTI_Line13) != RESET)   //Nrf2401中断,PC13
	{
		EXTI_ClearITPendingBit(EXTI_Line13);
		NRF24L01_IRQ();
	}
	
	
	if(EXTI_GetITStatus(EXTI_Line15) != RESET )   //左侧按键PC15中断
	{
		static uint32_t Key_Delay=0;
		EXTI_ClearITPendingBit(EXTI_Line15);
		if(Key_Delay<Timer3_Count-100) 
		{
			Key_Delay = Timer3_Count;
			roll_aux -= 20;
			if(roll_aux<1000)
			{roll_aux=1000;}
		}
	}
	
	if(EXTI_GetITStatus(EXTI_Line11) != RESET)   //右侧按键PB11
	{
		static uint32_t Key_Delay=0;
		EXTI_ClearITPendingBit(EXTI_Line11);
		if(Key_Delay<Timer3_Count-100) 
		{
			Key_Delay = Timer3_Count;
			roll_aux += 20;
			if(roll_aux>2000)
			{roll_aux=2000;}
		}
	}
	
	if(EXTI_GetITStatus(EXTI_Line10) != RESET)   //左侧stick1=PB10中断
	{
		static uint32_t Key_Delay=0;
		EXTI_ClearITPendingBit(EXTI_Line10);
		if(Key_Delay<Timer3_Count-100) 
		{
			Key_Delay = Timer3_Count;
			pitch_aux -= 20;
			if(pitch_aux<1000)
			{pitch_aux=1000;}
		}
	}
	
}


void EXTI9_5_IRQHandler(void)//右边的stick2按键中断 ，A8
{	
	static uint32_t Key_Delay=0;
	static uint8_t key_count=0;
	if(EXTI_GetITStatus(EXTI_Line8) != RESET) 
	{
		
		EXTI_ClearITPendingBit(EXTI_Line8);
		
		if(Key_Delay<Timer3_Count-100)//消抖
		{
			Key_Delay = Timer3_Count;
//			if(Fun)
//				Fun=0;
//			else
//				Fun=1;
//			  if(key_count==2) {key_count=0;}
			pitch_aux += 20;
			if(pitch_aux>2000)
			{pitch_aux=2000;}
			
//			  if(key_count==0)
//				{
//					pid[0].kp = 4.9f; //roll pitch
//					pid[0].ki = 0.02f;
//					pid[0].kd = 8.0f;

//					pid[1].kp = 1.4f; //gyro_roll gyro_pitch
//					pid[1].ki = 0.0f;
//					pid[1].kd = 2.0f;

//					pid[2].kp = 2.0f;  //gyro_yaw
//					pid[2].ki = 0.1f;
//					pid[2].kd = 0.4f;
//				}
//				else
//				{
//					pid[0].kp = 4.9f; //roll pitch
//					pid[0].ki = 0.02f;
//					pid[0].kd = 8.0f;

//					pid[1].kp = 1.4f; //gyro_roll gyro_pitch
//					pid[1].ki = 0.0f;
//					pid[1].kd = 2.0f;

//					pid[2].kp = 4.0f;  //gyro_yaw
//					pid[2].ki = 0.1f;
//					pid[2].kd = 0.4f;
//				}
//				key_count++;
//				Print_MSP_SET_PID();  //按键控制发送PID参数给四轴
		}
		
		
	}
}


/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
