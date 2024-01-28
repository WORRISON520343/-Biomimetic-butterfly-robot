
#include "struct_all.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_exti.h"
/*
#include "led_key.h"
#include "timer3.h"
#include "adc.h"
*/

uint8_t offset=1;//开机校正摇杆及旋钮中点
int16_t offset0,offset1,offset2,offset3;
int16_t offset4,offset5,offset6;
uint8_t Mode=0,Fun=0;
uint16_t roll_aux=1500, pitch_aux=1500;

/******************************************************************************
函数原型:	static void Delay_led(uint16_t n)
功　　能:	延时
*******************************************************************************/ 
static void Delay_led(uint16_t n)
{	
	uint16_t i,j;
	for(i=0;i<n;i++)
		for(j=0;j<8888;j++);
} 

/******************************************************************************
函数原型:	void LED_Init(void)
功　　能:	LED初始化
*******************************************************************************/ 
void LED_KEY_Init(void)
{
	GPIO_InitTypeDef GPIO_Structure;
	EXTI_InitTypeDef EXTI_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO, ENABLE);				 
//=============================================================================
//LED -> PB3  PB4
//BEEP  PC14
//KEY_R PC15
//=============================================================================	
	//重映射需要先使能AFIO时钟
  //RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO ,ENABLE);
	//只关闭JTAG而保留SWD 
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
	
	//以下是针对PC14 PC15作为普通IO口的设置，也有可能不需要，试一试
  PWR_BackupAccessCmd( ENABLE );/* 允许修改RTC和后备寄存器*/
  RCC_LSEConfig( RCC_LSE_OFF ); /* 关闭外部低速时钟,PC14+PC15可以用作普通IO*/
  BKP_TamperPinCmd(DISABLE);  /* 关闭入侵检测功能,PC13可以用作普通IO*/
	PWR_BackupAccessCmd(DISABLE);/* 禁止修改RTC和后备寄存器*/
	
	GPIO_Structure.GPIO_Pin =  Bee_Pin;	//蜂鸣器
	GPIO_Structure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Structure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_Init(Bee_GPIO, &GPIO_Structure);
		
	GPIO_Structure.GPIO_Pin =  LEDRed_Pin | LEDGreen_Pin;	//LED0 RED
	GPIO_Structure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Structure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_Init(LEDRed_GPIO, &GPIO_Structure);
	GPIO_Structure.GPIO_Pin =  LEDRed_Pin | LEDGreen_Pin;	//LED0 green
	GPIO_Structure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Structure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_Init(LEDGreen_GPIO, &GPIO_Structure);
	
	
	GPIO_Structure.GPIO_Pin=Key_Right_Pin;	//MODE按键,右边按键
  GPIO_Structure.GPIO_Mode=GPIO_Mode_IPU;
  GPIO_Init(Key_Right,&GPIO_Structure);
	
  GPIO_Structure.GPIO_Pin=Key_Left_Pin;	//FUN按键，左边按键
  GPIO_Structure.GPIO_Mode=GPIO_Mode_IPU;
  GPIO_Init(Key_Left,&GPIO_Structure);
	
	GPIO_Structure.GPIO_Pin=Key_Stick_Left_Pin;	//stick1按键,左边遥控按键
  GPIO_Structure.GPIO_Mode=GPIO_Mode_IPU;
  GPIO_Init(Key_Stick_Left,&GPIO_Structure);
	
  GPIO_Structure.GPIO_Pin=Key_Stick_Right_Pin;	//stick2按键,左边遥控按键
  GPIO_Structure.GPIO_Mode=GPIO_Mode_IPU;
  GPIO_Init(Key_Stick_Right,&GPIO_Structure);
	
	
	//对按键设置的中断
	  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource11); //PB11，右侧按键的中断源
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,GPIO_PinSource15); //PC15，左侧按键的中断源
		GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource10); //PB10，stick1,左边的遥控按键的中断源
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource8); //PA8，左侧stick2按键的中断源
    EXTI_InitStructure.EXTI_Line=EXTI_Line11|EXTI_Line15|EXTI_Line10|EXTI_Line8;
		//EXTI_InitStructure.EXTI_Line=EXTI_Line8;
    EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;//外部中断
    EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Falling;//下降沿触发,按键都是接地的
    EXTI_InitStructure.EXTI_LineCmd=ENABLE;
    EXTI_Init(&EXTI_InitStructure);
	


}

/******************************************************************************
函数原型:	void LED_ON_OFF(void)
功　　能:	LED开机闪烁
*******************************************************************************/ 
void LED_ON_OFF(void)
{
	uint8_t i ;
	for(i=0;i<3;i++)
	{
		LEDRed_ON;
		LEDGreen_OFF;
		Bee_ON;
		Delay_led(100);
		LEDGreen_ON;
		LEDRed_OFF;
		Bee_OFF;
		Delay_led(100);	
	}	
	Bee_OFF;
	LEDRed_OFF;
	LEDGreen_OFF;
	//Delay_led(100);
}

/******************************************************************************
函数原型：	void LEDRed_ON_Frequency(uint8_t Light_Frequency)
功    能：	让LED按照某个频率闪烁（需要循环扫描该函数）
参    数：   Light_Frequency，闪烁频率（HZ）
*******************************************************************************/ 
void LED3_ON_Frequency(uint8_t Light_Frequency)
{
	uint16_t time_count;
	static uint8_t Light_On;
	static uint32_t Last_time,Now_time;
	
	Now_time = Timer3_Count;
	if( Light_On )
	{
		time_count = (uint16_t)(Timer3_Frequency / Light_Frequency / 2 );	
		
		if( Now_time - Last_time >= time_count)
		{
			LEDRed_OFF;
			Light_On=0;
			Last_time = Timer3_Count;
		}
	}
	else
	{
		time_count = (uint16_t)(Timer3_Frequency / Light_Frequency / 2 );
		
		if( Now_time - Last_time >= time_count)
		{
			LEDRed_ON;
			Light_On=1;
			Last_time = Timer3_Count;
		}
	}
}

/******************************************************************************
函数原型：	void LED_Show(void)
功    能：	LED指示灯闪烁
*******************************************************************************/ 
void LED_Show(void)
{
	
	if( (Battery_Fly>200 && Battery_Fly<330) || (Battery_Rc>200 && Battery_Rc<360) )//低压报警
	{
		//Bee_ON;
		LED3_ON_Frequency(10);//红色LED10HZ闪烁
	}
	
	else if(Mode)//锁尾模式
	{
		Bee_OFF;
		LED3_ON_Frequency(4);//红色LED4HZ闪烁
	}
	else//非锁尾模式
	{
		Bee_OFF;
		LED3_ON_Frequency(1);//红色LED1HZ闪烁
	}
}




/******************************************************************************
函数原型:	void Do_Offset(void)
功　　能:	摇杆校准
*******************************************************************************/ 
void Do_Offset(void)
{
	if(offset)
	{		
		static uint8_t count=0;
		static int32_t count0,count1,count2,count3;
		static int32_t count4,count5,count6;
		if(count==0)
		{
			offset0 = 0;offset1 = 0;offset2 = 0;offset3 = 0;
			offset4 = 0;offset5 = 0;offset6 = 0;
			count0  = 0;count1  = 0;count2  = 0;count3  = 0;
			count4  = 0;count5  = 0;count6  = 0;
			count = 1;
			return;
		}
		else
		{
			count++;
			count0 += Rc.THROTTLE;
			count1 += Rc.YAW;
			count2 += Rc.PITCH;
			count3 += Rc.ROLL;
			count4 += Rc.AUX1;
			count5 += Rc.AUX2;
			count6 += Rc.AUX3;
		}
		if(count==51)
		{
			count--;
			offset0 = count0 / count - 1500;
			offset1 = count1 / count - 1500;
			offset2 = count2 / count - 1500;
			offset3 = count3 / count - 1500;
			offset4 = 0;//count4 / count - 1500;
			offset5 = 0;//count5 / count - 1500;
			offset6 = 0;//count6 / count - 1500;
			count = 0;
			offset = 0;
			Bee_ON;
			Delay_led(200);
		}
		
	}
}
