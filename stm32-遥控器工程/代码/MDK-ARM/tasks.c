
#include "tasks.h"
#include "filter.h"

/******************************************************************************
函数原型:	static void Nvic_Init(void)
功　　能:	NVIC初始化
*******************************************************************************/ 
static void Nvic_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	//NVIC_PriorityGroup 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//优先级分组
    //Timer3
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//先占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;//从优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	//Nrf2401中断
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn; //NRF_IRQ=PC13, 左侧按键PC15,右侧PB11，左侧stick1=PB10
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	/*
	//串口
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	*/
	
//	//MODE按键
//	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn; //可以接引脚PA1，PB1可用
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
	
	//下面的有可能是个问题，不确定，需要试一试
	//由于硬件上上的设计问题，导致左侧按键PC15,右侧PB11，左侧stick1=PB10都于nrf24L01在同一个中断线上
	//为了保证nrf24L01的中断正常使用，暂时舍弃这几个按键按键中断功能
	
	//FUN按键 右侧stick2按键
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;  //PA8，右侧stick2按键
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	
}

/******************************************************************************
函数原型：	void BSP_Int(void)
功    能：	硬件驱动初始化
*******************************************************************************/ 
void BSP_Int(void)
{
	LED_KEY_Init();//LED\按键初始化
	LED_ON_OFF();//LED闪烁
	
	USART_init(115200);	//串口初始化：波特率115200，8位数据，1位停止位，禁用奇偶校验
	PrintString("\r\n LED USART init!!! \r\n");
	Timer3_Init(500);//Timer3中断500HZ
	PrintString("\r\n Timer3 init!!! \r\n");
	Nvic_Init();//中断优先级初始化	
	PrintString("\r\n Nvic init!!! \r\n");
	
	
	ADC1_Init();//ADC及DMA初始化
	PrintString("\r\n ADC1 DMA init!!! \r\n");

	//SPI2_Init();//SPI2初始化
	//NRF24L01_Init(40,TX);//2401选择40通道，发送模式
	NRF24L01_IO_Init(); //里面包含了SPI1的初始化
	PrintString("\r\n NRF24L01 init!!! \r\n");
	NRF24L01_TX_Mode(); //发送模式
	//NRF24L01_Init(0,TX); //初始化为通道0，发射模式
 	NRF24L01_Check();//检测2401是否正常
	PrintString("\r\n nrf24L01 checked!!! \r\n");
	PrintString("\r\n Dream_Flyer_RC V1.1 \r\n");//版本号
	Bsp_Int_Ok = 1;
}

/******************************************************************************
函数原型：	void Task_500HZ(void)
功    能：	主循环中运行频率为500HZ任务
*******************************************************************************/ 
void Task_500HZ(void)
{
	//Rc_Filter(&Rc,&Filter_Rc);//计算遥控指令+滤波
	//Nrf_Connect();//NRF24L01连接函数
	
}

/******************************************************************************
函数原型：	void Task_100HZ(void)
功    能：	主循环中运行频率为100HZ任务
*******************************************************************************/ 
void Task_100HZ(void)
{
	//Print_MSP_RC('<');//发送遥控信号给飞控
	
}

/******************************************************************************
函数原型：	void Task_25HZ(void)
功    能：	主循环中运行频率为25HZ任务
*******************************************************************************/ 
void Task_25HZ(void)
{
	//Print_MSP_RC('>');//发送遥控信号给上位机
	LED_Show();//LED指示灯闪烁
	Rc_Filter(&Rc,&Filter_Rc);//计算遥控指令+滤波
	Nrf_Connect();//NRF24L01连接函数
	//Print_MSP_MOTOR();//发送电机转速给上位机
	//PrintString("\r\n 遥控器信号 \r\n");//版本号
}

/******************************************************************************
函数原型：	void Task_4HZ(void)
功    能：	主循环中运行频率为4HZ任务
*******************************************************************************/ 
void Task_5HZ(void)
{
	

  Print_MSP_RC('<');//发送遥控信号给飞控
	
	
	//static uint8_t turn_count;	
		//Print_MSP_RC('>');
//	turn_count++;
//	switch(turn_count)
//	{
//		case 1: PrintString("analog:"); Print_MSP_ANALOG();		break;
//		case 2: PrintString("ident:"); Print_MSP_IDENT();		break;
//		case 3: PrintString("analog"); Print_MSP_ANALOG();		break;				
//		case 4: PrintString("motor:"); Print_MSP_MOTOR_PINS();	
//				turn_count=0; 			break;
//	}
	
	
}


