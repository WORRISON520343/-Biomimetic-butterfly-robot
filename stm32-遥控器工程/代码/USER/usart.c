/*******************************************************************************************
										    声 明
    本项目代码仅供个人学习使用，可以自由移植修改，但必须保留此声明信息。移植过程中出现其他
	
不可估量的BUG，天际智联不负任何责任。请勿商用！

* 程序版本：V1.01
* 程序日期：2018-8-18
* 程序作者：愤怒的小孩
* 版权所有：西安天际智联信息技术有限公司
*******************************************************************************************/
#include "stm32f10x.h"
//#include "stdio.h"
#include "usart.h"

uint8_t TxCount=0;
uint8_t Count=0;

uint8_t Line0,Line1;	//串口接收双缓冲切换

static uint8_t TxBuff[256];	//串口发送缓冲区
//volatile uint8_t RxBuffer[50];//串口接收缓冲区
uint8_t RxBuff[2][50];		//串口接收缓冲区

/*****************************************************************************
* 函  数：void USART_init(uint32_t baudrate)
* 功  能：Usart1初始化为双工模式
* 参  数：baudrate 波特率
* 返回值：无
* 备  注：对于连续的数据帧的接收 接收中断与空闲中断配合能解决丢包问题，
          具体接收方式见stm32f1x_it.c 中的串口中断处理;
*****************************************************************************/
void USART_init(uint32_t baudrate)
{
	GPIO_InitTypeDef GPIO_InitStruct;   //定义GPIO结构体变量
	USART_InitTypeDef USART_InitStruct;   //定义串口结构体变量
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_USART1,ENABLE);   //使能GPIOA、USART1的时钟
	
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_9;   //配置TX引脚
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF_PP;   //配置PA9为复用推挽输出
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;   //配置PA9速率
	GPIO_Init(GPIOA,&GPIO_InitStruct);   //GPIO初始化函数
	
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_10;   //配置RX引脚
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_IN_FLOATING;   //配置PA10为浮空输入
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;   //配置PA10速率
	GPIO_Init(GPIOA,&GPIO_InitStruct);   //GPIO初始化函数
	
	
	USART_InitStruct.USART_Mode=USART_Mode_Tx|USART_Mode_Rx;   //发送接收模式
	USART_InitStruct.USART_Parity=USART_Parity_No;   //无奇偶校验
	USART_InitStruct.USART_BaudRate=baudrate;   //波特率
	USART_InitStruct.USART_StopBits=USART_StopBits_1;   //停止位1位
	USART_InitStruct.USART_WordLength=USART_WordLength_8b;   //字长8位
	USART_InitStruct.USART_HardwareFlowControl=USART_HardwareFlowControl_None;   //无硬件数据流控制
	USART_Init(USART1,&USART_InitStruct);   //串口初始化函数
	
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);		//串口接收中断
	USART_ITConfig(USART1,USART_IT_IDLE,ENABLE);		//串口空闲中断
	
	USART_Cmd(USART1,ENABLE);   //使能USART1
}



/******************************************************************************
函数原型：	void USART1_IRQHandler(void)
功    能：	串口中断
*******************************************************************************/ 
void USART1_IRQHandler(void)
{
	if(USART_GetFlagStatus(USART1, USART_FLAG_ORE) != RESET)//ORE中断
	{
		USART_ReceiveData(USART1);
	}
	
	if(USART1->SR & USART_SR_TC)//发送中断
	{
		USART1->DR = TxBuff[TxCount++];//写DR清除中断标志          
		if(TxCount == Count)
		{
			USART1->CR1 &= ~USART_CR1_TXEIE;//关闭TXE中断
		}
	}
	
	if(USART1->SR & USART_SR_RXNE)//接收中断 
	{
		volatile int8_t com_data ;
		com_data = USART1->DR;
	}
}




/*****************************************************************************
* 函  数：int fputc(int ch, FILE *f)
* 功  能：重定向 printf()函数
* 参  数：ch 要发送的数据
* 返回值：无
* 备  注：无
*****************************************************************************/
/*
int fputc(int ch,FILE *f)   //printf重定向函数
{
	USART_SendData(USART1,(uint8_t)ch);   //发送一字节数据
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);   //等待发送完成
	return ch;
}
*/
/*****************************************************************************
* 函  数：void usart_send(uint8_t *data,uint8_t len)
* 功  能：Usart发送指定长度数据
* 参  数：*data 要发送数据的地址
*         len   要发送数据的长度
* 返回值：无
* 备  注：无
*****************************************************************************/
void usart_send(uint8_t *data,uint8_t len)
{
	uint8_t i;
	
	for(i=0;i<len;i++)
	{
		while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);  //如果发送标志位没有在发送的时候
		USART_SendData(USART1,*(data+i));
		//USART1->DR = data;
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC) == RESET);
	}
}


/*void usart_send_mpu6050(int16_t data)
{
	
}
*/

//以HEX的形式输出U8型数据
void PrintHexU8(uint8_t data)
{
	/*TxBuff[Count++] = data;  
	if(!(USART1->CR1 & USART_CR1_TXEIE))
	USART_ITConfig(USART1, USART_IT_TXE, ENABLE); //打开TXE中断
	*/
	//USART1->DR = data ;
	
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);  //如果发送标志位没有在发送的时候
	USART_SendData(USART1, data);
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC) == RESET);
	//usart_send(TX_DATA, 10);
}
//以HEX的形式输出S16型数据
void PrintHexS16(int16_t num)
{
	PrintHexU8((uint8_t)(num & 0xff00) >> 8);//先发送高8位，再发送低8位
	PrintHexU8((uint8_t)(num & 0x00ff));
}
//以字符的形式输出S8型数据
void PrintS8(int8_t num)
{
	uint8_t  bai,shi,ge;
	if(num<0)
	{
		PrintHexU8('-');
		num=-num;
	}
	else 
		PrintHexU8(' ');	
	bai=num/100;
	shi=num%100/10;
	ge =num%10;
	PrintHexU8('0'+bai);
	PrintHexU8('0'+shi);
	PrintHexU8('0'+ge);
}
//以字符的形式输出U8型数据
void PrintU8(uint8_t num)
{
	uint8_t  bai,shi,ge;
	bai=num/100;
	shi=num%100/10;
	ge =num%10;
	PrintHexU8('0'+bai);
	PrintHexU8('0'+shi);
	PrintHexU8('0'+ge);
}
//以字符的形式输出S16型数据 
void PrintS16(int16_t num)
{	
	uint8_t w5,w4,w3,w2,w1;
	if(num<0)
	{
		PrintHexU8('-');
		num=-num;
	}
	else 
		PrintHexU8(' ');
	
	w5=num%100000/10000;
	w4=num%10000/1000;
	w3=num%1000/100;
	w2=num%100/10;
	w1=num%10;
	PrintHexU8('0'+w5);
	PrintHexU8('0'+w4);
	PrintHexU8('0'+w3);
	PrintHexU8('0'+w2);
	PrintHexU8('0'+w1);
}
//以字符的形式输出U16型数据
void PrintU16(uint16_t num)
{	
	uint8_t w5,w4,w3,w2,w1;
	
	w5=num%100000/10000;
	w4=num%10000/1000;
	w3=num%1000/100;
	w2=num%100/10;
	w1=num%10;
	PrintHexU8(' ');
	PrintHexU8('0'+w5);
	PrintHexU8('0'+w4);
	PrintHexU8('0'+w3);
	PrintHexU8('0'+w2);
	PrintHexU8('0'+w1);
}
//输出字符串
void PrintString(uint8_t *s)
{
	uint8_t *p;
	p=s;
	while(*p != '\0')
	{
		PrintHexU8(*p);
		p++;
	}
}

//串口发送数组中前len个数据
void Uart_Send(uint8_t *s ,uint8_t len)
{
	while(len)
	{
		//PrintHexU8(*s);
		PrintU8(*s);
		s++;
		len--;
	}
}


