
#include "spi.h"
//#include "usart.h"


/*****************************************************************************
* 函  数：void SPI_GPIO_Init(void)
* 功  能：配置SI24R1的 SCK、MISO、MOSI引脚，以及SPI2初始化
* 参  数：无
* 返回值：无
* 备  注：调试SPI通信时一定要分清主机从机模式
*         主机从机模式的 空闲状态 电平
*		  2.4G模块通信时，SPI速率一般不大于10Mbps
*****************************************************************************/
void SPI_GPIO_Init(void)
{
	SPI_InitTypeDef   SPI_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure;
	

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);
	//配置SPI的SCK，MISO和MOSI引脚为复用推挽模式
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	//SPI1的SCK,,MOSI对应PA5，PA7
	//SCK、MOSI,MISO配置为复用推挽模式
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_5|GPIO_Pin_7|GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	//MISO PA6
	//GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
  //GPIO_Init(GPIOA,&GPIO_InitStructure);
//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource5,GPIO_AF_SPI1); //PB3复用为 SPI1
//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_SPI1); //PB4复用为 SPI1
//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_SPI1); //PB5复用为 SPI1

	//RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1,ENABLE);//复位SPI1
	//RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1,DISABLE);//停止复位SPI1
	//SPI_Cmd(SPI1, DISABLE); //失能SPI外设
	SPI_InitStructure.SPI_Mode=SPI_Mode_Master;	//配置为主机模式
	SPI_InitStructure.SPI_NSS=SPI_NSS_Soft;		//NSS软件管理
	SPI_InitStructure.SPI_CPHA=SPI_CPHA_1Edge;	// SPI_CPHA_1Edge 第一个时钟沿捕获; SPI_CPHA_2Edge
	SPI_InitStructure.SPI_CPOL=SPI_CPOL_Low;	//空闲状态为低电平
	SPI_InitStructure.SPI_DataSize=SPI_DataSize_8b;						//8位数据帧
	SPI_InitStructure.SPI_BaudRatePrescaler=SPI_BaudRatePrescaler_16; 	//8,16, 256 //SPI波特率8分频 	36/8=4.5M
	SPI_InitStructure.SPI_Direction=SPI_Direction_2Lines_FullDuplex;	//全双工模式
	SPI_InitStructure.SPI_FirstBit=SPI_FirstBit_MSB;					//数据高位先行
	SPI_InitStructure.SPI_CRCPolynomial=7;								//CRC计算多项式
	SPI_Init(SPI1,&SPI_InitStructure);

	SPI_Cmd(SPI1,ENABLE);	//SPI1使能
	//SPI1_RW(0xff);//启动传输	
}



//SPI1速度设置函数
//SPI速度=fAPB2/分频系数
//@ref SPI_BaudRate_Prescaler:SPI_BaudRatePrescaler_2~SPI_BaudRatePrescaler_256  
//fAPB2时钟一般为84Mhz：
void SPI1_SetSpeed(uchar SPI_BaudRatePrescaler)
{
	//printf("设置spi1的速度***");
  assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));//判断有效性
	SPI1->CR1&=0XFFC7;//位3-5清零，用来设置波特率
	SPI1->CR1|=SPI_BaudRatePrescaler;	//设置SPI1速度 
	SPI_Cmd(SPI1,ENABLE); //使能SPI1
	//printf("设置spi1的速度完成");
} 


uchar SPI1_RW(uchar data)
{
	//printf("SPI1 RW开始");
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET){}//等待发送区空  
	SPI_I2S_SendData(SPI1, data); //通过外设SPIx发送一个byte  数据
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET){} //等待接收完一个byte  
	//printf("SPI1 RW结束");
	return SPI_I2S_ReceiveData(SPI1); //返回通过SPIx最近接收的数据	
}


/*
//使用软件模拟SPI
uchar SPI2_WriteReadByte(uchar byte)
{
	uchar bit_ctr;
	for(bit_ctr=0;bit_ctr<8;bit_ctr++)  // 输出8位
	{
		if((uchar)(byte&0x80)==0x80)
		NRF_MOSI=1; 			// MSB TO MOSI
		else 
			NRF_MOSI=0; 
		byte=(byte<<1);					// shift next bit to MSB
		NRF_SCK=1;
		byte|=NRF_MISO;	        		// capture current MISO bit
		NRF_SCK=0;
	}
	return byte;
}
*/
