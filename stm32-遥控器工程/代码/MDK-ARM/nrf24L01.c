#include "stm32f10x.h"
/*
#include "nrf24L01.h"
#include "spi.h"
#include "delay.h"
#include "stdio.h"
#include "stdlib.h"
#include "usart.h"
*/
#include "struct_all.h"
#include "stm32f10x_exti.h"
//第一、六组地址
static uint8_t TX_ADDRESS[5]={0x34,0x43,0x10,0x10,0x01}; //发送地址
static uint8_t RX_ADDRESS[5]={0x34,0x43,0x10,0x10,0x01}; //发送地址
//第二、七组地址
//static uint8_t TX_ADDRESS[5]={0x1A,0x3B,0x5C,0x7D,0x9E}; //发送地址
//static uint8_t RX_ADDRESS[5]={0x1A,0x3B,0x5C,0x7D,0x9E}; //发送地址
//第三、八组地址
//static uint8_t TX_ADDRESS[5]={0x22,0x44,0x66,0x88,0xAA}; //发送地址
//static uint8_t RX_ADDRESS[5]={0x22,0x44,0x66,0x88,0xAA}; //发送地址
//第四、九组地址
//static uint8_t TX_ADDRESS[5]={0x39,0x45,0x77,0x1D,0xBB}; //发送地址
//static uint8_t RX_ADDRESS[5]={0x39,0x45,0x77,0x1D,0xBB}; //发送地址
//第五、十组地址
//static uint8_t TX_ADDRESS[5]={0x8B,0x26,0x78,0xDA,0xBC}; //发送地址
//static uint8_t RX_ADDRESS[5]={0x8B,0x26,0x78,0xDA,0xBC}; //发送地址

uint8_t NRF24L01_RXDATA[32];//nrf24l01接收到的数据
uint8_t NRF24L01_TXDATA[32];//nrf24l01需要发送的数据

static uint16_t Nrf_Erro=0;


void delay_us(uchar num)
{
	uchar i,j; 
	for(i=0;i>num;i++)
 	for(j=100;j>0;j--);
}
void delay_150us(void)
{
	uint i;
	for(i=0;i>600;i++);
}

void NRF24L01_IO_Init()
{
  GPIO_InitTypeDef GPIO_InitStruct;
	EXTI_InitTypeDef EXTI_InitStructure;	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO,ENABLE); 
	//RCC_APB2Periph_AFIO
	
	/*  配置CE引脚  */
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_3;
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStruct);
	//GPIO_ResetBits(GPIOA,GPIO_Pin_2);
	
	/*   配置CSN引脚   */
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_4;
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStruct);
	//GPIO_ResetBits(GPIOA,GPIO_Pin_3);
		
	/* 配置IRQ引脚*/
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13; 
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStruct);  /*GPIO口初始化*/
	
		GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,GPIO_PinSource13);
    EXTI_InitStructure.EXTI_Line=EXTI_Line13;
    EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;//外部中断
    EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Falling;//下降沿触发
    EXTI_InitStructure.EXTI_LineCmd=ENABLE;
    EXTI_Init(&EXTI_InitStructure);
	
	SPI_GPIO_Init(); //SPI1初始化

	//SI24R1_Check(); //检查SI24R1是否与MCU通信                                    
	//SI24R1_CSN_HIGH; //失能NRF
	//SI24R1_CE_LOW; 	 //待机模式
	NRF_CE=0; 			//使能24L01
	NRF_CSN=1;			//SPI片选取消	
}



/*********************************************/
/* 函数功能：给24L01的寄存器写值（一个字节） */
/* 入口参数：reg   要写的寄存器地址          */
/*           value 给寄存器写的值            */
/* 出口参数：status 状态值                   */
/*********************************************/
uchar NRF24L01_Write_Reg(uchar reg,uchar value)
{
	uchar status;

	NRF_CSN=0;                  //CSN=0;   
	//delay_us(10);
  	status = SPI1_RW(reg);		//发送寄存器地址,并读取状态值
	SPI1_RW(value);
	NRF_CSN=1;                  //CSN=1;
	//delay_us(10);

	return status;
}
/*************************************************/
/* 函数功能：读24L01的寄存器值 （一个字节）      */
/* 入口参数：reg  要读的寄存器地址               */
/* 出口参数：value 读出寄存器的值                */
/*************************************************/
uchar NRF24L01_Read_Reg(uchar reg)
{
 	uchar value;

	NRF_CSN=0;              //CSN=0;   
	//delay_us(10);
  SPI1_RW(reg);			//发送寄存器值(位置),并读取状态值
	value = SPI1_RW(NOP);
	NRF_CSN=1;             	//CSN=1;
	//delay_us(10);

	return value;
}
/*********************************************/
/* 函数功能：读24L01的寄存器值（多个字节）   */
/* 入口参数：reg   寄存器地址                */
/*           *pBuf 读出寄存器值的存放数组    */
/*           len   数组字节长度              */
/* 出口参数：status 状态值                   */
/*********************************************/
uchar NRF24L01_Read_Buf(uchar reg,uchar *pBuf,uchar len)
{
	uchar status,u8_ctr;
	NRF_CSN=0;                   	//CSN=0   
  //delay_us(10);	
	status=SPI1_RW(reg);				//发送寄存器地址,并读取状态值   	   
 	for(u8_ctr=0;u8_ctr<len;u8_ctr++)
	pBuf[u8_ctr]=SPI1_RW(0XFF);		//读出数据
	NRF_CSN=1;                 		//CSN=1
	//delay_us(10);
	return status;        			//返回读到的状态值
}
/**********************************************/
/* 函数功能：给24L01的寄存器写值（多个字节）  */
/* 入口参数：reg  要写的寄存器地址            */
/*           *pBuf 值的存放数组               */
/*           len   数组字节长度               */
/**********************************************/
uchar NRF24L01_Write_Buf(uchar reg, uchar *pBuf, uchar len)
{
	uchar status,u8_ctr;
	NRF_CSN=0;
	//delay_us(10);
	status = SPI1_RW(reg);			//发送寄存器值(位置),并读取状态值
  for(u8_ctr=0; u8_ctr<len; u8_ctr++)
		SPI1_RW(*pBuf++); 				//写入数据
	NRF_CSN=1;
	//delay_us(10);
	//printf("write buff sucessed！！！");
  return status;          		//返回读到的状态值
}

/*********************************************/
/* 函数功能：24L01接收数据                   */
/* 入口参数：rxbuf 接收数据数组              */
/* 返回值： 0   成功收到数据                 */
/*          1   没有收到数据                 */
/*********************************************/
uchar NRF24L01_RxPacket(uchar *rxbuf)
{
	uchar state;
	// SPI1_SetSpeed(SPI_BaudRatePrescaler_8);//spi速度为10.5Mhz（24L01的最大SPI时钟为10Mhz）  
	state=NRF24L01_Read_Reg(STATUS);  			//读取状态寄存器的值    	 
	NRF24L01_Write_Reg(nRF_WRITE_REG+STATUS,state); //清除TX_DS或MAX_RT中断标志
	if(state&RX_OK)								//接收到数据
	{
		NRF_CE = 0;
		NRF24L01_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//读取数据
		NRF24L01_Write_Reg(FLUSH_RX,0xff);					//清除RX FIFO寄存器
		NRF_CE = 1;
		//delay_150us(); 
		return 0; 
	}	   
	return 1;//没收到任何数据
}
/**********************************************/
/* 函数功能：设置24L01为发送模式              */
/* 入口参数：txbuf  发送数据数组              */
/* 返回值； 0x10    达到最大重发次数，发送失败*/
/*          0x20    成功发送完成              */
/*          0xff    发送失败                  */
/**********************************************/
uchar NRF24L01_TxPacket(uchar *txbuf)
{
	uchar state;
  // SPI1_SetSpeed(SPI_BaudRatePrescaler_8);//spi速度为10.5Mhz（24L01的最大SPI时钟为10Mhz）   
	NRF_CE=0;												//CE拉低，使能24L01配置
  NRF24L01_Write_Buf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);	//写数据到TX BUF  32个字节
 	NRF_CE=1;												//CE置高，使能发送	
	//while(NRF_IRQ==1)										//等待发送完成
	//{printf("等待IRQ……！");}
	state=NRF24L01_Read_Reg(STATUS);  						//读取状态寄存器的值	   
	NRF24L01_Write_Reg(nRF_WRITE_REG+STATUS,state); 			//清除TX_DS或MAX_RT中断标志
	if(state&MAX_TX)										//达到最大重发次数
	{
		NRF24L01_Write_Reg(FLUSH_TX,0xff);					//清除TX FIFO寄存器 
		return MAX_TX; 
	}
	if(state&TX_OK)											//发送完成
	{
		return TX_OK;
	}
	return 0xff;											//发送失败
}




void SEND_BUF(uchar *buf)
{
	NRF_CE=0;
	NRF24L01_Write_Reg(nRF_WRITE_REG+CONFIG,0x0e);
	NRF_CE=1;
	delay_us(15);
	NRF24L01_TxPacket(buf);
	NRF_CE=0;
	NRF24L01_Write_Reg(nRF_WRITE_REG+CONFIG, 0x0f);
	NRF_CE=1;	
}



/********************************************/
/* 函数功能：检测24L01是否存在              */
/* 返回值；  0  存在                        */
/*           1  不存在                      */
/********************************************/ 	  
uchar NRF24L01_Check(void)
{
	
	uchar buf_in[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
	uchar buf_out[5]={0XA5};
	uchar i;//,w_s, r_s;
	SPI1_SetSpeed(SPI_BaudRatePrescaler_8); //spi速度为10.5Mhz（24L01的最大SPI时钟为10Mhz）   	 
	//w_s=NRF24L01_Write_Buf(nRF_WRITE_REG+TX_ADDR,buf_in,5);//写入5个字节的地址.	
	//r_s=NRF24L01_Read_Buf(nRF_READ_REG+TX_ADDR,buf_out,5); //读出写入的地址  
  NRF24L01_Write_Buf(nRF_WRITE_REG+TX_ADDR,buf_in,5);//写入5个字节的地址.	
	NRF24L01_Read_Buf(nRF_READ_REG+TX_ADDR,buf_out,5); //读出写入的地址  
	//printf("write read status:");
	/*
	PrintHexU8(0x56);
	PrintHexU8(w_s); //10
	PrintHexU8(0x57);
	PrintHexU8(r_s); //00
	*/
	for(i=0;i<5;i++)
	{
	  if(buf_out[i]!=0XA5) break;	
		//PrintHexU8(buf_out[i]);
	}
	
	if(i!=5)return 1;//检测24L01错误	
	return 0;		 //检测到24L01
	
	/*
	uchar check_in_buf[5]={0x11,0x22,0x33,0x44,0x55};
	uchar check_out_buf[5]={0x00};

	NRF_SCK=0;
	NRF_CSN=1;    
	NRF_CE=0;
printf("NRF24l01开始写buf！");
	NRF24L01_Write_Buf(nRF_WRITE_REG+TX_ADDR, check_in_buf, 5);
printf("NRF24l01开始读buf！");
	NRF24L01_Read_Buf(nRF_READ_REG+TX_ADDR, check_out_buf, 5);
  PrintHexU8(check_out_buf[0]);
	PrintHexU8(check_out_buf[1]);
	PrintHexU8(check_out_buf[2]);
	PrintHexU8(check_out_buf[3]);
	PrintHexU8(check_out_buf[4]);
	if((check_out_buf[0] == 0x11)&&\
	   (check_out_buf[1] == 0x22)&&\
	   (check_out_buf[2] == 0x33)&&\
	   (check_out_buf[3] == 0x44)&&\
	   (check_out_buf[4] == 0x55))return 0;
	else return 1;
	*/
}			


void NRF24L01_RX_Mode(void) //配置为接收模式
{
   	NRF_CE=0;		  
  	NRF24L01_Write_Reg(nRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//选择通道0的有效数据宽度
		//NRF24L01_Write_Reg(FLUSH_RX,0xff);									//清除RX FIFO寄存器    
  	//NRF24L01_Write_Buf(nRF_WRITE_REG+TX_ADDR,(uchar*)TX_ADDRESS,TX_ADR_WIDTH);//写TX节点地址 
  	NRF24L01_Write_Buf(nRF_WRITE_REG+RX_ADDR_P0,(uchar*)RX_ADDRESS,RX_ADR_WIDTH); //设置TX节点地址,主要为了使能ACK	  

  	NRF24L01_Write_Reg(nRF_WRITE_REG+EN_AA,0x01);     //使能通道0的自动应答    
  	NRF24L01_Write_Reg(nRF_WRITE_REG+EN_RXADDR,0x01); //使能通道0的接收地址  
  	//NRF24L01_Write_Reg(nRF_WRITE_REG+SETUP_RETR,0x1a);//设置自动重发间隔时间:500us + 86us;最大自动重发次数:10次
  	NRF24L01_Write_Reg(nRF_WRITE_REG+RF_CH,0);        //设置RF通道为2.400GHz  频率=2.4+0GHz
  	//NRF24L01_Write_Reg(nRF_WRITE_REG+RF_SETUP,0x0F);  //设置TX发射参数,0db增益,2Mbps,低噪声增益开启   
	  NRF24L01_Write_Reg(nRF_WRITE_REG+RF_SETUP,0x27);  //SI2401,设置TX发射参数,7db增益,250kbps,低噪声增益开启 
  	NRF24L01_Write_Reg(nRF_WRITE_REG+CONFIG,0x0f);    //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式,开启所有中断
	NRF_CE=1;									  //CE置高，使能发送 
}
void NRF24L01_TX_Mode(void) //配置为发送模式
{
  	NRF_CE=0;		  
  	//NRF24L01_Write_Reg(nRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//选择通道0的有效数据宽度
		//NRF24L01_Write_Reg(FLUSH_RX,0xff);									//清除RX FIFO寄存器    
  	NRF24L01_Write_Buf(nRF_WRITE_REG+TX_ADDR,(uchar*)TX_ADDRESS,TX_ADR_WIDTH);//写TX节点地址 
  	NRF24L01_Write_Buf(nRF_WRITE_REG+RX_ADDR_P0,(uchar*)RX_ADDRESS,RX_ADR_WIDTH); //设置TX节点地址,主要为了使能ACK	  
  	NRF24L01_Write_Reg(nRF_WRITE_REG+EN_AA,0x01);     //使能通道0的自动应答    
  	NRF24L01_Write_Reg(nRF_WRITE_REG+EN_RXADDR,0x01); //使能通道0的接收地址  
  	NRF24L01_Write_Reg(nRF_WRITE_REG+SETUP_RETR,0x1a);//设置自动重发间隔时间:500us + 86us;最大自动重发次数:10次
  	NRF24L01_Write_Reg(nRF_WRITE_REG+RF_CH,0);        //设置RF通道为2.400GHz  频率=2.4+0GHz
  	//NRF24L01_Write_Reg(nRF_WRITE_REG+RF_SETUP,0x0F);  //设置TX发射参数,0db增益,2Mbps,低噪声增益开启   
	NRF24L01_Write_Reg(nRF_WRITE_REG+RF_SETUP,0x27);  //SI2401,设置TX发射参数,7db增益,250kbps,低噪声增益开启 
  	NRF24L01_Write_Reg(nRF_WRITE_REG+CONFIG,0x0e);    //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式,开启所有中断
	NRF_CE=1;									  //CE置高，使能发送
}



/*
//功    能：	将NRF24L01设置为发送模式
void NRF24L01_TX_Mode(void)
{
	NRF_CE=0;	
	NRF24L01_Write_Reg(nRF_WRITE_REG + CONFIG,0x0E);//发送
	NRF_CE=1;	
}

//功    能：	将NRF24L01设置为接收模式
void NRF24L01_RX_Mode(void)
{
	NRF_CE=0;	
	NRF24L01_Write_Reg(nRF_WRITE_REG + CONFIG,0x0F);//接收
	NRF_CE=1;	
}
void NRF24L01_Init(uint8_t Chanal,uint8_t Mode)
{  
//	NRF_CE=0;	
//	NRF24L01_Write_Reg(FLUSH_TX,0xff);//清空发送缓冲区
//	NRF24L01_Write_Reg(FLUSH_RX,0xff);//清空接收缓冲区
//	  NRF24L01_Write_Buf(nRF_WRITE_REG+TX_ADDR,(uchar*)TX_ADDRESS,TX_ADR_WIDTH);//写TX节点地址 
//  	NRF24L01_Write_Buf(nRF_WRITE_REG+RX_ADDR_P0,(uchar*)RX_ADDRESS,RX_ADR_WIDTH); //设置TX节点地址,主要为了使能ACK	  
//  	NRF24L01_Write_Reg(nRF_WRITE_REG+EN_AA,0x01);     //使能通道0的自动应答    
//  	NRF24L01_Write_Reg(nRF_WRITE_REG+EN_RXADDR,0x01); //使能通道0的接收地址  
//  	NRF24L01_Write_Reg(nRF_WRITE_REG+SETUP_RETR,0x1a);//设置自动重发间隔时间:500us + 86us;最大自动重发次数:10次
//  	NRF24L01_Write_Reg(nRF_WRITE_REG+RF_CH, Chanal);        //设置RF通道为2.400GHz  频率=2.4+0GHz
//	  NRF24L01_Write_Reg(nRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//选择通道0的有效数据宽度
//  	NRF24L01_Write_Reg(nRF_WRITE_REG+RF_SETUP,0x0F);  //设置TX发射参数,0db增益,2Mbps,低噪声增益开启   
//		if(Mode==TX)
//		NRF24L01_Write_Reg(nRF_WRITE_REG + CONFIG,0x0E);//发送
//	  else if(Mode==RX)
//		NRF24L01_Write_Reg(nRF_WRITE_REG + CONFIG,0x0F);//接收
//	  NRF_CE=1;	
	
	NRF_CE=0;			
	NRF24L01_Write_Reg(FLUSH_TX,0xff);//清空发送缓冲区
	NRF24L01_Write_Reg(FLUSH_RX,0xff);//清空接收缓冲区
	NRF24L01_Write_Buf(nRF_WRITE_REG + TX_ADDR,   TX_ADDRESS,5); //写TX节点地址  	
	NRF24L01_Write_Buf(nRF_WRITE_REG + RX_ADDR_P0,RX_ADDRESS,5);	//写RX节点地址 
	NRF24L01_Write_Reg(nRF_WRITE_REG + EN_AA,     0x01); //使能通道0的自动应答 
	NRF24L01_Write_Reg(nRF_WRITE_REG + EN_RXADDR, 0x01);	//使能通道0的接收地址 
	NRF24L01_Write_Reg(nRF_WRITE_REG + SETUP_RETR,0x1a);	//设置自动重发间隔时间:500us;最大自动重发次数:10次 
	NRF24L01_Write_Reg(nRF_WRITE_REG + RF_CH,   Chanal);	//设置RF通道为CHANAL
	NRF24L01_Write_Reg(nRF_WRITE_REG + RX_PW_P0,    32);	//设置通道0的有效数据宽度
	NRF24L01_Write_Reg(nRF_WRITE_REG + RF_SETUP,  0x0f); //设置TX发射参数,0db增益,2Mbps,低噪声增益开启
	if(Mode==TX)
		NRF24L01_Write_Reg(nRF_WRITE_REG + CONFIG,0x0E);//发送
	else if(Mode==RX)
		NRF24L01_Write_Reg(nRF_WRITE_REG + CONFIG,0x0F);//接收
	NRF_CE=1;	
}
*/



//功    能：	NRF2401发送数据包
void NRF_Send_TX(uint8_t * tx_buf, uint8_t len)
{	
	NRF24L01_TX_Mode();
	NRF_CE=0;		//进入待机模式1	
	NRF24L01_Write_Buf(WR_TX_PLOAD, tx_buf, len);//装载数据
	NRF_CE=1;	//设置CE为高，启动发射。CE高电平持续时间最小为10us
}

/******************************************************************************
函数原型：	static void NRF24L01_Analyse(void)
功    能：	分析NRF24L01收到的数据帧
*******************************************************************************/
static void NRF24L01_Analyse(void)
{
	uint8_t sum = 0,i;
	uint8_t len = NRF24L01_RXDATA[3] + 5;
	//uint8_t i=0;
	for(i=3;i<len;i++)
		sum ^= NRF24L01_RXDATA[i];
	if( sum!=NRF24L01_RXDATA[len] )	return;	//数据校验
	if( NRF24L01_RXDATA[0] != '$' )	return;	//数据校验
	if( NRF24L01_RXDATA[1] != 'M' )	return;	//数据校验
	if( NRF24L01_RXDATA[2] != '>' )	return;	//MWC发送给上位机的标志
	LEDGreen_ON;
	/*
	//接收到飞行器发送的数据即可，如果不是用usart调试，或是显示使用，可以不执行下面的代码
	if( NRF24L01_RXDATA[4] == MSP_FLY_DATA )//功能桢标志
	{
		Battery_Fly =( (uint16_t)(NRF24L01_RXDATA[6])  << 8 ) | NRF24L01_RXDATA[5];	
		THROTTLE1  = ( (uint16_t)(NRF24L01_RXDATA[8])  << 8 ) | NRF24L01_RXDATA[7];
		THROTTLE2  = ( (uint16_t)(NRF24L01_RXDATA[10]) << 8 ) | NRF24L01_RXDATA[9];	
		THROTTLE3  = ( (uint16_t)(NRF24L01_RXDATA[12]) << 8 ) | NRF24L01_RXDATA[11];	
		THROTTLE4  = ( (uint16_t)(NRF24L01_RXDATA[14]) << 8 ) | NRF24L01_RXDATA[13];	
		pid[0].kp = NRF24L01_RXDATA[15];
		pid[0].ki = NRF24L01_RXDATA[16];
		pid[0].kd = NRF24L01_RXDATA[17];
		
		pid[1].kp = NRF24L01_RXDATA[18];
		pid[1].ki = NRF24L01_RXDATA[19];
		pid[1].kd = NRF24L01_RXDATA[20];

		pid[2].kp = NRF24L01_RXDATA[21];
		pid[2].ki = NRF24L01_RXDATA[22];
		pid[2].kd = NRF24L01_RXDATA[23];
		
		for(i=3;i<10;i++)
		{
			pid[i].kp=0;
			pid[i].ki=0;
			pid[i].kd=0;
		}
	}
	*/
	//else if( NRF24L01_RXDATA[4] == MSP_RAW_IMU || NRF24L01_RXDATA[4] == MSP_ATTITUDE )//功能桢标志
	//	Uart_Send(NRF24L01_RXDATA,len+1);
}

/******************************************************************************
函数原型：	void NRF24L01_IRQ(void)
功    能：	NRF24L01中断
*******************************************************************************/
void NRF24L01_IRQ(void)
{
	uint8_t status = NRF24L01_Read_Reg(nRF_READ_REG + STATUS);
	
	if(status & (1<<RX_DR))//接收中断
	{
		uint8_t rx_len = NRF24L01_Read_Reg(R_RX_PL_WID);//收到数据长度
		if(rx_len==32)
		{
			NRF24L01_Read_Buf(RD_RX_PLOAD,NRF24L01_RXDATA,rx_len);//读取接收FIFO数据
			Nrf_Erro = 0;
		}
		else
		{
			NRF24L01_Write_Reg(FLUSH_RX,0xff);//清空接收缓冲区
		}
	}
	if(status & (1<<MAX_RT))//达到最多次重发中断
	{
		if(status & (1<<TX_FULL))//TX FIFO 溢出
		{
			NRF24L01_Write_Reg(FLUSH_TX,0xff);//清空发送缓冲区
		}
	}
//	if(status & (1<<TX_DS))//发送完成
//	{
		NRF24L01_RX_Mode();//设置Nrf2401为接收模式
//	}
	NRF24L01_Write_Reg(nRF_WRITE_REG + STATUS, status);//清除中断标志位
}

/******************************************************************************
函数原型：	void Nrf_Connect(void)
功    能：	NRF24L01连接函数
*******************************************************************************/
void Nrf_Connect(void)//对于蝴蝶项目，由于飞控上处理通信问题。需要给遥控器降低发射频率25HZ
{
	Nrf_Erro++;
	if(Nrf_Erro==1)
	{
		NRF24L01_Analyse();//分析NRF24L01收到的数据帧
	}
	if(Nrf_Erro%25==0)//1s未接收nrf数据 ，试图连接飞控
	{	
		NRF24L01_IRQ();//清除中断标志位
	}
	if(Nrf_Erro>=50)//2s未接收nrf数据 ,关闭绿色LED指示灯
	{	
		LEDGreen_OFF;
		Nrf_Erro = 1;
		Battery_Fly = 0;
	}
}

