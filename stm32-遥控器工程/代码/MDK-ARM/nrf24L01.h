#ifndef    _NRF24L01_H
#define    _NRF24L01_H
#include "stdint.h"
#include "sys.h"
//#include "GPIOLIKE51.h"
#include "spi.h"

/******************************************************************************
							宏定义
*******************************************************************************/ 
#define TX	1
#define RX	2

#define RX_DR	6	//接收数据中断.当接收到有效数据后置一。写‘1’清除中断。 
#define TX_DS	5	//数据发送完成中断。当数据发送完成后产生中断。如果工作在自动应答模式下，只有当接收到应答信号后此位置一。写‘1’清除中断。 
#define MAX_RT	4	//达到最多次重发中断。写‘1’清除中断。如果MAX_RT中断产生则必须清除后系统才能进行通讯。
#define TX_FULL 0	//TX FIFO寄存器满标志。   1:TX FIFO  寄存器满   0: TX FIFO 寄存器未满, 有可用空间。 


/**********  NRF24L01寄存器操作命令  ***********/
#define nRF_READ_REG        0x00  //读配置寄存器,低5位为寄存器地址
#define nRF_WRITE_REG       0x20  //写配置寄存器,低5位为寄存器地址
#define ACTIVATE		0x50  	// follow with 0x73 to activate feature register
#define R_RX_PL_WID   	0x60	// 读接收缓冲区的长度
#define RD_RX_PLOAD     0x61  //读RX有效数据,1~32字节
#define WR_TX_PLOAD     0xA0  //写TX有效数据,1~32字节
#define FLUSH_TX        0xE1  //清除TX FIFO寄存器.发射模式下用
#define FLUSH_RX        0xE2  //清除RX FIFO寄存器.接收模式下用
#define REUSE_TX_PL     0xE3  //重新使用上一包数据,CE为高,数据包被不断发送.
#define NOP             0xFF  //空操作,可以用来读状态寄存器	 
/**********  NRF24L01寄存器地址   *************/
#define CONFIG          0x00  //配置寄存器地址                             
#define EN_AA           0x01  //使能自动应答功能 
#define EN_RXADDR       0x02  //接收地址允许
#define SETUP_AW        0x03  //设置地址宽度(所有数据通道)
#define SETUP_RETR      0x04  //建立自动重发
#define RF_CH           0x05  //RF通道
#define RF_SETUP        0x06  //RF寄存器
#define STATUS          0x07  //状态寄存器
#define OBSERVE_TX      0x08  // 发送检测寄存器
#define CD              0x09  // 载波检测寄存器
#define RX_ADDR_P0      0x0A  // 数据通道0接收地址
#define RX_ADDR_P1      0x0B  // 数据通道1接收地址
#define RX_ADDR_P2      0x0C  // 数据通道2接收地址
#define RX_ADDR_P3      0x0D  // 数据通道3接收地址
#define RX_ADDR_P4      0x0E  // 数据通道4接收地址
#define RX_ADDR_P5      0x0F  // 数据通道5接收地址
#define TX_ADDR         0x10  // 发送地址寄存器
#define RX_PW_P0        0x11  // 接收数据通道0有效数据宽度(1~32字节) 
#define RX_PW_P1        0x12  // 接收数据通道1有效数据宽度(1~32字节) 
#define RX_PW_P2        0x13  // 接收数据通道2有效数据宽度(1~32字节) 
#define RX_PW_P3        0x14  // 接收数据通道3有效数据宽度(1~32字节) 
#define RX_PW_P4        0x15  // 接收数据通道4有效数据宽度(1~32字节)
#define RX_PW_P5        0x16  // 接收数据通道5有效数据宽度(1~32字节)
#define FIFO_STATUS     0x17  // FIFO状态寄存器
/*————————————————————————————————————————————————————————————————————*/

/******   STATUS寄存器bit位定义      *******/
#define MAX_TX  	0x10  	  //达到最大发送次数中断
#define TX_OK   	0x20  	  //TX发送完成中断
#define RX_OK   	0x40  	  //接收到数据中断
/*——————————————————————————————————————————————————*/


/*********     24L01发送接收数据宽度定义	  ***********/
#define TX_ADR_WIDTH    5     //5字节地址宽度
#define RX_ADR_WIDTH    5     //5字节地址宽度
#define TX_PLOAD_WIDTH  32    //32字节有效数据宽度
#define RX_PLOAD_WIDTH  32    //32字节有效数据宽度

#define NRF_CE   	PAout(3)  //PC5 推完输出
#define NRF_CSN   PAout(4)  //片选信号，推挽输出  
#define NRF_IRQ   PCin(13)  //IRQ主机数据输入,上拉输入
//#define NRF_SCK   PBout(13)  //发送接收模式选择推挽输出
//#define NRF_MOSI  PBout(15)  //PC6 MOSI 主机推挽输出 （根据速率初始化PC_CR2寄存器）
//#define NRF_MISO  PBin(14)  //PC7 MISO

/******************************************************************************
							全局变量声明
*******************************************************************************/ 
extern 	uint8_t NRF24L01_RXDATA[32];//nrf24l01接收到的数据
extern 	uint8_t NRF24L01_TXDATA[32];//nrf24l01需要发送的数据

void delay_us(uchar num);
void delay_150us(void);
void NRF24L01_IO_Init(void);//初始化
void NRF24L01_Init(uint8_t Chanal,uint8_t Mode);
//uchar SPI_RW(uchar byte);
uchar NRF24L01_Write_Reg(uchar reg,uchar value);
uchar NRF24L01_Read_Reg(uchar reg);
uchar NRF24L01_Read_Buf(uchar reg,uchar *pBuf,uchar len);
uchar NRF24L01_Write_Buf(uchar reg, uchar *pBuf, uchar len);
uchar NRF24L01_RxPacket(uchar *rxbuf);
uchar NRF24L01_TxPacket(uchar *txbuf);
uchar NRF24L01_Check(void);
void NRF24L01_RT_Init(void);
void NRF24L01_RX_Mode(void);//配置为接收模式
void NRF24L01_TX_Mode(void);//配置为发送模式
void SEND_BUF(uchar *buf);

void NRF_Send_TX(uint8_t * tx_buf, uint8_t len);
void Nrf_Connect(void);
void NRF24L01_IRQ(void);
//static void NRF24L01_Analyse(void);

/*
void NRF24L01_Init(void);//初始化
void NRF24L01_RX_Mode(void);//配置为接收模式
void NRF24L01_TX_Mode(void);//配置为发送模式
u8 NRF24L01_Write_Buf(u8 reg, u8 *pBuf, u8 u8s);//写数据区
u8 NRF24L01_Read_Buf(u8 reg, u8 *pBuf, u8 u8s);//读数据区		  
u8 NRF24L01_Read_Reg(u8 reg);			//读寄存器
u8 NRF24L01_Write_Reg(u8 reg, u8 value);//写寄存器
u8 NRF24L01_Check(void);//检查24L01是否存在
u8 NRF24L01_TxPacket(u8 *txbuf);//发送一个包的数据
u8 NRF24L01_RxPacket(u8 *rxbuf);//接收一个包的数据
*/
#endif


