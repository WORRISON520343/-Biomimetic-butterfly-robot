#ifndef    _NRF24L01_H
#define    _NRF24L01_H
#include "stdint.h"
#include "sys.h"
//#include "GPIOLIKE51.h"
#include "spi.h"

/******************************************************************************
							�궨��
*******************************************************************************/ 
#define TX	1
#define RX	2

#define RX_DR	6	//���������ж�.�����յ���Ч���ݺ���һ��д��1������жϡ� 
#define TX_DS	5	//���ݷ�������жϡ������ݷ�����ɺ�����жϡ�����������Զ�Ӧ��ģʽ�£�ֻ�е����յ�Ӧ���źź��λ��һ��д��1������жϡ� 
#define MAX_RT	4	//�ﵽ�����ط��жϡ�д��1������жϡ����MAX_RT�жϲ�������������ϵͳ���ܽ���ͨѶ��
#define TX_FULL 0	//TX FIFO�Ĵ�������־��   1:TX FIFO  �Ĵ�����   0: TX FIFO �Ĵ���δ��, �п��ÿռ䡣 


/**********  NRF24L01�Ĵ�����������  ***********/
#define nRF_READ_REG        0x00  //�����üĴ���,��5λΪ�Ĵ�����ַ
#define nRF_WRITE_REG       0x20  //д���üĴ���,��5λΪ�Ĵ�����ַ
#define ACTIVATE		0x50  	// follow with 0x73 to activate feature register
#define R_RX_PL_WID   	0x60	// �����ջ������ĳ���
#define RD_RX_PLOAD     0x61  //��RX��Ч����,1~32�ֽ�
#define WR_TX_PLOAD     0xA0  //дTX��Ч����,1~32�ֽ�
#define FLUSH_TX        0xE1  //���TX FIFO�Ĵ���.����ģʽ����
#define FLUSH_RX        0xE2  //���RX FIFO�Ĵ���.����ģʽ����
#define REUSE_TX_PL     0xE3  //����ʹ����һ������,CEΪ��,���ݰ������Ϸ���.
#define NOP             0xFF  //�ղ���,����������״̬�Ĵ���	 
/**********  NRF24L01�Ĵ�����ַ   *************/
#define CONFIG          0x00  //���üĴ�����ַ                             
#define EN_AA           0x01  //ʹ���Զ�Ӧ���� 
#define EN_RXADDR       0x02  //���յ�ַ����
#define SETUP_AW        0x03  //���õ�ַ���(��������ͨ��)
#define SETUP_RETR      0x04  //�����Զ��ط�
#define RF_CH           0x05  //RFͨ��
#define RF_SETUP        0x06  //RF�Ĵ���
#define STATUS          0x07  //״̬�Ĵ���
#define OBSERVE_TX      0x08  // ���ͼ��Ĵ���
#define CD              0x09  // �ز����Ĵ���
#define RX_ADDR_P0      0x0A  // ����ͨ��0���յ�ַ
#define RX_ADDR_P1      0x0B  // ����ͨ��1���յ�ַ
#define RX_ADDR_P2      0x0C  // ����ͨ��2���յ�ַ
#define RX_ADDR_P3      0x0D  // ����ͨ��3���յ�ַ
#define RX_ADDR_P4      0x0E  // ����ͨ��4���յ�ַ
#define RX_ADDR_P5      0x0F  // ����ͨ��5���յ�ַ
#define TX_ADDR         0x10  // ���͵�ַ�Ĵ���
#define RX_PW_P0        0x11  // ��������ͨ��0��Ч���ݿ��(1~32�ֽ�) 
#define RX_PW_P1        0x12  // ��������ͨ��1��Ч���ݿ��(1~32�ֽ�) 
#define RX_PW_P2        0x13  // ��������ͨ��2��Ч���ݿ��(1~32�ֽ�) 
#define RX_PW_P3        0x14  // ��������ͨ��3��Ч���ݿ��(1~32�ֽ�) 
#define RX_PW_P4        0x15  // ��������ͨ��4��Ч���ݿ��(1~32�ֽ�)
#define RX_PW_P5        0x16  // ��������ͨ��5��Ч���ݿ��(1~32�ֽ�)
#define FIFO_STATUS     0x17  // FIFO״̬�Ĵ���
/*����������������������������������������������������������������������������������������������������������������������������������������*/

/******   STATUS�Ĵ���bitλ����      *******/
#define MAX_TX  	0x10  	  //�ﵽ����ʹ����ж�
#define TX_OK   	0x20  	  //TX��������ж�
#define RX_OK   	0x40  	  //���յ������ж�
/*����������������������������������������������������������������������������������������������������*/


/*********     24L01���ͽ������ݿ�ȶ���	  ***********/
#define TX_ADR_WIDTH    5     //5�ֽڵ�ַ���
#define RX_ADR_WIDTH    5     //5�ֽڵ�ַ���
#define TX_PLOAD_WIDTH  32    //32�ֽ���Ч���ݿ��
#define RX_PLOAD_WIDTH  32    //32�ֽ���Ч���ݿ��

#define NRF_CE   	PAout(3)  //PC5 �������
#define NRF_CSN   PAout(4)  //Ƭѡ�źţ��������  
#define NRF_IRQ   PCin(13)  //IRQ������������,��������
//#define NRF_SCK   PBout(13)  //���ͽ���ģʽѡ���������
//#define NRF_MOSI  PBout(15)  //PC6 MOSI ����������� ���������ʳ�ʼ��PC_CR2�Ĵ�����
//#define NRF_MISO  PBin(14)  //PC7 MISO

/******************************************************************************
							ȫ�ֱ�������
*******************************************************************************/ 
extern 	uint8_t NRF24L01_RXDATA[32];//nrf24l01���յ�������
extern 	uint8_t NRF24L01_TXDATA[32];//nrf24l01��Ҫ���͵�����

void delay_us(uchar num);
void delay_150us(void);
void NRF24L01_IO_Init(void);//��ʼ��
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
void NRF24L01_RX_Mode(void);//����Ϊ����ģʽ
void NRF24L01_TX_Mode(void);//����Ϊ����ģʽ
void SEND_BUF(uchar *buf);

void NRF_Send_TX(uint8_t * tx_buf, uint8_t len);
void Nrf_Connect(void);
void NRF24L01_IRQ(void);
//static void NRF24L01_Analyse(void);

/*
void NRF24L01_Init(void);//��ʼ��
void NRF24L01_RX_Mode(void);//����Ϊ����ģʽ
void NRF24L01_TX_Mode(void);//����Ϊ����ģʽ
u8 NRF24L01_Write_Buf(u8 reg, u8 *pBuf, u8 u8s);//д������
u8 NRF24L01_Read_Buf(u8 reg, u8 *pBuf, u8 u8s);//��������		  
u8 NRF24L01_Read_Reg(u8 reg);			//���Ĵ���
u8 NRF24L01_Write_Reg(u8 reg, u8 value);//д�Ĵ���
u8 NRF24L01_Check(void);//���24L01�Ƿ����
u8 NRF24L01_TxPacket(u8 *txbuf);//����һ����������
u8 NRF24L01_RxPacket(u8 *rxbuf);//����һ����������
*/
#endif


