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
//��һ�������ַ
static uint8_t TX_ADDRESS[5]={0x34,0x43,0x10,0x10,0x01}; //���͵�ַ
static uint8_t RX_ADDRESS[5]={0x34,0x43,0x10,0x10,0x01}; //���͵�ַ
//�ڶ��������ַ
//static uint8_t TX_ADDRESS[5]={0x1A,0x3B,0x5C,0x7D,0x9E}; //���͵�ַ
//static uint8_t RX_ADDRESS[5]={0x1A,0x3B,0x5C,0x7D,0x9E}; //���͵�ַ
//�����������ַ
//static uint8_t TX_ADDRESS[5]={0x22,0x44,0x66,0x88,0xAA}; //���͵�ַ
//static uint8_t RX_ADDRESS[5]={0x22,0x44,0x66,0x88,0xAA}; //���͵�ַ
//���ġ������ַ
//static uint8_t TX_ADDRESS[5]={0x39,0x45,0x77,0x1D,0xBB}; //���͵�ַ
//static uint8_t RX_ADDRESS[5]={0x39,0x45,0x77,0x1D,0xBB}; //���͵�ַ
//���塢ʮ���ַ
//static uint8_t TX_ADDRESS[5]={0x8B,0x26,0x78,0xDA,0xBC}; //���͵�ַ
//static uint8_t RX_ADDRESS[5]={0x8B,0x26,0x78,0xDA,0xBC}; //���͵�ַ

uint8_t NRF24L01_RXDATA[32];//nrf24l01���յ�������
uint8_t NRF24L01_TXDATA[32];//nrf24l01��Ҫ���͵�����

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
	
	/*  ����CE����  */
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_3;
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStruct);
	//GPIO_ResetBits(GPIOA,GPIO_Pin_2);
	
	/*   ����CSN����   */
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_4;
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStruct);
	//GPIO_ResetBits(GPIOA,GPIO_Pin_3);
		
	/* ����IRQ����*/
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13; 
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStruct);  /*GPIO�ڳ�ʼ��*/
	
		GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,GPIO_PinSource13);
    EXTI_InitStructure.EXTI_Line=EXTI_Line13;
    EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;//�ⲿ�ж�
    EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Falling;//�½��ش���
    EXTI_InitStructure.EXTI_LineCmd=ENABLE;
    EXTI_Init(&EXTI_InitStructure);
	
	SPI_GPIO_Init(); //SPI1��ʼ��

	//SI24R1_Check(); //���SI24R1�Ƿ���MCUͨ��                                    
	//SI24R1_CSN_HIGH; //ʧ��NRF
	//SI24R1_CE_LOW; 	 //����ģʽ
	NRF_CE=0; 			//ʹ��24L01
	NRF_CSN=1;			//SPIƬѡȡ��	
}



/*********************************************/
/* �������ܣ���24L01�ļĴ���дֵ��һ���ֽڣ� */
/* ��ڲ�����reg   Ҫд�ļĴ�����ַ          */
/*           value ���Ĵ���д��ֵ            */
/* ���ڲ�����status ״ֵ̬                   */
/*********************************************/
uchar NRF24L01_Write_Reg(uchar reg,uchar value)
{
	uchar status;

	NRF_CSN=0;                  //CSN=0;   
	//delay_us(10);
  	status = SPI1_RW(reg);		//���ͼĴ�����ַ,����ȡ״ֵ̬
	SPI1_RW(value);
	NRF_CSN=1;                  //CSN=1;
	//delay_us(10);

	return status;
}
/*************************************************/
/* �������ܣ���24L01�ļĴ���ֵ ��һ���ֽڣ�      */
/* ��ڲ�����reg  Ҫ���ļĴ�����ַ               */
/* ���ڲ�����value �����Ĵ�����ֵ                */
/*************************************************/
uchar NRF24L01_Read_Reg(uchar reg)
{
 	uchar value;

	NRF_CSN=0;              //CSN=0;   
	//delay_us(10);
  SPI1_RW(reg);			//���ͼĴ���ֵ(λ��),����ȡ״ֵ̬
	value = SPI1_RW(NOP);
	NRF_CSN=1;             	//CSN=1;
	//delay_us(10);

	return value;
}
/*********************************************/
/* �������ܣ���24L01�ļĴ���ֵ������ֽڣ�   */
/* ��ڲ�����reg   �Ĵ�����ַ                */
/*           *pBuf �����Ĵ���ֵ�Ĵ������    */
/*           len   �����ֽڳ���              */
/* ���ڲ�����status ״ֵ̬                   */
/*********************************************/
uchar NRF24L01_Read_Buf(uchar reg,uchar *pBuf,uchar len)
{
	uchar status,u8_ctr;
	NRF_CSN=0;                   	//CSN=0   
  //delay_us(10);	
	status=SPI1_RW(reg);				//���ͼĴ�����ַ,����ȡ״ֵ̬   	   
 	for(u8_ctr=0;u8_ctr<len;u8_ctr++)
	pBuf[u8_ctr]=SPI1_RW(0XFF);		//��������
	NRF_CSN=1;                 		//CSN=1
	//delay_us(10);
	return status;        			//���ض�����״ֵ̬
}
/**********************************************/
/* �������ܣ���24L01�ļĴ���дֵ������ֽڣ�  */
/* ��ڲ�����reg  Ҫд�ļĴ�����ַ            */
/*           *pBuf ֵ�Ĵ������               */
/*           len   �����ֽڳ���               */
/**********************************************/
uchar NRF24L01_Write_Buf(uchar reg, uchar *pBuf, uchar len)
{
	uchar status,u8_ctr;
	NRF_CSN=0;
	//delay_us(10);
	status = SPI1_RW(reg);			//���ͼĴ���ֵ(λ��),����ȡ״ֵ̬
  for(u8_ctr=0; u8_ctr<len; u8_ctr++)
		SPI1_RW(*pBuf++); 				//д������
	NRF_CSN=1;
	//delay_us(10);
	//printf("write buff sucessed������");
  return status;          		//���ض�����״ֵ̬
}

/*********************************************/
/* �������ܣ�24L01��������                   */
/* ��ڲ�����rxbuf ������������              */
/* ����ֵ�� 0   �ɹ��յ�����                 */
/*          1   û���յ�����                 */
/*********************************************/
uchar NRF24L01_RxPacket(uchar *rxbuf)
{
	uchar state;
	// SPI1_SetSpeed(SPI_BaudRatePrescaler_8);//spi�ٶ�Ϊ10.5Mhz��24L01�����SPIʱ��Ϊ10Mhz��  
	state=NRF24L01_Read_Reg(STATUS);  			//��ȡ״̬�Ĵ�����ֵ    	 
	NRF24L01_Write_Reg(nRF_WRITE_REG+STATUS,state); //���TX_DS��MAX_RT�жϱ�־
	if(state&RX_OK)								//���յ�����
	{
		NRF_CE = 0;
		NRF24L01_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//��ȡ����
		NRF24L01_Write_Reg(FLUSH_RX,0xff);					//���RX FIFO�Ĵ���
		NRF_CE = 1;
		//delay_150us(); 
		return 0; 
	}	   
	return 1;//û�յ��κ�����
}
/**********************************************/
/* �������ܣ�����24L01Ϊ����ģʽ              */
/* ��ڲ�����txbuf  ������������              */
/* ����ֵ�� 0x10    �ﵽ����ط�����������ʧ��*/
/*          0x20    �ɹ��������              */
/*          0xff    ����ʧ��                  */
/**********************************************/
uchar NRF24L01_TxPacket(uchar *txbuf)
{
	uchar state;
  // SPI1_SetSpeed(SPI_BaudRatePrescaler_8);//spi�ٶ�Ϊ10.5Mhz��24L01�����SPIʱ��Ϊ10Mhz��   
	NRF_CE=0;												//CE���ͣ�ʹ��24L01����
  NRF24L01_Write_Buf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);	//д���ݵ�TX BUF  32���ֽ�
 	NRF_CE=1;												//CE�øߣ�ʹ�ܷ���	
	//while(NRF_IRQ==1)										//�ȴ��������
	//{printf("�ȴ�IRQ������");}
	state=NRF24L01_Read_Reg(STATUS);  						//��ȡ״̬�Ĵ�����ֵ	   
	NRF24L01_Write_Reg(nRF_WRITE_REG+STATUS,state); 			//���TX_DS��MAX_RT�жϱ�־
	if(state&MAX_TX)										//�ﵽ����ط�����
	{
		NRF24L01_Write_Reg(FLUSH_TX,0xff);					//���TX FIFO�Ĵ��� 
		return MAX_TX; 
	}
	if(state&TX_OK)											//�������
	{
		return TX_OK;
	}
	return 0xff;											//����ʧ��
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
/* �������ܣ����24L01�Ƿ����              */
/* ����ֵ��  0  ����                        */
/*           1  ������                      */
/********************************************/ 	  
uchar NRF24L01_Check(void)
{
	
	uchar buf_in[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
	uchar buf_out[5]={0XA5};
	uchar i;//,w_s, r_s;
	SPI1_SetSpeed(SPI_BaudRatePrescaler_8); //spi�ٶ�Ϊ10.5Mhz��24L01�����SPIʱ��Ϊ10Mhz��   	 
	//w_s=NRF24L01_Write_Buf(nRF_WRITE_REG+TX_ADDR,buf_in,5);//д��5���ֽڵĵ�ַ.	
	//r_s=NRF24L01_Read_Buf(nRF_READ_REG+TX_ADDR,buf_out,5); //����д��ĵ�ַ  
  NRF24L01_Write_Buf(nRF_WRITE_REG+TX_ADDR,buf_in,5);//д��5���ֽڵĵ�ַ.	
	NRF24L01_Read_Buf(nRF_READ_REG+TX_ADDR,buf_out,5); //����д��ĵ�ַ  
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
	
	if(i!=5)return 1;//���24L01����	
	return 0;		 //��⵽24L01
	
	/*
	uchar check_in_buf[5]={0x11,0x22,0x33,0x44,0x55};
	uchar check_out_buf[5]={0x00};

	NRF_SCK=0;
	NRF_CSN=1;    
	NRF_CE=0;
printf("NRF24l01��ʼдbuf��");
	NRF24L01_Write_Buf(nRF_WRITE_REG+TX_ADDR, check_in_buf, 5);
printf("NRF24l01��ʼ��buf��");
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


void NRF24L01_RX_Mode(void) //����Ϊ����ģʽ
{
   	NRF_CE=0;		  
  	NRF24L01_Write_Reg(nRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//ѡ��ͨ��0����Ч���ݿ��
		//NRF24L01_Write_Reg(FLUSH_RX,0xff);									//���RX FIFO�Ĵ���    
  	//NRF24L01_Write_Buf(nRF_WRITE_REG+TX_ADDR,(uchar*)TX_ADDRESS,TX_ADR_WIDTH);//дTX�ڵ��ַ 
  	NRF24L01_Write_Buf(nRF_WRITE_REG+RX_ADDR_P0,(uchar*)RX_ADDRESS,RX_ADR_WIDTH); //����TX�ڵ��ַ,��ҪΪ��ʹ��ACK	  

  	NRF24L01_Write_Reg(nRF_WRITE_REG+EN_AA,0x01);     //ʹ��ͨ��0���Զ�Ӧ��    
  	NRF24L01_Write_Reg(nRF_WRITE_REG+EN_RXADDR,0x01); //ʹ��ͨ��0�Ľ��յ�ַ  
  	//NRF24L01_Write_Reg(nRF_WRITE_REG+SETUP_RETR,0x1a);//�����Զ��ط����ʱ��:500us + 86us;����Զ��ط�����:10��
  	NRF24L01_Write_Reg(nRF_WRITE_REG+RF_CH,0);        //����RFͨ��Ϊ2.400GHz  Ƶ��=2.4+0GHz
  	//NRF24L01_Write_Reg(nRF_WRITE_REG+RF_SETUP,0x0F);  //����TX�������,0db����,2Mbps,���������濪��   
	  NRF24L01_Write_Reg(nRF_WRITE_REG+RF_SETUP,0x27);  //SI2401,����TX�������,7db����,250kbps,���������濪�� 
  	NRF24L01_Write_Reg(nRF_WRITE_REG+CONFIG,0x0f);    //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ,���������ж�
	NRF_CE=1;									  //CE�øߣ�ʹ�ܷ��� 
}
void NRF24L01_TX_Mode(void) //����Ϊ����ģʽ
{
  	NRF_CE=0;		  
  	//NRF24L01_Write_Reg(nRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//ѡ��ͨ��0����Ч���ݿ��
		//NRF24L01_Write_Reg(FLUSH_RX,0xff);									//���RX FIFO�Ĵ���    
  	NRF24L01_Write_Buf(nRF_WRITE_REG+TX_ADDR,(uchar*)TX_ADDRESS,TX_ADR_WIDTH);//дTX�ڵ��ַ 
  	NRF24L01_Write_Buf(nRF_WRITE_REG+RX_ADDR_P0,(uchar*)RX_ADDRESS,RX_ADR_WIDTH); //����TX�ڵ��ַ,��ҪΪ��ʹ��ACK	  
  	NRF24L01_Write_Reg(nRF_WRITE_REG+EN_AA,0x01);     //ʹ��ͨ��0���Զ�Ӧ��    
  	NRF24L01_Write_Reg(nRF_WRITE_REG+EN_RXADDR,0x01); //ʹ��ͨ��0�Ľ��յ�ַ  
  	NRF24L01_Write_Reg(nRF_WRITE_REG+SETUP_RETR,0x1a);//�����Զ��ط����ʱ��:500us + 86us;����Զ��ط�����:10��
  	NRF24L01_Write_Reg(nRF_WRITE_REG+RF_CH,0);        //����RFͨ��Ϊ2.400GHz  Ƶ��=2.4+0GHz
  	//NRF24L01_Write_Reg(nRF_WRITE_REG+RF_SETUP,0x0F);  //����TX�������,0db����,2Mbps,���������濪��   
	NRF24L01_Write_Reg(nRF_WRITE_REG+RF_SETUP,0x27);  //SI2401,����TX�������,7db����,250kbps,���������濪�� 
  	NRF24L01_Write_Reg(nRF_WRITE_REG+CONFIG,0x0e);    //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ,���������ж�
	NRF_CE=1;									  //CE�øߣ�ʹ�ܷ���
}



/*
//��    �ܣ�	��NRF24L01����Ϊ����ģʽ
void NRF24L01_TX_Mode(void)
{
	NRF_CE=0;	
	NRF24L01_Write_Reg(nRF_WRITE_REG + CONFIG,0x0E);//����
	NRF_CE=1;	
}

//��    �ܣ�	��NRF24L01����Ϊ����ģʽ
void NRF24L01_RX_Mode(void)
{
	NRF_CE=0;	
	NRF24L01_Write_Reg(nRF_WRITE_REG + CONFIG,0x0F);//����
	NRF_CE=1;	
}
void NRF24L01_Init(uint8_t Chanal,uint8_t Mode)
{  
//	NRF_CE=0;	
//	NRF24L01_Write_Reg(FLUSH_TX,0xff);//��շ��ͻ�����
//	NRF24L01_Write_Reg(FLUSH_RX,0xff);//��ս��ջ�����
//	  NRF24L01_Write_Buf(nRF_WRITE_REG+TX_ADDR,(uchar*)TX_ADDRESS,TX_ADR_WIDTH);//дTX�ڵ��ַ 
//  	NRF24L01_Write_Buf(nRF_WRITE_REG+RX_ADDR_P0,(uchar*)RX_ADDRESS,RX_ADR_WIDTH); //����TX�ڵ��ַ,��ҪΪ��ʹ��ACK	  
//  	NRF24L01_Write_Reg(nRF_WRITE_REG+EN_AA,0x01);     //ʹ��ͨ��0���Զ�Ӧ��    
//  	NRF24L01_Write_Reg(nRF_WRITE_REG+EN_RXADDR,0x01); //ʹ��ͨ��0�Ľ��յ�ַ  
//  	NRF24L01_Write_Reg(nRF_WRITE_REG+SETUP_RETR,0x1a);//�����Զ��ط����ʱ��:500us + 86us;����Զ��ط�����:10��
//  	NRF24L01_Write_Reg(nRF_WRITE_REG+RF_CH, Chanal);        //����RFͨ��Ϊ2.400GHz  Ƶ��=2.4+0GHz
//	  NRF24L01_Write_Reg(nRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//ѡ��ͨ��0����Ч���ݿ��
//  	NRF24L01_Write_Reg(nRF_WRITE_REG+RF_SETUP,0x0F);  //����TX�������,0db����,2Mbps,���������濪��   
//		if(Mode==TX)
//		NRF24L01_Write_Reg(nRF_WRITE_REG + CONFIG,0x0E);//����
//	  else if(Mode==RX)
//		NRF24L01_Write_Reg(nRF_WRITE_REG + CONFIG,0x0F);//����
//	  NRF_CE=1;	
	
	NRF_CE=0;			
	NRF24L01_Write_Reg(FLUSH_TX,0xff);//��շ��ͻ�����
	NRF24L01_Write_Reg(FLUSH_RX,0xff);//��ս��ջ�����
	NRF24L01_Write_Buf(nRF_WRITE_REG + TX_ADDR,   TX_ADDRESS,5); //дTX�ڵ��ַ  	
	NRF24L01_Write_Buf(nRF_WRITE_REG + RX_ADDR_P0,RX_ADDRESS,5);	//дRX�ڵ��ַ 
	NRF24L01_Write_Reg(nRF_WRITE_REG + EN_AA,     0x01); //ʹ��ͨ��0���Զ�Ӧ�� 
	NRF24L01_Write_Reg(nRF_WRITE_REG + EN_RXADDR, 0x01);	//ʹ��ͨ��0�Ľ��յ�ַ 
	NRF24L01_Write_Reg(nRF_WRITE_REG + SETUP_RETR,0x1a);	//�����Զ��ط����ʱ��:500us;����Զ��ط�����:10�� 
	NRF24L01_Write_Reg(nRF_WRITE_REG + RF_CH,   Chanal);	//����RFͨ��ΪCHANAL
	NRF24L01_Write_Reg(nRF_WRITE_REG + RX_PW_P0,    32);	//����ͨ��0����Ч���ݿ��
	NRF24L01_Write_Reg(nRF_WRITE_REG + RF_SETUP,  0x0f); //����TX�������,0db����,2Mbps,���������濪��
	if(Mode==TX)
		NRF24L01_Write_Reg(nRF_WRITE_REG + CONFIG,0x0E);//����
	else if(Mode==RX)
		NRF24L01_Write_Reg(nRF_WRITE_REG + CONFIG,0x0F);//����
	NRF_CE=1;	
}
*/



//��    �ܣ�	NRF2401�������ݰ�
void NRF_Send_TX(uint8_t * tx_buf, uint8_t len)
{	
	NRF24L01_TX_Mode();
	NRF_CE=0;		//�������ģʽ1	
	NRF24L01_Write_Buf(WR_TX_PLOAD, tx_buf, len);//װ������
	NRF_CE=1;	//����CEΪ�ߣ��������䡣CE�ߵ�ƽ����ʱ����СΪ10us
}

/******************************************************************************
����ԭ�ͣ�	static void NRF24L01_Analyse(void)
��    �ܣ�	����NRF24L01�յ�������֡
*******************************************************************************/
static void NRF24L01_Analyse(void)
{
	uint8_t sum = 0,i;
	uint8_t len = NRF24L01_RXDATA[3] + 5;
	//uint8_t i=0;
	for(i=3;i<len;i++)
		sum ^= NRF24L01_RXDATA[i];
	if( sum!=NRF24L01_RXDATA[len] )	return;	//����У��
	if( NRF24L01_RXDATA[0] != '$' )	return;	//����У��
	if( NRF24L01_RXDATA[1] != 'M' )	return;	//����У��
	if( NRF24L01_RXDATA[2] != '>' )	return;	//MWC���͸���λ���ı�־
	LEDGreen_ON;
	/*
	//���յ����������͵����ݼ��ɣ����������usart���ԣ�������ʾʹ�ã����Բ�ִ������Ĵ���
	if( NRF24L01_RXDATA[4] == MSP_FLY_DATA )//�������־
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
	//else if( NRF24L01_RXDATA[4] == MSP_RAW_IMU || NRF24L01_RXDATA[4] == MSP_ATTITUDE )//�������־
	//	Uart_Send(NRF24L01_RXDATA,len+1);
}

/******************************************************************************
����ԭ�ͣ�	void NRF24L01_IRQ(void)
��    �ܣ�	NRF24L01�ж�
*******************************************************************************/
void NRF24L01_IRQ(void)
{
	uint8_t status = NRF24L01_Read_Reg(nRF_READ_REG + STATUS);
	
	if(status & (1<<RX_DR))//�����ж�
	{
		uint8_t rx_len = NRF24L01_Read_Reg(R_RX_PL_WID);//�յ����ݳ���
		if(rx_len==32)
		{
			NRF24L01_Read_Buf(RD_RX_PLOAD,NRF24L01_RXDATA,rx_len);//��ȡ����FIFO����
			Nrf_Erro = 0;
		}
		else
		{
			NRF24L01_Write_Reg(FLUSH_RX,0xff);//��ս��ջ�����
		}
	}
	if(status & (1<<MAX_RT))//�ﵽ�����ط��ж�
	{
		if(status & (1<<TX_FULL))//TX FIFO ���
		{
			NRF24L01_Write_Reg(FLUSH_TX,0xff);//��շ��ͻ�����
		}
	}
//	if(status & (1<<TX_DS))//�������
//	{
		NRF24L01_RX_Mode();//����Nrf2401Ϊ����ģʽ
//	}
	NRF24L01_Write_Reg(nRF_WRITE_REG + STATUS, status);//����жϱ�־λ
}

/******************************************************************************
����ԭ�ͣ�	void Nrf_Connect(void)
��    �ܣ�	NRF24L01���Ӻ���
*******************************************************************************/
void Nrf_Connect(void)//���ں�����Ŀ�����ڷɿ��ϴ���ͨ�����⡣��Ҫ��ң�������ͷ���Ƶ��25HZ
{
	Nrf_Erro++;
	if(Nrf_Erro==1)
	{
		NRF24L01_Analyse();//����NRF24L01�յ�������֡
	}
	if(Nrf_Erro%25==0)//1sδ����nrf���� ����ͼ���ӷɿ�
	{	
		NRF24L01_IRQ();//����жϱ�־λ
	}
	if(Nrf_Erro>=50)//2sδ����nrf���� ,�ر���ɫLEDָʾ��
	{	
		LEDGreen_OFF;
		Nrf_Erro = 1;
		Battery_Fly = 0;
	}
}

