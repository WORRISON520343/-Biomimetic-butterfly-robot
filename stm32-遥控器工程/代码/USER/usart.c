/*******************************************************************************************
										    �� ��
    ����Ŀ�����������ѧϰʹ�ã�����������ֲ�޸ģ������뱣����������Ϣ����ֲ�����г�������
	
���ɹ�����BUG��������������κ����Ρ��������ã�

* ����汾��V1.01
* �������ڣ�2018-8-18
* �������ߣ���ŭ��С��
* ��Ȩ���У��������������Ϣ�������޹�˾
*******************************************************************************************/
#include "stm32f10x.h"
//#include "stdio.h"
#include "usart.h"

uint8_t TxCount=0;
uint8_t Count=0;

uint8_t Line0,Line1;	//���ڽ���˫�����л�

static uint8_t TxBuff[256];	//���ڷ��ͻ�����
//volatile uint8_t RxBuffer[50];//���ڽ��ջ�����
uint8_t RxBuff[2][50];		//���ڽ��ջ�����

/*****************************************************************************
* ��  ����void USART_init(uint32_t baudrate)
* ��  �ܣ�Usart1��ʼ��Ϊ˫��ģʽ
* ��  ����baudrate ������
* ����ֵ����
* ��  ע����������������֡�Ľ��� �����ж�������ж�����ܽ���������⣬
          ������շ�ʽ��stm32f1x_it.c �еĴ����жϴ���;
*****************************************************************************/
void USART_init(uint32_t baudrate)
{
	GPIO_InitTypeDef GPIO_InitStruct;   //����GPIO�ṹ�����
	USART_InitTypeDef USART_InitStruct;   //���崮�ڽṹ�����
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_USART1,ENABLE);   //ʹ��GPIOA��USART1��ʱ��
	
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_9;   //����TX����
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF_PP;   //����PA9Ϊ�����������
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;   //����PA9����
	GPIO_Init(GPIOA,&GPIO_InitStruct);   //GPIO��ʼ������
	
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_10;   //����RX����
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_IN_FLOATING;   //����PA10Ϊ��������
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;   //����PA10����
	GPIO_Init(GPIOA,&GPIO_InitStruct);   //GPIO��ʼ������
	
	
	USART_InitStruct.USART_Mode=USART_Mode_Tx|USART_Mode_Rx;   //���ͽ���ģʽ
	USART_InitStruct.USART_Parity=USART_Parity_No;   //����żУ��
	USART_InitStruct.USART_BaudRate=baudrate;   //������
	USART_InitStruct.USART_StopBits=USART_StopBits_1;   //ֹͣλ1λ
	USART_InitStruct.USART_WordLength=USART_WordLength_8b;   //�ֳ�8λ
	USART_InitStruct.USART_HardwareFlowControl=USART_HardwareFlowControl_None;   //��Ӳ������������
	USART_Init(USART1,&USART_InitStruct);   //���ڳ�ʼ������
	
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);		//���ڽ����ж�
	USART_ITConfig(USART1,USART_IT_IDLE,ENABLE);		//���ڿ����ж�
	
	USART_Cmd(USART1,ENABLE);   //ʹ��USART1
}



/******************************************************************************
����ԭ�ͣ�	void USART1_IRQHandler(void)
��    �ܣ�	�����ж�
*******************************************************************************/ 
void USART1_IRQHandler(void)
{
	if(USART_GetFlagStatus(USART1, USART_FLAG_ORE) != RESET)//ORE�ж�
	{
		USART_ReceiveData(USART1);
	}
	
	if(USART1->SR & USART_SR_TC)//�����ж�
	{
		USART1->DR = TxBuff[TxCount++];//дDR����жϱ�־          
		if(TxCount == Count)
		{
			USART1->CR1 &= ~USART_CR1_TXEIE;//�ر�TXE�ж�
		}
	}
	
	if(USART1->SR & USART_SR_RXNE)//�����ж� 
	{
		volatile int8_t com_data ;
		com_data = USART1->DR;
	}
}




/*****************************************************************************
* ��  ����int fputc(int ch, FILE *f)
* ��  �ܣ��ض��� printf()����
* ��  ����ch Ҫ���͵�����
* ����ֵ����
* ��  ע����
*****************************************************************************/
/*
int fputc(int ch,FILE *f)   //printf�ض�����
{
	USART_SendData(USART1,(uint8_t)ch);   //����һ�ֽ�����
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);   //�ȴ��������
	return ch;
}
*/
/*****************************************************************************
* ��  ����void usart_send(uint8_t *data,uint8_t len)
* ��  �ܣ�Usart����ָ����������
* ��  ����*data Ҫ�������ݵĵ�ַ
*         len   Ҫ�������ݵĳ���
* ����ֵ����
* ��  ע����
*****************************************************************************/
void usart_send(uint8_t *data,uint8_t len)
{
	uint8_t i;
	
	for(i=0;i<len;i++)
	{
		while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);  //������ͱ�־λû���ڷ��͵�ʱ��
		USART_SendData(USART1,*(data+i));
		//USART1->DR = data;
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC) == RESET);
	}
}


/*void usart_send_mpu6050(int16_t data)
{
	
}
*/

//��HEX����ʽ���U8������
void PrintHexU8(uint8_t data)
{
	/*TxBuff[Count++] = data;  
	if(!(USART1->CR1 & USART_CR1_TXEIE))
	USART_ITConfig(USART1, USART_IT_TXE, ENABLE); //��TXE�ж�
	*/
	//USART1->DR = data ;
	
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);  //������ͱ�־λû���ڷ��͵�ʱ��
	USART_SendData(USART1, data);
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC) == RESET);
	//usart_send(TX_DATA, 10);
}
//��HEX����ʽ���S16������
void PrintHexS16(int16_t num)
{
	PrintHexU8((uint8_t)(num & 0xff00) >> 8);//�ȷ��͸�8λ���ٷ��͵�8λ
	PrintHexU8((uint8_t)(num & 0x00ff));
}
//���ַ�����ʽ���S8������
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
//���ַ�����ʽ���U8������
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
//���ַ�����ʽ���S16������ 
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
//���ַ�����ʽ���U16������
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
//����ַ���
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

//���ڷ���������ǰlen������
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


