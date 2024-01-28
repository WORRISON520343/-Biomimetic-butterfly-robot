#ifndef   _USART_H
#define   _USART_H
//#include "stdint.h"
#include "stm32f10x.h"
#include "stm32f10x_usart.h"

/******************************************************************************
							ȫ�ֱ�������
*******************************************************************************/ 
extern uint8_t RxBuff[2][50];//���ڽ��ջ�����
extern uint8_t Line0,Line1;	 //���ڽ���˫�����л�

void USART_init(uint32_t baudrate);
void usart_send(uint8_t *data,uint8_t len);
void usart_send_mpu6050(int16_t *data,uint8_t len);
void PrintHexU8(uint8_t data);
void PrintHexS16(int16_t num);
void PrintS8( int8_t num);
void PrintU8(uint8_t num);
void PrintS16( int16_t num);
void PrintU16(uint16_t num);
void PrintString(uint8_t *s);
void Uart_Send(uint8_t *s ,uint8_t len);	

#endif

