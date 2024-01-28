#ifndef   _SPI_H
#define   _SPI_H


#include "stm32f10x.h"
#include "stm32f10x_spi.h"

#define uchar unsigned char
#define uint unsigned int


void SPI_GPIO_Init(void);
void SPI1_SetSpeed(uchar SpeedSet); //����SPI1�ٶ�  
//void SPI2_SetSpeed(u8 SpeedSet); //����SPI2�ٶ� 

//uchar SPI1_WriteReadByte(uchar data);
uchar SPI1_RW(uchar byte);


//uint8_t SPI2_WriteReadByte(uint8_t data);
//uint8_t SPI2_RW(uint8_t byte);
#endif

