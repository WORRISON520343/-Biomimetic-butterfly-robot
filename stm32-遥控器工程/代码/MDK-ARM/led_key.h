#ifndef _LED_KEY_H_
#define _LED_KEY_H_

#include "stm32f10x.h"

/******************************************************************************
							�궨��
*******************************************************************************/ 
#define Bee_GPIO		GPIOC
#define Bee_Pin			GPIO_Pin_14

#define LEDRed_GPIO		GPIOB
#define LEDGreen_GPIO		GPIOB
//#define LED_GPIO		GPIOB
#define LEDRed_Pin		GPIO_Pin_4
#define LEDGreen_Pin	GPIO_Pin_3

#define Key_Right		GPIOC
#define Key_Right_Pin GPIO_Pin_15
#define Key_Left		GPIOB
#define Key_Left_Pin GPIO_Pin_11

#define Key_Stick_Left GPIOB
#define Key_Stick_Left_Pin GPIO_Pin_10
#define Key_Stick_Right GPIOA
#define Key_Stick_Right_Pin GPIO_Pin_8

#define Bee_ON 			Bee_GPIO->BSRR = Bee_Pin 		//������  //BSSR ���øߵ�ƽ
#define Bee_OFF  		Bee_GPIO->BRR  = Bee_Pin


#define LEDRed_ON 		LEDRed_GPIO->BRR    = LEDRed_Pin 	//��LED��,��ߵĵ�
#define LEDRed_OFF  	LEDRed_GPIO->BSRR   = LEDRed_Pin  //BSSR ����Ϊ�ߵ�ƽ��С����
#define LEDGreen_ON  	LEDGreen_GPIO->BRR  = LEDGreen_Pin	//��LED�ƣ��ұߵĵ�
#define LEDGreen_OFF 	LEDGreen_GPIO->BSRR = LEDGreen_Pin		


/******************************************************************************
							ȫ�ֺ�������
*******************************************************************************/ 
/******************************************************************************
							ȫ�ֱ�������
*******************************************************************************/ 
extern uint8_t offset;
extern int16_t offset0,offset1,offset2,offset3;
extern int16_t offset4,offset5,offset6;
extern uint8_t Mode,Fun;

extern uint16_t roll_aux, pitch_aux;


void LED_KEY_Init(void);
void LED_ON_OFF(void);
void LED3_ON_Frequency(uint8_t Light_Frequency);
void LED_Show(void);

//void KEY_Init(void);
void Do_Offset(void);

#endif

