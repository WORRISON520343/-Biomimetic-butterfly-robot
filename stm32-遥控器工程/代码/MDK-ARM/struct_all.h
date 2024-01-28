#ifndef _STRUCT_ALL_H_
#define _STRUCT_ALL_H_
#include "stm32f10x.h"


#include "led_key.h"
#include "usart.h" 
#include "maths.h" 
#include "timer3.h"
#include "adc.h"
#include "protocol.h"
#include "nrf24L01.h"
//#include ""

/******************************************************************************
							�궨��
*******************************************************************************/ 
#define American_Mode //ע�͵��ú��ʾʹ���ձ��֣�����Ĭ��ʹ��������
#define Lock_Mode (1<<0)//��β 1
#define Led_Mode  (1<<1)//ҹ��ģʽ 2

/******************************************************************************
							ȫ�ֺ�������
*******************************************************************************/ 
void BSP_Int(void);
//void EEPROM_INIT(void);
//void EEPROM_SAVE_OFFSET(void);
//void EEPROM_READ_OFFSET(void);
	
/******************************************************************************
							�ṹ������
*******************************************************************************/ 
/* pid���� */
struct _pid
{
	uint8_t kp;	//0.1
	uint8_t ki;	//0.001
	uint8_t kd;	//1
};
extern struct _pid pid[9];

/* ң��ͨ�� */
struct _Rc
{
	uint16_t THROTTLE;
	uint16_t YAW;
	uint16_t PITCH;
	uint16_t ROLL;
	
	uint16_t AUX1; 
	uint16_t AUX2;
	uint16_t AUX3;
	uint16_t AUX4;//��ص�ѹ
};
extern struct _Rc Rc;
extern struct _Rc Filter_Rc;


extern uint8_t Bsp_Int_Ok;

#endif

