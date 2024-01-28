
#include "tasks.h"
#include "filter.h"

/******************************************************************************
����ԭ��:	static void Nvic_Init(void)
��������:	NVIC��ʼ��
*******************************************************************************/ 
static void Nvic_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	//NVIC_PriorityGroup 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//���ȼ�����
    //Timer3
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//��ռ���ȼ�
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;//�����ȼ�
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	//Nrf2401�ж�
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn; //NRF_IRQ=PC13, ��ఴ��PC15,�Ҳ�PB11�����stick1=PB10
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	/*
	//����
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	*/
	
//	//MODE����
//	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn; //���Խ�����PA1��PB1����
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
	
	//������п����Ǹ����⣬��ȷ������Ҫ��һ��
	//����Ӳ�����ϵ�������⣬������ఴ��PC15,�Ҳ�PB11�����stick1=PB10����nrf24L01��ͬһ���ж�����
	//Ϊ�˱�֤nrf24L01���ж�����ʹ�ã���ʱ�����⼸�����������жϹ���
	
	//FUN���� �Ҳ�stick2����
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;  //PA8���Ҳ�stick2����
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	
}

/******************************************************************************
����ԭ�ͣ�	void BSP_Int(void)
��    �ܣ�	Ӳ��������ʼ��
*******************************************************************************/ 
void BSP_Int(void)
{
	LED_KEY_Init();//LED\������ʼ��
	LED_ON_OFF();//LED��˸
	
	USART_init(115200);	//���ڳ�ʼ����������115200��8λ���ݣ�1λֹͣλ��������żУ��
	PrintString("\r\n LED USART init!!! \r\n");
	Timer3_Init(500);//Timer3�ж�500HZ
	PrintString("\r\n Timer3 init!!! \r\n");
	Nvic_Init();//�ж����ȼ���ʼ��	
	PrintString("\r\n Nvic init!!! \r\n");
	
	
	ADC1_Init();//ADC��DMA��ʼ��
	PrintString("\r\n ADC1 DMA init!!! \r\n");

	//SPI2_Init();//SPI2��ʼ��
	//NRF24L01_Init(40,TX);//2401ѡ��40ͨ��������ģʽ
	NRF24L01_IO_Init(); //���������SPI1�ĳ�ʼ��
	PrintString("\r\n NRF24L01 init!!! \r\n");
	NRF24L01_TX_Mode(); //����ģʽ
	//NRF24L01_Init(0,TX); //��ʼ��Ϊͨ��0������ģʽ
 	NRF24L01_Check();//���2401�Ƿ�����
	PrintString("\r\n nrf24L01 checked!!! \r\n");
	PrintString("\r\n Dream_Flyer_RC V1.1 \r\n");//�汾��
	Bsp_Int_Ok = 1;
}

/******************************************************************************
����ԭ�ͣ�	void Task_500HZ(void)
��    �ܣ�	��ѭ��������Ƶ��Ϊ500HZ����
*******************************************************************************/ 
void Task_500HZ(void)
{
	//Rc_Filter(&Rc,&Filter_Rc);//����ң��ָ��+�˲�
	//Nrf_Connect();//NRF24L01���Ӻ���
	
}

/******************************************************************************
����ԭ�ͣ�	void Task_100HZ(void)
��    �ܣ�	��ѭ��������Ƶ��Ϊ100HZ����
*******************************************************************************/ 
void Task_100HZ(void)
{
	//Print_MSP_RC('<');//����ң���źŸ��ɿ�
	
}

/******************************************************************************
����ԭ�ͣ�	void Task_25HZ(void)
��    �ܣ�	��ѭ��������Ƶ��Ϊ25HZ����
*******************************************************************************/ 
void Task_25HZ(void)
{
	//Print_MSP_RC('>');//����ң���źŸ���λ��
	LED_Show();//LEDָʾ����˸
	Rc_Filter(&Rc,&Filter_Rc);//����ң��ָ��+�˲�
	Nrf_Connect();//NRF24L01���Ӻ���
	//Print_MSP_MOTOR();//���͵��ת�ٸ���λ��
	//PrintString("\r\n ң�����ź� \r\n");//�汾��
}

/******************************************************************************
����ԭ�ͣ�	void Task_4HZ(void)
��    �ܣ�	��ѭ��������Ƶ��Ϊ4HZ����
*******************************************************************************/ 
void Task_5HZ(void)
{
	

  Print_MSP_RC('<');//����ң���źŸ��ɿ�
	
	
	//static uint8_t turn_count;	
		//Print_MSP_RC('>');
//	turn_count++;
//	switch(turn_count)
//	{
//		case 1: PrintString("analog:"); Print_MSP_ANALOG();		break;
//		case 2: PrintString("ident:"); Print_MSP_IDENT();		break;
//		case 3: PrintString("analog"); Print_MSP_ANALOG();		break;				
//		case 4: PrintString("motor:"); Print_MSP_MOTOR_PINS();	
//				turn_count=0; 			break;
//	}
	
	
}


