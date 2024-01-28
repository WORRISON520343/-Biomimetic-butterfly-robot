
#include "spi.h"
//#include "usart.h"


/*****************************************************************************
* ��  ����void SPI_GPIO_Init(void)
* ��  �ܣ�����SI24R1�� SCK��MISO��MOSI���ţ��Լ�SPI2��ʼ��
* ��  ������
* ����ֵ����
* ��  ע������SPIͨ��ʱһ��Ҫ���������ӻ�ģʽ
*         �����ӻ�ģʽ�� ����״̬ ��ƽ
*		  2.4Gģ��ͨ��ʱ��SPI����һ�㲻����10Mbps
*****************************************************************************/
void SPI_GPIO_Init(void)
{
	SPI_InitTypeDef   SPI_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure;
	

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);
	//����SPI��SCK��MISO��MOSI����Ϊ��������ģʽ
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	//SPI1��SCK,,MOSI��ӦPA5��PA7
	//SCK��MOSI,MISO����Ϊ��������ģʽ
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_5|GPIO_Pin_7|GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	//MISO PA6
	//GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
  //GPIO_Init(GPIOA,&GPIO_InitStructure);
//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource5,GPIO_AF_SPI1); //PB3����Ϊ SPI1
//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_SPI1); //PB4����Ϊ SPI1
//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_SPI1); //PB5����Ϊ SPI1

	//RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1,ENABLE);//��λSPI1
	//RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1,DISABLE);//ֹͣ��λSPI1
	//SPI_Cmd(SPI1, DISABLE); //ʧ��SPI����
	SPI_InitStructure.SPI_Mode=SPI_Mode_Master;	//����Ϊ����ģʽ
	SPI_InitStructure.SPI_NSS=SPI_NSS_Soft;		//NSS�������
	SPI_InitStructure.SPI_CPHA=SPI_CPHA_1Edge;	// SPI_CPHA_1Edge ��һ��ʱ���ز���; SPI_CPHA_2Edge
	SPI_InitStructure.SPI_CPOL=SPI_CPOL_Low;	//����״̬Ϊ�͵�ƽ
	SPI_InitStructure.SPI_DataSize=SPI_DataSize_8b;						//8λ����֡
	SPI_InitStructure.SPI_BaudRatePrescaler=SPI_BaudRatePrescaler_16; 	//8,16, 256 //SPI������8��Ƶ 	36/8=4.5M
	SPI_InitStructure.SPI_Direction=SPI_Direction_2Lines_FullDuplex;	//ȫ˫��ģʽ
	SPI_InitStructure.SPI_FirstBit=SPI_FirstBit_MSB;					//���ݸ�λ����
	SPI_InitStructure.SPI_CRCPolynomial=7;								//CRC�������ʽ
	SPI_Init(SPI1,&SPI_InitStructure);

	SPI_Cmd(SPI1,ENABLE);	//SPI1ʹ��
	//SPI1_RW(0xff);//��������	
}



//SPI1�ٶ����ú���
//SPI�ٶ�=fAPB2/��Ƶϵ��
//@ref SPI_BaudRate_Prescaler:SPI_BaudRatePrescaler_2~SPI_BaudRatePrescaler_256  
//fAPB2ʱ��һ��Ϊ84Mhz��
void SPI1_SetSpeed(uchar SPI_BaudRatePrescaler)
{
	//printf("����spi1���ٶ�***");
  assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));//�ж���Ч��
	SPI1->CR1&=0XFFC7;//λ3-5���㣬�������ò�����
	SPI1->CR1|=SPI_BaudRatePrescaler;	//����SPI1�ٶ� 
	SPI_Cmd(SPI1,ENABLE); //ʹ��SPI1
	//printf("����spi1���ٶ����");
} 


uchar SPI1_RW(uchar data)
{
	//printf("SPI1 RW��ʼ");
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET){}//�ȴ���������  
	SPI_I2S_SendData(SPI1, data); //ͨ������SPIx����һ��byte  ����
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET){} //�ȴ�������һ��byte  
	//printf("SPI1 RW����");
	return SPI_I2S_ReceiveData(SPI1); //����ͨ��SPIx������յ�����	
}


/*
//ʹ�����ģ��SPI
uchar SPI2_WriteReadByte(uchar byte)
{
	uchar bit_ctr;
	for(bit_ctr=0;bit_ctr<8;bit_ctr++)  // ���8λ
	{
		if((uchar)(byte&0x80)==0x80)
		NRF_MOSI=1; 			// MSB TO MOSI
		else 
			NRF_MOSI=0; 
		byte=(byte<<1);					// shift next bit to MSB
		NRF_SCK=1;
		byte|=NRF_MISO;	        		// capture current MISO bit
		NRF_SCK=0;
	}
	return byte;
}
*/
