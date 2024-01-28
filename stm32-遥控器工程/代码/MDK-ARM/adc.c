#include "adc.h"
//#include "delay.h"
#include "stm32f10x_dma.h"
#include "usart.h"

uint16_t Battery_Fly, Battery_Rc;//四轴电压和遥控电压的100倍
uint16_t ADC_Value[M]; 

/******************************************************************************
函数原型：	static void ADC1_GPIO_Config(void)
功    能：	ADC的IO初始化
*******************************************************************************/ 
static void ADC1_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/* 打开GPIO和ADC、DMA部件的时钟 */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA |RCC_APB2Periph_GPIOB| RCC_APB2Periph_AFIO, ENABLE);
	RCC_ADCCLKConfig(RCC_PCLK2_Div6); //72M/6=12,ADC最大时间不能超过14M
	

	
	/* 将GPIO配置为输入模式 */	
  	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_0 | GPIO_Pin_1 |GPIO_Pin_2;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  	GPIO_Init(GPIOA , &GPIO_InitStructure);
	
	  GPIO_InitStructure.GPIO_Pin =GPIO_Pin_0 | GPIO_Pin_1 ;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  	GPIO_Init(GPIOB , &GPIO_InitStructure);
}

/******************************************************************************
函数原型：	static void ADC1_Mode_Config(void)
功    能：	ADC及DMA配置
*******************************************************************************/ 
static void ADC1_Mode_Config(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	
	/* DMA 通道1 配置 */
	DMA_DeInit(DMA1_Channel1);//将DMA的通道1寄存器重设为缺省值
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;//DMA外设ADC基地址;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ADC_Value;//DMA内存基地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;//内存作为数据传输的目的地
	DMA_InitStructure.DMA_BufferSize = M;//DMA通道的DMA缓存的大小
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设地址寄存器不变
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//内存地址寄存器递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//数据宽度为16位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//数据宽度为16位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//工作在循环缓存模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;//DMA通道x拥有高优先级
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;//DMA通道x没有设置为内存到内存传输
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);//根据DMA_InitStruct中指定的参数初始化DMA的通道

	/* 使能 DMA 通道1 */
	DMA_Cmd(DMA1_Channel1, ENABLE);
//////////////////////////////////////////////////////////////////////////////////////////////
	ADC_DeInit(ADC1); //将外设 ADC1 的全部寄存器重设为缺省值
	ADC_TempSensorVrefintCmd(ENABLE);//使能内部参照电压（1.2V）
	/* ADC1 配置 */
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;//独立模式 每个ADC独立工作
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;//使用扫描模式  scan位设置
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;//cont位设置 连续转换模式
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;//EXTSEL 选择启动规则通道组转换的外部事件 设置成有软件控制
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//数据对齐 由软件置位和清楚   这里设置成右对齐
	ADC_InitStructure.ADC_NbrOfChannel = M;//规则通道序列长度 这些位由软件定义在规则通道转换序列中的通道数目 1个转换 指定由多少个通道被转换
	ADC_Init(ADC1, &ADC_InitStructure);

	/* 设置 ADC1 使用8转换通道，在规则组里的转换顺序--->1，普通采样时间为 71.5 周期 */ 	
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_239Cycles5 );// (电池电压)
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_71Cycles5 );// ROLL
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_71Cycles5 );// PITCh
	ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 4, ADC_SampleTime_71Cycles5);// YAW
	ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 5, ADC_SampleTime_71Cycles5 );//THROTTLE
	ADC_RegularChannelConfig(ADC1, ADC_Channel_17,6, ADC_SampleTime_239Cycles5);//内部参照电压（1.2V）	

	/* 使能 ADC1 DMA */
	ADC_DMACmd(ADC1, ENABLE);
	/* 使能 ADC1 */
	ADC_Cmd(ADC1, ENABLE);
	/* 复位 ADC1 的校准寄存器 */   
	ADC_ResetCalibration(ADC1);
	/* 等待 ADC1 校准寄存器复位完成 */
	while(ADC_GetResetCalibrationStatus(ADC1));
	/* 开始 ADC1 校准 */
	ADC_StartCalibration(ADC1);
	/* 等待 ADC1 校准完成 */
	while(ADC_GetCalibrationStatus(ADC1));
	/* 使能指定的ADC1的软件转换启动功能 */ 
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

/******************************************************************************
函数原型：	void ADC1_Init(void)
功    能：	ADC1初始化
*******************************************************************************/ 
void ADC1_Init(void)
{
	ADC1_GPIO_Config();
	ADC1_Mode_Config();
}

/******************************************************************************
函数原型：	void Voltage_Printf(void)
功    能：	输出AD各通道电压值一百倍
*******************************************************************************/ 
void Voltage_Printf(void)
{
	uint8_t i;
	uint16_t Voltage;
	
	PrintString("\r\n");
	for(i=0;i<M-1;i++)
	{
		Voltage = (uint16_t)(2.0f*ADC_Value[i]/ADC_Value[M-1]*1.2f*100);
		PrintU8(i);
		PrintString("通道： ");
		PrintU16(Voltage); 
		PrintString("\r\n");
	}
}




/*
void Adc_Init(void) 				//ADC通道初始化
{
	
	GPIO_InitTypeDef  GPIO_Initstruct;
	ADC_InitTypeDef   ADC_Initstruct;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE); //打开PA AFOI ADC的时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB,ENABLE);
	
	RCC_ADCCLKConfig(RCC_PCLK2_Div6); //AD的分频因子确保不允许超过14Mhz 72/6=12
	
  GPIO_Initstruct.GPIO_Mode= GPIO_Mode_AIN;  //模拟输入  ADC专属模式
	GPIO_Initstruct.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2;
	GPIO_Initstruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_Initstruct);//初始化PA2
	
	GPIO_Initstruct.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1;
	GPIO_Initstruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_Initstruct);//初始化PA2
	
	ADC_Initstruct.ADC_Mode=ADC_Mode_Independent;  //模式选择，这里是独立就是不用ADC1和ADC2同时进行
	ADC_Initstruct.ADC_DataAlign=ADC_DataAlign_Right;  //寄存器数据对齐方式右
	ADC_Initstruct.ADC_ExternalTrigConv=ADC_ExternalTrigConv_None; //触发方式禁止外部，使用软件触发
	ADC_Initstruct.ADC_ContinuousConvMode=DISABLE;  //关闭连续转换所以是单次转换模式
  ADC_Initstruct.ADC_ScanConvMode=DISABLE; //非多通道扫面模式 单通道模式
  ADC_Initstruct.ADC_NbrOfChannel=1;  //常规则序列通道数量
	ADC_Init(ADC1,&ADC_Initstruct);
	
	ADC_Cmd(ADC1,ENABLE);  //ADC使能校准给ADC上电
	
	//以下的为软件触发程序
	ADC_ResetCalibration(ADC1);//复位指定的ADC的校准寄存器
	while(ADC_GetResetCalibrationStatus(ADC1) == SET);//获取ADC复位校准寄存器的状态  判断是否空循环秒如果是那么跳出然后获取标志位校准
	ADC_StartCalibration(ADC1);//开始指定ADC的校准状态
  while(ADC_GetCalibrationStatus(ADC1) == SET);//开始获取指定ADC的校准程序
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);//使能指定的ADC的软件转换启动功能

}


//获得ADC值
//ch: @ref ADC_channels 
//通道值 0~16取值范围为：ADC_Channel_0~ADC_Channel_16
//返回值:转换结果
u16 Get_Adc(u8 ch)   
{
	  	//设置指定ADC的规则组通道，一个序列，采样时间
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_71Cycles5 );	//ADC_SampleTime_239Cycles5，ADC1,ADC通道,239个周期,提高采样时间可以提高精确度			    
  
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//使能指定的ADC1的软件转换启动功能	
	 
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//等待转换结束

	return ADC_GetConversionValue(ADC1);	//返回最近一次ADC1规则组的转换结果
}


u16 Get_Adc_Average(u8 ch,u8 times)//得到某个通道给定次数采样的平均值  
{
	u32 temp_val=0;
	u8 t;
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_71Cycles5); //239
	for(t=0;t<times;t++)
	{
		ADC_SoftwareStartConvCmd(ADC1, ENABLE);
		while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));
		temp_val+=ADC_GetConversionValue(ADC1);
		Delay_us(1);
	}
	return temp_val/times;
}
*/
