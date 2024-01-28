//=============================================================================
//文件名称：main.h
//功能概要：STM32F103C8核心检测
//版权所有：源地工作室www.vcc-gnd.com
//版权更新：2013-02-20
//调试方式：J-Link OB ARM SW方式 5MHz
//=============================================================================

//头文件
#include "stm32f10x.h"

#include "tasks.h"
//#include "maths.h"
#include "filter.h"
#include "struct_all.h"


//=============================================================================
//文件名称：main
//功能概要：主函数
//参数说明：无
//函数返回：int
//=============================================================================
int main(void)
{
	BSP_Int();	//底层驱动初始化
	while(1)
	{
		/*
		//ADC DMA没有问题
		uint16_t thro, yaw, roll,pitch;
		thro=ADC_Value[4];
		yaw=ADC_Value[3];
		pitch=ADC_Value[2];
		roll=ADC_Value[1];
		if(thro>2300 || yaw>2300 || pitch>2300 || roll>2300)
		{		LEDRed_ON;
		LEDGreen_OFF;}
		else
		{
		LEDRed_OFF;
		LEDGreen_ON;
		}
		Voltage_Printf();
		*/
		
		/*
		uint16_t thro, yaw, roll, pitch;
		Rc_Filter(&Rc,&Filter_Rc);//计算遥控指令+滤波
		thro = Filter_Rc.THROTTLE;
		//PrintU16(thro);
		yaw=Filter_Rc.YAW;
		//PrintU16(yaw);
		pitch=Filter_Rc.PITCH;
		//PrintU16(pitch);
		roll=Filter_Rc.ROLL;
		//PrintU16(roll);
		if(thro>1600 || yaw>1600 || pitch>1600 || roll>1600)
		{		LEDRed_ON;
		LEDGreen_OFF;}
		else
		{
		LEDRed_OFF;
		LEDGreen_ON;
		}
		//Task_1HZ();
		*/
		
		//Uart_Check();//处理串口接收到的数据
//		if(Count_2ms>=1)
//		{
//			Count_2ms = 0;
//			Task_500HZ();
//		}
//		if(Count_10ms>=5)
//		{
//			Count_10ms = 0;
//			Task_100HZ();
//		}
		if(Count_40ms>=20)
		{
			Count_40ms = 0;
			Task_25HZ();
		}
		if(Count_200ms>=100)
		{
			Count_200ms = 0;
			Task_5HZ();
		}
		
		
		
	}
}










	/*
		//NRF24L01_TxPacket(TX_DATA);
	rece_buf[1]=0xc9;					//上
	rece_buf[2]=0xcf;					//上
	rece_buf[3]=0xba;					//海
	rece_buf[4]=0xa3;					//海
	rece_buf[5]=0xb1;					//宝
	rece_buf[6]=0xa6;					//宝
	rece_buf[7]=0xb1;					//宝
	rece_buf[8]=0xa6;					//宝
	//rece_buf[7]=0xc7;					//嵌
	//rece_buf[8]=0xb6;					//嵌
	rece_buf[9]=0xb5;					//电
	rece_buf[10]=0xe7;				//电
	rece_buf[11]=0xd7;				//子
	rece_buf[12]=0xd3;				//子
	rece_buf[0]=0xc9;						//一共要发送12个字节，rece_buf[0]必须是12！！！！！！
	//SEND_BUF(rece_buf);
			
			PBout(3)=1;
			PBout(4)=1;
			Delay(0xfffff);
		
			PBout(3)=0;
			PBout(4)=0;
			Delay(0xfffff);	
		
			if(NRF24L01_TxPacket(rece_buf)==TX_OK)
         {
					 printf("NRF24l01发送完成！");
					 printf("\n");  
         }
       else
         {                                               
             printf("Send failed !");
             printf("\n");            
         }
   */

	 //AD0=Get_ADC_Value(ADC_Channel_0,20);   //获取ADC的通道0
	 /*
	 AD1=Get_Adc_Average(ADC_Channel_1,5);   //获取ADC的通道1
	 AD2=Get_Adc_Average(ADC_Channel_2,5);   //获取ADC的通道2
	 AD3=Get_Adc_Average(ADC_Channel_8,5);   //获取ADC的通道1
	 AD4=Get_Adc_Average(ADC_Channel_9,5);   //获取ADC的通道2
	 */
	 /*
	 AD1=Get_Adc(ADC_Channel_1);
	 //printf("AD1");
	 //PrintU16(AD1);
	 if(AD1>2500)
	 {
	 PBout(3)=0;
	 PBout(4)=1;
	 }
	 else if(AD1<2000) 
	 {
	 PBout(3)=1;
	 PBout(4)=0;
	 }
	 else{
	 PBout(3)=1;
	 PBout(4)=1;
	 }
	 */
	 
	 //printf("\n");
	 /*
	 printf("AD2");
	 PrintU16(AD2);
	 printf("\n");
	 printf("AD3");
	 PrintU16(AD3);
	 printf("\n");
	 printf("AD4");
	 PrintU16(AD4);
	 printf("\n");
	 */
	 
	 /*
	 		PBout(3)=0;
			PBout(4)=1;
			Delay(0xfffff);
		
			PBout(3)=1;
			PBout(4)=0;
			Delay(0xfffff);	
		*/
		
	 /*
	 //ADv0=(float)AD0 / 4095 *3.3;    //这里获取值后需要除以刚才上面说的右对齐4096因为从0开始算所以是4095乘上电压值
	 ADv1=(float)AD1 / 4095 *3.3;
	 ADv2=(float)AD2 / 4095 *3.3;
	 ADv3=(float)AD3 / 4095 *3.3;
	 ADv4=(float)AD4 / 4095 *3.3;
		*/
		

