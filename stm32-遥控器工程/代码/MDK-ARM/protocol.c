//***************************************************************************************
//烈火微型四轴飞行器源码版权归烈火团队所有，未经烈火团队同意，请勿随意在网上传播本源码。
//与本软件相关书籍<<四轴飞行器DIY-基于STM32微控制器>>，由北航出版社正式出版，内容对本套包
//含的所有软件以及硬件相关都做了详细的讲解，有兴趣的网友可以从各大书店购买。
//与本软件配套的硬件：http://fire-dragon.taobao.com
//如果有网友做了各种有意义的改进，请随时与我们保持联系。
//QQ：16053729    烈火QQ群：234879071
//***************************************************************************************
#include "struct_all.h"

/******************************************************************************
							宏定义
*******************************************************************************/ 
#define VERSION 1.1	//版本信息
#define QUADX   3	

#define MSP_VERSION 0
#define CAP_PLATFORM_32BIT          ((uint32_t)1 << 31)
#define CAP_BASEFLIGHT_CONFIG       ((uint32_t)1 << 30)
#define CAP_DYNBALANCE              ((uint32_t)1 << 2)
#define CAP_FW_FLAPS                ((uint32_t)1 << 3)

uint16_t THROTTLE1,THROTTLE2,THROTTLE3,THROTTLE4;
/******************************************************************************
函数原型:	void Print_MSP_RC(uint8_t direction)
功　　能:	发送遥控数据
参    数：   direction：'>';发给上位机；'<';发给飞控
*******************************************************************************/ 
void Print_MSP_RC(uint8_t direction)
{
	uint8_t	data[32];
	uint8_t count;
	
	data[0] = '$';
	data[1] = 'M';
	data[2] = direction;//数据方向设置
	data[3] = 16;
	data[4] = MSP_RC;
	
	data[5] =  Filter_Rc.ROLL & 0xFF ;	
	data[6] = (Filter_Rc.ROLL >> 8)& 0xFF;
	data[7] =  Filter_Rc.PITCH & 0xFF ;	
	data[8] = (Filter_Rc.PITCH >> 8)& 0xFF;
	data[9] =  Filter_Rc.YAW & 0xFF ;	
	data[10] =(Filter_Rc.YAW >> 8)& 0xFF;
	data[11] = Filter_Rc.THROTTLE & 0xFF ;	
	data[12] =(Filter_Rc.THROTTLE >> 8)& 0xFF;
	
	
	//AU1~AU3为遥控器的三个微调旋钮，暂时没有硬件支持
	data[13] = pitch_aux & 0xFF ;//Rc.AUX1 & 0xFF ;	          //pitch
	data[14] = (pitch_aux >> 8)& 0xFF; //(Rc.AUX1 >> 8)& 0xFF;
	data[15] = roll_aux & 0xFF ; //Rc.AUX2 & 0xFF ;	          //roll
	data[16] = (roll_aux >> 8)& 0xFF; //(Rc.AUX2 >> 8)& 0xFF;
	data[17] = Rc.AUX3 & 0xFF ;	          //yaw
	data[18] = (Rc.AUX3 >> 8)& 0xFF;
	
	
	//AU4为遥控器的电池电压
	if(direction == '<')//发给四轴的话，不发送电池余量，而是将该位放入模式
	{
		Rc.AUX4 = 0;
		/*
		if(Mode)//Mode按键
			Rc.AUX4 |= 	Lock_Mode;//锁尾模式
		if(Fun)//Fun按键
			Rc.AUX4 |=  Led_Mode;//夜间模式
		*/
		Rc.AUX4 |= 	Lock_Mode;//锁尾模式   //暂时没有设置按键选择模式功能，默认为锁尾模式
		
		data[19] =  Rc.AUX4 & 0xFF ;	
		data[20] = (Rc.AUX4 >> 8)& 0xFF;
		data[21] = Get_Checksum(data);
		NRF_Send_TX(data,32); 
	}
	else if(direction == '>')//发给上位机
	{
		Rc.AUX4    = 1000+ Battery_Rc;
		
		data[19] =  Rc.AUX4 & 0xFF ;	
		data[20] = (Rc.AUX4 >> 8)& 0xFF;
		data[21] = Get_Checksum(data);
		for(count=0;count<22;count++)
		{
			PrintHexU8(data[count]);
			//PrintU8(data[count]);
		}
	}
}

/******************************************************************************
函数原型:	void Print_MSP_PID(void)
功　　能:	发送PID参数
*******************************************************************************/ 
void Print_MSP_PID(void)
{
	uint8_t	data[40];
	uint8_t count;

	data[0] = '$';
	data[1] = 'M';
	data[2] = '>';//发给上位机
	data[3] = 27;
	data[4] = MSP_PID;

	data[5] =  pid[0].kp & 0xFF;
	data[6] =  pid[0].ki & 0xFF;
	data[7] =  pid[0].kd & 0xFF;
	
	data[8] =  pid[1].kp & 0xFF;
	data[9] =  pid[1].ki & 0xFF;
	data[10]=  pid[1].kd & 0xFF;

	data[11] =  pid[2].kp & 0xFF;
	data[12] =  pid[2].ki & 0xFF;
	data[13] =  pid[2].kd & 0xFF;
	
	data[14] =  pid[3].kp & 0xFF;
	data[15] =  pid[3].ki & 0xFF;
	data[16] =  pid[3].kd & 0xFF;
	
	data[17] =  pid[4].kp & 0xFF;
	data[18] =  pid[4].ki & 0xFF;
	data[19] =  pid[4].kd & 0xFF;
	
	data[20] =  pid[5].kp & 0xFF;
	data[21] =  pid[5].ki & 0xFF;
	data[22] =  pid[5].kd & 0xFF;
	
	data[23] =  pid[6].kp & 0xFF;
	data[24] =  pid[6].ki & 0xFF;
	data[25] =  pid[6].kd & 0xFF;
	
	data[26] =  pid[7].kp & 0xFF;
	data[27] =  pid[7].ki & 0xFF;
	data[28] =  pid[7].kd & 0xFF;
	
	data[29] =  pid[8].kp & 0xFF;
	data[30] =  pid[8].ki & 0xFF;
	data[31] =  pid[8].kd & 0xFF;
	
	data[32] = Get_Checksum(data);
			
	for(count=0;count<33;count++)
	{
		PrintHexU8(data[count]);
	}
}

/******************************************************************************
函数原型:	void Print_MSP_IDENT(void)
功　　能:	发送版本信息
*******************************************************************************/ 
void Print_MSP_IDENT(void)
{
	uint8_t	data[13];
	uint8_t count;

	data[0] = '$';
	data[1] = 'M';
	data[2] = '>';//发给上位机
	data[3] = 7;
	data[4] = MSP_IDENT;
	
	data[5] =  VERSION;
	data[6] =  QUADX;
	data[7] =  MSP_VERSION;
	
	data[8] =  0;
	data[9] =  0;
	data[10]=  0;
	data[11] = 12;

	data[12] = Get_Checksum(data);
	for(count=0;count<13;count++)
	{
		PrintHexU8(data[count]);
	}
}

/******************************************************************************
函数原型:	void Print_MSP_MOTOR_PINS(void)
功　　能:	发送四轴电机IO配置
*******************************************************************************/ 
void Print_MSP_MOTOR_PINS(void)
{
	uint8_t	data[14];
	uint8_t count;

	data[0] = '$';
	data[1] = 'M';
	data[2] = '>';//发给上位机
	data[3] = 8;
	data[4] = MSP_MOTOR_PINS;
	
	data[5] =  3;
	data[6] =  1;
	
	data[7] =  2;
	data[8] =  0;
	
	data[9]  = 0;
	data[10] = 0;
	
	data[11] = 0;
	data[12] = 0;

	data[13] = Get_Checksum(data);
	for(count=0;count<14;count++)
	{
		PrintHexU8(data[count]);
	}	
}

/******************************************************************************
函数原型:	void Print_MSP_ANALOG(void)
功　　能:	发送电压数据
*******************************************************************************/ 
void Print_MSP_ANALOG(void)
{
	uint8_t	data[13];
	uint8_t count;

	data[0] = '$';
	data[1] = 'M';
	data[2] = '>';//发给上位机
	data[3] = 7;
	data[4] = MSP_ANALOG;
	
	data[5] =  (uint8_t)(Battery_Fly/10.0f)& 0xFF;
	
	data[6] =  0;
	data[7] =  0;
	
	data[8] =  0;
	data[9]  = 0;
	
	data[10] = 0;
	data[11] = 0;
	
	data[12] = Get_Checksum(data);
	for(count=0;count<13;count++)
	{
		PrintHexU8(data[count]);
	}
}

/******************************************************************************
函数原型:	void Print_MSP_SET_PID(void)
功　　能:	设置飞控PID参数
*******************************************************************************/ 
void Print_MSP_SET_PID(void)
{
	uint8_t	data[32];

	data[0] = '$';
	data[1] = 'M';
	data[2] = '<';//发给飞控
	data[3] = 9;
	data[4] = MSP_SET_PID;
	
	data[5] =  pid[0].kp & 0xFF;
	data[6] =  pid[0].ki & 0xFF;
	data[7] =  pid[0].kd & 0xFF;
	
	data[8] =  pid[1].kp & 0xFF;
	data[9] =  pid[1].ki & 0xFF;
	data[10]=  pid[1].kd & 0xFF;
	
	data[11] =  pid[2].kp & 0xFF;
	data[12] =  pid[2].ki & 0xFF;
	data[13] =  pid[2].kd & 0xFF;

	data[14] = Get_Checksum(data);
	
	NRF_Send_TX(data,32);
}

/******************************************************************************
函数原型:	void Print_MSP_MOTOR(void)
功　　能:	发送电机PWM值
*******************************************************************************/ 
void Print_MSP_MOTOR(void)
{
	uint8_t	data[32];
	uint8_t count;
	
	data[0] = '$';
	data[1] = 'M';
	data[2] = '>';//发给上位机
	data[3] = 16;
	data[4] = MSP_MOTOR;
	
	data[5] =   (THROTTLE4) & 0xFF;
	data[6] =  ((THROTTLE4) >> 8) & 0xFF;

	data[7] =   (THROTTLE2) & 0xFF;
	data[8] =  ((THROTTLE2) >> 8) & 0xFF;

	data[9] =   (THROTTLE3) & 0xFF;
	data[10] =  ((THROTTLE3) >> 8) & 0xFF;

	data[11] =   (THROTTLE1) & 0xFF;
	data[12] =  ((THROTTLE1) >> 8) & 0xFF;
/////////////////////////////////
	data[13] = 0;
	data[14] = 0;
	
	data[15] = 0;
	data[16] = 0;
	
	data[17] = 0;
	data[18] = 0;

	data[19] = 0;
	data[20] = 0;
	
	data[21] = Get_Checksum(data);
	
	for(count=0;count<22;count++)
	{
		PrintHexU8(data[count]);
	}	
}

/******************************************************************************
函数原型:	void Uart_Check(void)
功　　能:	串口缓冲数据处理
*******************************************************************************/ 
void Uart_Check(void)
{
	uint8_t Line_address;
	if(Line0==1 || Line1==1)
	{
		if(Line0==1)//确定缓冲队列
		{
			Line0 = 0;Line_address = 0;
		}
		else
		{
			Line1 = 0;Line_address = 1;
		}
		
		switch (RxBuff[Line_address][4])
		{
			case MSP_SET_PID : //设置PID参数
					pid[0].kp = RxBuff[Line_address][5];
					pid[0].ki = RxBuff[Line_address][6];
					pid[0].kd = RxBuff[Line_address][7];

					pid[1].kp = RxBuff[Line_address][8];
					pid[1].ki = RxBuff[Line_address][9];
					pid[1].kd = RxBuff[Line_address][10];

					pid[2].kp = RxBuff[Line_address][11];
					pid[2].ki = RxBuff[Line_address][12];
					pid[2].kd = RxBuff[Line_address][13];
					Print_MSP_SET_PID();
				break;
			case MSP_PID :	//读取PID参数
				Print_MSP_PID();
				break;
			case MSP_ACC_CALIBRATION : 	//校正加速度计
				NRF_Send_TX(RxBuff[Line_address],32);
				break;
			case MSP_MAG_CALIBRATION : 	//原是校正磁力计，这里用来校正陀螺仪
				NRF_Send_TX(RxBuff[Line_address],32);
				break;
			case MSP_RESET_CONF : 		//重置PID
			{
				NRF_Send_TX(RxBuff[Line_address],32);
				break;
			}
		}
	}
	//PrintString("\r\n Uart_Check  \r\n");
}
