//***************************************************************************************
//烈火微型四轴飞行器源码版权归烈火团队所有，未经烈火团队同意，请勿随意在网上传播本源码。
//与本软件相关书籍<<四轴飞行器DIY-基于STM32微控制器>>，由北航出版社正式出版，内容对本套包
//含的所有软件以及硬件相关都做了详细的讲解，有兴趣的网友可以从各大书店购买。
//与本软件配套的硬件：http://fire-dragon.taobao.com
//如果有网友做了各种有意义的改进，请随时与我们保持联系。
//QQ：16053729    烈火QQ群：234879071
//***************************************************************************************
#include "filter.h"
#include "struct_all.h"

/******************************************************************************
函数原型：	void RC_Limit(struct _Rc *rc)
功    能：	限制遥控指令范围
*******************************************************************************/ 
void RC_Limit(struct _Rc *rc)
{
	rc->THROTTLE = (rc->THROTTLE<=1000)?1000:rc->THROTTLE; 
	rc->THROTTLE = (rc->THROTTLE>=2000)?2000:rc->THROTTLE; 
	rc->PITCH = (rc->PITCH<=1000)?1000:rc->PITCH; 
	rc->PITCH = (rc->PITCH>=2000)?2000:rc->PITCH; 
	rc->ROLL = (rc->ROLL<=1000)?1000:rc->ROLL; 
	rc->ROLL = (rc->ROLL>=2000)?2000:rc->ROLL; 
	rc->YAW  = (rc->YAW<=1000)?1000:rc->YAW; 
	rc->YAW  = (rc->YAW>=2000)?2000:rc->YAW; 
	
	rc->AUX1 = (rc->AUX1<=1000)?1000:rc->AUX1; 
	rc->AUX1 = (rc->AUX1>=2000)?2000:rc->AUX1; 
	rc->AUX2 = (rc->AUX2<=1000)?1000:rc->AUX2; 
	rc->AUX2 = (rc->AUX2>=2000)?2000:rc->AUX2; 
	rc->AUX3 = (rc->AUX3<=1000)?1000:rc->AUX3; 
	rc->AUX3 = (rc->AUX3>=2000)?2000:rc->AUX3; 
	
}

#define Filter_Num 5
/******************************************************************************
函数原型：	void Rc_Filter( struct _Rc *Rc_in,struct _Rc *Rc_out)
功    能：	遥控指令滤波
*******************************************************************************/ 
void Rc_Filter( struct _Rc *Rc_in,struct _Rc *Rc_out)
{
	static uint16_t Filter_A[Filter_Num],Filter_B[Filter_Num],Filter_C[Filter_Num],Filter_D[Filter_Num];
	static uint8_t Filter_count;
	uint32_t Filter_sum_A=0,Filter_sum_B=0,Filter_sum_C=0,Filter_sum_D=0;
	uint8_t i=0;
//////////////////////////////////////////////////////////////////
//			由AD值计算遥控数据
//////////////////////////////////////////////////////////////////
//#ifdef American_Mode//美国手
	Rc_in->THROTTLE = 	1000 + (uint16_t)(0.25f*ADC_Value[4]) - offset0;//油门
	Rc_in->PITCH 	= 	2000 - (uint16_t)(0.25f*ADC_Value[2]) - offset2;//升降舵
//#else //日本手
//	Rc_in->THROTTLE = 	2000 - (uint16_t)(0.25f*ADC_Value[7]) - offset0;//油门
//	Rc_in->PITCH 	= 	1000 + (uint16_t)(0.25f*ADC_Value[4]) - offset2;//升降舵
//#endif
	
	Rc_in->YAW 		= 	1000 + (uint16_t)(0.25f*ADC_Value[3]) - offset1;//方向
	Rc_in->ROLL 	=	2000 - (uint16_t)(0.25f*ADC_Value[1]) - offset3;//副翼
	
	
	//V1.1版本的遥控器没有微调旋钮，但是实际飞行起步时会偏向一边，这里手动设置微调幅度，看看效果
	//1500，1400~1600等于没有设置 
	Rc_in->AUX1 	= 	1500;//1000 + (uint16_t)(0.25f*ADC_Value[2]) - offset4;//升降舵微调  //pitch
	Rc_in->AUX2	 	= 	1500;//1000 + (uint16_t)(0.25f*ADC_Value[0]) - offset5;//副翼微调    //roll
	Rc_in->AUX3 	= 	1500;//1000 + (uint16_t)(0.25f*ADC_Value[1]) - offset6;//方向微调    //yaw
	
	
	Battery_Rc = (uint16_t)(2.0f*ADC_Value[0]/ADC_Value[5]*1.2f*100);//遥控电压值的100倍 //adc_value的第5个值是内部参考电压
//////////////////////////////////////////////////////////////////
//			将遥控数据限制在1000~2000
//////////////////////////////////////////////////////////////////
	RC_Limit(Rc_in);
	Do_Offset();//遥控上电后自动对中，响一声后即对中完成	
//////////////////////////////////////////////////////////////////
//			遥控指令滤波
//////////////////////////////////////////////////////////////////
	Filter_A[Filter_count] = Rc_in->PITCH;
	Filter_B[Filter_count] = Rc_in->ROLL;
	Filter_C[Filter_count] = Rc_in->THROTTLE;
	Filter_D[Filter_count] = Rc_in->YAW;
	
	for(i=0;i<Filter_Num;i++)
	{
		Filter_sum_A += Filter_A[i];
		Filter_sum_B += Filter_B[i];
		Filter_sum_C += Filter_C[i];
		Filter_sum_D += Filter_D[i];
	}	
	
	Rc_out->PITCH 		= Filter_sum_A / Filter_Num;
	Rc_out->ROLL  		= Filter_sum_B / Filter_Num;
	Rc_out->THROTTLE 	= Filter_sum_C / Filter_Num;
	Rc_out->YAW 	 	= Filter_sum_D / Filter_Num;
	
	Filter_count++;
	if(Filter_count == Filter_Num)
		Filter_count=0;
}


