//***************************************************************************************
//烈火微型四轴飞行器源码版权归烈火团队所有，未经烈火团队同意，请勿随意在网上传播本源码。
//与本软件相关书籍<<四轴飞行器DIY-基于STM32微控制器>>，由北航出版社正式出版，内容对本套包
//含的所有软件以及硬件相关都做了详细的讲解，有兴趣的网友可以从各大书店购买。
//与本软件配套的硬件：http://fire-dragon.taobao.com
//如果有网友做了各种有意义的改进，请随时与我们保持联系。
//QQ：16053729    烈火QQ群：234879071
//***************************************************************************************
//#include "EEPROM.h"
#include "struct_all.h"

/* pid参数 */
struct _pid pid[9];

/* 遥控通道 */
struct _Rc Rc;
struct _Rc Filter_Rc;

uint8_t Bsp_Int_Ok = 0;

///////////////////////////////////////////////////////////
#define EEPROM_offset0		0
#define EEPROM_offset1		1
#define EEPROM_offset2		2
#define EEPROM_offset3		3
#define EEPROM_offset4		4
#define EEPROM_offset5		5
#define EEPROM_offset6		6

uint16_t data_0,data_1,data_2,data_3;

/*
uint16_t VirtAddVarTab[NumbOfVar] = {
	0x9A00,0x9A01,0x9A02,0x9A03,0x9A04,0x9A05,0x9A06
};//虚拟地址
*/

/*
void EEPROM_INIT(void)//初始化模拟的EEPROM，读取参数
{
	EE_Init();
	EEPROM_READ_OFFSET();
}

void EEPROM_SAVE_OFFSET(void)//保存摇杆零偏数据
{
	EE_WriteVariable(VirtAddVarTab[EEPROM_offset0], offset0);
	EE_WriteVariable(VirtAddVarTab[EEPROM_offset1], offset1);
	EE_WriteVariable(VirtAddVarTab[EEPROM_offset2], offset2);
	EE_WriteVariable(VirtAddVarTab[EEPROM_offset3], offset3);
	
	EE_WriteVariable(VirtAddVarTab[EEPROM_offset4], offset4);
	EE_WriteVariable(VirtAddVarTab[EEPROM_offset5], offset5);
	EE_WriteVariable(VirtAddVarTab[EEPROM_offset6], offset6);
}

void EEPROM_READ_OFFSET(void)//读取摇杆零偏数据
{
	EE_ReadVariable(VirtAddVarTab[EEPROM_offset0], &data_0);
	EE_ReadVariable(VirtAddVarTab[EEPROM_offset1], &data_1);
	EE_ReadVariable(VirtAddVarTab[EEPROM_offset2], &data_2);
	EE_ReadVariable(VirtAddVarTab[EEPROM_offset3], &data_3);
	offset0 = (int16_t) data_0;
	offset1 = (int16_t) data_1;
	offset2 = (int16_t) data_2;
	offset3 = (int16_t) data_3;
	
	EE_ReadVariable(VirtAddVarTab[EEPROM_offset4], &data_0);
	EE_ReadVariable(VirtAddVarTab[EEPROM_offset5], &data_1);
	EE_ReadVariable(VirtAddVarTab[EEPROM_offset6], &data_2);
	offset4 = (int16_t) data_0;
	offset5 = (int16_t) data_1;
	offset6 = (int16_t) data_2;
}
*/

