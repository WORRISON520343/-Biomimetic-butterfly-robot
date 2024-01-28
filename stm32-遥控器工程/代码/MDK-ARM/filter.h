#ifndef _FILTER_H_
#define _FILTER_H_
#include "stm32f10x.h"
#include "struct_all.h"

/******************************************************************************
							全局函数声明
*******************************************************************************/ 
void Rc_Filter( struct _Rc *Rc_in,struct _Rc *Rc_out);

#endif

