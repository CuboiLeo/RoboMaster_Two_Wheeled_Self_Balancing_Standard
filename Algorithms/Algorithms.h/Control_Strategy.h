 /**
 * @file Control_Strategy.h
 * @author Leo Liu
 * @brief header file for Control_Strategy.c
 * @version 1.0
 * @date 2023-01-07
 * 
 * @copyright Copyright (c) 2022
 * 
 */
 
#ifndef __CONTROL_STRATEGY_H
#define __CONTROL_STRATEGY_H
 
#include "Chassis_Control.h"
#include "PID.h"
 
#define Control_Strategy_Func_GroundInit		\
{																						\
		&Expert_PID_LQR_Combined,								\
}
 
typedef struct
{
	void (*Expert_PID_LQR_Combined)(void);
}Control_Strategy_Func_t;

extern Control_Strategy_Func_t Control_Strategy_Func;

#endif
