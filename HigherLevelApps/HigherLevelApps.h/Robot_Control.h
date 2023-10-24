/**
 * @file Robot_Control.h
 * @author Leo Liu
 * @brief header file for robot_control
 * @version 1.0
 * @date 2022-07-09
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef __ROBOT_CONTROL_H
#define __ROBOT_CONTROL_H

#include "Chassis_Control.h"
#include "Gimbal_Control.h"
#include "State_Machine.h"
#include "Shooting_Control.h"
#include "Jetson_Tx2.h"
#include "Control_Board_A.h"
#include "Referee_System.h"

#define Robot_Control_Func_GroundInit		\
		{																		\
					&Robot_Control_Start,					\
							&Robot_Control_Send,			\
							&Robot_Control_Disabled, 	\
		}
			
typedef struct
{
	void (*Robot_Control_Start)(void);
	void (*Robot_Control_Send)(void);
	void (*Robot_Control_Disabled)(void);
}Robot_Control_Func_t;

extern Robot_Control_Func_t Robot_Control_Func;

#endif
