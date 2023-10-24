/**
 * @file Shooting_Control.h
 * @author Leo Liu
 * @brief header file for shooting control
 * @version 1.0
 * @date 2022-07-16
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef __SHOOTING_CONTROL_H
#define __SHOOTING_CONTROL_H

#include "M3508_Motor.h"
#include "M2006_Motor.h"
#include "State_Machine.h"
#include "DR16_Remote.h"
#include "Ramp_Calc.h"

#define FRIC_SPEED_10 3000
#define FRIC_SPEED_15 3000 //4400 //Tested value for 15m/s
#define FRIC_SPEED_18	3000 //4900 //Tested value for 18m/s 
#define FRIC_SPEED_30	3000 //7200 //Tested value for 30m/s
#define TRIGGER_DIRECTION 1 //Trigger motor direction
#define FRIC_LEFT_DIRECTION -1 //Left friction wheel motor direction
#define FRIC_RIGHT_DIRECTION 1 //Right friction wheel motor direction

#define Shooting_Func_GroundInit	\
{																	\
		&Shooting_Init,								\
				&Trigger_Get_Data,				\
				&Shooting_Processing,			\
				&Shooting_Disabled,				\
}

typedef struct
{
	struct
	{
		uint8_t Single_Fire_Flag; //Signal that need to fire once
		uint8_t Single_Fired_Flag;	//Signal that it has fired once already
		uint8_t Burst_Flag;	//Signal continuous shooting
	}Type;
	
	struct
	{
		float Target_Angle;
		float Target_Speed;
	}Trigger;
	
	struct
	{
		uint16_t Target_Speed;
		uint8_t Turned_On;
	}Fric_Wheel;
	
	uint8_t Fric_Wheel_Ready_Flag;
}Shooting_t;

typedef struct
{
	void (*Shooting_Init)(void);
	void (*Trigger_Get_Data)(Shooting_t *Shooting);
	void (*Shooting_Processing)(Shooting_t *Shooting);
	void (*Shooting_Disabled)(void);
}Shooting_Func_t;

extern Shooting_t Shooting;
extern Shooting_Func_t Shooting_Func;

#endif
