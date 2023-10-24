/**
 * @file Chassis_Control.h
 * @author Leo Liu
 * @brief header file for Chassis_Control
 * @version 1.0
 * @date 2022-07-09
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef __CHASSIS_CONTROL_H
#define __CHASSIS_CONTROL_H

#include "PID.h"
#include "DR16_Remote.h"
#include "User_Defined_Math.h"
#include "Gimbal_Control.h"
#include "Robot_Control.h"
#include "Super_Capacitor.h"
#include "Control_Strategy.h"
#include "Control_Board_A.h"

#define WHEEL_DIAMETER 0.22f //m
#define TIME_INTERVAL 0.002f //s
#define YAW_SWING_LOWER_ANGLE 4570
#define YAW_SWING_UPPER_ANGLE 340
#define CHASSIS_SPINTOP_RATE_POWER_60 150.0f //Chassis spintop rate
#define CHASSIS_SPINTOP_RATE_POWER_80 200.0f //Chassis spintop rate
#define CHASSIS_SPINTOP_RATE_POWER_100 250.0f //Chassis spintop rate
#define CHASSIS_SPINTOP_RATE_POWER_150 300.0f //Chassis spintop rate
#define CHASSIS_TARGET_SPEED 0.0f //Balance speed
#define CHASSIS_TARGET_ANGLE 2.0f //Balance angle

#define Chassis_Func_GroundInit		\
{																	\
				&Chassis_Get_Data,				\
				&Chassis_State_Update,		\
				&Chassis_Processing,			\
}

typedef struct
{
	int Current_State;
	
	enum
	{
		Balancing,
		Moving,
		Braking,
	}Chassis_State;
	
	struct
	{
		float Prev_Vy;
		float Vx;
		float Vy;
		float Wz;
		float Forward_Speed; //m/s
		float Prev_Forward_Speed; //m/s
		float Forward_Distance;	//m
		float Pitch_Angle;	//degree
		float Prev_Pitch_Angle; //degree
		float Pitch_Angular_Rate;	//degree/s
		float Prev_Pitch_Angular_Rate;
		float Yaw_Angle;	//degree
		float Prev_Yaw_Angle; //degree
		float Yaw_Angular_Rate;	//degree/s
		float Prev_Yaw_Angular_Rate; //degree/s
		float Target_Yaw;
		float Swing_Target_Angle;
		float Spin_Rate;
	}Chassis_Coord; //Chassis coordinate
	
	struct
	{
		float Vx;
		float Vy;
		float Wz;
	}Gimbal_Coord; //Gimbal coordinate
	
	struct
	{
		float Left_Wheel;
		float Right_Wheel;
	}Target;
	
	struct
	{
		float Position_Loop;
		float Prev_Position_Loop;
		float Angle_Loop;
		float Speed_Loop;
		float Turning_Loop;
	}PID_Output;

	uint8_t Chassis_Offline_Flag;
}Chassis_t;

typedef struct
{
	void (*Chassis_Get_Data)(Chassis_t *Chassis);
	void (*Chassis_State_Update)(Chassis_t *Chassis);
	void (*Chassis_Processing)(Chassis_t *Chassis);
}Chassis_Func_t;

extern Chassis_t Chassis;
extern Chassis_Func_t Chassis_Func;

#endif
