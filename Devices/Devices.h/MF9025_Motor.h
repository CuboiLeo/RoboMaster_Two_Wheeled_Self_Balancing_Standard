/**
 * @file MF9025_Motor.h
 * @author Leo Liu
 * @brief header file for MF9025
 * @version 1.0
 * @date 2023-03-13
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef __MF9025_MOTOR_H
#define __MF9025_MOTOR_H

#include "can.h"
#include "CAN_Setup.h"
#include "Motor_Init.h"
#include <stdint.h>
#include <stdio.h>

#define MF9025_DEVICE_ID 0x140 //Single motor device identifier
#define MF9025_CHASSIS_LEFT_ID 0x01 //Starting ID for chassis motors
#define MF9025_CHASSIS_RIGHT_ID	0x02 //Ending ID for chassis motors
#define MF9025_MECH_ANGLE_MAX 65536.0f //16 bit encoder
#define MF9025_SPEED_MAX 330000.0f //MF9025 maximum speed (0.01 degree per second)

#define MF9025_Func_GroundInit					    \
		{																	   		\
				&MF9025_Chassis_Get_Data,			  		\
						&MF9025_Chassis_Send_Data,		  \
						&Check_MF9025_Chassis,			    \
		}
		
typedef struct
{
	 uint16_t Actual_Angle;
	 uint16_t Prev_Angle;
	 int16_t Actual_Speed;
	 int16_t Actual_Current;
	 int8_t Temperature;
	 
	 int32_t Target_Angle;
	 int32_t Target_Speed;
	 float Total_Turn;
	 int16_t Turn_Count;
	 
	 uint16_t Info_Update_Frame;
	 uint8_t Offline_Flag;
}MF9025_Chassis_t;

typedef struct
{
	void (*MF9025_Chassis_Get_Data)(CAN_Export_Data_t RxMessage);
	void (*MF9025_Chassis_Send_Data)(int32_t Left_Speed,int32_t Right_Speed);
	void (*Check_MF9025_Chassis)(void);
}MF9025_Func_t;

extern MF9025_Func_t MF9025_Func;
extern MF9025_Chassis_t MF9025_Chassis[2];

#endif
