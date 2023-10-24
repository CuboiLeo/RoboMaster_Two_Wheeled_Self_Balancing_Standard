/**
 * @file Gimbal_Control.c
 * @author Leo Liu
 * @brief control gimbal
 * @version 1.0
 * @date 2022-07-11
 * 
 * @copyright Copyright (c) 2022
 * 
 */
 
#include "Gimbal_Control.h"

void Gimbal_Init(void);
void Gimbal_Control_Get_Data(Gimbal_t *Gimbal);
void Gimbal_Processing(Gimbal_t *Gimbal);

Gimbal_Func_t Gimbal_Func = Gimbal_Func_GroundInit;
#undef Gimbal_Func_GroundInit

Gimbal_t Gimbal;

void Gimbal_Init(void)
{
	//Set the origin for yaw and pitch
	GM6020_Yaw.Target_Angle = Board_A_IMU.Export_Data.Total_Yaw;
	GM6020_Pitch.Target_Angle = PITCH_MID_ANGLE;
}

void Gimbal_Control_Get_Data(Gimbal_t *Gimbal)
{
	//The multiplying/dividing constant are tested value and can be changed
	if(State_Machine.Control_Source == Remote_Control)
	{
		Gimbal->Current_Yaw = YAW_DIRECTION * Board_A_IMU.Export_Data.Total_Yaw;
		Gimbal->Target_Yaw -= DR16_Export_Data.Remote_Control.Joystick_Right_Vx / 1000.0f;
		
		Gimbal->Current_Pitch = PITCH_DIRECTION * Board_A_IMU.Export_Data.Pitch;
		if((Gimbal->Current_Pitch > PITCH_LOWER_LIMIT && Gimbal->Current_Pitch < PITCH_UPPER_LIMIT) || \
			(Gimbal->Current_Pitch < PITCH_LOWER_LIMIT && DR16_Export_Data.Remote_Control.Joystick_Right_Vy > 0) || \
			(Gimbal->Current_Pitch > PITCH_UPPER_LIMIT && DR16_Export_Data.Remote_Control.Joystick_Right_Vy < 0))
					Gimbal->Target_Pitch += DR16_Export_Data.Remote_Control.Joystick_Right_Vy / 2200.0f;
		Gimbal->Target_Pitch = VAL_LIMIT(Gimbal->Target_Pitch,PITCH_UPPER_LIMIT,PITCH_LOWER_LIMIT);
	}
	
	else if(State_Machine.Control_Source == Computer)
	{
		Gimbal->Current_Yaw = YAW_DIRECTION * Board_A_IMU.Export_Data.Total_Yaw;
		Gimbal->Target_Yaw -= 0.01f * (float)DR16_Export_Data.Mouse.x;
		
		Gimbal->Current_Pitch = PITCH_DIRECTION * Board_A_IMU.Export_Data.Pitch;
		if((Gimbal->Current_Pitch > PITCH_LOWER_LIMIT && Gimbal->Current_Pitch < PITCH_UPPER_LIMIT) || \
			(Gimbal->Current_Pitch < PITCH_LOWER_LIMIT && DR16_Export_Data.Mouse.y < 0) || \
			(Gimbal->Current_Pitch > PITCH_UPPER_LIMIT && DR16_Export_Data.Mouse.y > 0))
					Gimbal->Target_Pitch -= (float)DR16_Export_Data.Mouse.y / 400.0f;
		Gimbal->Target_Pitch = VAL_LIMIT(Gimbal->Target_Pitch,PITCH_UPPER_LIMIT,PITCH_LOWER_LIMIT);
	}
}

void Gimbal_Processing(Gimbal_t *Gimbal)
{
	switch(State_Machine.Mode)
	{
		case(Follow_Gimbal):
		{	
			GM6020_Yaw.Output_Current = PID_Func.Positional_PID(&Yaw_Angle_PID,Gimbal->Target_Yaw,Gimbal->Current_Yaw);
			
			Gimbal->Pitch_Angle_Loop = PID_Func.Positional_PID(&Pitch_Angle_PID,Gimbal->Target_Pitch,Gimbal->Current_Pitch);
			GM6020_Pitch.Output_Current = PID_Func.Positional_PID(&Pitch_Speed_PID,Gimbal->Pitch_Angle_Loop,Board_A_IMU.Export_Data.Gyro_Pitch);	
			break;
		}
		
		case(Not_Follow_Gimbal):
		{
			GM6020_Yaw.Output_Current = PID_Func.Positional_PID(&Yaw_Angle_PID,Gimbal->Target_Yaw,Gimbal->Current_Yaw);
			
			Gimbal->Pitch_Angle_Loop = PID_Func.Positional_PID(&Pitch_Angle_PID,Gimbal->Target_Pitch,Gimbal->Current_Pitch);
			GM6020_Pitch.Output_Current = PID_Func.Positional_PID(&Pitch_Speed_PID,Gimbal->Pitch_Angle_Loop,Board_A_IMU.Export_Data.Gyro_Pitch);
			break;
		}
		
		case(Spin_Top):
		{	
			GM6020_Yaw.Output_Current = PID_Func.Positional_PID(&Yaw_Angle_PID,Gimbal->Target_Yaw,Gimbal->Current_Yaw);
			
			Gimbal->Pitch_Angle_Loop = PID_Func.Positional_PID(&Pitch_Angle_PID,Gimbal->Target_Pitch,Gimbal->Current_Pitch);
			GM6020_Pitch.Output_Current = PID_Func.Positional_PID(&Pitch_Speed_PID,Gimbal->Pitch_Angle_Loop,Board_A_IMU.Export_Data.Gyro_Pitch);	
			break;
		}
		
		case(Follow_Wheel):
		{
			GM6020_Yaw.Output_Current = PID_Func.Positional_PID(&Yaw_Angle_PID,Gimbal->Target_Yaw,Gimbal->Current_Yaw);
			
			Gimbal->Pitch_Angle_Loop = PID_Func.Positional_PID(&Pitch_Angle_PID,Gimbal->Target_Pitch,Gimbal->Current_Pitch);
			GM6020_Pitch.Output_Current = PID_Func.Positional_PID(&Pitch_Speed_PID,Gimbal->Pitch_Angle_Loop,Board_A_IMU.Export_Data.Gyro_Pitch);
			break;
		}
		
		case(Swing):
		{
			GM6020_Yaw.Output_Current = PID_Func.Positional_PID(&Yaw_Angle_PID,Gimbal->Target_Yaw,Gimbal->Current_Yaw);
			
			Gimbal->Pitch_Angle_Loop = PID_Func.Positional_PID(&Pitch_Angle_PID,Gimbal->Target_Pitch,Gimbal->Current_Pitch);
			GM6020_Pitch.Output_Current = PID_Func.Positional_PID(&Pitch_Speed_PID,Gimbal->Pitch_Angle_Loop,Board_A_IMU.Export_Data.Gyro_Pitch);
			break;
		}
		
		case(Escape):
		{
			GM6020_Yaw.Output_Current = PID_Func.Positional_PID(&Yaw_Angle_PID,Gimbal->Target_Yaw,Gimbal->Current_Yaw);
			
			Gimbal->Pitch_Angle_Loop = PID_Func.Positional_PID(&Pitch_Angle_PID,Gimbal->Target_Pitch,Gimbal->Current_Pitch);
			GM6020_Pitch.Output_Current = PID_Func.Positional_PID(&Pitch_Speed_PID,Gimbal->Pitch_Angle_Loop,Board_A_IMU.Export_Data.Gyro_Pitch);	
			break;
		}
		
		case(Disabled):
		{
			Gimbal->Target_Yaw = YAW_DIRECTION * Board_A_IMU.Export_Data.Total_Yaw;
			GM6020_Yaw.Output_Current = 0;
			GM6020_Pitch.Output_Current = 0;
			
			Gimbal->Prev_Mode = Disabled;			
			break;
		}
	}
}
