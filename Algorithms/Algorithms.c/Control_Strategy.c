	/**
 * @file Control_Strategy.c
 * @author Leo Liu
 * @brief different control strategy for chassis control of SBR
 * @version 1.0
 * @date 2023-01-07
 * 
 * @copyright Copyright (c) 2022
 * 
 */
 
#include "Control_Strategy.h"

void Expert_PID_LQR_Combined(void);

Control_Strategy_Func_t Control_Strategy_Func = Control_Strategy_Func_GroundInit;
#undef Control_Strategy_Func_GroundInit

float LQR_K_Matrix[4] = {9.9196,  21.3417, 395.9124,  54.9115};

void Expert_PID_LQR_Combined(void)
{
	switch(Chassis.Current_State)
	{
		case Balancing:
		{
			 Chassis_Speed_PID.Kp = 1000.0f;
			 Chassis_Speed_PID.Ki = 30.0;
			 Chassis_Speed_PID.I_Out_Max = 500.0f;
			 Chassis_Speed_PID.Output_Max = 10000.0f;
			 
			 Chassis.PID_Output.Speed_Loop = PID_Func.Positional_PID(&Chassis_Speed_PID, CHASSIS_TARGET_SPEED, Chassis.Chassis_Coord.Forward_Speed); 
			 Chassis.PID_Output.Angle_Loop = 0.75f * (Chassis.Chassis_Coord.Pitch_Angle - CHASSIS_TARGET_ANGLE) * LQR_K_Matrix[2] + 3.5f * Chassis.Chassis_Coord.Pitch_Angular_Rate * LQR_K_Matrix[3];
			 
			 Chassis_Turning_PID.Kp = 1.0f;
			 Chassis_Turning_PID.Kd = 3.0f;
			 Chassis_Turning_PID.Output_Max = 10000.0f;
			 Chassis.PID_Output.Turning_Loop = PID_Func.Positional_PID(&Chassis_Turning_PID, Chassis.Chassis_Coord.Wz,0);
			
			 Chassis.Target.Left_Wheel = Chassis.PID_Output.Speed_Loop + Chassis.PID_Output.Angle_Loop - 4.0f*Chassis.PID_Output.Turning_Loop; //Left Wheel
			 Chassis.Target.Right_Wheel = Chassis.PID_Output.Speed_Loop + Chassis.PID_Output.Angle_Loop + 4.0f*Chassis.PID_Output.Turning_Loop; //Right Wheel
			
			 break;
		}
		case Moving:
		{
			Chassis_Speed_PID.Kp = 1800.0f;
			Chassis_Speed_PID.Output_Max = 10000.0f;
			Chassis.PID_Output.Speed_Loop = PID_Func.Positional_PID(&Chassis_Speed_PID, Chassis.Chassis_Coord.Vy, Chassis.Chassis_Coord.Forward_Speed); 
			
			Chassis.PID_Output.Angle_Loop = 0.75f * (Chassis.Chassis_Coord.Pitch_Angle - CHASSIS_TARGET_ANGLE) * LQR_K_Matrix[2] + 3.5f * Chassis.Chassis_Coord.Pitch_Angular_Rate * LQR_K_Matrix[3];

			Chassis_Turning_PID.Kp = 1.2f;
			Chassis_Turning_PID.Kd = 2.0f;
			Chassis_Turning_PID.Output_Max = 10000.0f;
			Chassis.PID_Output.Turning_Loop = PID_Func.Positional_PID(&Chassis_Turning_PID, Chassis.Chassis_Coord.Wz,	0);

			Chassis.Target.Left_Wheel = Chassis.PID_Output.Speed_Loop + Chassis.PID_Output.Angle_Loop - 4.0f*Chassis.PID_Output.Turning_Loop;	//Left Wheel
			Chassis.Target.Right_Wheel = Chassis.PID_Output.Speed_Loop + Chassis.PID_Output.Angle_Loop + 4.0f*Chassis.PID_Output.Turning_Loop;	//Right Wheel
			break;
		}
		case Braking:
		{
			Chassis_Speed_PID.Kp = 2000.0f;
			Chassis_Speed_PID.Ki = 80.0;
			Chassis_Speed_PID.I_Out_Max = 6000.0f;
			Chassis_Speed_PID.Output_Max = 10000.0f;

			Chassis.PID_Output.Speed_Loop = PID_Func.Positional_PID(&Chassis_Speed_PID, CHASSIS_TARGET_SPEED, Chassis.Chassis_Coord.Forward_Speed); 
			Chassis.PID_Output.Angle_Loop = 0.9f * (Chassis.Chassis_Coord.Pitch_Angle - CHASSIS_TARGET_ANGLE) * LQR_K_Matrix[2] + 4.5f * Chassis.Chassis_Coord.Pitch_Angular_Rate * LQR_K_Matrix[3];
			
			Chassis_Turning_PID.Kp = 1.2f;
			Chassis_Turning_PID.Kd = 2.0f;
			Chassis_Turning_PID.Output_Max = 10000.0f;
			Chassis.PID_Output.Turning_Loop = PID_Func.Positional_PID(&Chassis_Turning_PID, Chassis.Chassis_Coord.Wz, 0);
			
			Chassis.Target.Left_Wheel = Chassis.PID_Output.Speed_Loop + Chassis.PID_Output.Angle_Loop - 4.0f*Chassis.PID_Output.Turning_Loop; //Left Wheel
			Chassis.Target.Right_Wheel = Chassis.PID_Output.Speed_Loop + Chassis.PID_Output.Angle_Loop + 4.0f*Chassis.PID_Output.Turning_Loop; //Right Wheel
			break;
		}
	}
}
