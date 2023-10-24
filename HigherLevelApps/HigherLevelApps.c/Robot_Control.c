/**
 * @file Robot_Control.c
 * @author Leo Liu
 * @brief general robot control
 * @version 1.0
 * @date 2022-07-09
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "Robot_Control.h"

void Robot_Control_Start(void); 
void Robot_Control_Send(void);
void Robot_Control_Disabled(void);

Robot_Control_Func_t Robot_Control_Func = Robot_Control_Func_GroundInit;
#undef Robot_Control_Func_GroundInit

//The function name tells you what it does
void Robot_Control_Start(void)
{
	State_Machine_Func.Remote_Control_Update(); 
	Referee_System_Func.Referee_Set_Robot_State();
	Tx2_Func.Jetson_Tx2_Get_Data();
	Tx2_Func.Jetson_Tx2_Send_Data();
	
	Chassis_Func.Chassis_Get_Data(&Chassis);
	Chassis_Func.Chassis_Processing(&Chassis);
	
	Gimbal_Func.Gimbal_Control_Get_Data(&Gimbal);
	Gimbal_Func.Gimbal_Processing(&Gimbal);
	
	Shooting_Func.Trigger_Get_Data(&Shooting);
	Shooting_Func.Shooting_Processing(&Shooting);
	
	Robot_Control_Func.Robot_Control_Send();
}

void Robot_Control_Send(void)
{
	MF9025_Func.MF9025_Chassis_Send_Data(-MF9025_Chassis[0].Target_Speed,-MF9025_Chassis[1].Target_Speed);
	M3508_Func.M3508_Fric_Wheel_Send_Data(M3508_Fric_Wheel[0].Output_Current,M3508_Fric_Wheel[1].Output_Current);
	GM6020_Func.GM6020_Gimbal_Send_Data(GM6020_Pitch.Output_Current, GM6020_Yaw.Output_Current);
	M2006_Func.M2006_Trigger_Send_Data(M2006_Trigger.Output_Current);
}

void Robot_Control_Disabled(void)
{
	State_Machine.Mode = Disabled;
	Shooting_Func.Shooting_Disabled();
}
