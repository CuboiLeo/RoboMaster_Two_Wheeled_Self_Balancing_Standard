/**
 * @file State_Machine.c
 * @author Leo Liu
 * @brief control the state of the robot
 * @version 1.0
 * @date 2022-07-12
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "State_Machine.h"

State_Machine_t State_Machine;

void Remote_Control_Update(void);
void Computer_Update(void);

State_Machine_Func_t State_Machine_Func = State_Machine_Func_GroundInit;
#undef State_Machine_Func_GroundInit

void Remote_Control_Update(void)
{
	/*
	Both switches down for disabling the robot
	Also disable the robot if remote control receives no data 
	*/
	if((DR16_Export_Data.Remote_Control.Left_Switch == SWITCH_DOWN && DR16_Export_Data.Remote_Control.Right_Switch == SWITCH_DOWN) 
		|| (DR16_Export_Data.Info_Update_Frame < 1))
	{
		Robot_Control_Func.Robot_Control_Disabled();
	}
	
	/*
	Right switch mid, left switch down: follow gimbal mode
	Right switch mid, left switch mid: not follow gimbal mode
	Right switch mid, left switch up: spintop mode
	*/
	else if(DR16_Export_Data.Remote_Control.Right_Switch == SWITCH_MID)
	{
		State_Machine.Control_Source = Remote_Control;
		
		switch(DR16_Export_Data.Remote_Control.Left_Switch)
		{
			case(SWITCH_DOWN):
			{
				State_Machine.Mode = Follow_Gimbal;
				State_Machine.Spin_Top_Flag = 0;
				State_Machine.Swing_Flag = 0;
				State_Machine.Follow_Wheel_Flag = 0;
				break;
			}
			case(SWITCH_MID):
			{
				State_Machine.Follow_Wheel_Flag = 1;
				State_Machine.Spin_Top_Flag = 0;
				State_Machine.Swing_Flag = 0;
				break;
			}
			case(SWITCH_UP):
			{
				State_Machine.Spin_Top_Flag = 1;
				State_Machine.Swing_Flag = 0;
				State_Machine.Follow_Wheel_Flag = 0;
				break;
			}
		}
	}
	
	/*
	Right switch up, left switch down: turn on friction wheel
	Right switch up, left switch mid: turn off friction wheel
	Right switch up, left switch up: enter computer control mode
	*/
	else if(DR16_Export_Data.Remote_Control.Right_Switch == SWITCH_UP)
	{
		switch(DR16_Export_Data.Remote_Control.Left_Switch)
		{
			case(SWITCH_DOWN):
			{
				State_Machine.Control_Source = Remote_Control;
				Shooting.Fric_Wheel.Turned_On = 0;				
				break;
			}	
			case(SWITCH_MID):
			{
				State_Machine.Control_Source = Remote_Control;				
				Shooting.Fric_Wheel.Turned_On = 1;				
				break;
			}
			case(SWITCH_UP):
			{
				State_Machine.Control_Source = Computer;				
				break;
			}
		}
	
	}
	Computer_Update();
}

void Computer_Update(void)
{
	/*
	Press R for follow gimbal mode, default mode is follow gimbal mode
	Press F for turn on/off not follow gimbal mode
	Press G for turn on/off spintop mode
	Press B for turn on/off friction wheel
	*/
	if(State_Machine.Control_Source == Computer)
	{
		if(DR16_Export_Data.Keyboard.Press_R.Switch_Flag)
		{
			State_Machine.Spin_Top_Flag = 0;
			State_Machine.Swing_Flag = 0;
			State_Machine.Mode = Follow_Gimbal;
		}
		
		else if(DR16_Export_Data.Keyboard.Press_F.Switch_Flag)
		{
			if(State_Machine.Mode == Swing)
			{
				State_Machine.Swing_Flag = 0;
				State_Machine.Mode = Follow_Gimbal;
			}
			else
			{
				State_Machine.Swing_Flag = 1;
				State_Machine.Spin_Top_Flag = 0;
				Chassis.Chassis_Coord.Swing_Target_Angle = YAW_SWING_LOWER_ANGLE;
				State_Machine.Mode = Swing;
			}
		}
		
		else if(DR16_Export_Data.Keyboard.Press_E.Switch_Flag)
		{
			if(State_Machine.Mode == Follow_Wheel)
			{
				State_Machine.Follow_Wheel_Flag = 0;
				State_Machine.Mode = Follow_Gimbal;
			}
			else
			{
				State_Machine.Follow_Wheel_Flag = 1;
				State_Machine.Spin_Top_Flag = 0;
				State_Machine.Swing_Flag = 0;
				State_Machine.Mode = Follow_Wheel;
			}
		}
		
		else if(DR16_Export_Data.Keyboard.Press_Q.Switch_Flag)
		{
			if(State_Machine.Mode == Escape)
			{
				State_Machine.Mode = Follow_Gimbal;
			}
			else
			{
				State_Machine.Mode = Escape;
			}
		}
		
		else if(DR16_Export_Data.Keyboard.Press_G.Switch_Flag)
		{
			if(State_Machine.Mode == Spin_Top)
			{
				State_Machine.Spin_Top_Flag = 0;
				State_Machine.Mode = Follow_Gimbal;
			}
			else
			{
				State_Machine.Spin_Top_Flag = 1;
				State_Machine.Swing_Flag = 0;
				State_Machine.Mode = Spin_Top;
			}
		}
		
		else if(DR16_Export_Data.Keyboard.Press_B.Switch_Flag)
		{
			if(Shooting.Fric_Wheel.Turned_On)
				Shooting.Fric_Wheel.Turned_On = 0;
			else 
				Shooting.Fric_Wheel.Turned_On = 1;
		}
		
		else if(DR16_Export_Data.Keyboard.Press_Z.Switch_Flag)
		{
			UI.Initialized_Flag = 0;
			UI.Global_PacketNumber = 0;
		}
	}
}
