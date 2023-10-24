/**
 * @file MF9025_Motor.c
 * @author Leo Liu
 * @brief MF9025 communication
 * @version 1.0
 * @date 2023-03-13
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include "MF9025_Motor.h"

MF9025_Chassis_t MF9025_Chassis[2];

void MF9025_Chassis_Get_Data(CAN_Export_Data_t RxMessage);
void MF9025_Chassis_Send_Data(int32_t Left_Speed,int32_t Right_Speed);
void Check_MF9025_Chassis(void);

MF9025_Func_t MF9025_Func = MF9025_Func_GroundInit;
#undef MF9025_Func_GroundInit

//Obtain chassis data from CAN
void MF9025_Chassis_Get_Data(CAN_Export_Data_t RxMessage)
{
	switch(RxMessage.CAN_RxHeader.StdId - MF9025_DEVICE_ID)
	{
		case MF9025_CHASSIS_LEFT_ID:
			MF9025_Chassis[0].Prev_Angle = MF9025_Chassis[0].Actual_Angle;
			MF9025_Chassis[0].Actual_Angle = (RxMessage.CANx_Export_RxMessage[7] << 8) + RxMessage.CANx_Export_RxMessage[6];
			MF9025_Chassis[0].Actual_Speed = (RxMessage.CANx_Export_RxMessage[5] << 8) + RxMessage.CANx_Export_RxMessage[4];
			MF9025_Chassis[0].Actual_Current = (RxMessage.CANx_Export_RxMessage[3] << 8) + RxMessage.CANx_Export_RxMessage[2];
			MF9025_Chassis[0].Temperature = RxMessage.CANx_Export_RxMessage[1];
			if((MF9025_Chassis[0].Actual_Angle - MF9025_Chassis[0].Prev_Angle) < -50000)
			MF9025_Chassis[0].Turn_Count++;
			else if((MF9025_Chassis[0].Actual_Angle - MF9025_Chassis[0].Prev_Angle) > 50000)
			MF9025_Chassis[0].Turn_Count--;
			MF9025_Chassis[0].Total_Turn = ((float)MF9025_Chassis[0].Actual_Angle / (float)MF9025_MECH_ANGLE_MAX) + MF9025_Chassis[0].Turn_Count;
		
			MF9025_Chassis[0].Info_Update_Frame++;
			break;
		case MF9025_CHASSIS_RIGHT_ID:
			MF9025_Chassis[1].Prev_Angle = MF9025_Chassis[1].Actual_Angle;
			MF9025_Chassis[1].Actual_Angle = (RxMessage.CANx_Export_RxMessage[7] << 8) + RxMessage.CANx_Export_RxMessage[6];
			MF9025_Chassis[1].Actual_Speed = (RxMessage.CANx_Export_RxMessage[5] << 8) + RxMessage.CANx_Export_RxMessage[4];
			MF9025_Chassis[1].Actual_Current = (RxMessage.CANx_Export_RxMessage[3] << 8) + RxMessage.CANx_Export_RxMessage[2];
			MF9025_Chassis[1].Temperature = RxMessage.CANx_Export_RxMessage[1];
			if((MF9025_Chassis[1].Actual_Angle - MF9025_Chassis[1].Prev_Angle) < -50000)
			MF9025_Chassis[1].Turn_Count++;
			else if((MF9025_Chassis[1].Actual_Angle - MF9025_Chassis[1].Prev_Angle) > 50000)
			MF9025_Chassis[1].Turn_Count--;
			
			MF9025_Chassis[1].Total_Turn = ((float)MF9025_Chassis[1].Actual_Angle / (float)MF9025_MECH_ANGLE_MAX) + MF9025_Chassis[1].Turn_Count;
			
			MF9025_Chassis[1].Info_Update_Frame++;
			break;
	}
}

//Send chassis data through specified identifier
void MF9025_Chassis_Send_Data(int32_t Left_Speed,int32_t Right_Speed)
{
	CAN_Func.CAN_0x140_Send_Data(&hcan1,MF9025_CHASSIS_LEFT_ID,0xA2,0,0,0,*(uint8_t *)(&Left_Speed),
					*((uint8_t *)(&Left_Speed)+1),*((uint8_t *)(&Left_Speed)+2),*((uint8_t *)(&Left_Speed)+3));
	
	CAN_Func.CAN_0x140_Send_Data(&hcan1,MF9025_CHASSIS_RIGHT_ID,0xA2,0,0,0,*(uint8_t *)(&Right_Speed),
					*((uint8_t *)(&Right_Speed)+1),*((uint8_t *)(&Right_Speed)+2),*((uint8_t *)(&Right_Speed)+3));
}

void Check_MF9025_Chassis(void)
{
	for(int i = 0; i < 4; i++)
	{
		if(MF9025_Chassis[i].Info_Update_Frame < 1)
			MF9025_Chassis[i].Offline_Flag = 1;
		else
			MF9025_Chassis[i].Offline_Flag = 0;
		MF9025_Chassis[i].Info_Update_Frame = 0;
	}
}
