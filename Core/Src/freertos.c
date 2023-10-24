/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Board_A_IMU.h"
#include "IMU_Temp_Control.h"
#include "DR16_Remote.h"
#include "M3508_Motor.h"
#include "MF9025_Motor.h"
#include "Robot_Control.h"
#include "MPU6050_IMU.h"
#include "Control_Board_A.h"
#include "Fusion.h"
#include "Buzzer.h"
#include "Referee_System.h"
#include "Jetson_Tx2.h"
#include "User_Interface.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId Task_IMUHandle;
osThreadId InitializationHandle;
osThreadId Task_CAN_SendHandle;
osThreadId Task_CAN1_ReceiHandle;
osThreadId Task_CAN2_ReceiHandle;
osThreadId Robot_ControlHandle;
osThreadId Task_UIHandle;
osMessageQId CAN_SendHandle;
osMessageQId CAN1_ReceiveHandle;
osMessageQId CAN2_ReceiveHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void IMU_Tasks(void const * argument);
void General_Initialization(void const * argument);
void CAN_Send_All(void const * argument);
void CAN1_Rec(void const * argument);
void CAN2_Rec(void const * argument);
void Robot_Control_All(void const * argument);
void UI_Draw(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of CAN_Send */
  osMessageQDef(CAN_Send, 32, CAN_Send_Data_t);
  CAN_SendHandle = osMessageCreate(osMessageQ(CAN_Send), NULL);

  /* definition and creation of CAN1_Receive */
  osMessageQDef(CAN1_Receive, 32, CAN_Export_Data_t);
  CAN1_ReceiveHandle = osMessageCreate(osMessageQ(CAN1_Receive), NULL);

  /* definition and creation of CAN2_Receive */
  osMessageQDef(CAN2_Receive, 32, CAN_Export_Data_t);
  CAN2_ReceiveHandle = osMessageCreate(osMessageQ(CAN2_Receive), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of Task_IMU */
  osThreadDef(Task_IMU, IMU_Tasks, osPriorityNormal, 0, 400);
  Task_IMUHandle = osThreadCreate(osThread(Task_IMU), NULL);

  /* definition and creation of Initialization */
  osThreadDef(Initialization, General_Initialization, osPriorityHigh, 0, 256);
  InitializationHandle = osThreadCreate(osThread(Initialization), NULL);

  /* definition and creation of Task_CAN_Send */
  osThreadDef(Task_CAN_Send, CAN_Send_All, osPriorityHigh, 0, 256);
  Task_CAN_SendHandle = osThreadCreate(osThread(Task_CAN_Send), NULL);

  /* definition and creation of Task_CAN1_Recei */
  osThreadDef(Task_CAN1_Recei, CAN1_Rec, osPriorityHigh, 0, 256);
  Task_CAN1_ReceiHandle = osThreadCreate(osThread(Task_CAN1_Recei), NULL);

  /* definition and creation of Task_CAN2_Recei */
  osThreadDef(Task_CAN2_Recei, CAN2_Rec, osPriorityHigh, 0, 256);
  Task_CAN2_ReceiHandle = osThreadCreate(osThread(Task_CAN2_Recei), NULL);

  /* definition and creation of Robot_Control */
  osThreadDef(Robot_Control, Robot_Control_All, osPriorityRealtime, 0, 1000);
  Robot_ControlHandle = osThreadCreate(osThread(Robot_Control), NULL);

  /* definition and creation of Task_UI */
  osThreadDef(Task_UI, UI_Draw, osPriorityNormal, 0, 256);
  Task_UIHandle = osThreadCreate(osThread(Task_UI), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_IMU_Tasks */
/**
  * @brief  Function implementing the Task_IMU thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_IMU_Tasks */
void IMU_Tasks(void const * argument)
{
  /* USER CODE BEGIN IMU_Tasks */
	portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  const TickType_t TimeIncrement = pdMS_TO_TICKS(1);
  /* Infinite loop */
  for(;;)
  {
		Board_A_IMU_Func.Board_A_IMU_Read_Data(&Board_A_IMU);
		Board_A_IMU_Func.Board_A_IMU_Calc_Angle(&Board_A_IMU);
		//Board_A_IMU_Func.Board_A_IMU_Calibrate(&Board_A_IMU);
		IMU_Temp_Control_Func.Board_A_IMU_Temp_Control();
		
		Control_Board_A_Func.Board_A_Send_Data();
		vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
  }
  /* USER CODE END IMU_Tasks */
}

/* USER CODE BEGIN Header_General_Initialization */
/**
* @brief Function implementing the Initialization thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_General_Initialization */
void General_Initialization(void const * argument)
{
  /* USER CODE BEGIN General_Initialization */
  /* Infinite loop */
	Buzzer_Func.Buzzer_Init();
	IMU_Temp_Control_Func.Board_A_IMU_Temp_Control_Init();
	Board_A_IMU_Func.Board_A_IMU_Init();	
	FusionAhrsInitialise(&Board_A_IMU_AHRS);
	DR16_Func.DR16_USART_Receive_DMA(&huart1);
	Control_Board_A_Func.Board_A_USART_Receive_DMA(&huart7);
	Referee_System_Func.Referee_UART_Receive_Interrupt(&huart6,Referee_System.Buffer,sizeof(Referee_System.Buffer));
	UI_Initialization();
	CAN_Func.CAN_IT_Init(&hcan1, CAN1_Type);
  CAN_Func.CAN_IT_Init(&hcan2, CAN2_Type);
	Tx2_Func.Jetson_Tx2_Initialization();
	Gimbal_Func.Gimbal_Init();
	Shooting_Func.Shooting_Init();
	vTaskDelete(NULL);
	
  /* USER CODE END General_Initialization */
}

/* USER CODE BEGIN Header_CAN_Send_All */
/**
* @brief Function implementing the Task_CAN_Send thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CAN_Send_All */
void CAN_Send_All(void const * argument)
{
  /* USER CODE BEGIN CAN_Send_All */
	CAN_Send_Data_t CAN_Send_Data;	
  /* Infinite loop */
  for(;;)
  {
    xQueueReceive(CAN_SendHandle, &CAN_Send_Data, portMAX_DELAY);
    HAL_CAN_AddTxMessage(CAN_Send_Data.Canx, &CAN_Send_Data.CAN_TxHeader, CAN_Send_Data.CANx_Send_RxMessage, (uint32_t *)CAN_TX_MAILBOX0);
  }
  /* USER CODE END CAN_Send_All */
}

/* USER CODE BEGIN Header_CAN1_Rec */
/**
* @brief Function implementing the Task_CAN1_Recei thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CAN1_Rec */
void CAN1_Rec(void const * argument)
{
  /* USER CODE BEGIN CAN1_Rec */
	CAN_Export_Data_t CAN_Export_Data;
	uint32_t ID;
  /* Infinite loop */
  for(;;)
  {
    xQueueReceive(CAN1_ReceiveHandle, &CAN_Export_Data, portMAX_DELAY);
		ID = CAN_Export_Data.CAN_RxHeader.StdId;
		if(ID == SUPERCAP_ID)
			Super_Capacitor_Func.Super_Capacitor_Get_Data(CAN_Export_Data);
		else if(ID == (MF9025_DEVICE_ID + MF9025_CHASSIS_LEFT_ID) || ID == (MF9025_DEVICE_ID + MF9025_CHASSIS_RIGHT_ID))
			MF9025_Func.MF9025_Chassis_Get_Data(CAN_Export_Data);
		else if(ID == GM6020_YAW_ID)
			GM6020_Func.GM6020_Yaw_Get_Data(CAN_Export_Data);
		else if(ID == GM6020_PITCH_ID)
			GM6020_Func.GM6020_Pitch_Get_Data(CAN_Export_Data);
		
		Monitor_CAN1.Info_Update_Frame++;
  }
  /* USER CODE END CAN1_Rec */
}

/* USER CODE BEGIN Header_CAN2_Rec */
/**
* @brief Function implementing the Task_CAN2_Recei thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CAN2_Rec */
void CAN2_Rec(void const * argument)
{
  /* USER CODE BEGIN CAN2_Rec */
	CAN_Export_Data_t CAN_Export_Data;
	uint32_t ID;
	//uint32_t ID;
  /* Infinite loop */
  for(;;)
  {
    xQueueReceive(CAN2_ReceiveHandle, &CAN_Export_Data, portMAX_DELAY);
		ID = CAN_Export_Data.CAN_RxHeader.StdId;
		if(ID == M2006_TRIGGER_ID)
			M2006_Func.M2006_Trigger_Get_Data(CAN_Export_Data);
		else if(ID == M3508_FRIC_WHEEL_LEFT_ID || ID == M3508_FRIC_WHEEL_RIGHT_ID)
			M3508_Func.M3508_Fric_Wheel_Get_Data(CAN_Export_Data);
		
		Monitor_CAN2.Info_Update_Frame++;
  }
  /* USER CODE END CAN2_Rec */
}

/* USER CODE BEGIN Header_Robot_Control_All */
/**
* @brief Function implementing the Robot_Control thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Robot_Control_All */
void Robot_Control_All(void const * argument)
{
  /* USER CODE BEGIN Robot_Control_All */
	portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  const TickType_t TimeIncrement = pdMS_TO_TICKS(2);
  /* Infinite loop */
  for(;;)
  {
    Robot_Control_Func.Robot_Control_Start();
		vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
  }
  /* USER CODE END Robot_Control_All */
}

/* USER CODE BEGIN Header_UI_Draw */
/**
* @brief Function implementing the Task_UI thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UI_Draw */
void UI_Draw(void const * argument)
{
  /* USER CODE BEGIN UI_Draw */
  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  const TickType_t TimeIncrement = pdMS_TO_TICKS(5);
  /* Infinite loop */
  for(;;)
  {
		if(Referee_Robot_State.ID == 3)
		{
			UI.Pilot_ID = UI_Data_CilentID_RStandard1;
			UI.Robot_ID = UI_Data_RobotID_RStandard1;
		}
		else if(Referee_Robot_State.ID == 103)
		{
			UI.Pilot_ID = UI_Data_CilentID_BStandard1;
			UI.Robot_ID = UI_Data_RobotID_BStandard1;
		}
		if(UI.Initialized_Flag == 0)
		{
			UI_AimingSystem();
			UI_GuidingSystem();
			UI_TextSystem_Init();
			UI.Initialized_Flag = 1;
		}
		osDelay (1000);
//		memcpy (TextSystem.Text, "A_Test", 6);
//		TextSystem.TextNumber = 6;
//		UI_AutoText_Update();
		switch(State_Machine.Mode)
		{
			case(Follow_Gimbal):
				memcpy (TextSystem.Text, "FLGB", 4);
				break;
			case(Spin_Top):
				memcpy (TextSystem.Text, "SPIN", 4);
				break;
			default:
				memcpy (TextSystem.Text, "FLGB", 4);
				break;
		}
		TextSystem.TextNumber = 4;
		UI_StateText_Update();
		
		
		if(Shooting.Fric_Wheel.Turned_On)
		{
			memcpy (TextSystem.Text, "FRIC ON", 7);
			TextSystem.TextNumber = 7;
			UI_AutoText_Update();
		}
		else if(Shooting.Fric_Wheel.Turned_On == 0)
		{
			memcpy (TextSystem.Text, "FRIC OFF", 8);
			TextSystem.TextNumber = 8;
			UI_AutoText_Update();
		}
		
//		memcpy (TextSystem.Text, "A_Test2", 7);
//		TextSystem.TextNumber = 7;
//		UI_AutoText_Update();
//		
//		memcpy (TextSystem.Text, "C_Test2", 7);
//		TextSystem.TextNumber = 7;
//		UI_SuperCapText_Update();
//		
//		memcpy (TextSystem.Text, "S_Test2", 7);
//		TextSystem.TextNumber = 7;
//		UI_StateText_Update();
		
		vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
  }
  /* USER CODE END UI_Draw */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
