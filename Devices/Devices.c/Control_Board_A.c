/**
 * @file Control_Board_A.c
 * @author Leo Liu
 * @brief communication between two boards
 * @version 1.0
 * @date 2023-03-19
 * 
 * @copyright Copyright (c) 2023
 * 
 */
 
 #include "Control_Board_A.h"
 
 void Board_A_Send_Data(void);
 void Board_A_Rec_Data(void);
 void Board_A_Handler(UART_HandleTypeDef *huart);
 void Board_A_USART_Receive_DMA(UART_HandleTypeDef *huartx);
 static int USART_Receive_DMA_NO_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint32_t Size);
 
 Control_Board_A_Func_t Control_Board_A_Func = Control_Board_A_Func_GroundInit;
 Control_Board_A_t Control_Board_A;
 
 #undef Control_Board_A_Func_GroundInit
 
 void Board_A_Send_Data(void)
 {
	 Control_Board_A.IMU_Data_Send[0].data = Board_A_IMU.Export_Data.Yaw;
	 Control_Board_A.IMU_Data_Send[1].data = Board_A_IMU.Export_Data.Roll;
	 Control_Board_A.IMU_Data_Send[2].data = Board_A_IMU.Export_Data.Pitch;
	 Control_Board_A.IMU_Data_Send[3].data = Board_A_IMU.Export_Data.Gyro_Yaw;
	 Control_Board_A.IMU_Data_Send[4].data = Board_A_IMU.Export_Data.Gyro_Roll;
	 Control_Board_A.IMU_Data_Send[5].data = Board_A_IMU.Export_Data.Gyro_Pitch;
	 
	 memcpy(&Control_Board_A.Tx_Buffer[0],&Control_Board_A.IMU_Data_Send[0].Data[0],24*sizeof(uint8_t));
	 HAL_UART_Transmit(&huart7, Control_Board_A.Tx_Buffer, sizeof(Control_Board_A.Tx_Buffer),10);
 }
 
 void Board_A_Rec_Data(void)
 {
	 memcpy(&Control_Board_A.IMU_Data_Rec.Data[0],&Control_Board_A.Rx_Buffer[0],24*sizeof(uint8_t));
	 
	 Control_Board_A.Rec.Prev_Yaw = Control_Board_A.Rec.Yaw;
	 Control_Board_A.Rec.Prev_Pitch = Control_Board_A.Rec.Roll;
	 Control_Board_A.Rec.Prev_Roll = Control_Board_A.Rec.Pitch;
	 if(fabs(*(float *)&Control_Board_A.IMU_Data_Rec.data[0]) <= 180)
			Control_Board_A.Rec.Yaw = *(float *)&Control_Board_A.IMU_Data_Rec.data[0];
	 if(fabs(*(float *)&Control_Board_A.IMU_Data_Rec.data[1]) <= 180)
			Control_Board_A.Rec.Pitch = *(float *)&Control_Board_A.IMU_Data_Rec.data[1];
	 if(fabs(*(float *)&Control_Board_A.IMU_Data_Rec.data[2]) <= 180)
			Control_Board_A.Rec.Roll = *(float *)&Control_Board_A.IMU_Data_Rec.data[2];
	 Control_Board_A.Rec.Gyro_Yaw = *(float *)&Control_Board_A.IMU_Data_Rec.data[3];
	 Control_Board_A.Rec.Gyro_Pitch = *(float *)&Control_Board_A.IMU_Data_Rec.data[4];
	 Control_Board_A.Rec.Gyro_Roll = *(float *)&Control_Board_A.IMU_Data_Rec.data[5];
	 
	 if((Control_Board_A.Rec.Yaw - Control_Board_A.Rec.Prev_Yaw) < - 300)
		Control_Board_A.Rec.Turn_Count++;
	else if((Control_Board_A.Rec.Yaw - Control_Board_A.Rec.Prev_Yaw) > 300)
		Control_Board_A.Rec.Turn_Count--;
	Control_Board_A.Rec.Total_Yaw = Control_Board_A.Rec.Yaw + 360.0f * Control_Board_A.Rec.Turn_Count;
 }
 
 static int USART_Receive_DMA_NO_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint32_t Size)
 {
	 if(huart->RxState == HAL_UART_STATE_READY)
	 {
		 if((pData == NULL) || (Size == 0))
			 return HAL_ERROR;
		 huart->pRxBuffPtr = pData;
		 huart->RxXferSize = Size;
		 huart->ErrorCode = HAL_UART_ERROR_NONE;
		 
		 HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR,(uint32_t)pData,Size);
		 SET_BIT(huart->Instance->CR3,USART_CR3_DMAR);
	 }
	 else
		 return HAL_BUSY;
	 return HAL_OK;
 }
 
 //Receive data if pass verification
 void Board_A_Handler(UART_HandleTypeDef *huart)
 {
	 __HAL_DMA_DISABLE(huart->hdmarx);
	 Control_Board_A_Func.Board_A_Rec_Data();
	 __HAL_DMA_ENABLE(huart->hdmarx);
 }
 
 void Board_A_USART_Receive_DMA(UART_HandleTypeDef *huartx)
 {
	 __HAL_UART_CLEAR_IDLEFLAG(huartx);
	 __HAL_UART_ENABLE(huartx);
	 __HAL_UART_ENABLE_IT(huartx,UART_IT_IDLE);
	 USART_Receive_DMA_NO_IT(huartx,Control_Board_A.Rx_Buffer,sizeof(Control_Board_A.Rx_Buffer));
 }

