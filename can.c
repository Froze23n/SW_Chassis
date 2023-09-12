#include "can.h"

/* USER CODE BEGIN 0 */
#include "gm6020.h"
#include "m3508.h"
///////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
0x200	chassis	1234	0x201-204			---can2------speeding
0x1FF	chassis	5678	0x205-0x208		---can2other
0x1FF	Gimbal	1234	0x205-0x208		---can1------steering
0x2FF	Gimbal	567		0x209-0x20A		---can1other
*/

//variables
//CAN1+GM6020
uint32_t CAN1_TxMailBox	;
CAN_TxHeaderTypeDef CAN1_TxHeader ;
CAN_RxHeaderTypeDef CAN1_RxHeader ;
uint8_t Set_GM6020_Voltage[8]={0,0,0,0,0,0,0,0};
uint8_t Get_GM6020_Data[8];
//CAN2+M3508
uint32_t CAN2_TxMailBox	;
CAN_TxHeaderTypeDef CAN2_TxHeader ;
CAN_RxHeaderTypeDef CAN2_RxHeader ;
uint8_t Set_M3508_Current[8]={0,0,0,0,0,0,0,0};
uint8_t Get_M3508_Data[8];
//anouncement for can1
void CAN1_Filter_Configuration(void);
void Set_CAN1_Tx_Message(void);
//... for can2
void CAN2_Filter_Configuration(void);
void Set_CAN2_Tx_Message(void);

//Codes
void CAN1_Filter_Configuration(void)
{
	//configure filter for can1 gm6020
	CAN_FilterTypeDef CAN1_Filter;
	CAN1_Filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	CAN1_Filter.FilterScale = CAN_FILTERSCALE_16BIT;
	CAN1_Filter.FilterBank = 0;
	CAN1_Filter.FilterMode = CAN_FILTERMODE_IDLIST;
	//CAN1_Filter.SlaveStartFilterBank = 0;
	CAN1_Filter.FilterActivation = CAN_FILTER_ENABLE;
	CAN1_Filter.FilterIdHigh = (0x0205)<<5;
	CAN1_Filter.FilterIdLow = (0x0206)<<5;
	CAN1_Filter.FilterMaskIdHigh= (0x0207)<<5;
	CAN1_Filter.FilterMaskIdLow = (0x0208)<<5;
	if (HAL_CAN_ConfigFilter(&hcan1,&CAN1_Filter)!= HAL_OK)
  {
    Error_Handler();
  }
}

void CAN2_Filter_Configuration(void)
{
	//configure filter for can2 m3508
	CAN_FilterTypeDef CAN2_Filter;
	CAN2_Filter.FilterFIFOAssignment = CAN_FILTER_FIFO1;
	CAN2_Filter.FilterScale = CAN_FILTERSCALE_16BIT;
	CAN2_Filter.FilterBank = 1;
	CAN2_Filter.FilterMode = CAN_FILTERMODE_IDLIST;
	//CAN2_Filter.SlaveStartFilterBank = 1;
	CAN2_Filter.FilterActivation = CAN_FILTER_ENABLE;
	CAN2_Filter.FilterIdHigh = (0x0201)<<5;
	CAN2_Filter.FilterIdLow = (0x0202)<<5;
	CAN2_Filter.FilterMaskIdHigh= (0x0203)<<5;
	CAN2_Filter.FilterMaskIdLow = (0x0204)<<5;
	if(HAL_CAN_ConfigFilter(&hcan2,&CAN2_Filter)!= HAL_OK)
  {
    Error_Handler();
  }
}

void Set_CAN1_Tx_Message(void)
{
	CAN1_TxMailBox = CAN_TX_MAILBOX0;
	
	CAN1_TxHeader.DLC = 8;
	CAN1_TxHeader.ExtId = 0;
	CAN1_TxHeader.IDE = CAN_ID_STD;
	CAN1_TxHeader.RTR = CAN_RTR_DATA;
	CAN1_TxHeader.StdId = 0x1FF;
	CAN1_TxHeader.TransmitGlobalTime = DISABLE;
}

void Set_CAN2_Tx_Message(void)
{
	CAN2_TxMailBox = CAN_TX_MAILBOX1;
	
	CAN2_TxHeader.DLC = 8;
	CAN2_TxHeader.ExtId = 0;
	CAN2_TxHeader.IDE = CAN_ID_STD;
	CAN2_TxHeader.RTR = CAN_RTR_DATA;
	CAN2_TxHeader.StdId = 0x200;
	CAN2_TxHeader.TransmitGlobalTime = DISABLE;
}

void Start_CAN1(void)
{
	CAN1_Filter_Configuration();
	Set_CAN1_Tx_Message();
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
	//HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO1_MSG_PENDING);
}

void Start_CAN2(void)
{
	CAN2_Filter_Configuration();
	Set_CAN2_Tx_Message();
	HAL_CAN_Start(&hcan2);
	//HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING	);
	HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO1_MSG_PENDING);
}

void CAN1_Tx(int16_t * tx_data)
{
	Set_GM6020_Voltage[0]=(uint8_t)(tx_data[0]>>8);
	Set_GM6020_Voltage[1]=(uint8_t)(tx_data[0]);
	Set_GM6020_Voltage[2]=(uint8_t)(tx_data[1]>>8);
	Set_GM6020_Voltage[3]=(uint8_t)(tx_data[1]);
	Set_GM6020_Voltage[4]=(uint8_t)(tx_data[2]>>8);
	Set_GM6020_Voltage[5]=(uint8_t)(tx_data[2]);
	Set_GM6020_Voltage[6]=(uint8_t)(tx_data[3]>>8);
	Set_GM6020_Voltage[7]=(uint8_t)(tx_data[3]);
	HAL_CAN_AddTxMessage(&hcan1,&CAN1_TxHeader,Set_GM6020_Voltage,&CAN1_TxMailBox);
}

void CAN2_Tx(int16_t * tx_data)
{
	Set_M3508_Current[0]=(uint8_t)(tx_data[0]>>8);
	Set_M3508_Current[1]=(uint8_t)(tx_data[0]);
	Set_M3508_Current[2]=(uint8_t)(tx_data[1]>>8);
	Set_M3508_Current[3]=(uint8_t)(tx_data[1]);
	Set_M3508_Current[4]=(uint8_t)(tx_data[2]>>8);
	Set_M3508_Current[5]=(uint8_t)(tx_data[2]);
	Set_M3508_Current[6]=(uint8_t)(tx_data[3]>>8);
	Set_M3508_Current[7]=(uint8_t)(tx_data[3]);
	HAL_CAN_AddTxMessage(&hcan2,&CAN2_TxHeader,Set_M3508_Current,&CAN2_TxMailBox);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(hcan==&hcan1)
	{
		HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0 ,&CAN1_RxHeader ,Get_GM6020_Data );
		GM6020_Data_Handler(CAN1_RxHeader.StdId,Get_GM6020_Data);
	}
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(hcan==&hcan2)
	{
		HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO1,&CAN2_RxHeader,Get_M3508_Data);
		M3508_Data_Handler(CAN2_RxHeader.StdId,Get_M3508_Data);
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* USER CODE END 0 */
