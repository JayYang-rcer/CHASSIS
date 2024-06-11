#ifndef  SERVICE_COMMUNICATION_H
#define SERVICE_COMMUNICATION_H

#include "service_config.h"
#include "drive_can.h"
#include "user_debug.h"

#ifdef __cplusplus
extern "C" {
#endif

void CAN1_Send_Task(void *pvParameters);
void CAN2_Send_Task(void *pvParameters);
void UART_Send_Task(void *pvParameters);

void CAN1_RxCallBack(CAN_RxBuffer *CAN_RxBuffer);	//CAN1接收回调函数
void CAN2_RxCallBack(CAN_RxBuffer *CAN_RxBuffer);	//CAN2接收回调函数

uint32_t ROS_UART3_RxCallback(uint8_t* Receive_data, uint16_t data_len);    //UART3接收回调函数

#ifdef __cplusplus 
}
#endif

#endif 
