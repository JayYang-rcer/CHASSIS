/**
 * @file service_communication.cpp
 * @author Yang Jianyi (2807643517@qq.com)
 * @brief   1) 该文件用于实现通信任务，包括CAN1、CAN2、UART的发送任务。
 *          2) 存放定义的CAN1、CAN2、UART的接收回调函数。
 *          3) 整个通讯协议的发送接收均采用freertos的框架实现，使用队列进行数据传输。
 * @version 0.1
 * @date 2024-04-09
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "service_communication.h"
#include "pid.h"
#include "motor.h"
#include "serial_tool.h"
#include "chassis.h"


void CAN1_Send_Task(void *pvParameters)
{
    CAN_TxMsg CAN_TxMsg;
    uint8_t free_can_mailbox;
    for(;;)
    {
        // comm_can_transmit_stdid(&hcan1, CAN_TxMsg.id, CAN_TxMsg.data, CAN_TxMsg.len);
        if(xQueueReceive(CAN1_TxPort, &CAN_TxMsg, portMAX_DELAY) == pdTRUE)
        {
            do{
                free_can_mailbox = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
            }while(free_can_mailbox == 0);
            comm_can_transmit_stdid(&hcan1, CAN_TxMsg.id, CAN_TxMsg.data, CAN_TxMsg.len);
        }
    }
}


void CAN2_Send_Task(void *pvParameters) 
{
    CAN_TxMsg CAN_TxMsg;
    uint8_t free_can_mailbox;
    for(;;)
    {
        // comm_can_transmit_stdid(&hcan1, CAN_TxMsg.id, CAN_TxMsg.data, CAN_TxMsg.len);
        if(xQueueReceive(CAN2_TxPort, &CAN_TxMsg, portMAX_DELAY) == pdTRUE)
        {
            do{
                free_can_mailbox = HAL_CAN_GetTxMailboxesFreeLevel(&hcan2);
            }while(free_can_mailbox == 0);
            comm_can_transmit_extid(&hcan2, CAN_TxMsg.id, CAN_TxMsg.data, CAN_TxMsg.len);
        }
    }
}


void UART_Send_Task(void *pvParameters)
{
    UART_TxMsg UART_TxMsg; 
    for(;;)
    {
        if(xQueueReceive(UART_TxPort, &UART_TxMsg, portMAX_DELAY) == pdTRUE)
        {
           HAL_UART_Transmit_DMA(UART_TxMsg.huart, (uint8_t *)UART_TxMsg.data_addr, UART_TxMsg.len);
        }
        osDelay(1);
    }
}


int can_flag=0;
/**
* @brief  Callback function in CAN Interrupt
* @param  None.
* @return None.
*/
void CAN1_RxCallBack(CAN_RxBuffer *RxBuffer)
{
    if(RxBuffer->header.IDE==CAN_ID_STD)
    {
        switch (RxBuffer->header.StdId)
        {   
            case 0x205:
            {
                RudderMotor[0].update(RxBuffer->data);
                can_flag++;
                break;
            }

            case 0x206:
            {
                RudderMotor[1].update(RxBuffer->data);
                break;
            }

            case 0x207:
            {
                RudderMotor[2].update(RxBuffer->data);
                break;
            }

            case 0x208:
            {
                RudderMotor[3].update(RxBuffer->data);
                break;
            }
        }
		
    }
}


void CAN2_RxCallBack(CAN_RxBuffer *RxBuffer)
{
    if(RxBuffer->header.IDE==CAN_ID_EXT)
    {   
        WheelMotor[0].update_vesc(RxBuffer);
        WheelMotor[1].update_vesc(RxBuffer);
        WheelMotor[2].update_vesc(RxBuffer);
        WheelMotor[3].update_vesc(RxBuffer);
    }
}


//串口DMA接收完毕回调函数，函数名字可以自定义，建议使用消息队列
uint32_t ROS_UART3_RxCallback(uint8_t* Receive_data, uint16_t data_len)
{
    UART_TxMsg Msg;
    if(Recieve_ROS_Port != NULL)
    {
        Msg.data_addr = Receive_data;
        Msg.len = data_len;
        Msg.huart = &huart1;
        if(Msg.data_addr != NULL)
            xQueueSendFromISR(Recieve_ROS_Port, &Msg, 0);
    }
    return 0;
}
