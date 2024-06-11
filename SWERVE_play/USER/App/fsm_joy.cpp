/**
 * @file fsm_joy.cpp
 * @author Yang JianYi
 * @brief 舵轮底盘应用文件，包括上位机控制接口的调用以及stm32手柄的调试，开关是通过宏定义来控制的(USE_ROS_CONTROL)。
 * @version 0.1
 * @date 2024-05-16
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "fsm_joy.h"
#include "drive_tim.h"
#include "chassis_task.h"

void Air_Joy_Task(void *pvParameters)
{
    for(;;)
    {
#if USE_ROS_CONTROL
            ROS_Cmd_Process();
#else 
            Robot_Twist_t twist;
            if(air_joy.LEFT_X>1400&&air_joy.LEFT_X<1600)
                air_joy.LEFT_X = 1500;
            if(air_joy.LEFT_Y>1400&&air_joy.LEFT_Y<1600)
                air_joy.LEFT_Y = 1500;
            if(air_joy.RIGHT_X>1400&&air_joy.RIGHT_X<1600)
                air_joy.RIGHT_X = 1500;
            if(air_joy.RIGHT_Y>1400&&air_joy.RIGHT_Y<1600)  
                air_joy.RIGHT_Y = 1500;

            if(air_joy.LEFT_X!=0||air_joy.LEFT_Y!=0||air_joy.RIGHT_X!=0||air_joy.RIGHT_Y!=0)
            {
                if(air_joy.SWA>1950&&air_joy.SWA<2050)
                {
                    if(air_joy.SWC>950&&air_joy.SWC<1050)
                    {
                        twist.chassis_mode = NORMAL;
                    }
                    else if(air_joy.SWC>1450&&air_joy.SWC<1550)
                    {
                        twist.chassis_mode = X_MOVE;
                    }
                    else if(air_joy.SWC>1950&&air_joy.SWC<2050)
                    {
                        twist.chassis_mode = Y_MOVE;
                    }
                    else
                    {
                        twist.chassis_mode = NORMAL;
                    }
                    
                    twist.linear.x = (air_joy.LEFT_Y - 1500)/500.0 * 3;
                    twist.linear.y = (air_joy.LEFT_X - 1500)/500.0 * 3;
                    twist.angular.z = (air_joy.RIGHT_X - 1500)/500.0 * 4;
                    xQueueSend(Chassia_Port, &twist, 0);
                }
            }
            else
            {
                twist = {0};
            }

#endif
        osDelay(1);
    }
}


void Broadcast_Task(void *pvParameters)
{
    static Broadcast broadcast;
    for(;;)
    {
        Robot_Status_t status = {STOP};
        if(xQueueReceive(Broadcast_Port, &status, 0) == pdPASS)
        {
            broadcast.Send_To_Broadcast(status);
        }
        // status.robot_init = AUTO_MODE;
        broadcast.Send_To_Broadcast(status);
        osDelay(1);
    }
}


// ROS ros;
void ROS_Cmd_Process(void)
{
    static ROS ros;
    static uint32_t dt=0,now=0,last=0;;
    dt = now - last;    //ms
    UART_TxMsg Msg;
    static Robot_Twist_t twist,twist_ros;
    static Robot_Status_t status;

    static uint8_t ctrl_flag=0;
    twist.chassis_mode = NORMAL;
    if(xQueueReceive(Recieve_ROS_Port, &Msg, 0) == pdPASS)
    {
        last = now;
        ros.Recieve_From_ROS((uint8_t *)(Msg.data_addr));
        twist.linear.x = ros.readFromRosData.x;
        twist.linear.y = ros.readFromRosData.y;
        twist.angular.z = ros.readFromRosData.z;
        twist.chassis_mode = (CHASSIS_MODE)ros.readFromRosData.ctrl_mode;
        ctrl_flag = ros.readFromRosData.ctrl_flag;
        status.robot_init = ros.readFromRosData.status.robot_init;
        status.path_mode = ros.readFromRosData.status.path_mode;
        status.sensor = ros.readFromRosData.status.sensor;
        status.control_mode = ros.readFromRosData.status.control_mode;
        twist_ros = twist;
    }

    if(dt>100)
    {
        twist = {0};
        ctrl_flag = 0;
    }

    if(ctrl_flag == 1)
        xQueueSend(Chassia_Port, &twist_ros, 0);
    now = ros.get_systemTick()/1000;    //ms

    xQueueSend(Broadcast_Port, &status, 0);
}
