/**
 * @file user_debug.cpp
 * @author Yang Jianyi
 * @brief 用于调试的任务
 * @version 0.1
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "user_debug.h"
#include "serial_tool.h"
#include "ROS.h"

void User_Debug_Task(void *pvParameters)
{
#if USE_DEBUG_TASK
    float angle1;
    PID PID_Speed,PID_Pos;
    PID_Speed.PID_Param_Init(12, 0.1, 0, 400, 30000);
    PID_Speed.D_of_Current = true;
    PID_Speed.Imcreatement_of_Out = true;
	
    PID_Pos.PID_Param_Init(150, 0.2, 0.8, 400, 30000);
    PID_Pos.D_of_Current = true;
    PID_Pos.Imcreatement_of_Out = false;
    PID_Pos.target = 0;
    PID_Pos.LowPass_d_err.Trust = 0.5;
    PID_Pos.DeadZone = 0.05;
	GM6020.encoder_offset=4096;
    
    for(;;)
    {
        PID_Speed.current = GM6020.get_speed();
        angle1 = PID_Pos.current = GM6020.get_angle();
        PID_Speed.target = PID_Pos.Adjust();
        GM6020.Out = PID_Speed.Adjust();
        RM_Motor_SendMsgs(&hcan1, GM6020);
        osDelay(1);
    }
#else
    for(;;)
    {
        osDelay(1);
    }
#endif
}
