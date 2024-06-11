/**
 * @file service_config.cpp
 * @author Yang JianYi
 * @brief 配置文件，用于初始化系统资源和我们自己定义的应用程序初始化函数
 * @version 0.1
 * @date 2024-05-16
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "service_config.h"
#include "chassis_task.h"

Swerve_Chassis chassis(0.055,0,0.321,4);

void System_Resource_Init(void)
{
    DataPool_Init();
    Timer_Init(&htim4,USE_HAL_DELAY);
    PWM_ReInit(4200-1,40000-1,&htim10,TIM_CHANNEL_1);
    CAN_Init(&hcan1,CAN1_RxCallBack);
    CAN_Init(&hcan2,CAN2_RxCallBack);
    CAN_Filter_Init(&hcan1,CanFilter_0|CanFifo_0|Can_STDID|Can_DataType,0,0);
    CAN_Filter_Init(&hcan2,CanFilter_14|CanFifo_0|Can_EXTID|Can_DataType,0,0);
    CAN_Filter_Init(&hcan1,CanFilter_1|CanFifo_1|Can_STDID|Can_DataType,0,0);
    CAN_Filter_Init(&hcan2,CanFilter_15|CanFifo_1|Can_EXTID|Can_DataType,0,0);
    Uart_Init(&huart3, Uart3_Rx_Buff, 21, ROS_UART3_RxCallback);
    App_Init();
}


void App_Init(void)
{
    Set_PwmDuty(&htim10, TIM_CHANNEL_1, 0);
    Chassis_Pid_Init();
    PidTimer::getMicroTick_regist(Get_SystemTimer);
    AirJoy::getMicroTick_regist(Get_SystemTimer);
    ROS::getMicroTick_regist(Get_SystemTimer);
    Chassis_Base::getMicroTick_regist(Get_SystemTimer);
    Broadcast::getMicroTick_regist(Get_SystemTimer);
}

