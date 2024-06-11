/**
 * @file omni_chassis.cpp
 * @author Yang JianYi (2807643517@qq.com)
 * @brief 全向轮底盘驱动文件，使用该文件，需要创建一个全向轮底盘类(由于这个工程是舵轮底盘工程，所以这个文件没有使用)。如果要使用这个类，需要将舵轮底盘的
 *        调用文件替换为全向轮底盘的调用文件。(chassis_task.cpp),同时把通信文件中的can接收函数进行更改。
 * @version 0.1
 * @date 2024-05-16
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "Chassis.h"

#if USE_THREE_OMNI_WHEEL
Motor_C620 Moror[3] = {Motor_C620(1), Motor_C620(2), Motor_C620(3)};
#endif
#if USE_FOUR_OMNI_WHEEL
Motor_C620 Motor[4] = {Motor_C620(1), Motor_C620(2), Motor_C620(3), Motor_C620(4)};
#endif

void Omni_Chassis::Control(Robot_Twist_t cmd_vel)
{
#if USE_FOUR_OMNI_WHEEL||USE_THREE_OMNI_WHEEL
    Velocity_Calculate(cmd_vel);
    for(int i=0; i<wheel_num; i++)
    {
        PID_Wheel[i].current = Motor[i].get_speed();
        PID_Wheel[i].target = wheel[i].wheel_vel;
        Motor[i].Out = PID_Wheel[i].Adjust();
    }
#endif
}

void Omni_Chassis::Motor_Control(void)
{
#if USE_FOUR_OMNI_WHEEL||USE_THREE_OMNI_WHEEL
    RM_Motor_SendMsgs(&hcan1, Motor);
#endif
}

void Omni_Chassis::Velocity_Calculate(Robot_Twist_t cmd_vel)
{
    if(wheel_num==4)
    {
        wheel[0].wheel_vel = (-cmd_vel.linear.y*COS45 + cmd_vel.linear.x*COS45 + cmd_vel.angular.z*Chassis_Radius) * ChassisVel_Trans_MotorRPM(Wheel_Radius, 19);
        wheel[1].wheel_vel = (-cmd_vel.linear.y*COS45 - cmd_vel.linear.x*COS45 + cmd_vel.angular.z*Chassis_Radius) * ChassisVel_Trans_MotorRPM(Wheel_Radius, 19);
        wheel[2].wheel_vel = ( cmd_vel.linear.y*COS45 - cmd_vel.linear.x*COS45 + cmd_vel.angular.z*Chassis_Radius) * ChassisVel_Trans_MotorRPM(Wheel_Radius, 19);
        wheel[3].wheel_vel = ( cmd_vel.linear.y*COS45 + cmd_vel.linear.x*COS45 + cmd_vel.angular.z*Chassis_Radius) * ChassisVel_Trans_MotorRPM(Wheel_Radius, 19);
    }
    else
    {
        wheel[0].wheel_vel = (cmd_vel.linear.y*COS30 - cmd_vel.linear.x*SIN30 + cmd_vel.angular.z*Chassis_Radius) * ChassisVel_Trans_MotorRPM(Wheel_Radius, 19);
        wheel[1].wheel_vel = (-cmd_vel.linear.y*SIN30 - cmd_vel.linear.x*COS30 + cmd_vel.angular.z*Chassis_Radius) * ChassisVel_Trans_MotorRPM(Wheel_Radius, 19);
        wheel[2].wheel_vel = (cmd_vel.linear.x + cmd_vel.angular.z*Chassis_Radius) * ChassisVel_Trans_MotorRPM(Wheel_Radius, 19);
    }
    
}


/**
 * @brief 底盘重置函数，根据自己的需求进行编写
 * 
 */
void Omni_Chassis::Reset(void)
{
    
}


bool Omni_Chassis::Pid_Param_Init(int num, float Kp, float Ki, float Kd, float Integral_Max, float OUT_Max, float DeadZone)
{
    PID_Wheel[num].PID_Param_Init(Kp, Ki, Kd, OUT_Max, Integral_Max,DeadZone);
    return true;
}

bool Omni_Chassis::Pid_Mode_Init(int num, float LowPass_error, float LowPass_d_err, bool D_of_Current, bool Imcreatement_of_Out)
{
    PID_Wheel[num].PID_Mode_Init(LowPass_error, LowPass_d_err, D_of_Current, Imcreatement_of_Out);
    return true;
}
