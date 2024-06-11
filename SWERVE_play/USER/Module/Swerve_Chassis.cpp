/**
 * @file Swerve_Chassis.cpp
 * @author Yang JinaYi (2807643517@qq.com)
 * @brief 舵轮底盘控制类，包括舵轮底盘的速度控制、PID参数初始化、电机控制等
 * @note 1)使用该文件，需要在main.c中调用Chassis_Pid_Init函数进行PID参数初始化，在任务循环中调用chassis.Control(twist)进行速度控制，调用chassis.Motor_Control()进行电机控制
 *       2)由于八期R2底盘采用GM6020作为舵向，VESC驱动轮向电机，因此需要为这个类创建两个对象，一个是Motor_GM6020，一个是VESC。并且需要在service_communication.cpp
 *         中的can接收函数更新电机的实时参数信息，包括速度、角度等。
 *       3)如果需要更改轮向电机类型，ChassisVel_Trans_MotorRPM函数中的电机极对数需要更改。(有人继承寄轩师兄的舵轮的话)
 * @version 0.1
 * @date 2024-04-09
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "Chassis.h"

Motor_GM6020 RudderMotor[4] = {Motor_GM6020(1), Motor_GM6020(2), Motor_GM6020(3), Motor_GM6020(4)};
VESC WheelMotor[4] = {VESC(1), VESC(2), VESC(3), VESC(4)};

SystemTick_Fun Chassis_Base::get_systemTick = NULL;

int32_t ABS(int32_t a)
{
    if(a<0)
        return -a;
    else
        return a;
}


/**
 * @brief 定时器获取函数初始化，在config文件中进行条用
 * 
 * @param getTick_fun 
 * @return uint8_t 
 */
uint8_t Chassis_Base::getMicroTick_regist(uint32_t (*getTick_fun)(void))
{
    if(getTick_fun != NULL)
    {
        Chassis_Base::get_systemTick = getTick_fun;
        return 1;
    }
    else
        return 0;
}


/**
 * @brief 计算任务循环时间
 * 
 * @return uint8_t 
 */
uint8_t Chassis_Base::update_timeStamp(void)
{
    uint32_t now_time=0;

    //Check get sysClock
    if(Chassis_Base::get_systemTick != NULL)
    {
        if(last_time == 0)
        {
            last_time = Chassis_Base::get_systemTick();
            return 1;
        }
        now_time = Chassis_Base::get_systemTick();

        //overflow
        if(now_time < last_time)
            dt = (float)((now_time + 0xFFFFFFFF) - last_time);  //now_time is 32bit, use 0XFFFFFFFF to avoid overflow
        else
            dt = (float)(now_time - last_time);
        
        last_time = now_time;
        
        dt *= 0.000001f;

        return 0;
    }
    else
    {
        dt = 0;
        return 1;
    }
}


/**
 * @brief 底盘控制函数
 * 
 * @param cmd_vel 底盘速度参数结构体
 */
void Swerve_Chassis::Control(Robot_Twist_t cmd_vel)
{
    static int32_t last_wheel_vel[4]={0};   //上一时刻给轮子的速度赋值
    static int32_t last_wheelmotor_speed[4]={0};    //上一时刻轮子的实际转速
    update_timeStamp();

    Reset();
    for(int i=0; i<4; i++)
    {
        if(chassis_is_init==true&&Chassis_Safety_Check(25000)==true)
        {
            //底盘速度限幅
            cmd_vel_.linear.x = cmd_vel.linear.x>Speed_Max.linear.x?Speed_Max.linear.x:cmd_vel.linear.x;
            cmd_vel_.linear.y = cmd_vel.linear.y>Speed_Max.linear.y?Speed_Max.linear.y:cmd_vel.linear.y;
            cmd_vel_.angular.z = cmd_vel.angular.z>Speed_Max.angular.z?Speed_Max.angular.z:cmd_vel.angular.z;
            
            
            //底盘模式选择，可能没太大用处
            switch (cmd_vel.chassis_mode)
            {
                case X_MOVE:
                    X_Velocity_Calculate(cmd_vel_,&swerve[i]);
                    break;

                case Y_MOVE:
                    Y_Velocity_Calculate(cmd_vel_,&swerve[i]);
                    break;

                case NORMAL:
                    Velocity_Calculate(cmd_vel_,&swerve[i]);
                    break;

                default:
                    break;
            }

            //使用加速度控制底盘速度
            #if USE_VEL_ACCEL
            if(swerve[i].wheel_vel > 0 && swerve[i].wheel_vel >= last_wheel_vel[i])
                swerve[i].wheel_vel = last_wheel_vel[i] + accel_vel*dt*ChassisVel_Trans_MotorRPM(Wheel_Radius, 21);
            else if (swerve[i].wheel_vel < 0 && swerve[i].wheel_vel <= last_wheel_vel[i])
                swerve[i].wheel_vel = last_wheel_vel[i] - accel_vel*dt*ChassisVel_Trans_MotorRPM(Wheel_Radius, 21);
            else
            {}

            last_wheel_vel[i] = swerve[i].wheel_vel;
            #endif      

            //在底盘运动速度比较低时，才能进行后退。防止反冲电流过大
            if(last_wheelmotor_speed[i]*swerve[i].wheel_vel<0 && ABS(WheelMotor[i].get_speed())-1000>=0)
            {
                WheelMotor[i].Mode = SET_eRPM;
                WheelMotor[i].Out = 0;
            }
            else
            {
                WheelMotor[i].Mode = SET_eRPM;
                WheelMotor[i].Out = swerve[i].wheel_vel;
            }

            //电机速度赋值
            PID_Rudder_Speed[i].current = RudderMotor[i].get_speed();
            PID_Rudder_Pos[i].current = RudderMotor[i].get_angle();
            PID_Rudder_Pos[i].target = swerve[i].target_angle;
            PID_Rudder_Speed[i].target = PID_Rudder_Pos[i].Adjust();
            RudderMotor[i].Out = PID_Rudder_Speed[i].Adjust();
            last_wheelmotor_speed[i] = WheelMotor[i].get_speed();
        }

        if(Chassis_Safety_Check(25000)==false)
        {
            WheelMotor[i].Mode = SET_CURRENT;
            WheelMotor[i].Out = 0;
        }
    }
    
}


/**
 * @brief 底盘电机控制函数
 * 
 * @return int 
 */
int Swerve_Chassis::Motor_Control(void)
{
    RM_Motor_SendMsgs(&hcan1, RudderMotor);

    //由于担心是一个任务同时发送太多can帧，防止阻塞严重，把can帧速度拉低。
    //方法比较简陋，希望后来者可以改善这个问题并做好封装
	static int send_flag=0;
    if(send_flag<1)
    {
        VESC_SendMsgs(&hcan2, WheelMotor[0]);
    }
    else if (send_flag>=1&&send_flag<2)
    {
        VESC_SendMsgs(&hcan2, WheelMotor[1]);
    }
    else if (send_flag>=2&&send_flag<3)
    {
        VESC_SendMsgs(&hcan2, WheelMotor[2]);
    }
    else if (send_flag>=3&&send_flag<4)
    {
        VESC_SendMsgs(&hcan2, WheelMotor[3]);
    }
    else
    {
        send_flag = -1;
    }

    send_flag++;
    return 0;
}


/**
 * @brief 底盘速度计算
 */
void Swerve_Chassis::Velocity_Calculate(Robot_Twist_t cmd_vel, Swerve_t *swerve)
{
    float wheel_Vx=0, wheel_Vy=0;
    switch(swerve->num)
    {
        case 1:
            wheel_Vx = (cmd_vel.linear.x + Chassis_Radius*SIN*cmd_vel.angular.z);
            wheel_Vy = (cmd_vel.linear.y + Chassis_Radius*COS*cmd_vel.angular.z);
            break;
        case 2:
            wheel_Vx = (cmd_vel.linear.x - Chassis_Radius*SIN*cmd_vel.angular.z);
            wheel_Vy = (cmd_vel.linear.y + Chassis_Radius*COS*cmd_vel.angular.z);
            break;
        case 3:
            wheel_Vx = (cmd_vel.linear.x - Chassis_Radius*SIN*cmd_vel.angular.z);
            wheel_Vy = (cmd_vel.linear.y - Chassis_Radius*COS*cmd_vel.angular.z);
            break;
        case 4:
            wheel_Vx = (cmd_vel.linear.x + Chassis_Radius*SIN*cmd_vel.angular.z);
            wheel_Vy = (cmd_vel.linear.y - Chassis_Radius*COS*cmd_vel.angular.z);
            break;
        default:
            break;
    }

    swerve->wheel_vel = sqrt(wheel_Vx*wheel_Vx + wheel_Vy*wheel_Vy)*ChassisVel_Trans_MotorRPM(Wheel_Radius, 21);
    swerve->target_angle = atan2f(wheel_Vy,wheel_Vx)/PI*180;   // -180~180
    
    //底盘速度赋值为0时，刹车
    if(ABS(cmd_vel.linear.x)-0.02<=0&&ABS(cmd_vel.linear.y)-0.02<=0&&ABS(cmd_vel.angular.z)-0.02<=0)
    {
        //设置刹车电流   
        WheelMotor[swerve->num-1].Mode = SET_BRAKE;
        WheelMotor[swerve->num-1].Out = 10;

        //四个轮子均小于100erpm时，超过2s锁住底盘
        Chassis_Lock(swerve);
    }

    //底盘舵向的劣弧计算
    RudderAngle_Adjust(swerve); 
    swerve->now_angle = swerve->target_angle;
}


/**
 * @brief 把底盘锁死在一个方向，但是能否走的直。取决于舵向精度
 * 
 * @param cmd_vel 
 * @param swerve 
 */
void Swerve_Chassis::X_Velocity_Calculate(Robot_Twist_t cmd_vel, Swerve_t *swerve)
{
    swerve->wheel_vel = cmd_vel.linear.x * ChassisVel_Trans_MotorRPM(Wheel_Radius, 21);

    if(cmd_vel.linear.x==0)
    {
        Chassis_Lock(swerve);
    }

    swerve->target_angle = 0;
    RudderAngle_Adjust(swerve); 
}


/**
 * @brief 把底盘锁死在一个方向，但是能否走的直。取决于舵向精度
 * 
 * @param cmd_vel 
 * @param swerve 
 */
void Swerve_Chassis::Y_Velocity_Calculate(Robot_Twist_t cmd_vel, Swerve_t *swerve)
{
    swerve->wheel_vel = cmd_vel.linear.y * ChassisVel_Trans_MotorRPM(Wheel_Radius, 21);

    if(cmd_vel.linear.y==0)
    {
        Chassis_Lock(swerve);
    }

    swerve->target_angle = 90;
    RudderAngle_Adjust(swerve); 
}


/**
 * @brief 舵向角度调整，保证舵向角度在一个周期，并进行劣弧计算
 * 
 * @param swerve 
 */
void Swerve_Chassis::RudderAngle_Adjust(Swerve_t *swerve)
{
    
    float error=0;
    int N1=0,N2=0;
    //保证电机的实时角度在一个360度的周期之内
    // N = (int)(swerve->now_angle/180.0f) - (int)(swerve->now_angle/360.0f);
    if(swerve->now_angle>=0)
        N1 = swerve->now_angle/360;
    else
        N1 = swerve->now_angle/360-1;

    if(swerve->target_angle>=0)
        N2 = swerve->target_angle/360;
    else
        N2 = swerve->target_angle/360-1;

    N = N1-N2;  //now_angle>0&&now_angle<180   ---->  N=0
                                                                            //now_angle>=180&&now_angle<360  ---->  N=1
                                                                            //now_angle>=360                 ---->  N=1
                                                                            //now_angle<=-360                 ---->  N=-1
                                                                            //now_angle>-360&&now_angle<=180  ---->  N=-1
                                                                            //now_angle>180&&now_angle<=0     ---->  N=0


    swerve->target_angle = swerve->target_angle + N*360.0f;
    error = abs(swerve->target_angle - swerve->now_angle);

    if(swerve->target_angle < swerve->now_angle)
    {
        if(error > 270)
        {
            swerve->target_angle = swerve->target_angle + 360.0f;
        }
        else if(error > 90)
        {
            swerve->target_angle = swerve->target_angle + 180.0f;
            swerve->wheel_vel = -swerve->wheel_vel;
        }
    }
    else
    {
        if(error > 270)
        {
            swerve->target_angle = swerve->target_angle - 360.0f;
        }
        else if(error > 90)
        {
            swerve->target_angle = swerve->target_angle - 180.0f;
            swerve->wheel_vel = -swerve->wheel_vel;
        }
    }
}


/**
 * @brief Chassis status reset
 */
void Swerve_Chassis::Reset(void)
{
    static uint32_t real_time=0;
    if(chassis_is_init==false&&reset_flag==2)
    {
        for(int i=0; i<4; i++)
        {
            Chassis_Lock(&swerve[i]);
        }
        reset_flag = 0;
    }

    if(reset_flag==0)
    {
        real_time = 0;
        reset_flag = 1;
    }

    real_time += dt*1000;

    if(real_time<1000)
    {
        for(int i=0; i<4; i++)
        {
            WheelMotor[i].Mode = SET_CURRENT;
            WheelMotor[i].Out = 0;
        }    
    }
    else if(real_time>=1000 && real_time<3000)
    {
        cmd_vel_.linear.x = 0;
        cmd_vel_.linear.y = 0;
        cmd_vel_.angular.z = 1;

        for(int i=0; i<4; i++)
        {
            Velocity_Calculate(cmd_vel_, &swerve[i]);
            WheelMotor[i].Mode = SET_eRPM;
            WheelMotor[i].Out = swerve[i].wheel_vel;

            //电机速度赋值
            PID_Rudder_Speed[i].current = RudderMotor[i].get_speed();
            PID_Rudder_Pos[i].current = RudderMotor[i].get_angle();
            PID_Rudder_Pos[i].target = swerve[i].target_angle;
            PID_Rudder_Speed[i].target = PID_Rudder_Pos[i].Adjust();
            RudderMotor[i].Out = PID_Rudder_Speed[i].Adjust();
        }
    }
    else if(real_time>=3000 && real_time<5000)
    {
        cmd_vel_.linear.x = 0;
        cmd_vel_.linear.y = 0;
        cmd_vel_.angular.z = -1;
        
        for(int i=0; i<4; i++)
        {
            Velocity_Calculate(cmd_vel_, &swerve[i]);
            WheelMotor[i].Mode = SET_eRPM;
            WheelMotor[i].Out = swerve[i].wheel_vel;

            //电机速度赋值
            PID_Rudder_Speed[i].current = RudderMotor[i].get_speed();
            PID_Rudder_Pos[i].current = RudderMotor[i].get_angle();
            PID_Rudder_Pos[i].target = swerve[i].target_angle;
            PID_Rudder_Speed[i].target = PID_Rudder_Pos[i].Adjust();
            RudderMotor[i].Out = PID_Rudder_Speed[i].Adjust();
        }
    }
    else
    {
        chassis_is_init = true;
        reset_flag=2;
    }
}


/**
 * @brief 舵轮底盘安全检测，检测电机的最大电流是否超过设定值。电流值过大持续时间超过1s，蜂鸣器报警
 * 
 * @param Current_Max 
 * @return true 电流安全
 * @return false 电流过大
 */
bool Swerve_Chassis::Chassis_Safety_Check(float Current_Max)
{
    static int beep_flag=0;
    static uint32_t start_time=0, now_time=0, real_time=0;
    for(int i=0; i<4; i++)
    {
        if(WheelMotor[i].get_tarque() > Current_Max)
        {
            
            if(beep_flag==0)
            {
                start_time = get_systemTick()/1000;
                beep_flag = 1;
            }

            now_time = get_systemTick()/1000;
            if(now_time<start_time)
            {
                real_time = 0xFFFFFFFF-start_time+now_time;
            }
            else
            {
                real_time = now_time-start_time;
            }
            
            if(real_time>2000&&beep_flag==1)
            {
                Set_PwmFreq(&htim10, 5);
                Set_PwmDuty(&htim10, TIM_CHANNEL_1, 50);
                beep_flag = 2;
                return false;
            }
        }
    }

    return true;
}


void Swerve_Chassis::Chassis_Lock(Swerve_t *swerve)
{
    static int reset_flag=0;
    static int32_t stop_start_time=0;
    
    if(ABS(WheelMotor[swerve->num-1].get_speed())<100)
    {   
        if(reset_flag==0)
        {
            reset_flag = 1;
            stop_start_time = get_systemTick()/1000;
        }

        if(get_systemTick()/1000-stop_start_time>2000) 
        {
            switch(swerve->num)
            {
                case 1:
                    swerve->target_angle = theta/2;
                    break;
                case 2:
                    swerve->target_angle = -theta/2;
                    break;
                case 3:
                    swerve->target_angle = theta/2;
                    break;
                case 4:
                    swerve->target_angle = -theta/2;
                    break;
                default:
                    break;
            }
        }
        else
        {
            //保证底盘停止时，舵向电机不会转动
            if(ABS(swerve->wheel_vel)< 100)
                swerve->target_angle = swerve->now_angle-N*360.0f;
        }
    }
    else
    {
        //保证底盘停止时，舵向电机不会转动
        if(ABS(swerve->wheel_vel)<100)
            swerve->target_angle = swerve->now_angle-N*360.0f;
        reset_flag = 0;
    }
}


/**
 * @brief 舵轮PID参数初始化
 * 
 * @param PID_Type 
 * @param Kp 
 * @param Ki 
 * @param Kd 
 * @param Integral_Max 积分限幅
 * @param OUT_Max 输出限幅
 */
void Swerve_Chassis::Pid_Param_Init(CHASSIS_PID_E PID_Type, float Kp, float Ki, float Kd, float Integral_Max, float Out_Max, float DeadZone)
{
    switch(PID_Type)
    {
        case RUDDER_LEFT_FRONT_Speed_E:
            PID_Rudder_Speed[0].PID_Param_Init(Kp, Ki, Kd, Integral_Max, Out_Max, DeadZone);
            break;
        case RUDDER_RIGHT_FRONT_Speed_E:
            PID_Rudder_Speed[1].PID_Param_Init(Kp, Ki, Kd, Integral_Max, Out_Max, DeadZone);
            break;
        case RUDDER_LEFT_REAR_Speed_E:
            PID_Rudder_Speed[2].PID_Param_Init(Kp, Ki, Kd, Integral_Max, Out_Max, DeadZone);
            break;
        case RUDDER_RIGHT_REAR_Speed_E:
            PID_Rudder_Speed[3].PID_Param_Init(Kp, Ki, Kd, Integral_Max, Out_Max, DeadZone);
            break;
        case RUDDER_LEFT_FRONT_Pos_E:
            PID_Rudder_Pos[0].PID_Param_Init(Kp, Ki, Kd, Integral_Max, Out_Max, DeadZone);
            break;
        case RUDDER_RIGHT_FRONT_Pos_E:
            PID_Rudder_Pos[1].PID_Param_Init(Kp, Ki, Kd, Integral_Max, Out_Max, DeadZone);
            break;
        case RUDDER_LEFT_REAR_Pos_E:
            PID_Rudder_Pos[2].PID_Param_Init(Kp, Ki, Kd, Integral_Max, Out_Max, DeadZone);
            break;
        case RUDDER_RIGHT_REAR_Pos_E:
            PID_Rudder_Pos[3].PID_Param_Init(Kp, Ki, Kd, Integral_Max, Out_Max, DeadZone);
            break;
        default:
            break;
    }
}


/**
 * @brief 舵轮底盘PID模式初始化
 * 
 * @param PID_Type 
 * @param LowPass_error 误差低通过滤器系数
 * @param LowPass_d_err 不完全微分系数
 * @param D_of_Current 是否开启微分先行
 * @param Imcreatement_of_Out 是否使用增量式输出
 */
void Swerve_Chassis::Pid_Mode_Init(CHASSIS_PID_E PID_Type, float LowPass_error, float LowPass_d_err, bool D_of_Current, bool Imcreatement_of_Out)
{
    switch(PID_Type)
    {
        case RUDDER_LEFT_FRONT_Pos_E:
            PID_Rudder_Pos[0].PID_Mode_Init(LowPass_error, LowPass_d_err, D_of_Current, Imcreatement_of_Out);
            break;
        case RUDDER_RIGHT_FRONT_Pos_E:
            PID_Rudder_Pos[1].PID_Mode_Init(LowPass_error, LowPass_d_err, D_of_Current, Imcreatement_of_Out);
            break;
        case RUDDER_LEFT_REAR_Pos_E:
            PID_Rudder_Pos[2].PID_Mode_Init(LowPass_error, LowPass_d_err, D_of_Current, Imcreatement_of_Out);
            break;
        case RUDDER_RIGHT_REAR_Pos_E:
            PID_Rudder_Pos[3].PID_Mode_Init(LowPass_error, LowPass_d_err, D_of_Current, Imcreatement_of_Out);
            break;
        case RUDDER_LEFT_FRONT_Speed_E:
            PID_Rudder_Speed[0].PID_Mode_Init(LowPass_error, LowPass_d_err, D_of_Current, Imcreatement_of_Out);
            break;
        case RUDDER_RIGHT_FRONT_Speed_E:
            PID_Rudder_Speed[1].PID_Mode_Init(LowPass_error, LowPass_d_err, D_of_Current, Imcreatement_of_Out);
            break;
        case RUDDER_LEFT_REAR_Speed_E:
            PID_Rudder_Speed[2].PID_Mode_Init(LowPass_error, LowPass_d_err, D_of_Current, Imcreatement_of_Out);
            break;
        case RUDDER_RIGHT_REAR_Speed_E:
            PID_Rudder_Speed[3].PID_Mode_Init(LowPass_error, LowPass_d_err, D_of_Current, Imcreatement_of_Out);
            break;
        default:
            break;
    }
}
