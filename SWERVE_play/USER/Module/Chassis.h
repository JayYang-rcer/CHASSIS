#pragma once 
#include "motor.h"
#include "math.h"
#include "pid.h"
#include "service_config.h"
#include "drive_tim.h"

typedef uint32_t (*SystemTick_Fun)(void);
#define PI 3.1415926f

#ifdef __cplusplus
extern "C" {
#endif 

typedef struct Swerve_t     //used for swerve chassis
{
    int num;    //the number of swerve
    float wheel_vel;    //the velocity of wheel
    float target_angle; //the target angle of rudder
    float now_angle;    //the now angle of rudder
}Swerve_t;

typedef struct Wheel_t    //used for mecanum chassis and omni chassis
{
    int num;
    float wheel_vel;
}Wheel_t;

enum CHASSIS_PID_E
{
    RUDDER_LEFT_FRONT_Speed_E,
    RUDDER_RIGHT_FRONT_Speed_E,
    RUDDER_LEFT_REAR_Speed_E,
    RUDDER_RIGHT_REAR_Speed_E,
    RUDDER_LEFT_FRONT_Pos_E,
    RUDDER_RIGHT_FRONT_Pos_E,
    RUDDER_LEFT_REAR_Pos_E,
    RUDDER_RIGHT_REAR_Pos_E
};

extern Motor_GM6020 RudderMotor[4];
extern VESC WheelMotor[4];

#ifdef __cplusplus
}

class Chassis_Base
{
public:
    Chassis_Base(float Wheel_Radius, float Wheel_Track, float Chassis_Radius,int wheel_num){}
    static uint8_t getMicroTick_regist(uint32_t (*getTick_fun)(void));

    Robot_Twist_t Speed_Max={0};

    Robot_Twist_t RoboSpeed_To_WorldSpeed(Robot_Twist_t RoboSpeed, float YawAngle_Now)
    {
        Robot_Twist_t WorldSpeed;
        TRANS_COS = cos (YawAngle_Now*PI/180);
        TRANS_SIN = sin (YawAngle_Now*PI/180);

        WorldSpeed.linear.x  = (RoboSpeed.linear.x * TRANS_COS + RoboSpeed.linear.y * TRANS_SIN);
        WorldSpeed.linear.y  = -(RoboSpeed.linear.x * TRANS_SIN - RoboSpeed.linear.y * TRANS_COS);
        WorldSpeed.angular.z = RoboSpeed.angular.z;
        return WorldSpeed;
    }

protected:
    static SystemTick_Fun get_systemTick;

    template<typename Type> 
    void Constrain(Type *x, Type Min, Type Max) 
    {
        if(*x < Min) 
            *x = Min;
        else if(*x > Max) 
            *x = Max;
        else{;}
    }

    /**
     * @brief 速度转轮子转速函数
     * @param gear_ratio 电机减速比，电机转子实际转速/轮子转速
     * @return float 
     */
    float ChassisVel_Trans_MotorRPM(float Wheel_Radius, float gear_ratio)
    {
        return ((60*gear_ratio)/(2*PI*Wheel_Radius));
    }

    /**
     * @brief 轮子转速转速度函数
     * @param gear_ratio 电机减速比，电机转子实际转速/轮子转速
     * @return 转换系数
     */
    float MotorRPM_Trans_ChassisVel(float Wheel_Radius, float gear_ratio)
    {
        return (2*PI*Wheel_Radius/(60*gear_ratio));
    }

    uint8_t update_timeStamp(void);
    
    Robot_Twist_t cmd_vel_={0};
    float  dt;
    uint32_t last_time;
private:
    double TRANS_SIN,TRANS_COS;
};


class Swerve_Chassis : public Chassis_Base
{
public:
    Swerve_Chassis(float Wheel_Radius, float Wheel_Track, float Chassis_Radius,int wheel_num) : Chassis_Base(Wheel_Radius, Wheel_Track, Chassis_Radius,wheel_num)
    {
        this->Wheel_Radius = Wheel_Radius;
        this->Wheel_Track = Wheel_Track;
        this->Chassis_Radius = Chassis_Radius;
        this->wheel_num = wheel_num;
        this->cmd_vel_.chassis_mode = NORMAL;
        swerve[0].num = 1;
        swerve[1].num = 2;
        swerve[2].num = 3;
        swerve[3].num = 4;
    }

    float theta=99.26;  //底盘两对对角轮连线的夹角，用于解算轮子速度
    float accel_vel=0; //底盘加速度
    bool chassis_is_init = false;
    void Control(Robot_Twist_t cmd_vel);
    int Motor_Control(void);
    void Pid_Param_Init(CHASSIS_PID_E PID_Type, float Kp, float Ki, float Kd, float Integral_Max, float Out_Max, float DeadZone);
    void Pid_Mode_Init(CHASSIS_PID_E PID_Type, float LowPass_error, float LowPass_d_err, bool D_of_Current, bool Imcreatement_of_Out);

private:
    int wheel_num = 0;
    Swerve_t swerve[4];
    float Wheel_Radius = 0.038;
    float Wheel_Track = 0;
    float Chassis_Radius = 0.641/2;
    float COS=cos(99.26/2),SIN=sin(99.26/2);
    int N=0;    //记录舵向转过的圈数
    uint8_t reset_flag=2;
    uint8_t lock_flag=0;
    bool Chassis_Safety_Check(float Current_Max);
    void Reset(void);
    void RudderAngle_Adjust(Swerve_t *swerve);
    void Chassis_Lock(Swerve_t *swerve);
    void Velocity_Calculate(Robot_Twist_t cmd_vel, Swerve_t *swerve);
    void X_Velocity_Calculate(Robot_Twist_t cmd_vel, Swerve_t *swerve);
    void Y_Velocity_Calculate(Robot_Twist_t cmd_vel, Swerve_t *swerve);

    PID PID_Rudder_Speed[4];
    PID PID_Rudder_Pos[4];
};


class Omni_Chassis : public Chassis_Base
{
public:
    Omni_Chassis(float Wheel_Radius, float Wheel_Track, float Chassis_Radius,int wheel_num) : Chassis_Base(Wheel_Radius, Wheel_Track, Chassis_Radius, wheel_num)
    {
        this->Wheel_Radius = Wheel_Radius;
        this->Wheel_Track = Wheel_Track;
        this->Chassis_Radius = Chassis_Radius;
        this->wheel_num = wheel_num;
        wheel[0].num = 1;
        wheel[1].num = 2;
        wheel[2].num = 3;
        wheel[3].num = 4;
    }

    bool Pid_Param_Init(int num, float Kp, float Ki, float Kd, float Integral_Max, float OUT_Max, float DeadZone);
    bool Pid_Mode_Init(int num, float LowPass_error, float LowPass_d_err, bool D_of_Current, bool Imcreatement_of_Out);

    void Control(Robot_Twist_t cmd_vel);
    void Motor_Control(void);
    void Reset(void);
private:
    PID PID_Wheel[4];
    Wheel_t wheel[4];
    int wheel_num = 0;
    float Wheel_Radius = 0.152f/2;
    float Wheel_Track = 0;
    float Chassis_Radius = 0.641/2;
    float COS45=cos(PI/4),SIN45=sin(PI/4);
    float COS30=cos(PI/6),SIN30=sin(PI/6);
    void Velocity_Calculate(Robot_Twist_t cmd_vel);
};


class Mecanum_Chassis : public Chassis_Base
{
public:
    
private:

};

#endif
