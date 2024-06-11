/**
 * @file pid.cpp
 * @author Yang JianYi 
 * @brief PID类的实现文件，该文件封装了基本PID的算法实现，包括微分先行，不完全微分，积分限幅，积分隔离以及增量式和位置式PID的实现。只需要修改PID的配置参数即可使用。
 *        使用发方法：配置PID参数(PID_Param_Init(),PID_Mode_Init())。 实时传入current值和target值，然后调用Adjust函数获取结果即可。
 * @version 0.1
 * @date 2024-05-16
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "pid.h"

SystemTick_Fun PidTimer::get_systemTick = NULL;

uint8_t PidTimer::update_timeStamp(void)
{
    uint32_t now_time=0;

    //Check get sysClock
    if(PidTimer::get_systemTick != NULL)
    {
        if(last_time == 0)
        {
            last_time = PidTimer::get_systemTick();
            return 1;
        }
        now_time = PidTimer::get_systemTick();

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

uint8_t PidTimer::getMicroTick_regist(uint32_t (*getTick_fun)(void))
{
    if(getTick_fun != NULL)
    {
        PidTimer::get_systemTick = getTick_fun;
        return 1;
    }
    else 
        return 0;
}

float PID::Adjust(void)
{
    //get real time failed
    if(update_timeStamp())
        return 0;
    
    //calculate error and deadzone
    error = target - current;
    if(_tool_Abs(error) < DeadZone)
    {
        Out=0;
        return Out;
    }
    
    //lowpass filter, change the trust value to adjust the filter
    error = LowPass_error.f(error);

    if(Imcreatement_of_Out)     //output increment mode
        P_Term = Kp * error - Kp * pre_error;
    else                        //output position mode
          P_Term = Kp * error;

    //calculate integral term, if use integral term
    if(Ki!=0)
    {
        if(Imcreatement_of_Out)
            integral_e = error;
        else
            integral_e  += error*dt;
        _tool_Constrain(&integral_e, -I_Term_Max/Ki, I_Term_Max/Ki);
    }
    else
    {
        integral_e = 0;
    }

    //integral separate
    if(_tool_Abs(error) < I_SeparThresh)
    {
        I_Term = Ki * integral_e;
        _tool_Constrain(&I_Term, -I_Term_Max, I_Term_Max);
    }
    else
    {
        I_Term = 0;
    }

    float d_err = 0;
    if(D_of_Current)
    {
        if(Imcreatement_of_Out)
            d_err = (current + eriler_Current - 2*pre_Current) / dt;
        else
            d_err = (current - pre_Current) / dt;
    }
    else
    {
        if(Imcreatement_of_Out)
            d_err = (error + eriler_error - 2*pre_error) / dt;
        else
            d_err = (error - pre_error) / dt;
    }

    d_err = LowPass_d_err.f(d_err);     //进行不完全微分
    D_Term = Kd * d_err;

    eriler_error = pre_error;
    pre_error = error;
    eriler_Current = pre_Current;
    pre_Current = current;
    if(Imcreatement_of_Out)
        Out = P_Term + I_Term + D_Term + last_out;
    else
        Out = P_Term + I_Term + D_Term;
    last_out = Out;

    _tool_Constrain(&Out, -Out_Max, Out_Max);

    return Out;
}

/**
     * @brief PID参数初始化
     * 
     * @param _Kp  比例系数
     * @param _Ki  积分系数
     * @param _Kd  微分系数
     * @param _I_Term_Max  I项限幅
     * @param _Out_Max  输出限幅
     * @param DeadZone  死区，fabs(error)小于DeadZone时，输出为0。设为负数不启用死区
     * @param I_SeparThresh  积分分离阈值，fabs(error)大于该阈值取消积分作用。
     */
    void PID::PID_Param_Init(float _Kp, float _Ki, float _Kd, float _I_Term_Max, float _Out_Max, float DeadZone)
    {
        DeadZone = DeadZone;
        Kp = _Kp;
        Ki = _Ki;
        Kd = _Kd;
        I_Term_Max = _I_Term_Max;
        Out_Max = _Out_Max;
    }
