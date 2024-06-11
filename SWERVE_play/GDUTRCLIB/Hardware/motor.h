/**
 * @file motor.h
 * @author Yang JianYi (287643517@qq.com)
 * @brief 电机调用函数，每款电机驱动器都已经封装成一个类，通过调用类的函数来实现电机的控制。包括C610、C620、GM6020、VESC等。
 *        如果要新增电机类，请务必继承Motor_Base类，以保证接口的统一性。
 *        考虑增加达妙电机的驱动文件。
 * @version 0.1
 * @date 2024-05-16
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#pragma once

#include <stdint.h>
#include "../Components/drive_can.h"
#include "../Components/drive_can.h"
#include "data_pool.h"
#include "tool.h"

#ifdef __cplusplus


typedef enum VESC_MODE
{
    SET_NULL,
    SET_eRPM,
    SET_CURRENT,
    SET_DUTY,
    SET_POS,
    SET_BRAKE,
}VESC_MODE;


template <typename T>
void motor_constraint(T *val, T min, T max)
{
    if(*val > max)
    {
        *val = max;
    }
    else if(*val < min)
    {
        *val = min;
    }
}


class Motor_Base
{
public:
    Motor_Base(uint8_t id) : ID(id){}
    virtual ~Motor_Base(){}
    const uint8_t ID = 0;

    virtual void update(uint8_t can_rx_data[])=0;
    virtual bool check_id(uint32_t StdID) const { return StdID == this->receive_id_init() + (uint8_t)ID; }
    float get_angle() const { return angle; }
	float get_encoder() const { return encoder; }

    float encoder_offset = 0;
    float motor_descritoion = 1.0f;
	float Out = 0; /*!< Output ampere value that sent to motor */
    uint16_t encoder = 0; 
protected:
    float angle = 0,  last_encoder = 0;
    bool encoder_is_init = false;

    virtual uint32_t receive_id_init() const { return 0; };
    virtual uint8_t motor_descritoion_init() const {return 1; };
    virtual void update_angle(uint8_t can_rx_data[])
    {
        encoder = (uint16_t)(can_rx_data[0] << 8 | can_rx_data[1]);
        if(encoder_is_init)
        {   
            if(this->encoder - this->last_encoder < -4096)
                this->round_cnt++;
            else if(this->encoder - this->last_encoder > 4096)
                this->round_cnt--;
            else{}
        }
        else 
        {
            encoder_offset = encoder;
            encoder_is_init = true;
        }

        this->last_encoder = this->encoder;
        int32_t total_encoder = round_cnt*8192 + encoder - encoder_offset;
        angle = total_encoder / ENCODER_ANGLE_RATIO();
    }

private:
    int32_t round_cnt = 0;
    virtual int16_t ENCODER_MAX() const { return 8192; }
    virtual float ENCODER_ANGLE_RATIO() const { return 8192.0f / 360.0f; }
    virtual float MAX_CURRENT() const { return 65535; }
};


class Motor_Speed : public Motor_Base
{
public:
    Motor_Speed(uint8_t id) : Motor_Base(id){}
    virtual ~Motor_Speed(){}
    virtual int32_t get_speed() const { return this->speed; }
    virtual void update(uint8_t can_rx_data[]) = 0;

protected:
    int16_t speed = 0;
    virtual void update_speed(uint8_t can_rx_data[])
    {
        speed = (int16_t)(can_rx_data[2] << 8 | can_rx_data[3]);
    }
};


class Motor_C610 : public Motor_Speed
{
public:
    virtual ~Motor_C610(){}
    virtual uint32_t receive_id_init() const { return 0x200; }
    virtual uint32_t send_id_high() const { return 0x1FF; }
    virtual uint32_t send_id_low()  const { return 0x200; }
    virtual float MAX_CURRENT() const { return 10000; }
    Motor_C610(uint8_t id) : Motor_Speed(id){}
    virtual void update(uint8_t can_rx_data[]) override
    {
        update_angle(can_rx_data);
        update_speed(can_rx_data);
        this->tarque = (int16_t)(can_rx_data[4] << 8 | can_rx_data[5]);    
    }
    int16_t get_tarque() const { return this->tarque; }
    uint8_t get_temperature() const { return this->temperature; }
private:
    int16_t tarque = 0;
    uint8_t temperature = 0;
};


class Motor_C620 : public Motor_Speed
{
public:
    virtual uint32_t receive_id_init() const { return 0x200; }
    virtual uint32_t send_id_high() const { return 0x1FF; }
    virtual uint32_t send_id_low()  const { return 0x200; }
    virtual float MAX_CURRENT() const { return 16384; }
    Motor_C620(uint8_t id) : Motor_Speed(id){}
    int16_t get_tarque() const { return this->tarque; }
    uint8_t get_temperature() const { return this->temperature; }

    virtual void update(uint8_t can_rx_data[])
    {
        update_angle(can_rx_data);
        update_speed(can_rx_data);
        this->tarque = (int16_t)(can_rx_data[4] << 8 | can_rx_data[5]);    
    }
private:
    int16_t tarque = 0;
    uint8_t temperature = 0;
};


class Motor_GM6020 : public Motor_Speed
{
public:
    virtual ~Motor_GM6020(){}
    virtual uint32_t recieve_id_init() const { return 0x204; }
    virtual uint32_t send_id_low() const { return 0x1ff; }
    virtual uint32_t send_id_high() const { return 0x2ff; }
    virtual float MAX_CURRENT() const { return 30000; }
    uint16_t set_encoder_offset(uint16_t offset)
    {
        this->encoder_offset = offset;
        this->last_encoder = offset;  
        this->encoder_is_init = true;
        return this->encoder;
    }
    Motor_GM6020(uint8_t id) : Motor_Speed(id){}
    virtual void update(uint8_t can_rx_data[])
    {
        this->update_angle(can_rx_data);
        this->update_speed(can_rx_data);
        this->tarque = (int16_t)(can_rx_data[4] << 8 | can_rx_data[5]);
        this->temperature = can_rx_data[6]; 
    }

    int16_t get_tarque() const { return this->tarque; }
    uint8_t get_temperature() const { return this->temperature; }
private:
    int16_t tarque = 0;
    uint8_t temperature = 0;
};


class VESC : public Motor_Speed, public Tools
{
public:
    VESC(uint8_t id) : Motor_Speed(id){} //括号中为VESC的CAN ID
    virtual ~VESC(){}
    VESC_MODE Mode;
    void update_vesc(CAN_RxBuffer* Buffer)
    { 
        ID_check = Buffer->header.ExtId & 0xff;
        if( ID_check == this->ID)
        {
            cmd = (Buffer->header.ExtId >> 8);   //获取对应的帧头
            update(Buffer->data);
        }
    }
    virtual int32_t get_speed() const { return this->speed; }
    int16_t get_tarque() const { return this->tarque; }

protected:
    virtual void update(uint8_t can_rx_data[])
    {
        switch(cmd)
        {
            case CAN_PACKET_STATUS:
                update_speed(can_rx_data);
                break;
            case CAN_PACKET_STATUS_4:
                update_angle(can_rx_data);
                break;
            default:
                break;
        }
    }
 
    virtual void update_speed(uint8_t can_rx_data[])
    {
        index=0;
        this->speed = _tool_buffer_get_int32(can_rx_data, &index);
        this->tarque = (float)_tool_buffer_get_int16(can_rx_data, &index)*100.0f;   //mA
        this->duty = (float)_tool_buffer_get_int16(can_rx_data, &index)/1000.0f;
    }

    virtual void update_angle(uint8_t can_rx_data[])
    {
        angle = (float)(can_rx_data[6] << 8 | can_rx_data[7])/50.0f;
    }

private:
    uint8_t ID_check;
    uint16_t cmd;
    int index=0;
    float tarque=0;
    float duty=0;
};


template <class Motor_Type, int N>
void RM_Motor_SendMsgs(CAN_HandleTypeDef *hcan, Motor_Type (&motor)[N])
{
    int16_t current_out=0;
    CAN_TxMsg CAN_TxMsg;

    bool low=false, high=false;
    for(int i=0; i<N; i++)
    {
        motor_constraint(&motor[i].Out, -motor[i].MAX_CURRENT(), motor[i].MAX_CURRENT());
        current_out = (int16_t)motor[i].Out;
        if(motor[i].ID<=4&&motor[i].ID>0)
        {
            low = true;
            CAN_TxMsg.data[motor[i].ID*2 - 2] = (uint8_t)(current_out >> 8) & 0xff;
            CAN_TxMsg.data[motor[i].ID*2 - 1] = (uint8_t)current_out & 0xff;
        }
        else if(motor[i].ID<=8&&motor[i].ID>4)
        {
            high = true;
            CAN_TxMsg.data[motor[i].ID*2 - 10] = (uint8_t)(current_out >> 8) & 0xff;
            CAN_TxMsg.data[motor[i].ID*2 - 9] = (uint8_t)current_out & 0xff;
        }
        else{}
    }

    if(low)
    {
        CAN_TxMsg.id = motor[0].send_id_low();
        CAN_TxMsg.len = 8;
        
        if(hcan == &hcan1)
            xQueueSend(CAN1_TxPort, &CAN_TxMsg, portMAX_DELAY);
        else if(hcan == &hcan2)
            xQueueSend(CAN2_TxPort, &CAN_TxMsg, portMAX_DELAY);
    }

    if(high)
    {
        CAN_TxMsg.id = motor[0].send_id_high();
        CAN_TxMsg.len = 8;
        if(hcan == &hcan1)
            xQueueSend(CAN1_TxPort, &CAN_TxMsg, portMAX_DELAY);
        else if(hcan == &hcan2)
            xQueueSend(CAN2_TxPort, &CAN_TxMsg, portMAX_DELAY);
    }

}


template <class Motor_Type>
void RM_Motor_SendMsgs(CAN_HandleTypeDef *hcan, Motor_Type &motor)
{
    Motor_Type motor_arr[1] = {motor};
    RM_Motor_SendMsgs(hcan, motor_arr);
}


template <class VESC_Type, int N>
void VESC_SendMsgs(CAN_HandleTypeDef *hcan, VESC_Type (&motor)[N])
{
    float current_out=0;
    CAN_TxMsg CAN_TxMsg;
    CAN_TxMsg.len = 8;
    for(int i=0; i<N; i++)
    {
        switch (motor[i].Mode)
        {
            case SET_eRPM:    //erpm = rpm * pole_pairs
            {
                int index=0;
                current_out = motor[i].Out;
                CAN_TxMsg.id = motor[i].ID | ((uint32_t)CAN_PACKET_SET_RPM << 8);
                motor[i]._tool_buffer_append_int32(CAN_TxMsg.data, (int32_t)current_out, &index);
                break;
            }

            case SET_CURRENT:
            {
                int index=0;
                current_out = motor[i].Out;
                CAN_TxMsg.id = motor[i].ID | ((uint32_t)CAN_PACKET_SET_CURRENT << 8);   //mA
                motor[i]._tool_buffer_append_int32(CAN_TxMsg.data, (int32_t)current_out, &index);
                break;
            }

            case SET_DUTY:      //0.01%
            {
                int index=0;
                current_out = motor[i].Out;
                CAN_TxMsg.id = motor[i].ID | ((uint32_t)CAN_PACKET_SET_DUTY << 8);
                motor[i]._tool_buffer_append_int32(CAN_TxMsg.data, (int32_t)(current_out * 100000), &index);
                break;
            }

            case SET_POS:   
            {
                int index=0;
                current_out = motor[i].Out;
                CAN_TxMsg.id = motor[i].ID | ((uint32_t)CAN_PACKET_SET_POS << 8);
                motor[i]._tool_buffer_append_int32(CAN_TxMsg.data, (int32_t)current_out*1000000, &index);
                break;
            }

            case SET_BRAKE:
            {
                int index=0;
                current_out = motor[i].Out;
                CAN_TxMsg.id = motor[i].ID | ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE << 8);
                motor[i]._tool_buffer_append_int32(CAN_TxMsg.data, (int32_t)current_out*1000, &index);
                break;
            }

            default:
                break;
        }
    }

    if(hcan == &hcan1)
        xQueueSend(CAN1_TxPort, &CAN_TxMsg, portMAX_DELAY);
    else if(hcan == &hcan2)
	    xQueueSend(CAN2_TxPort, &CAN_TxMsg, portMAX_DELAY);
}

template <class VESC_Type>
void VESC_SendMsgs(CAN_HandleTypeDef *hcan, VESC_Type &motor)
{
    VESC_Type motor_arr[1] = {motor};
    VESC_SendMsgs(hcan, motor_arr);
}

#endif 
