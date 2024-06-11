/**
 * @file air_joy.cpp
 * @author Yang JianYi
 * @brief 航模手柄的应用文件，使用PPM解析航模手柄的数据
 * @version 0.1
 * @date 2024-04-09
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "air_joy.h"

AirJoy air_joy;

SystemTick_Fun AirJoy::get_systemTick = NULL;
uint8_t AirJoy::getMicroTick_regist(uint32_t (*getTick_fun)(void))
{
    if(getTick_fun != NULL)
    {
        AirJoy::get_systemTick = getTick_fun;
        return 1;
    }
    else 
        return 0;
}


/**
 * @brief ppw signal data ubpack and update
 * @param GPIO_Pin The parameter of GPIO_EXTI callback function
 * @param GPIO_EXTI_USED_PIN the used GPIO_Pin for EXTI that we defined
 */
void AirJoy::data_update(uint16_t GPIO_Pin, uint16_t GPIO_EXTI_USED_PIN)
{
    if(GPIO_Pin == GPIO_EXTI_USED_PIN)		//使用的外部中断引脚
    {
        //get the system tick
		last_ppm_time=now_ppm_time;
		now_ppm_time=get_systemTick();  //获取当前时间
		ppm_time_delta=now_ppm_time-last_ppm_time;  //电平数据
    }

    //开始解包PPM信号
	if(ppm_ready==1)	//判断帧结束后，进行下一轮解析
	{
        //帧结束电平至少2ms=2000us(留点余量)
        //由于部分老版本遥控器、接收机输出PPM信号不标准，当出现解析异常时，尝试改小此值，该情况仅出现一例：使用天地飞老版本遥控器
		if(ppm_time_delta >= 2100)  //帧头
		{
			ppm_ready = 1;
			ppm_sample_cnt=0;   //对应的通道值
			ppm_update_flag=1;
		} 
		else if(ppm_time_delta>=950&&ppm_time_delta<=2050)//单个PWM脉宽在1000-2000us，这里设定950-2050，提升容错
		{         
			PPM_buf[ppm_sample_cnt++]=ppm_time_delta;//对应通道写入缓冲区 
			if(ppm_sample_cnt>=8)   //单次解析结束0-7表示8个通道。如果想要使用10通道，使用ibus协议(串口接收)
			{
                LEFT_X=PPM_buf[0]; LEFT_Y=PPM_buf[1]; RIGHT_X=PPM_buf[3]; RIGHT_Y=PPM_buf[2];
                SWA=PPM_buf[4]; SWB=PPM_buf[5]; SWC=PPM_buf[6]; SWD=PPM_buf[7];
				ppm_ready=0;
				ppm_sample_cnt=0;
			}
		}
		else  
            ppm_ready=0;
	}
	else if(ppm_time_delta>=2100)//帧尾电平至少2ms=2000us
	{
		ppm_ready=1;
		ppm_sample_cnt=0;
		ppm_update_flag=0;
	}
}


/**
 * @brief GPIO 的串口回调函数
 * 
 * @param GPIO_Pin 触发的IO口
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    air_joy.data_update(GPIO_Pin,GPIO_PIN_7);
}

