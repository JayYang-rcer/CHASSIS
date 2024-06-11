#pragma once
#include <stddef.h>
#include <limits.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "string.h"



#ifdef __cplusplus
typedef uint32_t (*SystemTick_Fun)(void);
class AirJoy
{
public:
    static uint8_t getMicroTick_regist(uint32_t (*getTick_fun)(void));
    void data_update(uint16_t GPIO_Pin, uint16_t GPIO_EXTI_USED_PIN);
    uint16_t SWA=0,SWB=0,SWC=0,SWD=0;
    uint16_t LEFT_X=0,LEFT_Y=0,RIGHT_X=0,RIGHT_Y=0;
private:
    static SystemTick_Fun get_systemTick;
    uint32_t last_ppm_time, now_ppm_time=0;
    uint8_t ppm_ready=0,ppm_sample_cnt=0;
    uint8_t ppm_update_flag=0;
	uint16_t ppm_time_delta=0;   //得到上升沿与下降沿的时间
    uint16_t PPM_buf[10]={0};   
};

extern "C" {
#endif

#ifdef __cplusplus
}
extern AirJoy air_joy;
#endif
