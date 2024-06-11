#ifndef DRIVE_TIM_H
#define DRIVE_TIM_H

#ifdef __cplusplus
extern "C" {
#endif  

#include "stm32f4xx_hal.h"
#include "tim.h"

/* Private macros ------------------------------------------------------------*/
#define microsecond()    Get_SystemTimer()

/* Private type --------------------------------------------------------------*/
typedef struct{
  uint32_t last_time;	/*!< Last recorded real time from systick*/
  float dt;				/*!< Differentiation of real time*/
}TimeStamp;

/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef enum 
{
	USE_MODULE_DELAY = 1,	/*!< Use module function to implement delay_ms_nos()*/
	USE_HAL_DELAY			/*!< Use HAL_Delay() for delay_ms_nos()*/
}EDelay_src;

/* Exported variables ---------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/
void Timer_Init(TIM_HandleTypeDef* htim, EDelay_src src);
void Update_SystemTick(void);
void PWM_ReInit(uint16_t period, uint16_t prescaler, TIM_HandleTypeDef* htim, uint32_t channel);
void Set_PwmDuty(TIM_HandleTypeDef* htim, uint32_t channel, uint16_t duty);
void Set_PwmFreq(TIM_HandleTypeDef* htim, uint32_t freq);
uint32_t Get_SystemTimer(void);
void delay_ms_nos(uint32_t cnt);
void delay_us_nos(uint32_t cnt);

#ifdef __cplusplus
}
#endif 

#endif //  DRIVE_TIM_H
