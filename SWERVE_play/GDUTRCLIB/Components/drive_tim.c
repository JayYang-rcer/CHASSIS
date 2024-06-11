/**
 * @file drive_tim.c
 * @author Yang Jianyi (2807643517@qq.com)
 * @brief 1)tim底层驱动文件，用于实现定时器的初始化和延时函数。一般不使用阻塞式延时。
 * 		  2)需要获取当前时间时，可以使用Get_SystemTimer()函数。Get_SystemTimer()在类中的使用可以参考PID类(PID.cpp)或者ROS的通讯类(ROS.cpp)
 * @version 0.1
 * @date 2024-03-30
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "drive_tim.h"

volatile uint32_t SystemTimerCnt;

struct timer_manager_t
{
	TIM_HandleTypeDef*	htim_x;
	EDelay_src	delay_ms_src;
}Timer_Manager;

static void Error_Handler(void);


/* function prototypes -------------------------------------------------------*/
/**
* @brief  Initialize Timer
* @param  htim_x : HAL Handler of timer x.
* @param  src : Choose the src for delay_ms().
* @retval None
*/
void Timer_Init(TIM_HandleTypeDef* htim, EDelay_src src)
{
	/* Check the parameters */
	assert_param(htim != NULL);
	
	Timer_Manager.htim_x = htim;
	Timer_Manager.delay_ms_src = src;
    
  	if(HAL_TIM_Base_Start_IT(Timer_Manager.htim_x)!=HAL_OK)
      	Error_Handler();
//   if(HAL_TIM_Base_Start(Timer_Manager.htim_x)!= HAL_OK)
//       Error_Handler();
}


/**
* @brief  Get the system tick from timer.
* @param  None
* @retval current tick.
*/
uint32_t Get_SystemTimer(void)
{
	return Timer_Manager.htim_x->Instance->CNT + SystemTimerCnt * 0xffff;
}


/**
* @brief  Update system tick that had run in Timer Interupt.
* @not    Add this function into Timer interupt function.
* @param  None
* @retval None
*/
void Update_SystemTick(void)
{
	SystemTimerCnt++;
}


/**
 * @brief 使用PWM输出时，需要重新初始化定时器的脉冲频率和重装载值。与Timer_Init()函数不同，PWM_Init()一起调用
 * 
 * @param period 重装载值
 * @param prescaler 预分频值
 * @param htim 定时器句柄
 * @param channel PWM输出通道
 */
void PWM_ReInit(uint16_t period, uint16_t prescaler, TIM_HandleTypeDef* htim, uint32_t channel)
{
	htim->Init.Period = period;
	htim->Init.Prescaler = prescaler;
	HAL_TIM_Base_Init(htim);
	HAL_TIM_PWM_Start(htim, channel);
}


/**
 * @brief 设置pwm占空比
 * 
 * @param htim 定时器句柄
 * @param channel 定时器通道
 * @param duty 占空比，0~100
 * @return int 
 */
void Set_PwmDuty(TIM_HandleTypeDef* htim, uint32_t channel, uint16_t duty)
{
	uint32_t tim_cnt=0, cmp;

	//获取定时器的重装载值
	tim_cnt = htim->Instance->ARR;
	cmp = (tim_cnt + 1) * duty / 100;
	__HAL_TIM_SET_COMPARE(htim, channel, cmp);
}


/**
 * @brief 设置PWM输出频率
 * @param htim 定时器句柄
 * @param freq 输出频率
 */
void Set_PwmFreq(TIM_HandleTypeDef* htim, uint32_t freq)
{
	htim->Init.Period = (htim->Instance->ARR/freq)-1;
	//重新初始化定时器
	HAL_TIM_Base_Init(htim);
	
	HAL_TIM_PWM_Init(htim);
}


/*如果硬件资源充足，可以在cubemsx中直接给系统分配一个硬件定时器，效果是一样的。当然了，在工程上，肯定是越省资源越好。一般来讲，最好不要用阻塞式延时*/
/**
* @brief  Delay microsecond.
* @param  cnt : microsecond to delay 
* @retval None
*/
void delay_us_nos(uint32_t cnt)
{
	uint32_t temp = cnt  + microsecond();

	while(temp >= microsecond());
}


/**
* @brief  Delay millisecond.
* @param  cnt : millisecond to delay
* @retval None
*/
void delay_ms_nos(uint32_t cnt)
{
	if(Timer_Manager.htim_x != NULL && Timer_Manager.delay_ms_src == USE_MODULE_DELAY)
	{
		uint32_t temp = cnt * 1000 + microsecond();
		while(temp >= microsecond());
	}
	else
		HAL_Delay(cnt);
}


/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
static void Error_Handler(void)
{
  /* Nromally the program would never run here. */
  while(1){}
}
