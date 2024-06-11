#pragma once

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "motor.h"
#include "pid.h"

#ifdef __cplusplus
// extern VESC MD4219;
extern Motor_GM6020 GM6020;

extern "C" {
#endif 

void User_Debug_Task(void *pvParameters);

#ifdef __cplusplus
}
#endif 
