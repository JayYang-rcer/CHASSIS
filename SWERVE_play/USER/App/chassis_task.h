#pragma once

#include "data_pool.h"
#include "chassis.h"


#ifdef __cplusplus
void Chassis_Pid_Init(void);
extern "C" {
#endif
void Chassis_Task(void *pvParameters);
extern Swerve_Chassis chassis;

#ifdef __cplusplus
}
#endif
