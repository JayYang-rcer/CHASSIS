#ifndef SERVICE_CONFIG_H
#define SERVICE_CONFIG_H

#include "drive_can.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "cmsis_os.h"
#include "drive_tim.h"
#include "service_communication.h"
#include "pid.h"
#include "data_pool.h"
#include "ROS.h"
#include "air_joy.h"
#include "Broadcast.h"


#define PriorityVeryLow       1
#define PriorityLow           2
#define PriorityBelowNormal   3
#define PriorityNormal        4
#define PriorityAboveNormal   5
#define PriorityHigh          6
#define PrioritySuperHigh     7
#define PriorityRealtime      8


#ifdef  __cplusplus
extern "C"{
#endif                                  
void System_Resource_Init(void);   
void App_Init(void);

#ifdef  __cplusplus
}
#endif                                


#endif //  SERVICE_CONFIG_H
