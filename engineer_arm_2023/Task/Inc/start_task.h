#ifndef __START_TASK_H
#define __START_TASK_H

#include "dev_system.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
	
#include "RTOSsystem_Task.h"
#include "Kinematics_task.h"
#include "guard_task.h"
#include "Chassis_Task.h"
#include "Message_Task.h"
#include "Correspondence_Task.h"	
	
void Start_Task(void *pvParameters);
void startTast(void);
void guard_task(void *pvParameters);

extern TaskHandle_t Kinematics_Handler;	
extern TaskHandle_t Guard_Handler;
	
extern QueueHandle_t IMU_Queue;
	
#ifdef __cplusplus
}
#endif

#endif /* __START_TASK_H */
