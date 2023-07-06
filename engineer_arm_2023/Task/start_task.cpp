#include "start_task.h"

/*创建任务*/
#define START_TASK_PRIO 1
#define START_STK_SIZE  64
static TaskHandle_t StartTask_Handler;

/*系统提示任务*/
#define RTOSsystem_TASK_PRIO     2
#define RTOSsystem_STK_SIZE      64
TaskHandle_t RTOSsystem_Handler;

///*运动学解算任务*/
//#define Kinematics_TASK_PRIO     5
//#define Kinematics_STK_SIZE      512
//TaskHandle_t Kinematics_Handler;


///*通信任务*/
//#define Correspondence_TASK_PRIO     10
//#define Correspondenced_STK_SIZE      512
//TaskHandle_t CorrespondenceTask_Handler;

/*运动学解算任务*/
#define Guard_TASK_PRIO     11
#define Guard_STK_SIZE      128
TaskHandle_t Guard_Handler;

/*通信任务*/
#define Message_TASK_PRIO     6
#define Message_STK_SIZE      512
TaskHandle_t MessageTask_Handler;

/*底盘任务*/
#define Chassis_TASK_PRIO 4//任务优先级，一共32位优先级，优先级越大代表优先级越高
#define Chassis_STK_SIZE 512//任务堆栈大小，以字为单位
TaskHandle_t ChassisTask_Handler;//任务句柄


void startTast(void) {
	xTaskCreate((TaskFunction_t)Start_Task,           //任务函数
		(const char*)"Start_Task",          //任务名称
		(uint16_t)START_STK_SIZE,            //任务堆栈大小
		(void*)NULL,                        //传递给任务函数的参数
		(UBaseType_t)START_TASK_PRIO,        //任务优先级
		(TaskHandle_t*)&StartTask_Handler); //任务句柄
}

QueueHandle_t IMU_Queue;

/*执行任务创建*/
void Start_Task(void* pvParameters) {
	taskENTER_CRITICAL();//进入临阶段
	
	IMU_Queue=xQueueCreate(2,sizeof(float));

	xTaskCreate((TaskFunction_t)RTOSsystem_Task,        //创建RTOS信息提示任务
		(const char*)"RTOSsystem_Task",
		(uint16_t)RTOSsystem_STK_SIZE,
		(void*)NULL,
		(UBaseType_t)RTOSsystem_TASK_PRIO,
		(TaskHandle_t*)&RTOSsystem_Handler);

//	xTaskCreate((TaskFunction_t)guard_task,      			  //禁戒任务
//		(const char*)"guard_task",
//		(uint16_t)Guard_STK_SIZE,
//		(void*)NULL,
//		(UBaseType_t)Guard_TASK_PRIO,
//		(TaskHandle_t*)&Guard_Handler);
		
//	xTaskCreate((TaskFunction_t)Kinematics_task
//		,        //运动学解算任务
//		(const char*)"Kinematics_task",
//		(uint16_t)Kinematics_STK_SIZE,
//		(void*)NULL,
//		(UBaseType_t)Kinematics_TASK_PRIO,
//		(TaskHandle_t*)&Kinematics_Handler);
		
	xTaskCreate((TaskFunction_t)Chassis_Task,              //创建底盘任务，
		(const char*)"ChassisTask",
		(uint16_t)Chassis_STK_SIZE,
		(void*)NULL,
		(UBaseType_t)Chassis_TASK_PRIO,
		(TaskHandle_t*)&ChassisTask_Handler);

	xTaskCreate((TaskFunction_t)Message_Task,
		(const char*)"Message_task",
		(uint16_t)Message_STK_SIZE,
		(void*)NULL,
		(UBaseType_t)Message_TASK_PRIO,
		(TaskHandle_t*)&MessageTask_Handler);		

//  xTaskCreate((TaskFunction_t)Correspondence_Task,//创建通信任务
//    (const char *)"Correspondence_Task",
//    (uint16_t)Correspondenced_STK_SIZE,
//    (void *)NULL,
//    (UBaseType_t)Correspondence_TASK_PRIO,
//    (TaskHandle_t *)&CorrespondenceTask_Handler);	

	vTaskDelete(StartTask_Handler); //删除开始任务
	taskEXIT_CRITICAL();            //退出临界区
}
