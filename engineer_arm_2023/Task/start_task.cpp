#include "start_task.h"

/*��������*/
#define START_TASK_PRIO 1
#define START_STK_SIZE  64
static TaskHandle_t StartTask_Handler;

/*ϵͳ��ʾ����*/
#define RTOSsystem_TASK_PRIO     2
#define RTOSsystem_STK_SIZE      64
TaskHandle_t RTOSsystem_Handler;

///*�˶�ѧ��������*/
//#define Kinematics_TASK_PRIO     5
//#define Kinematics_STK_SIZE      512
//TaskHandle_t Kinematics_Handler;


///*ͨ������*/
//#define Correspondence_TASK_PRIO     10
//#define Correspondenced_STK_SIZE      512
//TaskHandle_t CorrespondenceTask_Handler;

/*�˶�ѧ��������*/
#define Guard_TASK_PRIO     11
#define Guard_STK_SIZE      128
TaskHandle_t Guard_Handler;

/*ͨ������*/
#define Message_TASK_PRIO     6
#define Message_STK_SIZE      512
TaskHandle_t MessageTask_Handler;

/*��������*/
#define Chassis_TASK_PRIO 4//�������ȼ���һ��32λ���ȼ������ȼ�Խ��������ȼ�Խ��
#define Chassis_STK_SIZE 512//�����ջ��С������Ϊ��λ
TaskHandle_t ChassisTask_Handler;//������


void startTast(void) {
	xTaskCreate((TaskFunction_t)Start_Task,           //������
		(const char*)"Start_Task",          //��������
		(uint16_t)START_STK_SIZE,            //�����ջ��С
		(void*)NULL,                        //���ݸ��������Ĳ���
		(UBaseType_t)START_TASK_PRIO,        //�������ȼ�
		(TaskHandle_t*)&StartTask_Handler); //������
}

QueueHandle_t IMU_Queue;

/*ִ�����񴴽�*/
void Start_Task(void* pvParameters) {
	taskENTER_CRITICAL();//�����ٽ׶�
	
	IMU_Queue=xQueueCreate(2,sizeof(float));

	xTaskCreate((TaskFunction_t)RTOSsystem_Task,        //����RTOS��Ϣ��ʾ����
		(const char*)"RTOSsystem_Task",
		(uint16_t)RTOSsystem_STK_SIZE,
		(void*)NULL,
		(UBaseType_t)RTOSsystem_TASK_PRIO,
		(TaskHandle_t*)&RTOSsystem_Handler);

//	xTaskCreate((TaskFunction_t)guard_task,      			  //��������
//		(const char*)"guard_task",
//		(uint16_t)Guard_STK_SIZE,
//		(void*)NULL,
//		(UBaseType_t)Guard_TASK_PRIO,
//		(TaskHandle_t*)&Guard_Handler);
		
//	xTaskCreate((TaskFunction_t)Kinematics_task
//		,        //�˶�ѧ��������
//		(const char*)"Kinematics_task",
//		(uint16_t)Kinematics_STK_SIZE,
//		(void*)NULL,
//		(UBaseType_t)Kinematics_TASK_PRIO,
//		(TaskHandle_t*)&Kinematics_Handler);
		
	xTaskCreate((TaskFunction_t)Chassis_Task,              //������������
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

//  xTaskCreate((TaskFunction_t)Correspondence_Task,//����ͨ������
//    (const char *)"Correspondence_Task",
//    (uint16_t)Correspondenced_STK_SIZE,
//    (void *)NULL,
//    (UBaseType_t)Correspondence_TASK_PRIO,
//    (TaskHandle_t *)&CorrespondenceTask_Handler);	

	vTaskDelete(StartTask_Handler); //ɾ����ʼ����
	taskEXIT_CRITICAL();            //�˳��ٽ���
}
