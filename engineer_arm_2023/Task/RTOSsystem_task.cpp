#include "RTOSsystem_Task.h"
#include "app_led.h"

#if(Static_infobuffer == 1)
char InfoBuffer[1000];
#endif

void RTOSsystem_Task(void* pvParameters)
{
	LED.LED_Init();
	while (1)
	{
#if (Static_infobuffer == 1)
		vTaskList(InfoBuffer);	//��ӡ������
//		 printf("��ǰ�������������ͳ�ƣ�%s\r\n",InfoBuffer);
#endif
		LED.Toggle();
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}
