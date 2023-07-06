#include "guard_task.h"
#include "app_motor.h"
guard_base guarder;

void guard_task(void *pvParameters)
{
	while(1)
	{
		guarder.guarding();
		vTaskDelay(10);
	}
}

guard_base::guard_base()
{
	guardlist.list_size=0;
	guardlist.first_info=NULL;
}

bool guard_base::add_guard_task(TaskHandle_t &handle,void(*DealFunction_t)(void))
{
	//创建新的info
	guard_info_t  *guard_info=new guard_info_t;
	
	guard_info->handle=&handle;
	guard_info->DealFunction_t=DealFunction_t;
	guard_info->next_info=NULL;
	//添加到列表
	if(guardlist.list_size==0)
	{
		guardlist.first_info=guardlist.info_list=guard_info;
	}else
	{
		guardlist.info_list->next_info=guard_info;
	}
	
	guardlist.list_size++;
}

void guard_base::guarding()
{
	guard_info_t *ptr=guardlist.first_info;
	for(int i=0;i<guardlist.list_size;i++)
	{
		ptr->DealFunction_t();
		ptr=ptr->next_info;
	}
}













