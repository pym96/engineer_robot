#ifndef _GUARD_TASK_H_
#define _GUARD_TASK_H_

#include "dev_system.h"
#include "start_task.h"

#ifdef __cplusplus
extern "C" {
#endif

void guard_task(void *pvParameters);

#ifdef __cplusplus
}
#endif	
struct guard_info_t
{
	TaskHandle_t *handle;
	void(*DealFunction_t)(void);
	
	guard_info_t *next_info;
};

struct guard_list_t
{
	int list_size;
	guard_info_t *first_info;
	guard_info_t *info_list;
};

class guard_base
{
	public:
		guard_base();
		//��ӽ�������
		bool add_guard_task(TaskHandle_t &handle,void(*DealFunction_t)(void));
		void guarding();

	private:
		//������ӽ�������ķ���
		guard_list_t guardlist;
	
};



#endif 







