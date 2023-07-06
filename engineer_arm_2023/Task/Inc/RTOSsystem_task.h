#ifndef __RTOSsystem_TASK_H
#define __RTOSsystem_TASK_H

#include "dev_system.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef Static_infobuffer
#define Static_infobuffer  1
#endif
	
void RTOSsystem_Task(void *pvParameters);

#ifdef __cplusplus
}
#endif	

#endif /* __RTOSsystem_TASK_H */
