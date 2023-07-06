#ifndef __PLATFORM_H
#define __PLATFORM_H

#ifdef __cplusplus
extern "C" {
#endif
	
#define useFreeRTOS
	
#ifndef useFreeRTOS
#define osDelay( ms )   delay_ms( ms )
#define Delay_ms( ms )  delay_ms( ms )
#define Delay_us( us )  delay_us( us )
#else
#define osDelay( ms )   vTaskDelay( pdMS_TO_TICKS( ms ) )
#define Delay_ms( ms )  delay_xms( ms )
#define Delay_us( us )  delay_xus( us )
#endif
	
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "stm32f4xx.h"
#include "platform_io.h"
#include "platform_pwm.h"
#include "platform_exti.h"
#include "platform_timer.h"
#include "platform_systick.h"

	
#ifdef __cplusplus
}
#endif

#endif /* __PLATFORM_H */
