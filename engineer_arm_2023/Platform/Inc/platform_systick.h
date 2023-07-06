#ifndef __PLATFORM_SYSTICK_H
#define __PLATFORM_SYSTICK_H

#ifdef __cplusplus
extern "C" {
#endif
	
#include "platform.h"

#define F_CPU                  SystemCoreClock
#define CYCLES_PER_MICROSECOND (F_CPU / 1000000U)
#define CYCLES_PER_MILLISECOND (F_CPU / 1000U)

void Systick_Init( uint32_t sysTickPerTime );
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);
uint32_t Systick_Check_Underflow(void);
uint32_t micros(void);
uint32_t millis(void);
	
#ifdef useFreeRTOS
void delay_init( uint32_t TICK_RATE_HZ );
void delay_xms( uint16_t nms );
void delay_xus( uint16_t nus );
#endif
	
#ifdef __cplusplus
}
#endif

#endif /* __PLATFORM_SYSTICK_H */
