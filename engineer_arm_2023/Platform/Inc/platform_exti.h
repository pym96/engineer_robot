#ifndef __PLATFORM_EXTI_H
#define __PLATFORM_EXTI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platform.h"

#define EXTI_GetLine(Pin)            (1 << Pinx)
#define EXTI_GetPortSourceGPIOx(Pin) (GPIO_GetPortNum(Pin))
#define EXTI_GetPinSourcex(Pin)      (GPIO_GetPinNum(Pin))

#define CHANGE EXTI_Trigger_Rising_Falling
#define FALLING EXTI_Trigger_Falling
#define RISING EXTI_Trigger_Rising

#define EXTI_PreemptionPriority_Default 2
#define EXTI_SubPriority_Default 1

typedef void(*EXTI_CallbackFunction_t)(void);

void EXTIx_Init(uint8_t Pin, EXTI_CallbackFunction_t function, EXTITrigger_TypeDef Trigger_Mode, uint8_t PreemptionPriority, uint8_t SubPriority);
void attachInterrupt(uint8_t Pin, EXTI_CallbackFunction_t function, EXTITrigger_TypeDef Trigger_Mode);
void detachInterrupt(uint8_t Pin);

#ifdef __cplusplus
}// extern "C"
#endif

#endif /* __PLATFORM_EXTI_H */
