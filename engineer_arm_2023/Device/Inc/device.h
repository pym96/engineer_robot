#ifndef __DEVICE_H
#define __DEVICE_H

#ifdef __cplusplus
extern "C" {
#endif
	
#include "platform.h"
	
#ifdef __cplusplus
}
#endif

#include "dev_serial.h"
#include "dev_can.h"

#include "drivers_buffer.h"
#include "drivers_serial.h"
#include "drivers_can.h"
#include "drivers_remote.h"
#include "drivers_led.h"
#include "drivers_dma.h"

/* extren CAN class */
extern CANctrl CAN1_Ctrl;
extern CANctrl CAN2_Ctrl;

/* extren Serial class */
extern Serialctrl Serial3;
extern Serialctrl Serial6;
extern Serialctrl Serial7;
extern Serialctrl Serial8;
extern Serialctrl Serial1;

/* extren DMA class */
extern DMAdev usart3_TxDMA;
extern DMAdev usart3_RxDMA;
extern DMAdev usart6_TxDMA;
extern DMAdev usart6_RxDMA;
extern DMAdev usart7_TxDMA;
extern DMAdev usart7_RxDMA;
extern DMAdev usart8_TxDMA;
extern DMAdev usart8_RxDMA;
extern DMAdev usart1_TxDMA;
extern DMAdev usart1_RxDMA;

extern TaskHandle_t RTOSsystem_Handler;
extern TaskHandle_t GIMBALTask_Handler;
extern TaskHandle_t MessageTask_Handler;
extern TaskHandle_t VersionTask_Handler;

/* extern CTRL class */
#endif /* __DEVICE_H */

