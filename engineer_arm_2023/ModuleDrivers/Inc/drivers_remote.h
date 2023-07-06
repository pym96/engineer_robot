#ifndef __DRIVERS_REMOTE_H
#define __DRIVERS_REMOTE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platform.h"
	
#define RC_NVIC 8

void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
void RC_unable(void);
void RC_restart(uint16_t dma_buf_num);
	
#ifdef __cplusplus
}
#endif

#endif /* __DRIVERS_REMOTE_H */
