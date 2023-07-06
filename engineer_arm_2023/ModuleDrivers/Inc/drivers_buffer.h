#ifndef __DRIVERS_BUFFER_H
#define __DRIVERS_BUFFER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platform.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
	
#ifdef __cplusplus
}
#endif

typedef struct{
    uint8_t *fifo;
    volatile uint16_t pw;
    volatile uint16_t pr;
    uint16_t buf_size;
}RingBuffer, *ptRingBuffer;

class Buffer{
protected:
	int newBuffer( ptRingBuffer buffer, uint16_t size );
  int Buffer_Write( ptRingBuffer buffer, const uint8_t data );
  int Buffer_Write( ptRingBuffer buffer, const uint8_t *data_stream, uint8_t len );
  
  int Buffer_Read( ptRingBuffer buffer, uint8_t *data );
  int Buffer_Read( ptRingBuffer buffer, uint8_t *data_stream, uint8_t len );
  int Buffer_Clean( ptRingBuffer buffer );
};

#endif /* __DRIVERS_BUFFER_H */
