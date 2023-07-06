#ifndef __DRIVERS_SERIAL_H
#define __DRIVERS_SERIAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platform.h"
#include "drivers_buffer.h"
	
#ifdef __cplusplus
}
#endif



#define SERIAL_Config_Default SERIAL_8N1
#define USART_ITPending_Default USART_IT_RXNE
#define USART_PreemptionPriority_Default 10
#define USART_SubPriority_Default 0

#define USART_GetWordLength(SERIAL_x)    ((uint16_t)(SERIAL_x&0xF000))
#define USART_GetParity(SERIAL_x)        ((uint16_t)(SERIAL_x&0x0F00))
#define USART_GetStopBits(SERIAL_x)      ((uint16_t)((SERIAL_x&0x00F0)<<8))

typedef enum {
    SERIAL_8N1 = USART_WordLength_8b | USART_Parity_No | (USART_StopBits_1 >> 8),
    SERIAL_8N2 = USART_WordLength_8b | USART_Parity_No | (USART_StopBits_2 >> 8),
    SERIAL_8E1 = USART_WordLength_8b | USART_Parity_Even | (USART_StopBits_1 >> 8),
    SERIAL_8E2 = USART_WordLength_8b | USART_Parity_Even | (USART_StopBits_2 >> 8),
    SERIAL_8O1 = USART_WordLength_8b | USART_Parity_Odd | (USART_StopBits_1 >> 8),
    SERIAL_8O2 = USART_WordLength_8b | USART_Parity_Odd | (USART_StopBits_2 >> 8),
    SERIAL_8N0_5 = USART_WordLength_8b | USART_Parity_No | (USART_StopBits_0_5 >> 8),
    SERIAL_8N1_5 = USART_WordLength_8b | USART_Parity_No | (USART_StopBits_1_5 >> 8),
    SERIAL_8E0_5 = USART_WordLength_8b | USART_Parity_Even | (USART_StopBits_0_5 >> 8),
    SERIAL_8E1_5 = USART_WordLength_8b | USART_Parity_Even | (USART_StopBits_1_5 >> 8),
    SERIAL_8O0_5 = USART_WordLength_8b | USART_Parity_Odd | (USART_StopBits_0_5 >> 8),
    SERIAL_8O1_5 = USART_WordLength_8b | USART_Parity_Odd | (USART_StopBits_1_5 >> 8),

    SERIAL_9N1 = USART_WordLength_9b | USART_Parity_No | (USART_StopBits_1 >> 8),
    SERIAL_9N2 = USART_WordLength_9b | USART_Parity_No | (USART_StopBits_2 >> 8),
    SERIAL_9E1 = USART_WordLength_9b | USART_Parity_Even | (USART_StopBits_1 >> 8),
    SERIAL_9E2 = USART_WordLength_9b | USART_Parity_Even | (USART_StopBits_2 >> 8),
    SERIAL_9O1 = USART_WordLength_9b | USART_Parity_Odd | (USART_StopBits_1 >> 8),
    SERIAL_9O2 = USART_WordLength_9b | USART_Parity_Odd | (USART_StopBits_2 >> 8),
    SERIAL_9N0_5 = USART_WordLength_9b | USART_Parity_No | (USART_StopBits_0_5 >> 8),
    SERIAL_9N1_5 = USART_WordLength_9b | USART_Parity_No | (USART_StopBits_1_5 >> 8),
    SERIAL_9E0_5 = USART_WordLength_9b | USART_Parity_Even | (USART_StopBits_0_5 >> 8),
    SERIAL_9E1_5 = USART_WordLength_9b | USART_Parity_Even | (USART_StopBits_1_5 >> 8),
    SERIAL_9O0_5 = USART_WordLength_9b | USART_Parity_Odd | (USART_StopBits_0_5 >> 8),
    SERIAL_9O1_5 = USART_WordLength_9b | USART_Parity_Odd | (USART_StopBits_1_5 >> 8),
} SERIAL_Config;
/*
   串口的初始化
   以及保护成员 串口号 开启的中断位 接受的buf 

*/
class Serialdev : public Buffer
{
public:
		Serialdev( USART_TypeDef *_USARTx, uint32_t BufferSize );
    Serialdev( USART_TypeDef *_USARTx, uint32_t BufferSize, uint16_t USART_ITPending );

		void Serial_Init( uint32_t BaudRate, SERIAL_Config Config, uint8_t PreemptionPriority, uint8_t SubPriority );
    void Serial_Init( uint32_t BaudRate, uint8_t PreemptionPriority, uint8_t SubPriority );
    void Serial_Init( uint32_t BaudRate, SERIAL_Config Config );
    void Serial_Init( uint32_t BaudRate );
    RingBuffer _rx_buffer;

protected:
	  typedef void(*USART_CallbackFunction_t)(void);
   
		USART_TypeDef *USARTx;                      //串口
		USART_CallbackFunction_t USART_Function;    //
    uint16_t USART_ITPending;
		
};

#endif /* __DRIVERS_SERIAL_H */
