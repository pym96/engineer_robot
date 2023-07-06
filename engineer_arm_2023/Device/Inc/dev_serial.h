#ifndef __APP_SERIAL_H
#define __APP_SERIAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platform.h"

	
#ifdef __cplusplus
}
#endif
#include "drivers_serial.h"


typedef union 
{
	float go2pos_f;
	u8 go2pose_u[4];
}go2data_u;

extern uint8_t seirl7_mpu_data;
extern go2data_u go2data;

#define Serial3_Buffer_Size 16
#define Serial6_Buffer_Size 16
#define Serial7_Buffer_Size 16
#define Serial8_Buffer_Size 16
#define Serial1_Buffer_Size 16

class Serialctrl : public Serialdev
{
public:
	Serialctrl( USART_TypeDef *_USARTx, uint32_t BufferSize ) : Serialdev( _USARTx, BufferSize ){}
	Serialctrl( USART_TypeDef *_USARTx, uint32_t BufferSize, uint16_t USART_ITPending ) : Serialdev( _USARTx, BufferSize, USART_ITPending ){}

	void attachInterrupt( USART_CallbackFunction_t Function );
  void IRQHandler( void );

  void sendData( uint8_t ch );
  void sendData( const void *str );
  void sendData( const void *buf,uint8_t len );
	
	uint8_t read( void );
  int peek( void );
  void flush( void );
		
	int available( void );

};

#endif /* __APP_SERIAL_H */
