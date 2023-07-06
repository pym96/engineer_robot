#ifndef __DEVICE_CAN_H
#define __DEVICE_CAN_H

#ifdef __cplusplus
extern "C" {
#endif
	
#ifdef __cplusplus
}
#endif

#include "drivers_can.h"

class CANctrl : public CANdev
{
public:
	CANctrl( CAN_TypeDef *CANx, uint32_t StdId ) : CANdev( CANx ), StdId( StdId ){}
	
	void attachInterrupt( CAN_CallbackFunction_t Function );	
		
	void SendData( int16_t data1, int16_t data2, int16_t data3, int16_t data4 );
	void SendData( int16_t data1, int16_t data2, int16_t data3 );
	void SendData( int16_t data1, int16_t data2 );
	void SendData( int16_t data1 );

	void IRQHandler( void );
private:
	uint32_t StdId;
};

#endif /* __DEVICE_CAN_H */
