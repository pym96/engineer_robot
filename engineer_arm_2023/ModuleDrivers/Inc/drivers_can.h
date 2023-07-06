#ifndef __DRIVERS_CAN_H
#define __DRIVERS_CAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platform.h"
	
#ifdef __cplusplus
}
#endif

#define CAN_PreemptionPriority_Default 4
#define CAN_SubPriority_Default 0

class CANdev{
public:
	CANdev( CAN_TypeDef *CANx );

	void CANx_Init( uint8_t PreemptionPriority, uint8_t SubPriority );
  void CANx_Init( void );
protected:
	typedef void(*CAN_CallbackFunction_t)( CanRxMsg* RxMessage );

	CAN_TypeDef *CANx;
  CAN_CallbackFunction_t CAN_Function;
};

#endif /* __DRIVERS_CAN_H */
