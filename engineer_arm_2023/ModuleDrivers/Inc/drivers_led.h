#ifndef __DRIVERS_LED_H
#define __DRIVERS_LED_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platform.h"
	
#ifdef __cplusplus
}
#endif

class LEDdev{
public:
	LEDdev();

	void LED_Init( void );
protected:
  uint8_t Pins_Num;
};

const static Pin_TypeDef LED_Pins[] = {PG1, PG2, PG3, PG4, PG5, PG6, PG7, PG8};

#endif /* __DRIVERS_LED_H */
