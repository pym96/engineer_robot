#include "drivers_led.h"

LEDdev::LEDdev()
{
	  this->Pins_Num = sizeof( LED_Pins ) / sizeof( LED_Pins[0] );
}

void LEDdev::LED_Init( void )
{
		for( uint8_t i = 0; i < Pins_Num; i++ )
		{
			if( IS_PIN( LED_Pins[i] ) == 0 ){
				return;
			}else{
				GPIOx_Init( LED_Pins[i], OUTPUT, GPIO_Speed_100MHz );
			}
		}
}
