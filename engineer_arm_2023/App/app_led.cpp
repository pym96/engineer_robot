#include "app_led.h"

LEDctrl LED;

void LEDctrl::Turn_On( void )
{
		for( uint8_t i = 0; i < Pins_Num; i++ )
		{
			GPIO_LOW( PIN_MAP[LED_Pins[i]].GPIOx, PIN_MAP[LED_Pins[i]].GPIO_Pin_x );
		}
}

void LEDctrl::Turn_On( Pin_TypeDef Pin )
{
	  GPIO_LOW( PIN_MAP[Pin].GPIOx, PIN_MAP[Pin].GPIO_Pin_x );
}

void LEDctrl::Turn_Off( void )
{
		for( uint8_t i = 0; i < Pins_Num; i++ )
		{
			GPIO_HIGH( PIN_MAP[LED_Pins[i]].GPIOx, PIN_MAP[LED_Pins[i]].GPIO_Pin_x );
		}
}

void LEDctrl::Turn_Off( Pin_TypeDef Pin )
{
	  GPIO_HIGH(PIN_MAP[Pin].GPIOx, PIN_MAP[Pin].GPIO_Pin_x );
}

void LEDctrl::Toggle( void )
{
		for( uint8_t i = 0; i < Pins_Num; i++ )
		{
			GPIO_TOGGLE( PIN_MAP[LED_Pins[i]].GPIOx, PIN_MAP[LED_Pins[i]].GPIO_Pin_x );
			osDelay( 100 );
		}
}

void LEDctrl::Toggle( Pin_TypeDef Pin )
{
	  GPIO_TOGGLE(PIN_MAP[Pin].GPIOx, PIN_MAP[Pin].GPIO_Pin_x );
	  osDelay( 100 );
}
