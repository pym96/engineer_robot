#include "drivers_key.h"
#include "drivers_dma.h"

static void EXTI2_CallbackFunction( void );

Keydev Key( PA2 );

Keydev::Keydev( Pin_Type Key_Pin )
{
  this->Key_Pin = Key_Pin;
}

void Keydev::Key_Init( void )
{
  GPIOx_Init( Key_Pin, INPUT_PULLUP, HIGH_SPEED );
	
	EXTIx_Init( Key_Pin, EXTI2_CallbackFunction, EXTI_Trigger_Falling, 1, 0 );

}

static void EXTI2_CallbackFunction( void )
{
  DMA_Cmd(DMA1_Channel4, DISABLE);
	DMA_SetCurrDataCounter( DMA1_Channel4, 128 );
	DMA_Cmd(DMA1_Channel4, ENABLE); //开启DMA的通道
}
