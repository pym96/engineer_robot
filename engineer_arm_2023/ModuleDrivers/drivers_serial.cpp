#include "drivers_serial.h"
#include "app_usart7.h"

extern "C" {
	#pragma import(__use_no_semihosting)                
	void _sys_exit(int returncode) 
	{ 
		//printf("exit return code is: %d\r\n", returncode); 
		abort();
		while(1);
	} 

	int fputc(int ch, FILE *f )
	{ 	
		while((USART1->SR&0X40)==0);  
		USART1->DR = (u8) ch;      
		return ch;
	}
}

Serialdev::Serialdev( USART_TypeDef *_USARTx, uint32_t BufferSize )
{
    this->USARTx = _USARTx;
	  this->USART_ITPending = USART_ITPending_Default;
    USART_Function = 0;
	  newBuffer( &_rx_buffer, BufferSize );
}

Serialdev::Serialdev( USART_TypeDef *_USARTx, uint32_t BufferSize, uint16_t USART_ITPending )
{
    this->USARTx = _USARTx;
	
	  if( IS_USART_CONFIG_IT( USART_ITPending ) )
	    this->USART_ITPending = USART_ITPending;
		else
			USART_ITPending = USART_ITPending_Default;
		
    USART_Function = 0;
	  newBuffer( &_rx_buffer, BufferSize );
}

void Serialdev::Serial_Init( uint32_t BaudRate, SERIAL_Config Config, uint8_t PreemptionPriority, uint8_t SubPriority )
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    uint16_t Tx_Pin, Rx_Pin;
    uint16_t ItChannel;
    uint32_t RCC_AHB1Periph_GPIOx;
    GPIO_TypeDef *GPIOx;
    uint8_t GPIO_AF_USARTx;

    if(USARTx == USART3)
    {
        Tx_Pin = GPIO_Pin_8;
        Rx_Pin = GPIO_Pin_9;
        GPIOx = GPIOD;
        GPIO_AF_USARTx = GPIO_AF_USART3;
        RCC_AHB1Periph_GPIOx = RCC_AHB1Periph_GPIOD;
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
#if defined (STM32F40_41xxx) || defined (STM32F427_437xx) || defined (STM32F429_439xx)
        ItChannel = USART3_IRQn;
#endif
    }
    else if(USARTx == USART6)
    {
        Tx_Pin = GPIO_Pin_14;
        Rx_Pin = GPIO_Pin_9;
        GPIOx = GPIOG;
        GPIO_AF_USARTx = GPIO_AF_USART6;
        RCC_AHB1Periph_GPIOx = RCC_AHB1Periph_GPIOG;
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
        ItChannel = USART6_IRQn;
    }
    else if(USARTx == UART7)
    {
        Tx_Pin = GPIO_Pin_8;
        Rx_Pin = GPIO_Pin_7;
        GPIOx = GPIOE;
        GPIO_AF_USARTx = GPIO_AF_UART7;
        RCC_AHB1Periph_GPIOx = RCC_AHB1Periph_GPIOE;
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART7, ENABLE);
			
			  ItChannel = UART7_IRQn;
    }
		else if(USARTx == UART8)
    {
        Tx_Pin = GPIO_Pin_1;
        Rx_Pin = GPIO_Pin_0;
        GPIOx = GPIOE;
        GPIO_AF_USARTx = GPIO_AF_UART8;
        RCC_AHB1Periph_GPIOx = RCC_AHB1Periph_GPIOE;
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART8, ENABLE);
			
			  ItChannel = UART8_IRQn;
    }
    else return;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOx, ENABLE);

    GPIO_PinAFConfig(GPIOx, GPIO_GetPinSource(Tx_Pin), GPIO_AF_USARTx);
    GPIO_PinAFConfig(GPIOx, GPIO_GetPinSource(Rx_Pin), GPIO_AF_USARTx);

    GPIO_InitStructure.GPIO_Pin =  Tx_Pin | Rx_Pin;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOx, &GPIO_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = ItChannel;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PreemptionPriority ;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = SubPriority;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_InitStructure.USART_BaudRate = BaudRate;
    USART_InitStructure.USART_WordLength = USART_GetWordLength(Config);
    USART_InitStructure.USART_StopBits = USART_GetStopBits(Config);
    USART_InitStructure.USART_Parity = USART_GetParity(Config);
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USARTx, &USART_InitStructure);
    USART_ITConfig(USARTx, USART_ITPending, ENABLE);
    USART_Cmd(USARTx, ENABLE);
}

void Serialdev::Serial_Init( uint32_t BaudRate, uint8_t PreemptionPriority, uint8_t SubPriority  )
{
    Serial_Init( BaudRate, SERIAL_Config_Default, PreemptionPriority, SubPriority );
}

void Serialdev::Serial_Init( uint32_t BaudRate, SERIAL_Config Config )
{
    Serial_Init( BaudRate, Config, USART_PreemptionPriority_Default, USART_SubPriority_Default );
}

void Serialdev::Serial_Init( uint32_t BaudRate )
{
    Serial_Init( BaudRate, SERIAL_Config_Default, USART_PreemptionPriority_Default, USART_SubPriority_Default );
}





