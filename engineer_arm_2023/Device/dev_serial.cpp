#include "dev_serial.h"

Serialctrl Serial7( UART7,  Serial7_Buffer_Size );
Serialctrl Serial8( UART8,  Serial8_Buffer_Size );
Serialctrl Serial3( USART3, Serial3_Buffer_Size );
Serialctrl Serial6( USART6, Serial6_Buffer_Size );

uint8_t seirl7_mpu_data=0;
uint8_t seirl7_mpu_data1[Serial7_Buffer_Size]={0};
uint8_t cnt7=0;

uint8_t cnt6=0;
go2data_u go2data;

void Serialctrl::attachInterrupt( USART_CallbackFunction_t Function )
{
    USART_Function = Function;
}

void Serialctrl::IRQHandler( void )
{
    if(USART_GetITStatus(USARTx, USART_IT_RXNE) != RESET)
    {       
        uint8_t data = USART_ReceiveData( USARTx );
			  if(USARTx==UART7) 
				{
					seirl7_mpu_data=data;
					if(cnt7<Serial7_Buffer_Size)
					seirl7_mpu_data1[cnt7++]=data;
				}
				if(USARTx==USART6) 
				{
					go2data.go2pose_u[cnt6++] = data;
					if(cnt6 == 4)
					{
						cnt6 = 0;
					}
				}
				
   			Buffer_Write( &_rx_buffer, data );

        if(USART_Function)
        {
            USART_Function();
        }

       USART_ClearITPendingBit(USARTx, USART_IT_RXNE);
    }
		else if(USART_GetITStatus(USARTx, USART_IT_IDLE) != RESET)
		{
			   if(USART_Function)
        {
            USART_Function();
        }
		}
}

void Serialctrl::sendData( uint8_t ch )
{
	USART_SendData(this->USARTx,ch);
	while (USART_GetFlagStatus(this->USARTx, USART_FLAG_TXE) == RESET);	
}

void Serialctrl::sendData( const void *str )
{
	unsigned int k=0;
  do 
  {
      sendData(*((uint8_t *)str + k) );
      k++;
  } while(*((uint8_t *)str + k)!='\0');
  while(USART_GetFlagStatus(this->USARTx, USART_FLAG_TC)==RESET){}
}

void Serialctrl::sendData( const void *buf, uint8_t len )
{
	uint8_t *ch = (uint8_t *)buf;
	while(len--){
		sendData( *ch++ );
	}
}

int Serialctrl::available( void )
{
    return ((unsigned int)( _rx_buffer.buf_size + _rx_buffer.pw - _rx_buffer.pr )) % _rx_buffer.buf_size;
}

uint8_t Serialctrl::read( void )
{
		uint8_t c = 0;
	  Buffer_Read( &_rx_buffer, &c );
		return c;
}

int Serialctrl::peek( void )
{
    if ( _rx_buffer.pr == _rx_buffer.pw )
    { 
        return -1;
    }
    else
    {
        return _rx_buffer.fifo[_rx_buffer.pr];
    }
}

void Serialctrl::flush(void)
{
    _rx_buffer.pr= _rx_buffer.pw;
}

extern "C"{
    void UART7_IRQHandler(void)
		{
			uint32_t ulReturn;

		/* 进入临界段，临界段可以嵌套 */
			ulReturn = taskENTER_CRITICAL_FROM_ISR();
			
      Serial7.IRQHandler();
						
			taskEXIT_CRITICAL_FROM_ISR( ulReturn );
		}
		void USART6_IRQHandler(void)
		{
			uint32_t ulReturn;

		/* 进入临界段，临界段可以嵌套 */
			ulReturn = taskENTER_CRITICAL_FROM_ISR();
			
      Serial6.IRQHandler();
						
			taskEXIT_CRITICAL_FROM_ISR( ulReturn );
		}
		void UART8_IRQHandler(void)
		{
			uint32_t ulReturn;

		/* 进入临界段，临界段可以嵌套 */
			ulReturn = taskENTER_CRITICAL_FROM_ISR();
			
      Serial8.IRQHandler();
						
			taskEXIT_CRITICAL_FROM_ISR( ulReturn );
		}
}
