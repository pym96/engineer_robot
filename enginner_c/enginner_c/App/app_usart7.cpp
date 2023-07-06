#include "app_usart7.h"
#include "main.h"
#include "stm32f4xx_hal.h"




/*串口6接受上位机串口数据*************************************************************************************************************/
//#define FRAME_HEADER 0xFF
//#define FRAME_TAIL   0xFE

//#define DATA_NUM 20
//uint8_t USART6_Rx_Buf[DATA_NUM] = {0}; //USART6存储接收正确数据
//uint8_t USART6_data_length = 0;//DMA剩余字节数据
//bool USART6_Rx_flg = true;  //串口6接受判断标志

//DMA使能

void USART6CallBackFun()
{
	
}

//struct GOMotor_t *get_GOMotor_Point(void){
//	return &GOMotor;
//}

/*数据发送函数**********************************************************************************************************************************/
//void send_data(uint8_t *data, uint16_t len,USART_TypeDef *_USARTx)
//{
//  uint16_t i;
//  uint8_t checksum = 0;
//  // Send frame header
//  while((_USARTx->SR & USART_SR_TC) == 0); // Wait for transmission complete
//  USART_SendData(_USARTx, FRAME_HEADER);
//	
//		checksum = FRAME_HEADER;

//  // Send data
//  for (i = 0; i < len; i++) {
//    checksum += data[i];
//    while((_USARTx->SR & USART_SR_TC) == 0); // Wait for transmission complete
//    USART_SendData(_USARTx, data[i]);
//  }
//	if(checksum ==0xFF){
//	 char i =0;
//	return ;
//	}
////  // Send checksum
//  while((_USARTx->SR & USART_SR_TC) == 0); // Wait for transmission complete
//  USART_SendData(_USARTx, checksum);

//  // Send frame tail
//  while((_USARTx->SR & USART_SR_TC) == 0); // Wait for transmission complete
//  USART_SendData(_USARTx, FRAME_TAIL);
//}

const u8  FRAME_HEADER =0xFF;
const u8 FRAME_TAIL =0xFE;

void send_data(uint8_t *data, uint16_t len, UART_HandleTypeDef *huart)
{
  uint16_t i;
  uint8_t checksum = 0;

  // Send frame header
  HAL_UART_Transmit(huart, &FRAME_HEADER, 1, 1000);
  checksum = FRAME_HEADER;

  // Send data
  for (i = 0; i < len; i++) {
    checksum += data[i];
    HAL_UART_Transmit(huart, &data[i], 1, 1000);
  }

  // Send checksum
  checksum = ~checksum + 1;
  HAL_UART_Transmit(huart, &checksum, 1, 1000);

  // Send frame tail
  HAL_UART_Transmit(huart, &FRAME_TAIL, 1, 1000);
}

void send_float(float data,UART_HandleTypeDef *huart)
{
  send_data((uint8_t *)&data, sizeof(float),huart);
}

void send_double(double data,UART_HandleTypeDef *huart)
{
  send_data((uint8_t *)&data, sizeof(double),huart);
}

void send_int(int data,UART_HandleTypeDef *huart)
{
  send_data((uint8_t *)&data, sizeof(int),huart);
}

void send_u8(int data,UART_HandleTypeDef *huart)
{
  send_data((uint8_t *)&data, sizeof(u8),huart);
}




