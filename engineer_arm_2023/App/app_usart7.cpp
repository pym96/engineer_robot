#include "app_usart7.h"

/*串口7接受陀螺仪数据*/
uint8_t ucRxData7[16];	
static uint8_t ucRxCnt=0;
struct SGyro  stcgimbalGyro={0};
struct SAngle stcgimbalAngle={0};
void CopeSerial7Data(void)
{
	ucRxData7[ucRxCnt++]=seirl7_mpu_data;
	
	if (ucRxData7[0]!=0x55) //数据头不对，则重新开始寻找0x55数据头
	{
		ucRxCnt=0;
		
	  return;
	}
	if (ucRxCnt<11)
	{
	  return;
	}//数据不满11个，则返回
	else
	{
		switch(ucRxData7[1])
		{
			//角速度
			case 0x52:	memcpy(&stcgimbalGyro,&ucRxData7[2],8);break;
			//角度
			//
			case 0x53:	memcpy(&stcgimbalAngle,&ucRxData7[2],8);break;
			
			default:break;
		}
		ucRxCnt=0;
	}
}
struct SGyro *get_imu_gyro_Point(void){
	return &stcgimbalGyro;
}
struct SAngle *get_imu_angle_Point(void){
	return &stcgimbalAngle;
}
u8 set_gimbal_imu_zero[5]={0xff,0xaa,0x76,0x00,0x00};
void imu_gimbal_set_zero(void){
	
	Serial7.sendData(set_gimbal_imu_zero,sizeof(set_gimbal_imu_zero));
}


/*串口6接受上位机串口数据*************************************************************************************************************/
#define FRAME_HEADER 0xFF
#define FRAME_TAIL   0xFE

#define DATA_NUM 20
uint8_t USART6_Rx_Buf[DATA_NUM] = {0}; //USART6存储接收正确数据
uint8_t USART6_data_length = 0;//DMA剩余字节数据
bool USART6_Rx_flg = true;  //串口6接受判断标志

struct GOMotor_t GOMotor;
//DMA使能
void DMA_USART6(void)
{
	//串口6DMA初始化
	usart6_RxDMA.dmaInit(PTM_CR_SGSG_DS, (uint32_t*)&(USART6->DR), (uint32_t*)USART6_Rx_Buf, DATA_NUM, \
		DMA_PeripheralInc_Disable, DMA_MemoryInc_Enable, DMA_PeripheralDataSize_Byte, DMA_MemoryDataSize_Byte, \
		DMA_Priority_VeryHigh, DMA_FIFOThreshold_1QuarterFull);
	//使能
	USART_DMACmd(USART6, USART_DMAReq_Rx, ENABLE);// OPEN DMA
}
void USART6CallBackFun()
{
	
}

struct GOMotor_t *get_GOMotor_Point(void){
	return &GOMotor;
}

/*数据发送函数**********************************************************************************************************************************/
void send_data(uint8_t *data, uint16_t len,USART_TypeDef *_USARTx)
{
  uint16_t i;
  uint8_t checksum = 0;
  // Send frame header
  while((_USARTx->SR & USART_SR_TC) == 0); // Wait for transmission complete
  USART_SendData(_USARTx, FRAME_HEADER);
	
		checksum = FRAME_HEADER;

  // Send data
  for (i = 0; i < len; i++) {
    checksum += data[i];
    while((_USARTx->SR & USART_SR_TC) == 0); // Wait for transmission complete
    USART_SendData(_USARTx, data[i]);
  }
	if(checksum ==0xFF){
	 char i =0;
	return ;
	}
//  // Send checksum
  while((_USARTx->SR & USART_SR_TC) == 0); // Wait for transmission complete
  USART_SendData(_USARTx, checksum);

  // Send frame tail
  while((_USARTx->SR & USART_SR_TC) == 0); // Wait for transmission complete
  USART_SendData(_USARTx, FRAME_TAIL);
}

void send_float(float data,USART_TypeDef *_USARTx)
{
  send_data((uint8_t *)&data, sizeof(float),_USARTx);
}

void send_double(double data,USART_TypeDef *_USARTx)
{
  send_data((uint8_t *)&data, sizeof(double),_USARTx);
}

void send_int(int data,USART_TypeDef *_USARTx)
{
  send_data((uint8_t *)&data, sizeof(int),_USARTx);
}

void send_u8(int data,USART_TypeDef *_USARTx)
{
  send_data((uint8_t *)&data, sizeof(u8),_USARTx);
}




