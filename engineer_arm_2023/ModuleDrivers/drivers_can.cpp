#include "drivers_can.h"

CANdev::CANdev( CAN_TypeDef *CANx )
{
    this->CANx = CANx;
	  this->CAN_Function = 0;
}

void CANdev::CANx_Init( uint8_t PreemptionPriority, uint8_t SubPriority )
{
    CAN_InitTypeDef CAN_InitStructure;
    CAN_FilterInitTypeDef CAN_FilterInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
	
	  uint32_t RCC_APB1Periph;
	  uint8_t CAN_Tx_Pin, CAN_Rx_Pin;
	  uint8_t GPIO_AF_CANx;
	  uint8_t CANx_RX0_IRQn;
	  uint8_t CANx_FilterNumber;
	
	  if( CANx == CAN1 )
		{
			  CAN_Tx_Pin = PD1;
			  CAN_Rx_Pin = PD0;
			  GPIO_AF_CANx = GPIO_AF_CAN1;
			  CANx_RX0_IRQn = CAN1_RX0_IRQn;
		    RCC_APB1Periph = RCC_APB1Periph_CAN1;
			  CANx_FilterNumber = 0;
		}else if( CANx == CAN2 )
		{
			  CAN_Tx_Pin = PB13;//PB13;6
			  CAN_Rx_Pin = PB12;//PB12;7
			  GPIO_AF_CANx = GPIO_AF_CAN2;
			  CANx_RX0_IRQn = CAN2_RX0_IRQn;
		    RCC_APB1Periph = RCC_APB1Periph_CAN2;
			  CANx_FilterNumber = 14;
		}

    RCC_APB1PeriphClockCmd(RCC_APB1Periph, ENABLE);

    RCC_APB1PeriphResetCmd(RCC_APB1Periph, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph, DISABLE);
		
		GPIOx_Init( CAN_Tx_Pin, OUTPUT_AF, GPIO_Speed_100MHz );
		GPIOx_Init( CAN_Rx_Pin, OUTPUT_AF, GPIO_Speed_100MHz );

    GPIO_PinAFConfig( PIN_MAP[CAN_Tx_Pin].GPIOx, GPIO_GetPinSource( PIN_MAP[CAN_Tx_Pin].GPIO_Pin_x ), GPIO_AF_CANx );
    GPIO_PinAFConfig( PIN_MAP[CAN_Rx_Pin].GPIOx, GPIO_GetPinSource( PIN_MAP[CAN_Rx_Pin].GPIO_Pin_x ), GPIO_AF_CANx );

    CAN_InitStructure.CAN_TTCM = DISABLE;
    CAN_InitStructure.CAN_ABOM = ENABLE;
    CAN_InitStructure.CAN_AWUM = DISABLE;
    CAN_InitStructure.CAN_NART = DISABLE;
    CAN_InitStructure.CAN_RFLM = DISABLE;
    CAN_InitStructure.CAN_TXFP = DISABLE;
    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
    CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
    CAN_InitStructure.CAN_BS1 = CAN_BS1_6tq;
    CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
    CAN_InitStructure.CAN_Prescaler = 5;
//		CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
//    CAN_InitStructure.CAN_BS1 = CAN_BS1_10tq;
//    CAN_InitStructure.CAN_BS2 = CAN_BS2_3tq;
    //CAN_InitStructure.CAN_Prescaler = 3;
    CAN_Init(CANx, &CAN_InitStructure);

    CAN_FilterInitStructure.CAN_FilterNumber = CANx_FilterNumber;
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
//		CAN_FilterInitStructure.CAN_FilterActivation = DISABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);

    CAN_ITConfig(CANx, CAN_IT_FMP0, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = CANx_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PreemptionPriority;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = SubPriority;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void CANdev::CANx_Init( void )
{
    CANx_Init( CAN_PreemptionPriority_Default, CAN_SubPriority_Default );
}
