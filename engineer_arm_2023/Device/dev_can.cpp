#include "dev_can.h"
#include "app_motor.h"

CANctrl CAN1_Ctrl( CAN1, CAN_CHASSIS_ALL_ID  );
CANctrl CAN2_Ctrl( CAN2, CAN_UP_PUSH_ID );

void CANctrl::attachInterrupt( CAN_CallbackFunction_t Function )
{
    this->CAN_Function = Function;
}

void CANctrl::SendData( int16_t data1, int16_t data2, int16_t data3, int16_t data4 )
{
    CanTxMsg TxMessage;
    TxMessage.StdId = StdId;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = data1 >> 8;
    TxMessage.Data[1] = data1;
    TxMessage.Data[2] = data2 >> 8;
    TxMessage.Data[3] = data2;
    TxMessage.Data[4] = data3 >> 8;
    TxMessage.Data[5] = data3;
    TxMessage.Data[6] = data4 >> 8;
    TxMessage.Data[7] = data4;

    CAN_Transmit( CANx, &TxMessage );
}

void CANctrl::SendData( int16_t data1, int16_t data2, int16_t data3 )
{
    CanTxMsg TxMessage;
    TxMessage.StdId = StdId;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = data1 >> 8;
    TxMessage.Data[1] = data1;
    TxMessage.Data[2] = data2 >> 8;
    TxMessage.Data[3] = data2;
    TxMessage.Data[4] = data3 >> 8;
    TxMessage.Data[5] = data3;

    CAN_Transmit( CANx, &TxMessage );
}

void CANctrl::SendData( int16_t data1, int16_t data2 )
{
    CanTxMsg TxMessage;
    TxMessage.StdId = StdId;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = data1 >> 8;
    TxMessage.Data[1] = data1;
    TxMessage.Data[2] = data2 >> 8;
    TxMessage.Data[3] = data2;

    CAN_Transmit( CANx, &TxMessage );
}

void CANctrl::SendData( int16_t data1 )
{
    CanTxMsg TxMessage;
    TxMessage.StdId = StdId;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = data1 >> 8;
    TxMessage.Data[1] = data1;

    CAN_Transmit( CANx, &TxMessage );
}

void CANctrl::IRQHandler( void )
{
    static CanRxMsg Rx_Message;

    if (CAN_GetITStatus(CANx, CAN_IT_FMP0) != RESET)
    {
        CAN_ClearITPendingBit( CANx, CAN_IT_FMP0 );
        CAN_Receive( CANx, CAN_FIFO0, &Rx_Message );
        if( CAN_Function )
				{
				   CAN_Function( &Rx_Message );
				}
    }
}

extern"C"
{
    void CAN1_RX0_IRQHandler(void)
    {
			uint32_t ulReturn;

		/* 进入临界段，临界段可以嵌套 */
			ulReturn = taskENTER_CRITICAL_FROM_ISR();
			
      CAN1_Ctrl.IRQHandler();
						
			taskEXIT_CRITICAL_FROM_ISR( ulReturn );
    }
    void CAN2_RX0_IRQHandler(void)
    {
     	uint32_t ulReturn;

		/* 进入临界段，临界段可以嵌套 */
			ulReturn = taskENTER_CRITICAL_FROM_ISR();
			
      CAN2_Ctrl.IRQHandler();
						
			taskEXIT_CRITICAL_FROM_ISR( ulReturn );
    }	
}
