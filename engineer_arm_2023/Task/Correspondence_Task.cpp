//#include "Correspondence_Task.h"
//#include "device.h"



//void Correspondence_Task(void *pvParameters)
//{
//	while(1)
//	{
//    
//		vTaskDelay(2);
//	}
//}



//void correspondence_ctrl::Corres_Send(void)
//{
//  CAN_Corres_CAN.SendData(limit.dowm_limit,limit.up_limit);	
//}


//void correspondence_ctrl::Corres_Feedback(void)
//{
//	limit.dowm_limit=GPIO_ReadInputDataBit(GPIOH,GPIO_Pin_11);
//  limit.up_limit  =GPIO_ReadInputDataBit(GPIOH,GPIO_Pin_12);
//}

//void correspondence_ctrl::SendData( int16_t data1, int16_t data2, int16_t data3, int16_t data4 )
//{
//    CanTxMsg TxMessage;
//    TxMessage.StdId = 0x100;
//    TxMessage.IDE = CAN_ID_STD;
//    TxMessage.RTR = CAN_RTR_DATA;
//    TxMessage.DLC = 0x08;
//    TxMessage.Data[0] = data1 >> 8;
//    TxMessage.Data[1] = data1;
//    TxMessage.Data[2] = data2 >> 8;
//    TxMessage.Data[3] = data2;
//    TxMessage.Data[4] = data3 >> 8;
//    TxMessage.Data[5] = data3;
//    TxMessage.Data[6] = data4 >> 8;
//    TxMessage.Data[7] = data4;

//    CAN_Transmit( CAN2, &TxMessage );
//}

//void correspondence_ctrl::SendData( int16_t data1, int16_t data2, int16_t data3 )
//{
//    CanTxMsg TxMessage;
//    TxMessage.StdId = 0x100;
//    TxMessage.IDE = CAN_ID_STD;
//    TxMessage.RTR = CAN_RTR_DATA;
//    TxMessage.DLC = 0x08;
//    TxMessage.Data[0] = data1 >> 8;
//    TxMessage.Data[1] = data1;
//    TxMessage.Data[2] = data2 >> 8;
//    TxMessage.Data[3] = data2;
//    TxMessage.Data[4] = data3 >> 8;
//    TxMessage.Data[5] = data3;

//    CAN_Transmit( CAN2, &TxMessage );
//}

//void correspondence_ctrl::SendData( int16_t data1, int16_t data2 )
//{
//    CanTxMsg TxMessage;
//    TxMessage.StdId = 0x100;
//    TxMessage.IDE = CAN_ID_STD;
//    TxMessage.RTR = CAN_RTR_DATA;
//    TxMessage.DLC = 0x08;
//    TxMessage.Data[0] = data1 >> 8;
//    TxMessage.Data[1] = data1;
//    TxMessage.Data[2] = data2 >> 8;
//    TxMessage.Data[3] = data2;

//    CAN_Transmit( CAN2, &TxMessage );
//}

//void correspondence_ctrl::SendData( int16_t data1 )
//{
//    CanTxMsg TxMessage;
//    TxMessage.StdId = 0x100;
//    TxMessage.IDE = CAN_ID_STD;
//    TxMessage.RTR = CAN_RTR_DATA;
//    TxMessage.DLC = 0x08;
//    TxMessage.Data[0] = data1 >> 8;
//    TxMessage.Data[1] = data1;

//    CAN_Transmit( CAN2, &TxMessage );
//}