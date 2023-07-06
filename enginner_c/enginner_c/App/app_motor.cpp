#include "app_motor.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern Message_bag message_bag;


Torque_Motor_Ctrl    Arm;

int float_to_uint(float x, float x_min, float x_max, int bits){
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
    }
    
    
float uint_to_float(int x_int, float x_min, float x_max, int bits){
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
    }


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
	if(hcan->Instance==CAN2)
	{
      CAN_RxHeaderTypeDef RxHeader;
      uint8_t RxData[8];	
	    if (HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
    {
	switch (RxHeader.StdId)
	{//2.792
		case CAN_JOINT1_ID:
		case CAN_JOINT2_ID:
		case CAN_JOINT3_ID:
		case CAN_JOINT4_ID:
		{
			static uint8_t i = 0;
			//处理电机ID号
			i = RxHeader.StdId - 0x04;
			//处理电机数据宏函数
			Arm.Torque_Motor_Measure[i].ID=(RxData[0])&0x0F; 
			Arm.Torque_Motor_Measure[i].state=(RxData[0]>>4);		
			Arm.Torque_Motor_Measure[i].POS.Idata=(RxData[1]<<8)|RxData[2];
			Arm.Torque_Motor_Measure[i].VEl.Idata=(RxData[3]<<4)|(RxData[4]>>4);
			Arm.Torque_Motor_Measure[i].Torque.Idata=((RxData[4]&0xF)<<8)|RxData[5];
			Arm.Torque_Motor_Measure[i].temperature_MOS = (float)(RxData[6]);
			Arm.Torque_Motor_Measure[i].temperature_Rotro = (float)(RxData[7]);
			
			//处理电机数据为浮点型数据
			Arm.POS_[i] = uint_to_float(Arm.Torque_Motor_Measure[i].POS.Idata, JointPLimit[i][MINVALUE], JointPLimit[i][MAXVALUE], 16);
			Arm.VEl_[i] = uint_to_float(Arm.Torque_Motor_Measure[i].VEl.Idata, JointVLimit[i][MINVALUE], JointVLimit[i][MAXVALUE], 12); 
			Arm.Torque_Motor_Measure[i].Torque.fdata = uint_to_float(Arm.Torque_Motor_Measure[i].Torque.Idata, T_MIN, T_MAX, 12); 
			break;			
		}
	}
   }
 }
	if(hcan->Instance==CAN1)
	{
      CAN_RxHeaderTypeDef RxHeader;
      uint8_t RxData[8];	
		 if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
    {
	switch (RxHeader.StdId)
	{
	  case 0x126:
		{  
       get_limit_measuer(&message_bag, RxData);
			    break;
      
    }
		case 201:
		case 202:
		case 203:
		case 204:
		{
			break;
		}
	}
	   }
	 }
	 
	
//	if(hcan->Instance==CAN2)
//	{
//      CAN_RxHeaderTypeDef RxHeader;
//      uint8_t RxData[8];	
//	    if (HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
//    {
//	     switch (RxHeader.StdId)
//	    {//2.792
//		    case 0x100:
//		   {  
//       get_limit_measuer(&message_bag, RxData);
//			    break;			
//		   }
//      
//      }
//	  }
//	 }


}

//位置速度控制模式
void ctrl_motor_pv(uint16_t id ,float _pos, float _vel)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t TxMailbox;  
		uint8_t *pbuf,*vbuf;
		pbuf=(uint8_t*)&_pos;
		vbuf=(uint8_t*)&_vel;
	
    TxHeader.StdId = id+0x100;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 0x08;
    TxData[0] = *pbuf;
    TxData[1] = *(pbuf+1);
    TxData[2] = *(pbuf+2);
    TxData[3] = *(pbuf+3);
    TxData[4] = *vbuf;
	  TxData[5] = *(vbuf+1);
	  TxData[6] = *(vbuf+2);
	  TxData[7] = *(vbuf+3);
	
  if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox) != HAL_OK)
  {
//    Error_Handler();
  }
}

void start_motor(uint16_t id)
{
	
  CAN_TxHeaderTypeDef TxHeader;
  uint8_t TxData[8];
  uint32_t TxMailbox;

  TxHeader.StdId =  id+0x100;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.DLC = 0x08;
  TxData[0] = 0xFF;
  TxData[1] = 0xFF;
  TxData[2] = 0xFF;
  TxData[3] = 0xFF;
  TxData[4] = 0xFF;
  TxData[5] = 0xFF;
  TxData[6] = 0xFF;
  TxData[7] = 0xFC;

  if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox) != HAL_OK)
  {
    //Error_Handler();
  }
}	

void lock_motor(uint16_t id)
{
  CAN_TxHeaderTypeDef TxHeader;
  uint8_t TxData[8];
  uint32_t TxMailbox;

  TxHeader.StdId =  id+0x100;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.DLC = 0x08;
  TxData[0] = 0xFF;
  TxData[1] = 0xFF;
  TxData[2] = 0xFF;
  TxData[3] = 0xFF;
  TxData[4] = 0xFF;
  TxData[5] = 0xFF;
  TxData[6] = 0xFF;
  TxData[7] = 0xFD;

  if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox) != HAL_OK)
  {
//    Error_Handler();
  }
}	

void setpoint_motor(uint16_t id)
{
  CAN_TxHeaderTypeDef TxHeader;
  uint8_t TxData[8];
  uint32_t TxMailbox;

  TxHeader.StdId =  id+0x100;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.DLC = 0x08;
  TxData[0] = 0xFF;
  TxData[1] = 0xFF;
  TxData[2] = 0xFF;
  TxData[3] = 0xFF;
  TxData[4] = 0xFF;
  TxData[5] = 0xFF;
  TxData[6] = 0xFF;
  TxData[7] = 0xFE;

  if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox) != HAL_OK)
  {
//    Error_Handler();
  }
}	



