#include "app_motor.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern Message_bag message_bag;

MGmotor mgmotor ;
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


		
u8 errorcan[2]={0,0};
		
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
				case 0x141 :
				{
					if(RxData[0]==0xA3 || RxData[0]==0xA6 || RxData[0]==0xA5 ||RxData[0]==0xA4)
					{
						mgmotor.MG_Motor_measure.tem=RxData[1];
						mgmotor.MG_Motor_measure.iq=RxData[2]+(RxData[3]<<8);
						mgmotor.MG_Motor_measure.speed=RxData[4]+(RxData[5]<<8);
						mgmotor.MG_Motor_measure.encoder=RxData[6]+(RxData[7]<<8);
					}
					
					if(RxData[0]==0x92)
					{
						mgmotor.MG_Motor_measure.mutilAngle=(int64_t)(RxData[1]|(RxData[2]<<8)|(RxData[3]<<8)|(RxData[4]<<8)|(RxData[5]<<8)|(RxData[6]<<8)|(RxData[7]<<8));
					}
					
					if(RxData[0]==0x94)
					{
						mgmotor.MG_Motor_measure.circleAngle=(int16_t)(RxData[4]|(RxData[5]<<8));
					}
					
					if(RxData[0]==0x33)
					{
						mgmotor.MG_Motor_measure.accle=(int32_t)((RxData[4])|(RxData[5]<<8)|(RxData[6]<<16)|(RxData[7]<<24));
					}
				}
				default :
				{
					errorcan[0]++;
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
              message_bag.chassis.limit.dowm_limit =   (RxData[0]) ;          
              message_bag.chassis.limit.up_limit  =   (RxData[1]) ;         
			        message_bag.chassis.limit.back_limit  =   (RxData[2]) ;         
			        message_bag.chassis.limit.front_limit  =   (RxData[3]) ;         
			        message_bag.chassis.sucker_state  =   (RxData[4]) ;         
			        message_bag.chassis.speed_level  =   (RxData[5]) ;         	
			       break;
          }
					default :
					{
						errorcan[1]++;
						break;
					}
				}
	  }
	 }

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

void MGmotor::motor_cmd(uint8_t _cmd)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t TxMailbox;  
	
    TxHeader.StdId = 1+0x140;
    TxHeader.RTR = CAN_RTR_DATA;
		TxHeader.IDE = CAN_ID_STD;
		TxHeader.ExtId = 0x00;
    TxHeader.DLC = 0x08;
    TxData[0] = (uint8_t)_cmd;
	  TxData[1] = 0;
    TxData[2] = 0;
    TxData[3] = 0;
    TxData[4] = 0;
	  TxData[5] = 0;
	  TxData[6] = 0;
	  TxData[7] = 0;
	
  if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox) != HAL_OK)
  {
//    Error_Handler();
  }
}

void MGmotor::motor_send(uint16_t _pos)
{
//	angle_control.angle =(int32_t)((_pos/3.14f)*18000);
	   angle_control.angle = _pos;
	if(angle_control.last_angle<angle_control.angle)
		angle_control.spinDirection=0x00;
	else 
		angle_control.spinDirection=0x01;
	
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t TxMailbox;  
	
    TxHeader.StdId = 1+0x140;
    TxHeader.RTR = CAN_RTR_DATA;
		TxHeader.IDE = CAN_ID_STD;
		TxHeader.ExtId = 0x00;
    TxHeader.DLC = 0x08;
    TxData[0] = 0xA5;
    TxData[1] = angle_control.spinDirection;
    TxData[2] = 0;
    TxData[3] = 0;
    TxData[4] = *((uint8_t *)(&angle_control.angle) + 0);
	  TxData[5] = *((uint8_t *)(&angle_control.angle) + 1);
	  TxData[6] = 0;
	  TxData[7] = 0;
	
  if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox) != HAL_OK)
  {
//    Error_Handler();
  }
	angle_control.last_angle=angle_control.angle;
}
void MGmotor::motor_SPsend(uint16_t _pos,uint16_t _vel)
{
	angle_control.angle = _pos*100;
	angle_control.vel = _vel;
	if(angle_control.last_angle<angle_control.angle)
		angle_control.spinDirection=0x00;
	else if(angle_control.last_angle>angle_control.angle)
		angle_control.spinDirection=0x01;
	
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t TxMailbox;  
	
    TxHeader.StdId = 0x01+0x140;
    TxHeader.RTR = CAN_RTR_DATA;
		TxHeader.IDE = CAN_ID_STD;
		TxHeader.ExtId = 0x00;
    TxHeader.DLC = 0x08;
    TxData[0] = 0xA6;
    TxData[1] = angle_control.spinDirection;
    TxData[2] =  *((uint8_t *)(&angle_control.vel) + 0);
    TxData[3] =  *((uint8_t *)(&angle_control.vel) + 1);
    TxData[4] =  *((uint8_t *)(&angle_control.angle) + 0);
	  TxData[5] =  *((uint8_t *)(&angle_control.angle) + 1);
	  TxData[6] = 0;
	  TxData[7] = 0;
	
  if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox) != HAL_OK)
  {
//    Error_Handler();
  }
	angle_control.last_angle=angle_control.angle;
}
void MGmotor::motor_start(void)
{
	  int start_angle =20000;
	  angle_control.spinDirection=0x01;
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t TxMailbox;  
	
    TxHeader.StdId = 1+0x140;
    TxHeader.RTR = CAN_RTR_DATA;
		TxHeader.IDE = CAN_ID_STD;
		TxHeader.ExtId = 0x00;
    TxHeader.DLC = 0x08;
    TxData[0] = 0xA5;
    TxData[1] = angle_control.spinDirection;
    TxData[2] = 0;
    TxData[3] = 0;
    TxData[4] =(uint8_t)start_angle;
	  TxData[5] =(uint8_t)start_angle>>8;
	  TxData[6] = 0;
	  TxData[7] = 0;
	
  if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox) != HAL_OK)
  {
//    Error_Handler();
  }
	angle_control.last_angle=start_angle;
	
}
void MGmotor::motor_MVP(float _pos,uint16_t _vel)
{
    _pos=constrain(_pos,-2.52 ,0.22f);
		angle_control.angle = (int32_t)((_pos*RAD2DEG*100)*10);
		angle_control.vel = (uint16_t)(_vel);
	
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t TxMailbox;  
	
    TxHeader.StdId = 0x01+0x140;
    TxHeader.RTR = CAN_RTR_DATA;
		TxHeader.IDE = CAN_ID_STD;
		TxHeader.ExtId = 0x00;
    TxHeader.DLC = 0x08;
    TxData[0] = 0xA4;
    TxData[1] = 0;
    TxData[2] = *((uint8_t *)(&angle_control.vel) + 0);
    TxData[3] = *((uint8_t *)(&angle_control.vel) + 1);
    TxData[4] = *((uint8_t *)(&angle_control.angle) + 0);
	  TxData[5] = *((uint8_t *)(&angle_control.angle) + 1);
	  TxData[6] = *((uint8_t *)(&angle_control.angle) + 2);
	  TxData[7] = *((uint8_t *)(&angle_control.angle) + 3);
	
  if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox) != HAL_OK)
  {
  }
}

void MGmotor::motor_accle(int32_t _accle)
{
		angle_control.accle = (int32_t)(_accle);
	
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t TxMailbox;  
	
    TxHeader.StdId = 0x01+0x140;
    TxHeader.RTR = CAN_RTR_DATA;
		TxHeader.IDE = CAN_ID_STD;
		TxHeader.ExtId = 0x00;
    TxHeader.DLC = 0x08;
    TxData[0] = 0x34;
    TxData[1] = 0;
    TxData[2] = 0;
    TxData[3] = 0;
    TxData[4] = *((uint8_t *)(&angle_control.accle) + 0);
	  TxData[5] = *((uint8_t *)(&angle_control.accle) + 1);
	  TxData[6] = *((uint8_t *)(&angle_control.accle) + 2);
	  TxData[7] = *((uint8_t *)(&angle_control.accle) + 3);
	
  if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox) != HAL_OK)
  {
  }
}

void MGmotor::motor_test(void)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t TxMailbox;  

	  uint16_t ctlValue = 10 * 100;
	
    TxHeader.StdId = 1+0x140;
    TxHeader.RTR = CAN_RTR_DATA;
		TxHeader.IDE = CAN_ID_STD;
		TxHeader.ExtId = 0x00;
    TxHeader.DLC = 0x08;
		TxHeader.StdId = 0x140 + 1;
		TxHeader.ExtId = 0x00;
		TxHeader.RTR = CAN_RTR_DATA;
		TxHeader.IDE = CAN_ID_STD;
		TxHeader.DLC = 8;
		
		TxData[0] = 0xA7;
		TxData[1] = 0x00;
		TxData[2] = 0x00;
		TxData[3] = 0x00;
		TxData[4] = *((uint8_t *)&ctlValue + 0);
		TxData[5] = *((uint8_t *)&ctlValue + 1);
		TxData[6] = 0x00;
		TxData[7] = 0x00;
		
		/* send can command */
		HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox);
}
