#include "app_motor.h"

CAN_Ctrl CAN_Cmd;
extern Message_bag message_bag;

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
static void CAN2_Hook(CanRxMsg* Rx_Message)
{
	switch (Rx_Message->StdId)
	{
		case 0x126:
		{			
     get_limit_measuer(&message_bag, Rx_Message);			
	break;			
		}
		case CAN_3510_PUSH1_ID:
		case CAN_3510_PUSH2_ID:
		case CAN_3508_UP1_ID:
		case CAN_3508_UP2_ID:
		{
			static uint8_t i = 0;
			//处理电机ID号
			i = Rx_Message->StdId - CAN_3510_PUSH1_ID
			;
			//处理电机数据宏函数
			get_motor_measure(&CAN_Cmd.Up_Push.Up_Push_Motor_Measure[i], Rx_Message);
			break;
		}	
	}
}

static void CAN1_Hook(CanRxMsg* Rx_Message)
{
	switch (Rx_Message->StdId)
	{//2.792
		case CAN_3508_M1_ID:
		case CAN_3508_M2_ID:
		case CAN_3508_M3_ID:
		case CAN_3508_M4_ID:
		{
			static uint8_t i = 0;
			//处理电机ID号
			i = Rx_Message->StdId - CAN_3508_M1_ID;
			//处理电机数据宏函数
			get_motor_measure(&CAN_Cmd.Chassis.Chassis_Measure[i], Rx_Message);
			break;
		}
	}
}

//位置速度控制模式
void ctrl_motor_pv(uint16_t id ,float _pos, float _vel)
{
    CanTxMsg TxMessage;  
		uint8_t *pbuf,*vbuf;
		pbuf=(uint8_t*)&_pos;
		vbuf=(uint8_t*)&_vel;
	
    TxMessage.StdId = id+0x100;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = *pbuf;
    TxMessage.Data[1] = *(pbuf+1);
    TxMessage.Data[2] = *(pbuf+2);
    TxMessage.Data[3] = *(pbuf+3);
    TxMessage.Data[4] = *vbuf;
	  TxMessage.Data[5] = *(vbuf+1);
	  TxMessage.Data[6] = *(vbuf+2);
	  TxMessage.Data[7] = *(vbuf+3);
	
    CAN_Transmit( CAN1, &TxMessage );
}

void start_motor(uint16_t id)
{
    CanTxMsg TxMessage;
	
    TxMessage.StdId = id+0x100;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = 0xFF;
    TxMessage.Data[1] = 0xFF;
    TxMessage.Data[2] = 0xFF;
    TxMessage.Data[3] = 0xFF;
    TxMessage.Data[4] = 0xFF;
	  TxMessage.Data[5] = 0xFF;
	  TxMessage.Data[6] = 0xFF;
	  TxMessage.Data[7] = 0xFC;
	
    CAN_Transmit( CAN1, &TxMessage );
}	

void lock_motor(uint16_t id)
{
    CanTxMsg TxMessage;
	
    TxMessage.StdId = id+0x100;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = 0xFF;
    TxMessage.Data[1] = 0xFF;
    TxMessage.Data[2] = 0xFF;
    TxMessage.Data[3] = 0xFF;
    TxMessage.Data[4] = 0xFF;
	  TxMessage.Data[5] = 0xFF;
	  TxMessage.Data[6] = 0xFF;
	  TxMessage.Data[7] = 0xFD;
	
    CAN_Transmit( CAN1, &TxMessage );
}	

void setpoint_motor(uint16_t id)
{
    CanTxMsg TxMessage;
	
    TxMessage.StdId = id+0x100;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = 0xFF;
    TxMessage.Data[1] = 0xFF;
    TxMessage.Data[2] = 0xFF;
    TxMessage.Data[3] = 0xFF;
    TxMessage.Data[4] = 0xFF;
	  TxMessage.Data[5] = 0xFF;
	  TxMessage.Data[6] = 0xFF;
	  TxMessage.Data[7] = 0xFE;
	
    CAN_Transmit( CAN1, &TxMessage );
}	

void CAN_ALL_Init(void)
{
	//必须先初始化CAN1，在初始化CAN2
	CAN1_Ctrl.CANx_Init();
	CAN2_Ctrl.CANx_Init();

	CAN1_Ctrl.attachInterrupt(CAN1_Hook);
	CAN2_Ctrl.attachInterrupt(CAN2_Hook);
}


