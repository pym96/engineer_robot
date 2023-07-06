#include "main.h"
#include "motor_control.h"
#include "crc_ccitt.h"
#include "stdio.h"
#include "math.h"
#include "Kinematics_task.h"

extern UART_HandleTypeDef huart1;
MOTOR_send ys_motor={0};
MOTOR_recv yr_motor={0};
//错误处理标志位
u8 IsGoWorkNormal =1;
u8 GOTimeOut = 0;


#define SATURATE(_IN, _MIN, _MAX) {\
 if (_IN < _MIN)\
 _IN = _MIN;\
 else if (_IN > _MAX)\
 _IN = _MAX;\
 } 

int modify_data(MOTOR_send *motor_s)
{
    motor_s->hex_len = 17;
    motor_s->motor_send_data.head[0] = 0xFE;
    motor_s->motor_send_data.head[1] = 0xEE;
	
		//SATURATE(motor_s->id,   0,    15);
		//SATURATE(motor_s->mode, 0,    7);
	  motor_s->id = 0;
	  motor_s->mode = 1;
		SATURATE(motor_s->K_P,  0.0f,   25.599f);
		SATURATE(motor_s->K_W,  0.0f,   25.599f);
		//SATURATE(motor_s->T,   -127.99f,  127.99f);
		//SATURATE(motor_s->W,   -804.00f,  804.00f);
		//SATURATE(motor_s->Pos, -411774.0f,  411774.0f);
	  SATURATE(motor_s->T,   -17.99f,  17.99f);
	  SATURATE(motor_s->W,   - 3.14f,   3.14f);
	  SATURATE(motor_s->Pos, -14.0f,  3.0f);

    motor_s->motor_send_data.mode.id   = motor_s->id;
    motor_s->motor_send_data.mode.status  = motor_s->mode;
    motor_s->motor_send_data.comd.k_pos  = motor_s->K_P/25.6f*32768;
    motor_s->motor_send_data.comd.k_spd  = motor_s->K_W/25.6f*32768;
    motor_s->motor_send_data.comd.pos_des  = motor_s->Pos/6.2832f*32768;
    motor_s->motor_send_data.comd.spd_des  = motor_s->W/6.2832f*256;
    motor_s->motor_send_data.comd.tor_des  = motor_s->T*256;
//    motor_s->motor_send_data
    motor_s->motor_send_data.CRC16 = crc_ccitt(0, (uint8_t *)&motor_s->motor_send_data, 15);
    return 0;
}

int extract_data(MOTOR_recv *motor_r)
{
//    if(motor_r->motor_recv_data.CRC16 !=
//        crc_ccitt(0, (uint8_t *)&motor_r->motor_recv_data, 14)){
//        motor_r->correct = 0;
//        return motor_r->correct;
//    }
//    else
//		{
        motor_r->motor_id = motor_r->motor_recv_data.mode.id;
        motor_r->mode = motor_r->motor_recv_data.mode.status;
        motor_r->Temp = motor_r->motor_recv_data.fbk.temp;
        motor_r->MError = motor_r->motor_recv_data.fbk.MError;
        motor_r->W = ((float)motor_r->motor_recv_data.fbk.speed/256)*6.2832f ;
        motor_r->T = ((float)motor_r->motor_recv_data.fbk.torque) / 256;
        motor_r->Pos = 6.2832f*((float)motor_r->motor_recv_data.fbk.pos) / 32768;
				motor_r->footForce = motor_r->motor_recv_data.fbk.force;
				motor_r->correct = 1;
        return motor_r->correct;
//    }
}

uint16_t rxlen = 0;
uint8_t *rp = NULL;
HAL_StatusTypeDef SERVO_Send_recv(MOTOR_send *pData, MOTOR_recv *rData)
{
    PID_cala8010(pData,rData);
    modify_data(pData);	
		SET_485_DE_UP();
    HAL_UART_Transmit(&huart1, (uint8_t *)pData, sizeof(pData->motor_send_data), 10); 	
		
	  SET_485_DE_DOWN();
	
	  HAL_UARTEx_ReceiveToIdle(&huart1, (uint8_t *)rData, 14, &rxlen, 10);				

    if(rxlen == 0)
      return HAL_TIMEOUT;

    if(rxlen != 14)
			return HAL_ERROR;

    rp = (uint8_t *)&rData->motor_recv_data;
    if(rp[0] == 0xFD && rp[1] == 0xEE)
    {
			  if(rData->motor_recv_data.mode.id==0 && rData->motor_recv_data.mode.status==1 && rData->motor_recv_data.fbk.MError ==0)
				{
					rData->correct = 1;
					//只有工作模式正常的时候我才能发送数据
						extract_data(rData);
					
					return HAL_OK;
				}else
				{
					GOTimeOut++;
					if(GOTimeOut > GOTIMEOUTLIMIT)
					{
						 //如果数据不对超过一定时间认为堵转了 直接把宇树挂起
						 pData->K_P=0;
					   pData->K_W=0;
					   SET_485_DE_UP();
             HAL_UART_Transmit(&huart1, (uint8_t *)pData, sizeof(pData->motor_send_data), 10); 
						 GOTimeOut = 0;
						 IsGoWorkNormal =0;
					}
					return HAL_ERROR;
				}
    }
    
    return HAL_ERROR;
}

void PID_cala8010(MOTOR_send *pData, MOTOR_recv *rData)
{
	 Pid_m pid;
	 pid.maxIout = 0;
   pid.error_ = pData->Pos - rData->Pos;
//      if (fabs(pid.error_) >= 5) {
//        pData->K_P = 0.3;

//        if (pid.error_ > 0)
//          pData->W = 0.5;
//        else
//          pData->W = -0.5;
//          
//        pid.ki = 0.004;
//      } else if (fabs(pid.error_) > 3 && fabs(pid.error_) < 5) {
//        pData->K_P = 0.4;

//        if (pid.error_ > 0)
//          pData->W = 0.2;
//        else
//          pData->W = -0.2;

//        pid.ki = 0.003;
//      } else {
//        pData->K_P = 0.7;
//        pData->K_W = 0.045;
//        pData->W = 0.0;
//        pid.ki = 0.0045;
//      }
  if (fabs(pid.error_) >= 9)
	{
		   pData->K_P=0.2;
       pData->K_W = 0.15;
		   pid.ki = 0.003;
       if (pid.error_ > 0)
         pData->W = 0.1;
       else
         pData->W = -0.1;
			 
			 pid.Iout += pid.ki * pid.error_;
       pid.Iout = constrain(pid.Iout, -pid.maxIout, pid.maxIout);

			 //pData->Pos= (pData->Pos)-0.2f*pid.error_;
			 pData->Pos= (pData->Pos + pid.Iout);
	}
  else if (fabs(pid.error_)>= 7 && fabs(pid.error_)< 9)
	{
		   pData->K_P=0.25;
       pData->K_W = 0.12;
		   pid.ki = 0.0035;
       if (pid.error_ > 0)
         pData->W = 0.2;
       else
         pData->W = -0.2;
			 
			 pid.Iout += pid.ki * pid.error_;
       pid.Iout = constrain(pid.Iout, -pid.maxIout, pid.maxIout);

		
			 pData->Pos= (pData->Pos + pid.Iout);
	}
  else if (fabs(pid.error_) >= 5 && fabs(pid.error_)< 7)
	{
		   pData->K_P=0.35;
       pData->K_W = 0.08;
		   pid.ki = 0.004;
       if (pid.error_ > 0)
         pData->W = 0.3;
       else
         pData->W = -0.3;
			 
			 pid.Iout += pid.ki * pid.error_;
       pid.Iout = constrain(pid.Iout, -pid.maxIout, pid.maxIout);

			 pData->Pos= (pData->Pos + pid.Iout);
	
	}
  else if (fabs(pid.error_) >= 3 && fabs(pid.error_)< 5)
	{
		   pData->K_P=0.45;
       pData->K_W = 0.06;
		   pid.ki = 0.0045;
       if (pid.error_ > 0)
         pData->W = 0.4;
       else
         pData->W = -0.4;
			 
			 pid.Iout += pid.ki * pid.error_;
       pid.Iout = constrain(pid.Iout, -pid.maxIout, pid.maxIout);

			 pData->Pos= (pData->Pos + pid.Iout);
	
	}
  else
	{
		   pData->K_P=0.5;
       pData->K_W = 0.05;
		   pData->W =0.5;
       pid.ki = 0.005;
		
       pid.Iout += pid.ki * pid.error_;
       pid.Iout = constrain(pid.Iout, -pid.maxIout, pid.maxIout);
     	 pData->Pos= (pData->Pos)+pid.Iout;
	}
}

