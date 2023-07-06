#ifndef __APP_USART7_H
#define __APP_USART7_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif
void CopeSerial7Data(void);
	
//���ڷ��ͺ���
void send_float(float data,UART_HandleTypeDef *huart);
void send_double(double data,UART_HandleTypeDef *huart);
void send_int(int data,UART_HandleTypeDef *huart);
void send_u8(int data,UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

struct SGyro//���ٶ�
{
	short w[3];//xyz
	short T;
};
struct SAngle//�Ƕ�
{
	short Angle[3];//xyz
	short T;
};

//struct GOMotor_t
//{
//	float POS_;
//	float VEL_;
//	float W_;
//	float Tor_;
//	float Tem_;
//};

#endif







