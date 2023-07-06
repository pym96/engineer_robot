#ifndef __APP_USART7_H
#define __APP_USART7_H

#include "dev_system.h"

#ifdef __cplusplus
extern "C" {
#endif
void CopeSerial7Data(void);

//��̨����������
struct SGyro *get_imu_gyro_Point(void);
struct SAngle *get_imu_angle_Point(void);
void imu_gimbal_set_zero(void);
struct GOMotor_t *get_GOMotor_Point(void);
void USART6CallBackFun();
	
//���ڷ��ͺ���
void send_float(float data,USART_TypeDef *_USARTx);
void send_double(double data,USART_TypeDef *_USARTx);
void send_int(int data,USART_TypeDef *_USARTx);
void send_u8(int data,USART_TypeDef *_USARTx);

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

struct GOMotor_t
{
	float POS_;
	float VEL_;
	float W_;
	float Tor_;
	float Tem_;
};

#endif







