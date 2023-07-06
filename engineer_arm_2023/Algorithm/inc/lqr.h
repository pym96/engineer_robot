#ifndef _LQR_H__
#define _LQR_H__

#include "dev_system.h"
#include "app_motor.h"
#include "arm_math.h"
#include "stdint.h"

#define torque_constant2Nm 0.741 // ת�س��� ��λ N*M/A
#define torque_constant2mNm 741 // ת�س��� ��λ mN*M/A

#define A_Columns  2 //��
#define A_Rows     2 //��

#define B_Columns  1 //��
#define B_Rows     2 //��

typedef struct
{
	fp32 error;
	fp32 motor_rmp;
}state_variable;//ѡȡ��״̬����

typedef struct
{
	fp32 K1;
	fp32 K2;

	fp32 Fc;//ǰ��ϵ��
}match_variable;//ѡȡ��ƥ�����

//typedef struct
//{
//	matrix_f32 A;//״̬����
//	matrix_f32 B;//�������
//	matrix_f32 C;//�������
//	matrix_f32 D;//ֱ�Ӵ��ݾ���

//	/*����ϵ��*/
//	matrix_f32 Quadratic;
//	matrix_f32 Regulator;
//};
class motor_6020_modle
{
public:
	motor_6020_modle(CAN_Ctrl* motor, float &p_rotational_inertia)
	{
		this->motor = motor;
		this->rotational_inertia = p_rotational_inertia;
		reciprocal2r_i = (1 / rotational_inertia);
	}

	int16_t Out_function(float& rad, float& rad_set);

	match_variable match_K1K2;

	float  torque;//����
	float  last_rad;//��һ�ε����
	float  rotational_inertia;//ת������
	float  reciprocal2r_i;//ת�������ĵ���

	CAN_Ctrl* motor;
};























#endif