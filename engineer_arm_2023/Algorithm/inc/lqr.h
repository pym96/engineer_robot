#ifndef _LQR_H__
#define _LQR_H__

#include "dev_system.h"
#include "app_motor.h"
#include "arm_math.h"
#include "stdint.h"

#define torque_constant2Nm 0.741 // 转矩常数 单位 N*M/A
#define torque_constant2mNm 741 // 转矩常数 单位 mN*M/A

#define A_Columns  2 //列
#define A_Rows     2 //行

#define B_Columns  1 //列
#define B_Rows     2 //行

typedef struct
{
	fp32 error;
	fp32 motor_rmp;
}state_variable;//选取的状态变量

typedef struct
{
	fp32 K1;
	fp32 K2;

	fp32 Fc;//前馈系数
}match_variable;//选取的匹配参量

//typedef struct
//{
//	matrix_f32 A;//状态矩阵
//	matrix_f32 B;//输入矩阵
//	matrix_f32 C;//输出矩阵
//	matrix_f32 D;//直接传递矩阵

//	/*代价系数*/
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

	float  torque;//力矩
	float  last_rad;//上一次的输出
	float  rotational_inertia;//转动惯量
	float  reciprocal2r_i;//转动惯量的倒数

	CAN_Ctrl* motor;
};























#endif