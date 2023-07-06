#ifndef _ADAPTIVE_EXERCISE_H
#define _ADAPTIVE_EXERCISE_H

/*
*	@自适应非线性控制器
*/

#include "dev_system.h"
#include "math.h"

#define CONTROLPREIOD 1 //控制时间 ms

#ifndef torque_constant2Nm
#define torque_constant2Nm 0.741f // 转矩常数 单位 N*M/A
#endif // !torque_constant2Nm



#ifndef Motor_Ecd_to_Rad
#define Motor_Ecd_to_Rad 0.000766990394f //      2*  PI  /8191
#endif // !Motor_Ecd_to_Rad


class Adapt_controller
{

public:
	Adapt_controller(const fp32& real_gyor, const fp32& real_theta)
	{
		/*
		*	前端质量 1.5kg 前段长度 200mm
		*	后端质量 2.5kg 后端长度 130mm
		*	J=((m1*L1^2)-(m2*L2^2))/3 
		*/
		this->J = 0.005917;

		this->control_param.k1 = 8;
		this->control_param.k2 = 5;

		this->real_gyor = real_gyor / 60;//rad/s
		//this->real_theta = real_theta * Motor_Ecd_to_Rad;//rad
		this->real_theta=0;

	}

	fp32 out_put_torque(fp32& Theta1_d);
	void updata(const fp32& real_gyor, const fp32& real_theta);
private:
	struct control_param_t//控制系数
	{
		fp32 k1;
		fp32 k2;

		fp32 error1;
		fp32 error2;


		control_param_t()
		{
			this->k1 = 0;
			this->k2 = 0;
			this->error1 = 0;
			this->error2 = 0;
		}
	};

	struct destination_param_t//目标值参数
	{
		fp32 Theta1_d[2];
		fp32 dot_Theta1_d[2];
		fp32 dot_dot_Theta1_d;

		destination_param_t()
		{
			Theta1_d[0] = 0;
			Theta1_d[1] = 0;

			dot_Theta1_d[0] = 0;
			dot_Theta1_d[1] = 0;

			dot_dot_Theta1_d = 0;
		}
	};

	control_param_t control_param;//控制系数
	destination_param_t destination_param;//目标值参数

	inline void calc_dot();
	inline void calc_dot_dot();

	inline void calc_error1(fp32& Theta1_d);
	inline void calc_error2();

	fp32 J;//转动惯量

	fp32 current_set;
	fp32 torque;

	fp32 real_gyor;
	fp32 real_theta;

};



extern Adapt_controller adapt_controller;


















#endif // !_ADAPTIVE_EXERCISE_H
