#include "Adaptive_exercise.h"
#include "app_motor.h"
Adapt_controller adapt_controller(CAN_Cmd.Gimbal.Yaw_Measure.speed_rpm, CAN_Cmd.Gimbal.Yaw_Measure.angle_ecd);

uint16_t add=0;
void Adapt_controller::updata(const fp32& real_gyor, const fp32& real_theta)
{
	this->real_gyor = (fp32)real_gyor / 60;//rad/s
	//this->real_theta = (fp32)((real_theta - add) * Motor_Ecd_to_Rad);//rad
	this->real_theta = (fp32)real_theta;
}

int32_t a = 10000;
uint32_t integral_time = 1;
bool switch1 = true;
uint32_t temp_arrange = 1000;
fp32 error_arange=0.024;
fp32 Adapt_controller::out_put_torque(fp32& Theta1_d)
{

	//T=J(dot (wd) + error1 -cos(theta_now)*(1/s){-error2*cos(theta_now)+k2*error2})
	this->destination_param.Theta1_d[1] = this->destination_param.Theta1_d[0];
	this->destination_param.Theta1_d[0] = Theta1_d;

	calc_dot();
	calc_dot_dot();

	calc_error1(Theta1_d);
	calc_error2();

	torque = J * (destination_param.dot_dot_Theta1_d + control_param.k1 * control_param.error2 \
		- control_param.k1 * control_param.k1 * control_param.error1 + control_param.error1 + control_param.k2 * control_param.error2\
		+ control_param.error2 * pow(cos(Theta1_d * RAD_TO_DEG), 2) * integral_time * CONTROLPREIOD);

	if (switch1)
	{
		if (abs(control_param.error1) < error_arange)
		{
			integral_time = 1;
		}
		integral_time++;
	}
	else
	{
		integral_time++;
		if (integral_time > temp_arrange)
		{
			integral_time = temp_arrange;
		}

	}

	current_set = torque / torque_constant2Nm;

	current_set = current_set * a;

	return current_set;
}
inline void Adapt_controller::calc_dot()
{
	this->destination_param.dot_Theta1_d[1] = this->destination_param.dot_Theta1_d[0];
	this->destination_param.dot_Theta1_d[0] = ((this->destination_param.Theta1_d[0] - this->destination_param.Theta1_d[1]) / CONTROLPREIOD);
	//return this->destination_param.dot_Theta1_d[0];
}

inline void Adapt_controller::calc_dot_dot()
{
	this->destination_param.dot_dot_Theta1_d = ((this->destination_param.dot_Theta1_d[0] - this->destination_param.dot_Theta1_d[1]) / CONTROLPREIOD);
	//return this->destination_param.dot_dot_Theta1_d;
}


inline void Adapt_controller::calc_error1(fp32& Theta1_d)
{
	//error1=theta_d-real_theta
	this->control_param.error1 = Theta1_d - (real_theta);
}

inline void Adapt_controller::calc_error2()
{
	//error2=dot(theta_d)+k1*error1-real_gyor
	this->control_param.error2 = this->destination_param.dot_Theta1_d[0] + this->control_param.k1 * this->control_param.error1 - (real_gyor);
}



















