#include "lqr.h"

int16_t motor_6020_modle::Out_function(float& rad, float& rad_set)
{
	torque = reciprocal2r_i * (match_K1K2.K1 * (rad_set - motor->Gimbal.Yaw_Measure.angle_ecd * DEG_TO_RAD) + \
							   match_K1K2.K2 * (last_rad - motor->Gimbal.Yaw_Measure.angle_ecd * DEG_TO_RAD));
	last_rad = motor->Gimbal.Yaw_Measure.angle_ecd * DEG_TO_RAD;

	return ((torque / torque_constant2Nm) * 10000);
}