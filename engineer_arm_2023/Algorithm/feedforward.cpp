#include "feedforward.h"

void FeedforwardController_Init(FFC &vFFC,float &T_Alpha,float &T_beta)
{
	vFFC.lastRin = 0;
	vFFC.perrRin = 0;
	vFFC.rin = 0;
	vFFC.ffc_p.Alpha = T_Alpha;
	vFFC.ffc_p.beta = T_beta;
}

/*实现前馈控制器*/
float FeedforwardController(FFC vFFC)
{
	float result;
	result = vFFC.ffc_p.Alpha * (vFFC.rin - vFFC.lastRin) + vFFC.ffc_p.beta * (vFFC.rin - 2 * vFFC.lastRin + vFFC.perrRin);
	vFFC.perrRin = vFFC.lastRin;
	vFFC.lastRin = vFFC.rin;
	return result;
}

void Lead_lag_control_Init(Lead_lag_control &llc,float &pole,float &zero)
{
	llc.pole_point = pole;
	llc.zero_point = zero;

	llc.lastout = 0;
	llc.out = 0;

	llc.lastrin = 0;
	llc.rin = 0;
}

/*超前滞后补偿器*/
float LLC_calc(Lead_lag_control& llc, float& in)
{
	llc.out = (llc.out - llc.lastout + llc.zero_point * llc.rin + llc.rin - llc.lastrin) * llc.pole_point;

	llc.lastout = llc.out;
	llc.lastrin = llc.rin;

	return llc.out;
}
