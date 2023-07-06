#ifndef _FEEDFORWARD_H__
#define _FEEDFORWARD_H__

#include "dev_system.h"


/*
*		前馈控制适用于抵消已知部分的干扰力矩或者干扰输入
*		例如惯性矩，重力等等
*/
/*前馈参数*/
typedef struct
{
	float Alpha;
	float beta;
}feedforward_param_t;

/*定义前馈控制器的结构体*/
typedef struct
{
	float rin;
	float lastRin;
	float perrRin;
	feedforward_param_t ffc_p;
}FFC;
/*
*		超前滞后补偿器 Leadand lag control
*			超前补偿器：(p<z<0)可以提高系统的响应速度
*			滞后补偿器：(z<p<0)可以提高系统的稳态误差
*/
typedef struct
{
	float out;
	float lastout;

	float rin;
	float lastrin;

	double pole_point;
	double zero_point;
}Lead_lag_control;


#endif // !1


