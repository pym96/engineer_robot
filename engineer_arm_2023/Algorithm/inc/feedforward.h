#ifndef _FEEDFORWARD_H__
#define _FEEDFORWARD_H__

#include "dev_system.h"


/*
*		ǰ�����������ڵ�����֪���ֵĸ������ػ��߸�������
*		������Ծأ������ȵ�
*/
/*ǰ������*/
typedef struct
{
	float Alpha;
	float beta;
}feedforward_param_t;

/*����ǰ���������Ľṹ��*/
typedef struct
{
	float rin;
	float lastRin;
	float perrRin;
	feedforward_param_t ffc_p;
}FFC;
/*
*		��ǰ�ͺ󲹳��� Leadand lag control
*			��ǰ��������(p<z<0)�������ϵͳ����Ӧ�ٶ�
*			�ͺ󲹳�����(z<p<0)�������ϵͳ����̬���
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


