#include "algorithm_pid.h"

/*
******************************************************************************
  * @attention
  *
  * 调用此库时请注意:
  * # 带有操作系统时请使用绝对延时，如vTaskDelayUntil
  * # 不使用操作系统请提供一个基准定时器，周期为1ms或者1us
  * # 更改定时器周期后，请更改pid.h中 TIMERPERCYCLE
	* # 切换操作模式请在sys.h中定义 useFreeRTOS 为 Disable或Enable
  * # 使用PID时请先执行pidFunPointerInit
	*
  * <h2><center>&copy; Copyright (c) 2022 - ~, USC	MA Development Team.
  * All rights reserved.</center></h2>
  ******************************************************************************
  */

#ifndef useSimplePID

float PID_Position_Calc(PidTypeDef* pid, float set, float ref);
float PID_Delta_Calc(PidTypeDef* pid, float set, float ref);
static float variableIntegralCoefficientCalc(float* maxInterval, float* minInterval, float* thiserror);

static void Membership_Calc(PidTypeDef* pid);
static void Fuzzification(PidTypeDef* pid, float set, float ref);
static void Defuzzification(PidTypeDef* pid);

static PidMode getPIDWorkMode(PidTypeDef* pid);
static PidChangerTypeDef getPIDChangerState(PidTypeDef* pid);

/**
  * @brief          pid函数指针初始化函数
  * @param[in]      *pid: 结构体指针
  * @retval         none
  */
void pidFunPointerInit(PidTypeDef* pid)
{
	if (pid == NULL)
	{
		return;
	}
	pid->vParmaInitFun = PID_Init;
	pid->vChangerInitFun = PIDChangerInit;
	pid->vClearFun = PID_Clear;
	pid->fCalcFun = PID_Calc;

	pid->pmGetFun = getPIDWorkMode;
	pid->pcGetFun = getPIDChangerState;
}

/**
  * @brief          PID初始化
	* @param[in]      pid: PID结构数据指针
  * @param[in]      PID_Coefficient[3]: PID三参数，依次为Kp，Ki，Kd。当启用DR_PID时，最后一个参数不使用
	* @param[in]      max_Iout: 最大积分输出
	* @param[in]      max_out: PID最大输出
	* @param[in]      band_I: 积分分离阈值
	* @param[in]      dead_band: pid死区
	* @param[in]      filter_D: 微分一阶惯性系统系数。此值越小效果越好，但系统灵敏度会下降。
																	 反之，效果变差，灵敏度提高。0为不使用惯性系统。
	* @param[in]      pvCoefficient: 微分先行滤波系数
	* @param[in]      minInterval: 变速积分区间下限
	* @param[in]      maxInterval: 变速积分区间上限
	* @param[in]      Cycle: pid控制周期 单位:s
  * @retval         none
  */
void PID_Init(PidTypeDef* pid, const float PID_Coefficient[3], float max_Iout,
	float max_out, float band_I, float dead_band, float pvCoefficient,
	float filter_D, float	minInterval, float maxInterval, float Cycle)
{
	if (pid == NULL || PID_Coefficient == NULL)
	{
		return;
	}

	pid->max_Iout = max_Iout;
	pid->max_out = max_out;
	pid->band_I = band_I;
	pid->dead_band = dead_band;
	pid->filter_D = pid->filter_D;
	pid->pvCoefficient = pvCoefficient;
	pid->minInterval = minInterval;
	pid->maxInterval = maxInterval;

#ifndef useFreeRTOS
	pid->pidCycle.sampleTime = Cycle * TIMERPERCYCLE;
#endif

	if (pid->PidChanger.disturbanceRejection == false)
	{
		pid->Kp = PID_Coefficient[0];
		pid->Ki = PID_Coefficient[1] * Cycle;
		pid->Kd = PID_Coefficient[2] / Cycle;
	}
	else if (pid->PidChanger.disturbanceRejection == true)
	{
		pid->Kp = PID_Coefficient[0];
		pid->wc = PID_Coefficient[1];

		pid->Kp = 2 * pid->Kp * pid->wc;
		pid->Ki = pid->wc * pid->wc * pid->Kp * Cycle;
		pid->Kd = pid->Kp / Cycle;
	}

}

/**
  * @brief          PID功能开关函数
  * @param[in]      pid: PID结构数据指针
  * @param[in]      tlBool: 梯形积分开关
  * @param[in]      pvBool: 微分先行开关
  * @param[in]      drBool: DR_PID开关
  * @retval         none
  */
void PIDChangerInit(PidTypeDef* pid, bool tlBool, bool pvBool, bool fcBool, bool drBool)
{
	if (pid == NULL)
	{
		return;
	}
	pid->PidChanger.trapezoidalintegral = tlBool;
	pid->PidChanger.processVariable = pvBool;
	pid->PidChanger.fuzzyController = fcBool;
	pid->PidChanger.disturbanceRejection = drBool;
}

/**
  * @brief          PID计算函数
  * @param[in]      pid: PID结构数据指针
  * @param[in]      set: 设定值
  * @param[in]      ref: 反馈值
  * @param[in]      mode: PID模式 INIT为初始模式，POSITION为位置式，DELTA为增量式
  * @retval         PID输出值
  */
float PID_Calc(PidTypeDef* pid, float set, float ref, PidMode mode)
{
#ifdef useFreeRTOS
	if (pid == NULL)
	{
		return 0.0f;
	}

	if (mode == POSITION)
	{
		return PID_Position_Calc(pid, set, ref);
	}
	else if (mode == DELTA)
	{
		return PID_Delta_Calc(pid, set, ref);
	}
	else
	{
		return 0;
	}

#else
	pid->pidCycle.nowTime = millis();
	pid->pidCycle.passTime = pid->pidCycle.nowTime - pid->pidCycle.lastTime;
	if (pid->pidCycle.passTime >= pid->pidCycle.sampleTime)
	{
		if (mode == POSITION)
		{
			return PID_Position_Calc(pid, set, ref);
		}
		else if (mode == DELTA)
		{
			return PID_Delta_Calc(pid, set, ref);
		}
		else
		{
			pid->pidCycle.lastTime = millis();
			return 0;
		}
	}
	else
	{
		pid->pidCycle.lastTime = millis();
		return 0;
	}
#endif
}

/**
  * @brief          位置式PID
  * @param[in]      pid: PID结构数据指针
  * @param[in]      set: 设定值
  * @param[in]      ref: 反馈值
  * @retval         位置式PID输出值
  */
float PID_Position_Calc(PidTypeDef* pid, float set, float ref)
{
	pid->set = set;
	pid->ref = ref;

	pid->error[0] = set - ref;

	if (pid->PidChanger.fuzzyController == true)
	{
		Fuzzification(pid, set, ref);
		Defuzzification(pid);
	}

	if (ABS(pid->error[0]) < pid->dead_band) //PID死区
	{
		PID_Clear(pid);
#ifndef useFreeRTOS
		pid->pidCycle.lastTime = millis();
#endif
		return 0.0f;
	}
	else
	{
		pid->P_out = pid->Kp * pid->error[0];
		//积分
		if (ABS(pid->error[0]) < pid->band_I) //积分分离
		{
			if (pid->PidChanger.trapezoidalintegral == false)//没有开启梯形积分，使用抗饱和积分和变速积分
			{
				//变速积分
				if (pid->out > pid->max_out) {
					if (pid->error[0] < 0) {
						pid->I_out += pid->Ki * pid->error[0]\
							* variableIntegralCoefficientCalc(&pid->maxInterval, &pid->minInterval, &pid->error[0]);
					}
				}
				else {
					pid->I_out += pid->Ki * pid->error[0]\
						* variableIntegralCoefficientCalc(&pid->maxInterval, &pid->minInterval, &pid->error[0]);
				}
			}
			else if (pid->PidChanger.trapezoidalintegral == true)//开启梯形积分，使用抗饱和积分，变速积分和梯形积分
			{
				//梯形积分
				if (pid->out > pid->max_out) {
					if (pid->error[0] < 0) {
						pid->I_out += pid->Ki * (pid->error[0] + pid->error[1]) / 2\
							* variableIntegralCoefficientCalc(&pid->maxInterval, &pid->minInterval, &pid->error[0]);
					}
				}
				else {
					pid->I_out += pid->Ki * (pid->error[0] + pid->error[1]) / 2\
						* variableIntegralCoefficientCalc(&pid->maxInterval, &pid->minInterval, &pid->error[0]);
				}
			}
			else//状态保护
			{
				pid->I_out += pid->Ki * pid->error[0];
			}
		}
		else//积分分离
		{
			pid->I_out = 0.0f;
		}
		
		//微分
		if (pid->PidChanger.processVariable == false)//没有开启微分先行，使用不完全微分
		{
			pid->D_out = pid->Kd * (1 - pid->filter_D) * pid->error[0] + pid->filter_D * pid->D_out
				- pid->Kd * (1 - pid->filter_D) * pid->error[1];
		}
		else if (pid->PidChanger.processVariable == true)//开启微分先行，使用微分先行
		{
			pid->D_out = pid->pvCoefficient * pid->Kd / (pid->pvCoefficient * pid->Kd + pid->Kp) * pid->D_out\
				+ (pid->Kd + pid->Kp) / (pid->pvCoefficient * pid->Kd + pid->Kp) * pid->ref\
				+ pid->Kd / (pid->pvCoefficient * pid->Kd + pid->Kp) * pid->lastRef;
		}
		else//状态保护
		{
			pid->D_out = pid->Kd * (pid->error[0] - pid->error[1]);
		}

		pid->Dbuf[0] = pid->D_out;

		LimitMax(pid->I_out, pid->max_Iout);

		pid->out = pid->P_out + pid->I_out + pid->D_out;

		pid->lastRef = pid->ref;

		pid->error[1] = pid->error[0];
		pid->lastRef = pid->ref;

		pid->Dbuf[2] = pid->Dbuf[1];
		pid->Dbuf[1] = pid->Dbuf[0];

		if (pid->out >= pid->max_out) //输出限幅
		{
#ifndef useFreeRTOS
			pid->pidCycle.lastTime = millis();
#endif
			return pid->max_out;
		}
		else
		{
#ifndef useFreeRTOS
			pid->pidCycle.lastTime = millis();
#endif
			return pid->out;
		}
	}
}

/**
  * @brief          增量式PID
  * @param[in]      pid: PID结构数据指针
  * @param[in]      set: 设定值
  * @param[in]      ref: 反馈值
  * @retval         增量式PID输出值
  */
float PID_Delta_Calc(PidTypeDef* pid, float set, float ref)
{
	pid->set = set;
	pid->ref = ref;

	pid->error[0] = set - ref;

	if (pid->PidChanger.fuzzyController == true)
	{
		Fuzzification(pid, set, ref);
		Defuzzification(pid);
	}

	if (ABS(pid->error[0]) < pid->dead_band)//PID����
	{
		PID_Clear(pid);
#ifndef useFreeRTOS
		pid->pidCycle.lastTime = millis();
#endif
		return 0.0f;
	}
	else
	{
		pid->P_out = pid->Kp * (pid->error[0] - pid->error[1]);

		if (ABS(pid->error[0]) < pid->band_I)//���ַ���
		{
			if (pid->PidChanger.trapezoidalintegral == false)//û�п������λ��֣�ʹ�ÿ����ͻ��ֺͱ��ٻ���
			{
				if (pid->out > pid->max_out) {
					if (pid->error[0] < 0) {
						pid->I_out = pid->Ki * pid->error[0]\
							* variableIntegralCoefficientCalc(&pid->maxInterval, &pid->minInterval, &pid->error[0]);
					}
				}
				else {
					pid->I_out = pid->Ki * pid->error[0]\
						* variableIntegralCoefficientCalc(&pid->maxInterval, &pid->minInterval, &pid->error[0]);
				}
			}
			else if (pid->PidChanger.trapezoidalintegral == true)//�������λ��֣�ʹ�ÿ����ͻ��֣����ٻ��ֺ����λ���
			{
				if (pid->out > pid->max_out) {
					if (pid->error[0] < 0) {
						pid->I_out += pid->Ki * (pid->error[0] + pid->error[1]) / 2\
							* variableIntegralCoefficientCalc(&pid->maxInterval, &pid->minInterval, &pid->error[0]);
					}
				}
				else {
					pid->I_out += pid->Ki * (pid->error[0] + pid->error[1]) / 2\
						* variableIntegralCoefficientCalc(&pid->maxInterval, &pid->minInterval, &pid->error[0]);
				}
			}
			else
			{
				pid->I_out = pid->Ki * pid->error[0];
			}
		}
		else
		{
			pid->I_out = 0.0f;
		}

		if (pid->PidChanger.processVariable == false)//û�п���΢�����У�ʹ�ò���ȫ΢��
		{
			pid->D_out = pid->Kd * (1 - pid->filter_D) * (pid->error[0] - 2 * pid->error[1] + pid->error[2])\
				+ pid->filter_D * (pid->Dbuf[1] - pid->Dbuf[2]);
		}
		else if (pid->PidChanger.processVariable == true)//����΢�����У�ʹ��΢������
		{
			pid->D_out = pid->pvCoefficient * pid->Kd / (pid->pvCoefficient * pid->Kd + pid->Kp) * (pid->Dbuf[1] - pid->Dbuf[2])\
				+ (pid->Kd + pid->Kp) / (pid->pvCoefficient * pid->Kd + pid->Kp) * (pid->ref - pid->lastRef)\
				+ pid->Kd / (pid->pvCoefficient * pid->Kd + pid->Kp) * (pid->lastRef - pid->perRef);
		}
		else//״̬����
		{
			pid->D_out = pid->Kd * (pid->error[0] - 2 * pid->error[1] + pid->error[2]);
		}

		LimitMax(pid->I_out, pid->max_Iout);

		pid->Dbuf[0] = pid->D_out;

		pid->out += pid->P_out + pid->I_out + pid->D_out;

		pid->error[2] = pid->error[1];
		pid->error[1] = pid->error[0];

		pid->perRef = pid->lastRef;
		pid->lastRef = pid->ref;

		pid->Dbuf[2] = pid->Dbuf[1];
		pid->Dbuf[1] = pid->Dbuf[0];

		if (pid->out > pid->max_out)//PID����޷�
		{
#ifndef useFreeRTOS
			pid->pidCycle.lastTime = millis();
#endif
			return pid->max_out;
		}
		else
		{
#ifndef useFreeRTOS
			pid->pidCycle.lastTime = millis();
#endif
			return pid->out;
		}
	}
}

/**
  * @brief          pid输出清除
  * @param[in]      pid: PID结构数据指针
  * @retval         none
  */
void PID_Clear(PidTypeDef* pid)
{
	if (pid == NULL)
	{
		return;
	}

	pid->set = pid->ref = pid->lastRef = pid->perRef = 0.0f;
	pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
	pid->out = pid->P_out = pid->I_out = pid->D_out = 0.0f;
	pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;

}

/**
  * @brief          获取pid工作模式
  * @param[in]      pid: PID结构数据指针
  * @retval         pid工作模式
  */
static PidMode getPIDWorkMode(PidTypeDef* pid)
{
	if (pid == NULL)
	{
		return INIT;
	}

	return pid->mode;
}

/**
  * @brief          获取pid开关变量
  * @param[in]      pid: PID结构数据指针
  * @retval         PidChanger结构体
  */
static PidChangerTypeDef getPIDChangerState(PidTypeDef* pid)
{
	if (pid == NULL)
	{
		static PidChangerTypeDef PIDChangerNULL;

		return PIDChangerNULL;
	}

	return pid->PidChanger;
}

/**
  * @brief          变速积分系数计算函数
  * @param[in]      *maxInterval: 积分区间上限
  * @param[in]      *minInterval: 积分区间下限
  * @param[in]      *thiserror: 	本次误差
  * @retval         变速积分系数
  */
static float variableIntegralCoefficientCalc(float* maxInterval, float* minInterval, float* thiserror)
{
	if (maxInterval == NULL || minInterval == NULL || thiserror == NULL)
	{
		return 0;
	}
	if (ABS(*thiserror) <= *minInterval)
	{
		return 1.0;
	}
	else if (ABS(*thiserror) > *maxInterval)
	{
		return 0;
	}
	else
	{
		return ((*maxInterval - ABS(*thiserror)) / (*maxInterval - *minInterval));
	}
}

/**
  * @brief          清晰化处理
  * @param[in]      pid: pid结构数据指针
  * @retval         none
  */
static void Defuzzification(PidTypeDef* pid)
{
	pid->fuzzyPID.deltaKp = (Kp_Rules_Table[pid->fuzzyPID.IndexE[0]][pid->fuzzyPID.IndexER[0]] * pid->fuzzyPID.MembershipE[0] * pid->fuzzyPID.MembershipER[0]) +
		(Kp_Rules_Table[pid->fuzzyPID.IndexE[0]][pid->fuzzyPID.IndexER[1]] * pid->fuzzyPID.MembershipE[0] * pid->fuzzyPID.MembershipER[1]) +
		(Kp_Rules_Table[pid->fuzzyPID.IndexE[1]][pid->fuzzyPID.IndexER[0]] * pid->fuzzyPID.MembershipE[1] * pid->fuzzyPID.MembershipER[0]) +
		(Kp_Rules_Table[pid->fuzzyPID.IndexE[1]][pid->fuzzyPID.IndexER[1]] * pid->fuzzyPID.MembershipE[1] * pid->fuzzyPID.MembershipER[1]);

	pid->fuzzyPID.deltaKi = (Ki_Rules_Table[pid->fuzzyPID.IndexE[0]][pid->fuzzyPID.IndexER[0]] * pid->fuzzyPID.MembershipE[0] * pid->fuzzyPID.MembershipER[0]) +
		(Ki_Rules_Table[pid->fuzzyPID.IndexE[0]][pid->fuzzyPID.IndexER[1]] * pid->fuzzyPID.MembershipE[0] * pid->fuzzyPID.MembershipER[1]) +
		(Ki_Rules_Table[pid->fuzzyPID.IndexE[1]][pid->fuzzyPID.IndexER[0]] * pid->fuzzyPID.MembershipE[1] * pid->fuzzyPID.MembershipER[0]) +
		(Ki_Rules_Table[pid->fuzzyPID.IndexE[1]][pid->fuzzyPID.IndexER[1]] * pid->fuzzyPID.MembershipE[1] * pid->fuzzyPID.MembershipER[1]);

	pid->fuzzyPID.deltaKd = (Kd_Rules_Table[pid->fuzzyPID.IndexE[0]][pid->fuzzyPID.IndexER[0]] * pid->fuzzyPID.MembershipE[0] * pid->fuzzyPID.MembershipER[0]) +
		(Kd_Rules_Table[pid->fuzzyPID.IndexE[0]][pid->fuzzyPID.IndexER[1]] * pid->fuzzyPID.MembershipE[0] * pid->fuzzyPID.MembershipER[1]) +
		(Kd_Rules_Table[pid->fuzzyPID.IndexE[1]][pid->fuzzyPID.IndexER[0]] * pid->fuzzyPID.MembershipE[1] * pid->fuzzyPID.MembershipER[0]) +
		(Kd_Rules_Table[pid->fuzzyPID.IndexE[1]][pid->fuzzyPID.IndexER[1]] * pid->fuzzyPID.MembershipE[1] * pid->fuzzyPID.MembershipER[1]);

	pid->Kp = pid->Kp + pid->fuzzyPID.deltaKp * 0.005f;
	pid->Ki = pid->Ki + pid->fuzzyPID.deltaKi * 0.000f;
	pid->Kd = pid->Kd + pid->fuzzyPID.deltaKd * 0.005f;

	LimitBand(pid->Kp, 20.0f, -20.0f);
	LimitBand(pid->Kd, 20.0f, -20.0f);
}

/**
  * @brief          模糊化处理
  * @param[in]      pid: pid结构数据指针
  * @param[in]      Set: 设定值
  * @param[in]      Ref: 反馈值
  * @retval         none
  */
static void Fuzzification(PidTypeDef* pid, float set, float ref)
{
	pid->fuzzyPID.errorRate = pid->error[0] - pid->error[1];
	Membership_Calc(pid);
}

/**
  * @brief          隶属度计算函数
  * @param[in]      pid: pid结构数据指针
  * @retval         none
  */
static void Membership_Calc(PidTypeDef* pid)
{
	if (pid->error[0] < NB)
	{
		pid->fuzzyPID.IndexE[0] = 0;
		pid->fuzzyPID.IndexE[1] = 0;
		pid->fuzzyPID.MembershipE[0] = 1;
		pid->fuzzyPID.MembershipE[1] = 0;
	}
	else if ((pid->error[0] >= NB) && (pid->error[0] < NM))
	{
		pid->fuzzyPID.IndexE[0] = 0;
		pid->fuzzyPID.IndexE[1] = 1;
		pid->fuzzyPID.MembershipE[0] = (NM - pid->error[0]) / (NM - NB);
		pid->fuzzyPID.MembershipE[1] = 1 - pid->fuzzyPID.MembershipE[0];
	}
	else if ((pid->error[0] >= NM) && (pid->error[0] < NS))
	{
		pid->fuzzyPID.IndexE[0] = 1;
		pid->fuzzyPID.IndexE[1] = 2;
		pid->fuzzyPID.MembershipE[0] = (NS - pid->error[0]) / (NS - NM);
		pid->fuzzyPID.MembershipE[1] = 1 - pid->fuzzyPID.MembershipE[0];
	}
	else if ((pid->error[0] >= NS) && (pid->error[0] < ZO))
	{
		pid->fuzzyPID.IndexE[0] = 2;
		pid->fuzzyPID.IndexE[1] = 3;
		pid->fuzzyPID.MembershipE[0] = (ZO - pid->error[0]) / (ZO - NS);
		pid->fuzzyPID.MembershipE[1] = 1 - pid->fuzzyPID.MembershipE[0];
	}
	else if ((pid->error[0] >= ZO) && (pid->error[0] < PS))
	{
		pid->fuzzyPID.IndexE[0] = 3;
		pid->fuzzyPID.IndexE[1] = 4;
		pid->fuzzyPID.MembershipE[0] = (PS - pid->error[0]) / (PS - ZO);
		pid->fuzzyPID.MembershipE[1] = 1 - pid->fuzzyPID.MembershipE[0];
	}
	else if ((pid->error[0] >= PS) && (pid->error[0] < PM))
	{
		pid->fuzzyPID.IndexE[0] = 4;
		pid->fuzzyPID.IndexE[1] = 5;
		pid->fuzzyPID.MembershipE[0] = (PM - pid->error[0]) / (PM - PS);
		pid->fuzzyPID.MembershipE[1] = 1 - pid->fuzzyPID.MembershipE[0];
	}
	else if ((pid->error[0] >= PM) && (pid->error[0] < PB))
	{
		pid->fuzzyPID.IndexE[0] = 5;
		pid->fuzzyPID.IndexE[1] = 6;
		pid->fuzzyPID.MembershipE[0] = (PB - pid->error[0]) / (PB - PM);
		pid->fuzzyPID.MembershipE[1] = 1 - pid->fuzzyPID.MembershipE[0];
	}
	else if (pid->error[0] >= PB)
	{
		pid->fuzzyPID.IndexE[0] = 6;
		pid->fuzzyPID.IndexE[1] = 6;
		pid->fuzzyPID.MembershipE[0] = 0;
		pid->fuzzyPID.MembershipE[1] = 1;
	}

	if (pid->fuzzyPID.errorRate < NB)
	{
		pid->fuzzyPID.IndexER[0] = 0;
		pid->fuzzyPID.IndexER[1] = 0;
		pid->fuzzyPID.MembershipER[0] = 1;
		pid->fuzzyPID.MembershipER[1] = 0;
	}
	else if ((pid->fuzzyPID.errorRate >= NB) && (pid->fuzzyPID.errorRate < NM))
	{
		pid->fuzzyPID.IndexER[0] = 0;
		pid->fuzzyPID.IndexER[1] = 1;
		pid->fuzzyPID.MembershipER[0] = (NM - pid->fuzzyPID.errorRate) / (NM - NB);
		pid->fuzzyPID.MembershipER[1] = 1 - pid->fuzzyPID.MembershipER[0];
	}
	else if ((pid->fuzzyPID.errorRate >= NM) && (pid->fuzzyPID.errorRate < NS))
	{
		pid->fuzzyPID.IndexER[0] = 1;
		pid->fuzzyPID.IndexER[1] = 2;
		pid->fuzzyPID.MembershipER[0] = (NS - pid->fuzzyPID.errorRate) / (NS - NM);
		pid->fuzzyPID.MembershipER[1] = 1 - pid->fuzzyPID.MembershipER[0];
	}
	else if ((pid->fuzzyPID.errorRate >= NS) && (pid->fuzzyPID.errorRate < ZO))
	{
		pid->fuzzyPID.IndexER[0] = 2;
		pid->fuzzyPID.IndexER[1] = 3;
		pid->fuzzyPID.MembershipER[0] = (ZO - pid->fuzzyPID.errorRate) / (ZO - NS);
		pid->fuzzyPID.MembershipER[1] = 1 - pid->fuzzyPID.MembershipER[0];
	}
	else if ((pid->fuzzyPID.errorRate >= ZO) && (pid->fuzzyPID.errorRate < PS))
	{
		pid->fuzzyPID.IndexER[0] = 3;
		pid->fuzzyPID.IndexER[1] = 4;
		pid->fuzzyPID.MembershipER[0] = (PS - pid->fuzzyPID.errorRate) / (PS - ZO);
		pid->fuzzyPID.MembershipER[1] = 1 - pid->fuzzyPID.MembershipER[0];
	}
	else if ((pid->fuzzyPID.errorRate >= PS) && (pid->fuzzyPID.errorRate < PM))
	{
		pid->fuzzyPID.IndexER[0] = 4;
		pid->fuzzyPID.IndexER[1] = 5;
		pid->fuzzyPID.MembershipER[0] = (PM - pid->fuzzyPID.errorRate) / (PM - PS);
		pid->fuzzyPID.MembershipER[1] = 1 - pid->fuzzyPID.MembershipER[0];
	}
	else if ((pid->fuzzyPID.errorRate >= PM) && (pid->fuzzyPID.errorRate < PB))
	{
		pid->fuzzyPID.IndexER[0] = 5;
		pid->fuzzyPID.IndexER[1] = 6;
		pid->fuzzyPID.MembershipER[0] = (PB - pid->fuzzyPID.errorRate) / (PB - PM);
		pid->fuzzyPID.MembershipER[1] = 1 - pid->fuzzyPID.MembershipER[0];
	}
	else if (pid->fuzzyPID.errorRate >= PB)
	{
		pid->fuzzyPID.IndexER[0] = 6;
		pid->fuzzyPID.IndexER[1] = 6;
		pid->fuzzyPID.MembershipER[0] = 0;
		pid->fuzzyPID.MembershipER[1] = 1;
	}
}

#else

void PID_Init(PidTypeDef* pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout)
{
	if (pid == NULL || PID == NULL)
	{
		return;
	}
	pid->mode = mode;
	pid->Kp = PID[0];
	pid->Ki = PID[1];
	pid->Kd = PID[2];
	pid->max_out = max_out;
	pid->max_iout = max_iout;
	pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
	pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}
//mode : 0��ʾ�����趨ref set ֵ  1����ʾ����errֵ����ͨ��setֵ����
fp32 PID_Calc(PidTypeDef* pid, fp32 ref, fp32 set, uint8_t mode)
{
	if (pid == NULL)
	{
		return 0.0f;
	}

	pid->error[2] = pid->error[1];
	pid->error[1] = pid->error[0];
	pid->set = set;
	pid->fdb = ref;
	if (mode == 0) {
		pid->error[0] = set - ref;
	}
	else if (mode == 1) {
		pid->error[0] = set;
	}
	if (pid->mode == PID_POSITION)
	{
		pid->Pout = pid->Kp * pid->error[0];
		pid->Iout += pid->Ki * pid->error[0];
		pid->Dbuf[2] = pid->Dbuf[1];
		pid->Dbuf[1] = pid->Dbuf[0];
		pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
		pid->Dout = pid->Kd * pid->Dbuf[0];
		LimitMax(pid->Iout, pid->max_iout);
		pid->out = pid->Pout + pid->Iout + pid->Dout;
		LimitMax(pid->out, pid->max_out);
	}
	else if (pid->mode == PID_DELTA)
	{
		pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
		pid->Iout = pid->Ki * pid->error[0];
		pid->Dbuf[2] = pid->Dbuf[1];
		pid->Dbuf[1] = pid->Dbuf[0];
		pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
		pid->Dout = pid->Kd * pid->Dbuf[0];
		pid->out += pid->Pout + pid->Iout + pid->Dout;
		LimitMax(pid->out, pid->max_out);
	}
	return pid->out;
}

void PID_clear(PidTypeDef* pid)
{
	if (pid == NULL)
	{
		return;
	}

	pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
	pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
	pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
	pid->fdb = pid->set = 0.0f;
}

#endif
