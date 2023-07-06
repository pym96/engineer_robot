#ifndef __PID_H
#define __PID_H

#include "dev_system.h"

#define useSimplePID 

#define rad_format(Ang) loop_float_constrain((Ang), -PI, PI)
#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }
		
#define LimitBand(input, max, min)   \
    {                                \
        if (input > max)             \
        {                            \
            input = max;             \
        }                            \
        else if (input < min)        \
        {                            \
            input = min;             \
        }                            \
    }

#define TIMERPERCYCLE TIMERPER_1MS

#define TIMERPER_1US	1000000	
#define TIMERPER_1MS	1000

#ifndef useSimplePID
/**********论域宏定义**********/
#define NB                -0.3f
#define NM                -0.2f
#define NS                -0.1f
#define ZO                 0.0f
#define PS                 0.1f
#define PM                 0.2f
#define PB                 0.3f

/***********Kp规则表***********/
static const float Kp_Rules_Table[7][7]= 
{
  PB, PB, PM, PM, PS, ZO, ZO,
  PB, PB, PM, PS, PS, ZO, NS,
  PM, PM, PM, PS, ZO, NS, NS,
  PM, PM, PS, ZO, NS, NM, NM,
  PS, PS, ZO, NS, NS, NM, NM,
  PS, ZO, NS, NM, NM, NM, NB,
  ZO, ZO, NM, NM, NM, NB, NB
};
 
/***********Ki规则表***********/
static const float Ki_Rules_Table[7][7]=
{
  NB, NB, NM, NM, NS, ZO, ZO,
  NB, NB, NM, NS, NS, ZO, ZO,
  NB, NM, NS, NS, ZO, PS, PS,
  NM, NM, NS, ZO, PS, PM, PM, 
  NM, NS, ZO, PS, PS, PM, PB,
  ZO, ZO, PS, PS, PM, PB, PB,
  ZO, ZO, PS, PM, PM, PB, PB
};
 
/***********Kd规则表***********/
static const float Kd_Rules_Table[7][7]=
{
  PS, NS, NB, NB, NB, NM, PS,
  PS, NS, NB, NM, NM, NS, ZO,
  ZO, NS, NM, NM, NS, NS, ZO,
  ZO, NS, NS, NS, NS, NS, ZO,
  ZO, ZO, ZO, ZO, ZO, ZO, ZO,
  PB, NS, PS, PS, PS, PS, PB,
  PB, PM, PM, PM, PS, PS, PB
};
		
typedef enum
{
	INIT = 0x00,
	POSITION,
	DELTA
}PidMode;

typedef struct
{
	uint32_t sampleTime;
	uint32_t nowTime;
	uint32_t lastTime;
	uint32_t passTime;

}PidTimerDef;

typedef struct
{
	bool trapezoidalintegral;  //梯形积分开关
	bool processVariable;      //微分先行开关
	bool fuzzyController;      //模糊PID开关
	bool disturbanceRejection; //DR_PID开关
}PidChangerTypeDef;

typedef struct
{
	float errorRate;
	
	float deltaKp;
	float deltaKi;
	float deltaKd;
	
	uint8_t IndexE[2];
	uint8_t IndexER[2];
	uint8_t MembershipE[2];
	uint8_t MembershipER[2];
}FuzzyPidTypeDef;

typedef struct PidTypeDef
{
	PidMode mode;
	
  float Kp;
	float Ki;
	float Kd;
	float wc;         //启用DR_PID时，wc决定闭环响应速度，kp决定抗扰性能，因此先调wc，再调kp
									 //速度环wc > 1, 角度环wc < 1。若系统超调，可以通过在输入端增加低通滤波器来消除超调。
	float set;
	float ref;
	float lastRef;
	float perRef;
	float error[3];
	float Dbuf[3];
	
	float out;
	float P_out;
	float I_out;
	float D_out;
	
	float max_Iout;
	float max_out;
	float band_I;
	float dead_band;
	float filter_D;       //越小效果越好，但系统灵敏度会下降。反之，效果变差，灵敏度提高。0为关闭惯性一阶惯性系统。
	float pvCoefficient;  //微分先行滤波系数
	float minInterval;    //变速积分区间
	float maxInterval;
	
	PidChangerTypeDef PidChanger;
	FuzzyPidTypeDef fuzzyPID;
	
	void (*vParmaInitFun)( struct PidTypeDef *pid, const float PID_Coefficient[3], float max_Iout, 
												 float max_out, float band_I, float dead_band, float pvCoefficient,
												 float filter_D, float	minInterval, float maxInterval, float Cycle );
	void (*vChangerInitFun)( struct PidTypeDef *pid, bool tlBool, bool pvBool, bool fcBool, bool drBool );
	void (*vClearFun)( struct PidTypeDef *pid );
	float (*fCalcFun)( struct PidTypeDef *pid, float set, float ref, PidMode mode );
	PidMode (*pmGetFun)( struct PidTypeDef *pid );
	PidChangerTypeDef (*pcGetFun)( struct PidTypeDef *pid );
	
	#ifndef useFreeRTOS
	PidTimerDef pidCycle;
	#endif
	
}PidTypeDef;

void pidFunPointerInit( PidTypeDef *pid );
void PIDChangerInit(PidTypeDef *pid, bool tlBool, bool pvBool, bool fcBool, bool drBool );
void PID_Clear( PidTypeDef *pid );
void PID_Init( PidTypeDef *pid, const float PID_Coefficient[3], float max_Iout, 
	             float max_out, float band_I, float dead_band, float pvCoefficient,
							 float filter_D, float	minInterval, float maxInterval, float Cycle );
float PID_Calc( PidTypeDef *pid, float set, float ref, PidMode mode );

#else							 
enum PID_MODE
{
    PID_POSITION = 0,//位置式PID
    PID_DELTA//增量试
};

typedef struct
{
    uint8_t mode;
    //PID 三参数
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;  //最大输出
    fp32 max_iout; //最大积分输出

    fp32 set;
    fp32 fdb;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  //微分项 0最新 1上一次 2上上次
    fp32 error[3]; //误差项 0最新 1上一次 2上上次

} PidTypeDef;
void PID_Init(PidTypeDef *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout);
fp32 PID_Calc(PidTypeDef *pid, fp32 ref, fp32 set,uint8_t mode);
void PID_clear(PidTypeDef *pid);

#endif

#endif
