#ifndef _KINEEMATICS_TASK_H_ 
#define _KINEEMATICS_TASK_H_


//#include "app_usart7.h"
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif
#include "led.h"	
void Kinematics_task(void *pvParameters);

#ifdef __cplusplus
}
#endif	

#define Biggest(a,b,c) a > b ? (a > c ? a : c) : (b > c ? b : a)

#define Interpolation_scale 3 //插值倍数

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

#ifndef M_PI
	#define M_PI 3.1415926535897932384626433832795
#endif
#define ECD_RANGE 8191

#ifndef RAD2DEG
	#define RAD2DEG 57.324840764331f
#endif
#define ECD_RANGE 8191 

#define Secons2Milliseconds (1000.0f)
#define Milliseconds2microsecond (0.001)
#define PER_CALC      1000.0f

#define Motor_Ecd_to_Rad 2*M_PI/ECD_RANGE

#define RCCHMAX 660 
#define RCCHMIN -660 

#define HEADFRAME 0xff
#define TAILFRAME 0xfe
#define BUFSIZE  6
//typedef arm_matrix_instance_f32 mat_f32;
//typedef arm_matrix_instance_f64 mat_f64;
//typedef arm_matrix_instance_q15 mat_q16;
//typedef arm_matrix_instance_q31 mat_q31;
	
typedef enum
{
	normal=0,
	Over_voltage=8,
	Under_voltage,
	Over_current,
	Over_temperature_mos,
	Over_temperature_Rotro,
	Lost_Message,
	Over_load
}Torque_Motor_state_e;

enum
{
	error =0,
	fault =1,
	success =2,
};

struct GOMotor_t
{
	float POS_;
	float VEL_;
	float W_;
	float Tor_;
	float Tem_;
};

typedef struct
{
	float px;
	float py;
	float pz;
}Pose;

typedef struct
{
	float q1;
	float q2;
	float q3;
	float q4;
}Rad_t;

typedef struct
{
	float c0;
	float c1;
	float c2;
	float c3;
	float c4;
	float c5;
}Quintic_polynomial_trajectory_planning_factors_t;

typedef struct
{
	float c0;
	float c1;
	float c2;
	float c3;
}Cubic_polynomial_trajectory_planning_factors_t;

class Kinematics_base 
{
	public:
		Kinematics_base(){};
		//运动保护
		bool Is_Reach_aim(float *Current_pose,float *Aim_pose,float error);
		//更新目标位姿
//		mat_f32 transl(float &x,float &y,float &z);
//		mat_f32 trotx(float &y_rad);
//		mat_f32 troty(float &y_rad);
//		mat_f32 trotz(float &y_rad);
//		void updata_aim_pose(mat_f32 &aim_pose);
	  void updata_aim_pose(float aim_pose[4][4]);
		//逆运动学解算器
		void IK_slover();
		//正运动学
		void fkine(Pose &pose,Rad_t &rad);
		//多项式插值
		bool polynomial_trajectory_slover(float* rad,float* vel,const int &T,float theat,float dot_theta,float dott_theta,float aim,float dot_aim,float dott_aim);
		bool polynomial_trajectory_slover(float* rad,float* vel,const int &T,float theat,float dot_theta,float aim,float dot_aim);
	
		//线性映射函数
		float linear_map(float value, float in_min, float in_max, float out_min, float out_max);
	private:
//	  mat_f32 Aim_pose;
		
		float Link1;//机械臂连杆1
		float Link2;//机械臂连杆2
		Quintic_polynomial_trajectory_planning_factors_t Qptpf;//五次多项式系数
		Cubic_polynomial_trajectory_planning_factors_t Cptpf;//三次多项式系数
};

class ArmControl_base:public Kinematics_base
{
	public:
		ArmControl_base();
			
		//状态处理任务
		void RCArmContorl();//遥控器处理
		void PCArmContorl();//遥控器处理
		void ArmMoveGroupSolver(float *pose,int Time);//连续动作组解决器
		void ArmControl(float *pose);//关节角度控制
		void Joint0adjust();//GO2电机的矫正
		
		//状态切换
		void ArmModeGoing();//机械臂模式执行
		int Times__;
		
		GOMotor_t *goMotor;//宇树电机
	
		float vel_base;//基本速度
		float JointPose[4];
	
		//存储插值器密度
		
		//存储发送给宇树电机的buf
		uint8_t GOMotorbuf[BUFSIZE];
		//存储解算器的正确性
		bool joint1_slover;
		bool joint2_slover;
		bool joint3_slover;
		bool joint4_slover;
	//存储目标的位置与速度
		float aim_pose[4][10];
		float aim_vel[4][10];	
		
		float GO2adjust;
};

extern ArmControl_base ArmControl;




#endif

