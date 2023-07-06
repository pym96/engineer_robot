#include "Kinematics_task.h"
#include "motor_control.h"
#include "Message_Task.h"
#include "math.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app_usart7.h"

static TickType_t xLastWakeTime;
static const TickType_t xDelay1s = pdMS_TO_TICKS(420);
extern Torque_Motor_Ctrl   Arm;
extern MOTOR_send ys_motor;
extern MOTOR_recv yr_motor;

float InitGroup[4]       =			{-9,       0,     -0.6,0};
float AirCatchGroup[4]   =			{-1.5	,       0,		-1.57,0};
float PutBackGroup[4]    =			{-10,       0,		-1.7,0};
float CatchGroundGroup[4]=			{ 1,       0,		  0.57,0};
float CatchMidGroup[4]   =			{-10,       0,		-1.7,0};
float StopGroup[4]       =			{-10,       0,		-1.7,0};
float stop[4] = {2,0,0,0};
	
//机械臂
ArmControl_base Armcontrol;
u8 kinematics_flag = 0;

void Kinematics_task(void *pvParameters)
{
  xLastWakeTime = xTaskGetTickCount();
	
		for(int i=0;i<=3;i++)
		{
			start_motor(i);
			vTaskDelay(pdMS_TO_TICKS(1000));
		}
		
//		for(int i=0;i<3;i++)
//		{
//			ctrl_motor_pv(i,(float)InitGroup[i+1],0);
//			vTaskDelay(pdMS_TO_TICKS(1000));
//		}
		
		ys_motor.id=0; 		
		ys_motor.mode=1;
		ys_motor.T=0;
		ys_motor.W=0;
		ys_motor.Pos=0;
		ys_motor.K_P=0;
		ys_motor.K_W=0;
	
//	while(go2data.go2pos_f <= 1 || go2data.go2pos_f >= 3)
//	{//只有第一个轴位于1-3之间才执行
//		//Armcontrol.ArmControl(stop);
//		vTaskDelay(pdMS_TO_TICKS(10));
//	}
	
	while (1)
	{ 
		kinematics_flag++;
		//Armcontrol.Joint0adjust();//先拨杆向下 进入修正模式
		Armcontrol.ArmModeGoing();
		

//		SERVO_Send_recv(&ys_motor,&yr_motor);

		vTaskDelay(pdMS_TO_TICKS(15));
	}
}

ArmControl_base::ArmControl_base()
{
	Times__=5;
	vel_base=0.5;//rad/s
	GO2adjust = 0;
	//初始化动作组

	//存储解算器的正确性
 joint1_slover=0;
 joint2_slover=0;
 joint3_slover=0;
 joint4_slover=0;
	//存储目标的位置与速度
	for(int i = 0;i<sizeof(JointPose);i++)
		JointPose[i] = InitGroup[i];
	
	//初始化buf
	GOMotorbuf[0] = HEADFRAME;
	GOMotorbuf[BUFSIZE-1] = TAILFRAME;
	
//	goMotor = get_GOMotor_Point();
//  aim_pose[3][10]={};
//  aim_vel[3][10]={};	
}
void ArmControl_base::ArmModeGoing()
{
#if USE_RC==1
	if(message_bag.RobotBehavior.RobotMode==RobotArm_m)//遥控器控制机械臂
	{
		RCArmContorl();
	}else
	{
		switch(message_bag.RobotBehavior.ActionGroup)//判断要执行的动作组
		{
			case	AirCatch:
			{
				ArmMoveGroupSolver(AirCatchGroup,this->Times__);
				break;
			}
			case	Init:
			{
				ArmMoveGroupSolver(InitGroup,this->Times__);
				break;
			}
			case	PutBack:
			{
				ArmMoveGroupSolver(PutBackGroup,this->Times__);
				break;
			}
			case	CatchGround:
			{
				ArmMoveGroupSolver(CatchGroundGroup,this->Times__);
				break;
			}
			case	CatchMid:
			{
				ArmMoveGroupSolver(CatchMidGroup,this->Times__);
				break;
			}
			case	Stop:
			{
				ArmMoveGroupSolver(StopGroup,this->Times__);
				break;
			}
			case	KEEP:
			{
				float temppose[4] = {JointPose[0],Arm.POS_[1],Arm.POS_[2],Arm.POS_[3]};
					
				ArmControl(temppose);//保持当前数据不变
				break;
			}
		}		
	}
#else 
	PCArmContorl();
#endif
}


void ArmControl_base::Joint0adjust()
{
	float delta__[4] = {0};
	if((switch_is_down(message_bag.RC_ctrl->rc.s[S2_CHANNEL]) && switch_is_down(message_bag.RC_ctrl->rc.s[S1_CHANNEL])))//全部朝下 进入关节零的位置矫正
	{
		delta__[0]=linear_map(message_bag.RC_ctrl->rc.ch[0],-1000,1320,JointPLimit[0][MINVALUE],JointPLimit[0][MAXVALUE]);//遥控器调控
		ArmControl(delta__);
		
		GO2adjust = -9;
		
		JointPose[0] = GO2adjust;//初始为矫正值 则后面的值都是在这个基础上加减\
		
		InitGroup[0]       =GO2adjust;   
		AirCatchGroup[0]   =GO2adjust;   
		PutBackGroup[0]    =GO2adjust;   
		CatchGroundGroup[0]=GO2adjust;   
		CatchMidGroup[0]   =GO2adjust;   
		StopGroup[0]       =GO2adjust;   
	}else
	{
		GO2adjust = -9;
		InitGroup[0]       =GO2adjust;   
		AirCatchGroup[0]   =GO2adjust;   
		PutBackGroup[0]    =GO2adjust;   
		CatchGroundGroup[0]=GO2adjust;   
		CatchMidGroup[0]   =GO2adjust;   
		StopGroup[0]       =GO2adjust;   
		return ;
	}
}

void ArmControl_base::RCArmContorl()
{
	//遥控器映射
	JointPose[0]=linear_map(message_bag.RC_ctrl->rc.ch[0],-1000,1320,JointPLimit[0][MINVALUE],JointPLimit[0][MAXVALUE]);//右右
	JointPose[1]=linear_map(message_bag.RC_ctrl->rc.ch[1],RCCHMIN,RCCHMAX,JointPLimit[1][MINVALUE],JointPLimit[1][MAXVALUE]);//右上
	JointPose[2]=linear_map(message_bag.RC_ctrl->rc.ch[2],RCCHMIN,RCCHMAX,JointPLimit[2][MINVALUE],JointPLimit[2][MAXVALUE]);//左右
	JointPose[3]=linear_map(message_bag.RC_ctrl->rc.ch[3],RCCHMIN,RCCHMAX,JointPLimit[3][MINVALUE],JointPLimit[3][MAXVALUE]);//左上
	
	JointPose[0] = JointPose[0] - 0.3f;
	
	//控制输出
	ArmControl(JointPose);
}
void ArmControl_base::PCArmContorl()
{ 
	static char last_state = MAX_ActionGroup;
	if(message_bag.RobotBehaviorPC.ActionGroup != last_state) //如果相等就不做切换
	{
		switch(message_bag.RobotBehaviorPC.ActionGroup)
		{
			case Init:
			{
				for(int i =0;i<4;i++)
				{
					JointPose[i]=InitGroup[i];
				}
				break;
			}
			case PutBack:
			{
				for(int i =0;i<4;i++)
				{
					JointPose[i]=PutBackGroup[i];
				}
				break;
			}
			case CatchGround:
			{
				for(int i =0;i<4;i++)
				{
					JointPose[i]=CatchGroundGroup[i];
				}
				break;
			}
			case AirCatch:
			{
				for(int i =0;i<4;i++)
				{
					JointPose[i]=AirCatchGroup[i];
				}
				break;
			}
		}		
	}
	
	last_state = message_bag.RobotBehaviorPC.ActionGroup;

	if(message_bag.ctrl_neg.key_count)//反转
	{//
		JointPose[0]-=0.04*(message_bag.key_z.key_count);//*(1+message_bag.key_z.key_longcount);
		JointPose[1]-=0.02*(message_bag.key_x.key_count);//*(1+message_bag.key_x.key_longcount);
		JointPose[2]-=0.02*(message_bag.key_c.key_count);//*(1+message_bag.key_c.key_longcount);
		JointPose[3]-=0.02*(message_bag.key_f.key_count);//*(1+message_bag.key_f.key_longcount);

	}
	else if(message_bag.shift_pos.key_count)//正转
	{
		JointPose[0]+=0.04*(message_bag.key_z.key_count);//*(1+message_bag.key_z.key_longcount);
		JointPose[1]+=0.02*(message_bag.key_x.key_count);//*(1+message_bag.key_x.key_longcount);
		JointPose[2]+=0.02*(message_bag.key_c.key_count);//*(1+message_bag.key_c.key_longcount);
		JointPose[3]+=0.02*(message_bag.key_f.key_count);//*(1+message_bag.key_f.key_longcount);
	}
	else//停止
	{
	}
	
	//输出限幅
	JointPose[3]=constrain(JointPose[3],-3.25f,3.25f);
	JointPose[2]=constrain(JointPose[2],-1.75f,1.75f);
	JointPose[1]=constrain(JointPose[1],-3.25f,3.25f);
	JointPose[0]=constrain(JointPose[0],-14.0f ,4.0f);
	
	//控制输出
	ArmControl(JointPose);

}
/**
  * @brief          判断是否到达目标
  * @param[in]      *pose 目标动作组 
	*									Time	插值密度
  * @retval         none
  */
bool Kinematics_base::Is_Reach_aim(float *Current_pose,float *Aim_pose,float error)
{
	int flag =0;
	for(int i =0;i<3;i++)
	{
			if(fabs(Current_pose[i]-Aim_pose[i])<=error && flag==2)
			{
				flag++;
				return 1;
			}
			else
			{
				return 0;
			}
	}
	
	return 0;
}

/**
  * @brief          连续动作组解决器
  * @param[in]      *pose 目标动作组 
	*									Time	插值密度
  * @retval         none
  */
void ArmControl_base::ArmControl(float *pose)
{
	vTaskDelay(pdMS_TO_TICKS(5));
	ctrl_motor_pv(2,(float)pose[3],(float)vel_base);
	vTaskDelay(pdMS_TO_TICKS(5));
	ctrl_motor_pv(1,(float)pose[2],(float)vel_base);	
	vTaskDelay(pdMS_TO_TICKS(5));
	ctrl_motor_pv(0,(float)pose[1],(float)vel_base);
	vTaskDelay(pdMS_TO_TICKS(5));
	ys_motor.Pos=pose[0];
	SERVO_Send_recv(&ys_motor,&yr_motor);
}
/**
  * @brief          连续动作组解决器
  * @param[in]      *pose 目标动作组 

	*									Time	插值密度
  * @retval         none
  */
void ArmControl_base::ArmMoveGroupSolver(float *pose,int Time)
{		
		//判断是否到达目标
		if(Is_Reach_aim((float *)Arm.POS_,pose,0.02))
		{
			return;
		}
	
    joint1_slover = polynomial_trajectory_slover(aim_pose[1],aim_vel[1],Time,Arm.POS_[0],Arm.VEl_[0],pose[0],0);
    joint2_slover = polynomial_trajectory_slover(aim_pose[2],aim_vel[2],Time,Arm.POS_[1],Arm.VEl_[1],pose[1],0);
    joint3_slover = polynomial_trajectory_slover(aim_pose[3],aim_vel[3],Time,Arm.POS_[2],Arm.VEl_[2],pose[2],0);				
	
//		joint1_slover = polynomial_trajectory_slover(aim_pose[1],aim_vel[1],Time,Arm.POS_[0],Arm.VEl_[0],0.436,pose[1],0,-0.436);
//    joint2_slover = polynomial_trajectory_slover(aim_pose[2],aim_vel[2],Time,Arm.POS_[1],Arm.VEl_[1],0.436,pose[2],0,-0.436);
//    joint3_slover = polynomial_trajectory_slover(aim_pose[3],aim_vel[3],Time,Arm.POS_[2],Arm.VEl_[2],0.436,pose[3],0,-0.436);
//    joint4_slover = polynomial_trajectory_slover(aim_pose[0],aim_vel[0],Time,Arm.POS_[0],Arm.VEl_[3],pose[2],0);
		
		for(int i=0;i<=Time;i++)
		{
			//限幅
			aim_pose[2][i]=constrain(aim_pose[2][i],-3.14f,3.14f);
			aim_pose[1][i]=constrain(aim_pose[1][i],-1.75f,1.75f);
			aim_pose[0][i]=constrain(aim_pose[0][i],-3.14f,3.14f);
			//输出
			ctrl_motor_pv(2,(float)aim_pose[3][i],(float)aim_vel[3][i]);
			ctrl_motor_pv(1,(float)aim_pose[2][i],(float)aim_vel[2][i]);
			ctrl_motor_pv(0,(float)aim_pose[1][i],(float)aim_vel[1][i]);
    	ys_motor.Pos=aim_pose[0][i];
    	SERVO_Send_recv(&ys_motor,&yr_motor);
			//串口输出
			//send_float(aim_pose[0][i],USART6);
			
			vTaskDelayUntil( &xLastWakeTime, xDelay1s );
		}
}
/**
  * @brief          五次多项式插值
  * @param[in]      &rad 存储目标弧度 
	*									&vel存储目标速度
	*								  &插值密度
	*								  &当前角度 速度 加速度
	*								  &目标角度 速度 加速度
  * @retval         none
  */
bool Kinematics_base::polynomial_trajectory_slover(float* rad,float* vel,const int &T,float theat,float dot_theta,float dott_theta,float aim,float dot_aim,float dott_aim)
{
	int times = T ;
	int param_times = T;
	
	Qptpf.c0 = theat;
	Qptpf.c1 = dot_theta;
	Qptpf.c2 = dott_theta/2;
	Qptpf.c3 = (10*aim)/powf(param_times,3) - (4*dot_aim)/powf(param_times,2) + dott_aim/(2*param_times) - (6*dot_theta)/powf(param_times,2) - (3*dott_theta)/(2*param_times) - (10*theat)/powf(param_times,3);
	Qptpf.c4 = (7*dot_aim)/powf(param_times,3) - (15*aim)/powf(param_times,4) - dott_aim/powf(param_times,2) + (8*dot_theta)/powf(param_times,3) + (3*dott_theta)/(2*powf(param_times,2)) + (15*theat)/powf(param_times,4);
	Qptpf.c5 = (6*aim)/powf(param_times,5) - (3*dot_aim)/powf(param_times,4) + dott_aim/(2*powf(param_times,3)) - (3*dot_theta)/powf(param_times,4) - dott_theta/(2*powf(param_times,3)) - (6*theat)/powf(param_times,5);
	
	if(!rad||!vel)
	{return error;}
	
	for(int i=0;i<=times;i++)
	{
		rad[i] = Qptpf.c0 + Qptpf.c1*i + Qptpf.c2*powf(i,2) + Qptpf.c3*powf(i,3) + Qptpf.c4*powf(i,4) + Qptpf.c5*powf(i,5);
		vel[i] = Qptpf.c1 + 2*Qptpf.c2*i + 3*Qptpf.c3*powf(i,2) + 4*Qptpf.c4*powf(i,3) + 5*Qptpf.c5*powf(i,4);
		
		if(((rad+1== NULL)||(vel+1== NULL))&&(i!= times))
		{return fault;}
	}
	
	return success;
}



bool Kinematics_base::polynomial_trajectory_slover(float* rad,float* vel,const int &T,float theat,float dot_theta,float aim,float dot_aim)
{
	uint16_t param_time = 0;//s
	uint16_t cnt = 0;

	cnt = (T * Secons2Milliseconds) / PER_CALC;//ms
	param_time = T;


	//计算插值系数
	Cptpf.c0 = theat;
	Cptpf.c1 = dot_theta;
	Cptpf.c3 = -((2 * (aim - Cptpf.c0) - Cptpf.c1 * param_time - dot_aim * param_time) / (param_time*param_time*param_time));
	Cptpf.c2 = (dot_aim - Cptpf.c1 - 3 * Cptpf.c3 * (param_time*param_time)) / (2 * param_time);
	//计算每一个时间段所要到达的值
	for (int i = 0; i < cnt + 1; i++)
	{
		rad[i] = Cptpf.c0 + Cptpf.c1 * i\
			+ Cptpf.c2 * (i*i)\
			+ Cptpf.c3 * (i*i*i);
		vel[i] = Cptpf.c1 \
			+ 2*Cptpf.c2 * i\
			+ 3*Cptpf.c3 * (i*i);
	}
	
	return 1;
}

/**
  * @brief          线性映射函数
  * @param[in]      &value 要映射的值 
	*									&in_min 输入的最小值
	*								  &in_max	输入的最大值
	*								  &out_min 映射区间的最小值
	*								  &out_max 映射区间的最大值
  * @retval         none
  */
float Kinematics_base::linear_map(float value, float in_min, float in_max, float out_min, float out_max) {
    float out_value = (value - in_min) / (in_max - in_min) * (out_max - out_min) + out_min;
    if (out_min < out_max) {
        if (out_value < out_min) {
            out_value = out_min;
        }
        if (out_value > out_max) {
            out_value = out_max;
        }
    } else {
        if (out_value > out_min) {
            out_value = out_min;
        }
        if (out_value < out_max) {
            out_value = out_max;
        }
    }
    return out_value;
}
/**
  * @brief          正运动学
  * @param[in]      &pose 正运动学解算的坐标 &rad 弧度
  * @retval         none
  */
void Kinematics_base::fkine(Pose &pose,Rad_t &rad)
{
	pose.px = Link2*(cos(rad.q3)*sin(rad.q1) + cos(rad.q1)*cos(rad.q2)*sin(rad.q3)) + Link1*sin(rad.q1);
	pose.py = Link2*sin(rad.q2)*sin(rad.q3);
	pose.pz = Link2*(cos(rad.q1)*cos(rad.q3) - cos(rad.q2)*sin(rad.q1)*sin(rad.q3)) + Link1*cos(rad.q1);
}

//void Kinematics_base::updata_aim_pose(mat_f32 &aim_pose)
//{
//	this->Aim_pose=aim_pose;
//}

//void Kinematics_base::updata_aim_pose(float aim_pose[4][4])
//{
//	arm_mat_init_f32(&this->Aim_pose,4,4,aim_pose[0]);
//}

//mat_f32 Kinematics_base::transl(float &x,float &y,float &z)
//{
//	float arr[4][4]={1,0,0,x,
//									 0,1,0,y,
//									 0,0,1,z,
//									 0,0,0,1};
//	mat_f32 mat;
//	
//	arm_mat_init_f32(&mat,4,4,arr[0]);
//	
//	return mat;
//}
//mat_f32 Kinematics_base::trotx(float &x_rad)
//{
//	float x_angle=x_rad*180/M_PI;
//	float arr[4][4]={1,0,0,0,
//									 0,cosf(x_angle),-sinf(x_angle),0,
//									 0,sinf(x_angle),cosf(x_angle),0,
//									 0,0,0,1};
//	mat_f32 mat;
//	
//	arm_mat_init_f32(&mat,4,4,arr[0]);
//	
//	return mat;					 
//}
//mat_f32 Kinematics_base::troty(float &y_rad)
//{
//	float y_angle=y_rad*180/M_PI;
//	float arr[4][4]={cosf(y_angle),0,sinf(y_angle),0,
//									 0,1,0,0,
//									 -sinf(y_angle),0,cosf(y_angle),0,
//									 0,0,0,1};
//	mat_f32 mat;
//	
//	arm_mat_init_f32(&mat,4,4,arr[0]);
//	
//	return mat;					 
//}
//mat_f32 Kinematics_base::trotz(float &z_rad)
//{
//	float z_angle=z_rad*180/M_PI;
//	float arr[4][4]={cosf(z_angle),-sinf(z_angle),0,0,
//									 sinf(z_angle),cosf(z_angle),0,0,
//									 0,0,0,0,
//									 0,0,0,1};
//	mat_f32 mat;
//	
//	arm_mat_init_f32(&mat,4,4,arr[0]);
//	
//	return mat;					 
//}

	





