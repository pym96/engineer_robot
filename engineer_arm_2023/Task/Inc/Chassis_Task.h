#ifndef __CHASSIS_TASK_H
#define __CHASSIS_TASK_H

#include "dev_system.h"
#include "protocol_dbus.h"

#include "app_motor.h"

#include "algorithm_user_lib.h"
#include "algorithm_pid.h"

#include "Message_Task.h"
#ifdef __cplusplus
extern "C" {
#endif

void Chassis_Task(void* pvParameters);

#ifdef __cplusplus
}
#endif

#define useMecanum

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

//任务开始空闲一段时间
#define CHASSIS_TASK_INIT_TIME 357

//前后的遥控器通道号码
#define CHASSIS_X_CHANNEL 3
//左右的遥控器通道号码
#define CHASSIS_Y_CHANNEL 2
//在特殊模式下，可以通过遥控器控制旋转
#define CHASSIS_WZ_CHANNEL 1

//选择底盘状态 开关通道号
#define MODE_CHANNEL 0
//遥控器前进摇杆（max 660）转化成车体前进速度（m/s）的比例
#define CHASSIS_VX_RC_SEN 0.003030303f//0.005f
//遥控器左右摇杆（max 660）转化成车体左右速度（m/s）的比例
#define CHASSIS_VY_RC_SEN 0.003030303f//0.005f
//跟随底盘yaw模式下，遥控器的yaw遥杆（max 660）增加到车体角度的比例
#define CHASSIS_ANGLE_Z_RC_SEN 0.000002f
//不跟随云台的时候 遥控器的yaw遥杆（max 660）转化成车体旋转速度的比例
#define CHASSIS_WZ_RC_SEN 0.01f

//这两个宏用于低通滤波//值越大底盘越软
#define CHASSIS_ACCEL_X_NUM 0.3f//0.5f
#define CHASSIS_ACCEL_Y_NUM 0.3f//0.5f
#define CHASSIS_ACCEL_Z_NUM 0.15f
//遥控器死区
#define CHASSIS_RC_DEADLINE 10

#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f

#define MOTOR_DISTANCE_TO_CENTER 0.2f

//底盘任务控制间隔 2ms
#define CHASSIS_CONTROL_TIME_MS 2
//底盘任务控制间隔 0.002s
#define CHASSIS_CONTROL_TIME 0.002
//底盘任务控制频率，尚未使用这个宏
#define CHASSIS_CONTROL_FREQUENCE 500.0f
//底盘3508最大can发送电流值
#define MAX_MOTOR_CAN_CURRENT 10000.0f


//底盘前后左右控制按键+加速键

//功率优先
#define POWER_FIRST 1 
//血量优先
#define HP_FIRST 0 

#define CHASSIS_FRONT_KEY        KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY         KEY_PRESSED_OFFSET_S
#define CHASSIS_LEFT_KEY         KEY_PRESSED_OFFSET_A
#define CHASSIS_RIGHT_KEY        KEY_PRESSED_OFFSET_D

#define CHASSIS_SPEEDUP_KEY      KEY_PRESSED_OFFSET_SHIFT
#define CHASSIS_SPEEDDOWN_KEY    KEY_PRESSED_OFFSET_CTRL

#define CHASSIS_FRONT_CH3				 RC_CH_VALUE_MAX
#define CHASSIS_STOP_CH3				 RC_CH_VALUE_OFFSET
#define CHASSIS_BACK_CH3				 RC_CH_VALUE_MIN

#define CHASSIS_RIGHT_CH2				 RC_CH_VALUE_MAX 
#define CHASSIS_STOP_CH2         RC_CH_VALUE_OFFSET
#define CHASSIS_LEFT_CH2       	 RC_CH_VALUE_MIN

#define RC_CH_STEP 0.005//0.0861//      7/10             //0.013636  //				10/660

#define FRONT_ECD 887
#define BACK_ECD  4974
#define LEFT_ECD  2935
#define RIGHT_ECD 7034

#define FRONT_LEFT_ECD  3977
#define FRONT_RIGHT_ECD 2050
#define BACK_RIGHT_ECD  6022
#define BACK_LEFT_ECD   7930


//m3508转化成底盘速度(m/s)的比例，做两个宏 是因为可能换电机需要更换比例
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

//底盘电机最大速度
#define MAX_WHEEL_SPEED 4.0f//10.0f//4.0
//底盘运动过程最大前进速度
#define NORMAL_MAX_CHASSIS_SPEED_X 2.0f//10.0f//2.9
//底盘运动过程最大平移速度
#define NORMAL_MAX_CHASSIS_SPEED_Y 2.0f//10.0f//2.9
//底盘运动过程最大旋速度
#define NORMAL_MAX_CHASSIS_SPEED_Z 4.0f//10.0f//2.9

//底盘设置旋转速度，设置前后左右轮不同设定速度的比例分权 0为在几何中心，不需要补偿
#define CHASSIS_WZ_SET_SCALE 0.1f

//摇摆原地不动摇摆最大角度(rad)
#define SWING_NO_MOVE_ANGLE 0.1f
//摇摆过程底盘运动最大角度(rad)
#define SWING_MOVE_ANGLE 0.5f

//底盘电机速度环PID
#define M3505_MOTOR_SPEED_PID_KP 18000.0f
#define M3505_MOTOR_SPEED_PID_KI 5.0f
#define M3505_MOTOR_SPEED_PID_KD 0.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

//底盘旋转跟随PID
#define CHASSIS_FOLLOW_GIMBAL_PID_KP 5.5f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 4.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 0.2f
//angle position pid
#define CHASSIS_ANGLE_POSITION_PID_KP 6.0f
#define CHASSIS_ANGLE_POSITION_PID_KI 0.003f
#define CHASSIS_ANGLE_POSITION_PID_KD 0.0f
#define CHASSIS_ANGLE_POSITION_PID_MAX_OUT 8.0f
#define CHASSIS_ANGLE_POSITION_PID_MAX_IOUT 1.0f
//angle speed pid
#define CHASSIS_GYRO_SPEED_PID_KP 1.0f
#define CHASSIS_GYRO_SPEED_PID_KI 0.0f
#define CHASSIS_GYRO_SPEED_PID_KD 0.0f
#define CHASSIS_GYRO_SPEED_PID_MAX_OUT 	6.0f
#define CHASSIS_GYRO_SPEED_PID_MAX_IOUT 0.2f


#define STEP_SPEED 0.001
typedef struct
{
	const motor_measure_t* chassis_motor_measure;
	fp32 accel;
	fp32 speed;
	fp32 speed_set;
	int16_t give_current;
} Chassis_Motor_t;//底盘接收编码器数据 以及 设定数据

typedef struct
{
	const motor_measure_t* up_push_motor_measure;
	fp32 accel;
	fp32 speed;
	fp32 speed_set;
	int16_t give_current;
} UP_PUSH_Motor_t;//抬升推出接收编码器数据 以及 设定数据

typedef struct {
	fp32 vx;      //底盘速度 前进方向 前为正，单位 m/s
	fp32 vy;      //底盘速度 左右方向 左为正  单位 m/s
	fp32 wz;      //底盘旋转角速度，逆时针为正 单位 rad/s
	fp32 vx_set;  //底盘设定速度 前进方向 前为正，单位 m/s
	fp32 vy_set;  //底盘设定速度 左右方向 左为正，单位 m/s
	fp32 wz_set;  //底盘设定旋转角速度，逆时针为正 单位 rad/s

	fp32 vx_max_speed;  //前进方向最大速度 单位m/s
	fp32 vx_min_speed;  //前进方向最小速度 单位m/s
	fp32 vy_max_speed;  //左右方向最大速度 单位m/s
	fp32 vy_min_speed;  //左右方向最小速度 单位m/s

	fp32    Speed_Set[3];//底盘速度 [0]前进速度 [1]后退速度 [2]平移速度

	uint8_t Speed_up_sum;
	uint8_t Speed_down_sum;
} Chassis_Velocity_t;

class Chassis_Ctrl
{
public:
	Message_bag* Chassis_message;

	void Chassis_Init(void);//底盘初始化
	void Chassia_Motor_Feedback_Update(void);//底盘电机数据更新
	void Chassis_rc_to_control_vector(fp32* vx_set, fp32* vy_set);//遥控器数据处理为xyw速度
	void Chassis_Motor_ctrl(void);
	void Limit_power_chassis_output(void);
	void caclc_chassia_front_angle(void);
	void chassis_set_contorl(void);
	void chassis_behaviour_control_set(fp32* vx_set, fp32* vy_set, fp32* angle_set);
	void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4]);
	void chassis_control_loop(void);
  void Up_Push_Move(void);
  void Up_Push_loop(void);
	void up_push_set_contorl(void);
	void UP_PUSH_Motor_Feedback_Update(void);
private:

	void Chassis_speed_change(void);
	Chassis_Motor_t Motor[Chassis_Motor_Numbers];
  UP_PUSH_Motor_t gMotor[4];

	PidTypeDef  Speed_Pid[Chassis_Motor_Numbers];
	PidTypeDef  Follow_Gimbal_Pid;
	PidTypeDef  chassis_setangle;
	PidTypeDef  chassis_setangle_gyro;
  PidTypeDef  Up_Push_Pid[4];

	//滤波之后的速度
	first_order_filter_type_t Filter_vx;
	first_order_filter_type_t Filter_vy;
	first_order_filter_type_t Filter_vw;

	Chassis_Velocity_t   Velocity;

	struct SGyro* chassis_imu_gyro;        //底盘陀螺仪
	struct SAngle* chassis_imu_angle;       //底盘陀螺仪

	fp32                        Chassis_angle;
	fp32                        Chassis_angle_set;
	fp32                        Chassis_gyor;
	fp32                        Chassis_gyor_set;
	const fp32* relative_angle;
	//功率相关
	fp32                        level_abgle_set;//小陀螺旋转设定值
	fp32                        level_limit_speed_max;
	//底盘由于轮子到云台的距离不一样 所以需要一个修正系数
	float head_wheel;
	float tail_wheel;
};



#endif /* __CHASSIS_TASK_H */
