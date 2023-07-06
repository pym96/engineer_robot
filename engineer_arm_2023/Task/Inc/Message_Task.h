#ifndef  __MESSAGE_TASK_H
#define  __MESSAGE_TASK_H

//选择底盘状态 开关通道号
#define S1_CHANNEL  0
#define S2_CHANNEL  1

#define S1_UP 			1
#define S1_DOWN			2
#define S1_MID			3

#define S2_UP 			1
#define S2_DOWN			2
#define S2_MID			3
//是用遥控器控制
#define USE_RC      0

#include "dev_system.h"
#include "app_motor.h"

#define CAN_Message_CAN             ( CAN2_Ctrl )
#ifdef __cplusplus
extern "C" {
#endif

#include "platform.h"
#include "protocol_dbus.h"

void Message_Task(void);

#ifdef __cplusplus
}
#endif

typedef enum
{
	Chassis_NO_MOVE = 0,
	Chassis_FOLLOW_L1,
	Chassis_FOLLOW_L2,
	Chassis_SPIN,
	Chassis_UP_PUSH,
	Chassis_KEEP
}Chassis_State_t;

	
typedef enum
{
	RobotArm_m = 0,//机械臂控制模式
	RobotChassis_m,//底盘控制模式
	RobotChassisAction_m1,//动作组1模式
	RobotChassisAction_m2,//动作组2模式
}RobotMode_t;


typedef enum
{
	AirCatch = 0,
	Init,
	PutBack,
	CatchGround,
	CatchMid,
	Stop,
	KEEP
}ActionGroup_t;

class RobotBehaviorRC_t
{
public:
	RobotBehaviorRC_t()
	{
		RobotMode = RobotChassis_m;
		
		IsSuckerON = 0;
		IsChassisON = 1;
		IsGimablON = 0;
		IsArmON = 0;
		IsArmKeep = 0;
		
		ActionGroup = Init;
	}
	
	void RobotBehaviorReset()
	{
		IsSuckerON = 0;
		IsChassisON = 0;
		IsGimablON = 0;
		IsArmON = 0;
	}
	
	bool IsSuckerON;//是否允许使能吸盘
	bool IsChassisON;//是否使能底盘
	bool IsGimablON;//是否使能云台
	bool IsArmON;//是否使能机械臂
	bool IsArmKeep;//摇杆切换时是否保持动作
	
	RobotMode_t RobotMode;
	ActionGroup_t ActionGroup;
};

class RobotBehaviorPC_t
{
public:
	RobotBehaviorPC_t()
	{
		IsSuckerON = 0;
		IsFollowON = 0;
		PosNegFlag = 0;
		IsArmInitON		= 0;
		SpeedLevel =0;
		ActionGroup = Init;
		ChassisState = Chassis_NO_MOVE;
	}
	
	void RobotBehaviorReset()
	{
		PosNegFlag = 0;
		IsSuckerON = 0;
		IsFollowON = 0;
		IsArmInitON = 0;
	}
	
	bool IsSuckerON;//是否允许使能吸盘
	bool IsFollowON;//是否使能底盘跟随
	bool IsArmInitON;//是否初始化
	bool IsArmAirCatchON;//是否空接
	bool IsArmCatchMidON;//是否小资源岛
	bool IsArmCatchGroundON;//是否小资源岛
	char PosNegFlag;//关节的正负
	u8 SpeedLevel;//控制速度挡位
	Chassis_State_t ChassisState;//底盘状态
	ActionGroup_t ActionGroup;
};

struct key_t : InitStruct<key_t>    //按键结构体
{
	uint8_t key_flag;
	uint8_t key_longflag;
	uint8_t key_count;
	uint8_t key_longcount;
	uint8_t last_key;
};

class key
{
public:

	/*新按键*/
	void key_num_sum(uint8_t key_num, key_t* temp_key);
	void key_clear(key_t* temp_key);
	void key_long_press(uint8_t key_num, key_t* temp_key);

	key_t key_c;
	key_t key_f;
	key_t key_v;
	key_t key_x;
	key_t key_q;
	key_t key_e;
	key_t key_r;
	key_t key_g;
	key_t key_z;
	key_t mouse_l;
	key_t mouse_r;
	key_t shift_pos;
	key_t ctrl_neg;
	key_t key_b;
};

struct _limit
{
	u8 dowm_limit;
	u8 up_limit;
	u8 front_limit;
	u8 back_limit;
	u8 last_dowm_limit;
	u8 last_up_limit;
	u8 last_front_limit;
	u8 last_back_limit;
};

class Message_bag : public key
{
public:
	uint8_t                          Hero_colour_flag;//英雄颜色状态标志
	void Messag_bag_Init(void);  //初始化
	void Operate_to_robotstate(void); //通过遥控器获取来设置机器状态	
	
	void PC_to_robotstate(void);//电脑控制
	void RC_to_robotstate(void);//遥控控制

	void ARMControl();
	void SolveLeftPlunk();
	void SolveRightPlunk();
	/* 上 1 下 2 中 3 */
	bool s1_flag[3];//左通道              
	bool s2_flag[3];//右通道
	/*云台 底盘状态*/
  RobotBehaviorRC_t   RobotBehaviorRC;
	RobotBehaviorPC_t   RobotBehaviorPC;
	Chassis_State_t   Chassis_state;
	/*遥控信息*/
	const RC_ctrl_t* RC_ctrl;

    _limit limit;
    void Message_Send(void);
    void Message_Feedback(void);
		void SendData(uint32_t StdId, int16_t data1, int16_t data2, int16_t data3, int16_t data4 ,int16_t data5,int16_t data6);
	  void SendData(uint32_t StdId, int16_t data1, int16_t data2, int16_t data3, int16_t data4 );
	  void SendData(uint32_t StdId, int16_t data1, int16_t data2, int16_t data3 );
	  void SendData(uint32_t StdId, int16_t data1, int16_t data2 );
	  void SendData(uint32_t StdId, int16_t data1 );
	
};

extern Message_bag message_bag;

#endif



