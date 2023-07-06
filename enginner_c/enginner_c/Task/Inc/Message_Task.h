#ifndef  __MESSAGE_TASK_H
#define  __MESSAGE_TASK_H

#include "app_motor.h"
#include "Kinematics_task.h"
//#include "remote_control.h"

//ѡ�����״̬ ����ͨ����
#define S1_CHANNEL  0
#define S2_CHANNEL  1

#define S1_UP 			1
#define S1_DOWN			2
#define S1_MID			3

#define S2_UP 			1
#define S2_DOWN			2
#define S2_MID			3
//����ң��������
#define USE_RC      0


#include "motor_control.h"
#ifdef __cplusplus
extern "C" {
#endif
#include "RGB.h"
//#include "platform.h"
//#include "protocol_dbus.h"
#include "remote_control.h"
#include "led.h" 
void Message_task(void);

#ifdef __cplusplus
}
#endif

typedef enum
{
	Chassis_NO_MOVE = 0,
	Chassis_FOLLOW,
	Chassis_SPIN,
	Chassis_UP_PUSH,
	Chassis_KEEP
}Chassis_State_t;

	
typedef enum
{
	RobotArm_m = 0,//��е�ۿ���ģʽ
	RobotChassis_m,//���̿���ģʽ
	RobotChassisAction_m1,//������1ģʽ
	RobotChassisAction_m2,//������2ģʽ
}RobotMode_t;


typedef enum
{
	AirCatch = 0,
	Init,
	PutBack,
	CatchGround,
	CatchMid,
	Stop,
	KEEP,
	MAX_ActionGroup
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
	
	bool IsSuckerON;//�Ƿ�����ʹ������
	bool IsChassisON;//�Ƿ�ʹ�ܵ���
	bool IsGimablON;//�Ƿ�ʹ����̨
	bool IsArmON;//�Ƿ�ʹ�ܻ�е��
	bool IsArmKeep;//ҡ���л�ʱ�Ƿ񱣳ֶ���
	
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
		
		ActionGroup = Init;

	}
	
	void RobotBehaviorReset()
	{
		PosNegFlag = 0;
		IsSuckerON = 0;
		IsFollowON = 0;
		IsArmInitON = 0;
	}
	
	bool IsSuckerON;//�Ƿ�����ʹ������
	bool IsFollowON;//�Ƿ�ʹ�ܵ��̸���
	bool IsArmInitON;//�Ƿ��ʼ��
	bool IsArmAirCatchON;//�Ƿ�ս�
	bool IsArmCatchMidON;//�Ƿ�С��Դ��
	bool IsArmCatchGroundON;//�Ƿ�С��Դ��
	char PosNegFlag;//�ؽڵ�����
	
	ActionGroup_t ActionGroup;
};

struct key_t    //�����ṹ��
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

	/*�°���*/
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
	uint8_t                          Hero_colour_flag;//Ӣ����ɫ״̬��־
	void Messag_bag_Init(void);  //��ʼ��
	void Operate_to_robotstate(void); //ͨ��ң������ȡ�����û���״̬	
	
	void PC_to_robotstate(void);//���Կ���
	void RC_to_robotstate(void);//ң�ؿ���

	void ARMControl();
	void SolveLeftPlunk();
	void SolveRightPlunk();
	/* �� 1 �� 2 �� 3 */
	bool s1_flag[3];//��ͨ��              
	bool s2_flag[3];//��ͨ��
	/*��̨ ����״̬*/
  RobotBehaviorRC_t   RobotBehaviorRC;
	RobotBehaviorPC_t   RobotBehaviorPC;
	/*ң����Ϣ*/
	const RC_ctrl_t* RC_ctrl;

    _limit limit;
    void Message_Send(void);
    void Message_Feedback(void);
	  void SendData(uint32_t StdId, int16_t data1, int16_t data2, int16_t data3, int16_t data4 );
	  void SendData(uint32_t StdId, int16_t data1, int16_t data2, int16_t data3 );
	  void SendData(uint32_t StdId, int16_t data1, int16_t data2 );
	  void SendData(uint32_t StdId, int16_t data1 );
		void image_control(void);
	
};

extern Message_bag message_bag;

#endif



