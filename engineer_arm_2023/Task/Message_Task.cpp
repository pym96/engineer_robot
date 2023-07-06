#include "Message_Task.h"
#include "app_usart7.h"
#include "start_task.h"
#include "app_gomotor.h"

uint8_t Message_flag = 0;

Message_bag message_bag;
MOTOR_send cmd; 

static void System_Reset(void);

void Message_Task()
{
	static BaseType_t err = pdFALSE;

	message_bag.Messag_bag_Init();//初始化

	CAN_ALL_Init();
	vTaskDelay(200);
	while (1)
	{
		Message_flag++;
		message_bag.Operate_to_robotstate();//遥控器模式控制
		message_bag.ARMControl();
		//message_bag.RC_to_robotstate();
    message_bag.Message_Feedback();
    message_bag.Message_Send();		
		vTaskDelay(10);
	}
}

void Message_bag::Messag_bag_Init(void)
{
	RC_ctrl = get_remote_control_point();
}
void key::key_num_sum(uint8_t key_num, key_t* temp_key)
{
	if (key_num == 1 && temp_key->key_flag == 0) {
		temp_key->key_flag = 1;
	}
	if (temp_key->key_flag == 1 && key_num == 0) {
		temp_key->key_count = 1;
		temp_key->key_flag = 0;
	}
	
	//判断长按
	if(key_num == 1 )
	{
		temp_key->key_longflag++;
	}else
	{
		temp_key->key_longflag = 0;		
	}
	
	if(temp_key->key_longflag>50)//s
	{
		temp_key->key_longcount++;
	}
	
	if(temp_key->key_longflag == 0)
	{
		temp_key->key_longcount = 0;
	}
}

void key::key_long_press(uint8_t key_num, key_t* temp_key)
{
	if (key_num == 1)
	{
		temp_key->key_count = 1;
	}
	else if (key_num == 0)
	{
		temp_key->key_count = 0;
	}
}

void key::key_clear(key_t* temp_key)
{
	temp_key->key_count = 0;
	temp_key->key_flag = 0;
}

void Message_bag::Operate_to_robotstate(void)
{
#if USE_RC==1
	RC_to_robotstate();
#else
	PC_to_robotstate();
#endif
}

/*
*		左拨杆 													右拨杆
*		上：底盘动作组控制模式					上：底盘开启动作组一 中：底盘启动动作组二 下：底盘关闭动作组三	   遥感：吸盘和底盘方向
*		中：机械臂与吸盘控制模式				上：关吸盘操控机械臂 中：开吸盘操控机械臂 下：关吸盘操控抬升推出   遥感：四轴机械臂或抬升推出机构
*		下：底盘动作组控制模式					上：底盘开启动作组四 中：底盘启动动作组五 下：底盘关闭动作组六	   遥感：吸盘和底盘方向
*/
void Message_bag::SolveLeftPlunk()
{
	static u8 LeftPlunkFlag = 0;
	//debug提示或者UI使用
	if(LeftPlunkFlag != switch_is_up(RC_ctrl->rc.s[S1_CHANNEL]))
	{
			RobotBehaviorRC.RobotBehaviorReset();
	}
	//左拨杆
	if (switch_is_up(RC_ctrl->rc.s[S1_CHANNEL]))//底盘动作组控制模式	
	{
		RobotBehaviorRC.RobotMode = RobotChassisAction_m1;
		SolveRightPlunk();
	}
	else if (switch_is_mid(RC_ctrl->rc.s[S1_CHANNEL]))//机械臂与吸盘控制模式
	{
		RobotBehaviorRC.RobotMode = RobotArm_m;
		SolveRightPlunk();
	}
	else if (switch_is_down(RC_ctrl->rc.s[S1_CHANNEL]))//底盘与抬升控制模式
	{
		//RobotBehaviorRC.RobotMode = RobotChassisAction_m2;
		RobotBehaviorRC.RobotMode = RobotChassis_m;
		SolveRightPlunk();
	}
	//存储上一次的左拨杆数值
	LeftPlunkFlag = RC_ctrl->rc.s[S2_CHANNEL];
};
void Message_bag::SolveRightPlunk()
{
	static u8 RightPlunkFlag = 0;
	//debug提示或者UI使用
	if(RightPlunkFlag != switch_is_up(RC_ctrl->rc.s[S2_CHANNEL]))
	{
			RobotBehaviorRC.RobotBehaviorReset();
	}
	
	switch(RobotBehaviorRC.RobotMode)
	{
		case RobotArm_m:
		{
			RobotBehaviorRC.IsArmON =1;
			Chassis_state = Chassis_KEEP;
			//右拨杆 
			//机械臂与吸盘控制模式
			//当右拨杆位于中间时候关闭吸盘并且取消动作保护
			if (switch_is_up(RC_ctrl->rc.s[S2_CHANNEL]))
			{
				RobotBehaviorRC.IsSuckerON=true;
				RobotBehaviorRC.IsArmKeep=true;
			}
			else if (switch_is_mid(RC_ctrl->rc.s[S2_CHANNEL]))
			{
				RobotBehaviorRC.IsSuckerON=false;
				RobotBehaviorRC.IsArmKeep=false;
			}
			else if (switch_is_down(RC_ctrl->rc.s[S2_CHANNEL]))
			{
				RobotBehaviorRC.IsSuckerON=true;
				RobotBehaviorRC.IsArmKeep=true;
			}			
			break;
		}
		case RobotChassisAction_m1:
		{
			RobotBehaviorRC.IsSuckerON=true;
			RobotBehaviorRC.IsChassisON = true;
			Chassis_state = Chassis_FOLLOW_L1;
			
			if(RobotBehaviorRC.IsArmKeep==true)
			{
				RobotBehaviorRC.ActionGroup = KEEP;
				break;//动作保护
			}
			//右拨杆
			//底盘动作组控制模式
			if (switch_is_up(RC_ctrl->rc.s[S2_CHANNEL]))
			{			
				RobotBehaviorRC.ActionGroup = AirCatch;//空接
			}
			else if (switch_is_mid(RC_ctrl->rc.s[S2_CHANNEL]))
			{			
				RobotBehaviorRC.ActionGroup = Init;//初始动作
			}
			else if (switch_is_down(RC_ctrl->rc.s[S2_CHANNEL]))
			{
				RobotBehaviorRC.ActionGroup = PutBack;//放矿
			}			
			break;
		}
		case RobotChassisAction_m2:
		{
			RobotBehaviorRC.IsSuckerON=true;
			RobotBehaviorRC.IsChassisON = true;
			Chassis_state = Chassis_KEEP;
			
			if(RobotBehaviorRC.IsArmKeep==true)
			{
				RobotBehaviorRC.ActionGroup = KEEP;
				break;//动作保护
			}
			//右拨杆
			//底盘动作组控制模式
			if (switch_is_up(RC_ctrl->rc.s[S2_CHANNEL]))
			{
				RobotBehaviorRC.ActionGroup = CatchGround;//抓地上
			}
			else if (switch_is_mid(RC_ctrl->rc.s[S2_CHANNEL]))
			{
				RobotBehaviorRC.ActionGroup = Stop;//停止在原地
			}
			else if (switch_is_down(RC_ctrl->rc.s[S2_CHANNEL]))
			{
				RobotBehaviorRC.ActionGroup = CatchMid;//抓小资源岛
			}			
			break;
		}
		case RobotChassis_m:
		{
			RobotBehaviorRC.IsSuckerON=false;
			RobotBehaviorRC.IsChassisON = true;
			Chassis_state = Chassis_FOLLOW_L1;
			//抬升机构处理标志 ch0 ch1
			//右拨杆
			//底盘与抬升控制模式
			if (switch_is_up(RC_ctrl->rc.s[S2_CHANNEL]))
			{
				Chassis_state = Chassis_UP_PUSH;
			}
			else if (switch_is_mid(RC_ctrl->rc.s[S2_CHANNEL]))
			{
				Chassis_state = Chassis_FOLLOW_L1;
			}
			else if (switch_is_down(RC_ctrl->rc.s[S2_CHANNEL]))
			{
				Chassis_state = Chassis_NO_MOVE;
			}	
			break;
		}
		default:
		{
			//出大问题 复位
			System_Reset();
			break;
		}
	}
	
	//存储上一次的右拨杆数值
	RightPlunkFlag = RC_ctrl->rc.s[S2_CHANNEL];
};
void Message_bag::RC_to_robotstate(void)
{
	SolveLeftPlunk();
	//底盘模式下开关吸盘
	if(RobotBehaviorRC.IsSuckerON )
	{
		if(ABS(RC_ctrl->rc.ch[1])>200)
		{
			GPIO_SetBits(GPIOI,GPIO_Pin_2);
		}else
		{
			GPIO_ResetBits(GPIOI,GPIO_Pin_2);
		}
	} 
	
	
	if(RobotBehaviorRC.IsArmON == 1 && RobotBehaviorRC.IsSuckerON == true)
	{ 
		GPIO_SetBits(GPIOI,GPIO_Pin_2);
	}else if(RobotBehaviorRC.IsArmON == 1 && RobotBehaviorRC.IsSuckerON == false)
	{
		GPIO_ResetBits(GPIOI,GPIO_Pin_2);
	}
}

void Message_bag::PC_to_robotstate(void)
{
	if (RC_ctrl->key.v & KEY_PRESSED_OFFSET_F) {
		//key_num_sum(1, &key_f);//j3
		key_long_press(1, &key_f);
	}
	else {
		//key_num_sum(0, &key_f);
		key_long_press(0, &key_f);
	}
	
	if (RC_ctrl->key.v & KEY_PRESSED_OFFSET_C) {
		//key_num_sum(1, &key_c);//j2
		key_long_press(1, &key_c);
	}
	else {
		//key_num_sum(0, &key_c);
		key_long_press(0, &key_c);
	}
	
	if (RC_ctrl->key.v & KEY_PRESSED_OFFSET_X) {
		//key_num_sum(1, &key_x);//j1
		key_long_press(1, &key_x);
	}
	else {
		//key_num_sum(0, &key_x);
		key_long_press(0, &key_x);
	}
	
	if (RC_ctrl->key.v & KEY_PRESSED_OFFSET_Z) {
		//key_num_sum(1, &key_z);//j0
		key_long_press(1, &key_z);
	}
	else {
		//key_num_sum(0, &key_z);
		key_long_press(0, &key_z);
	}
	
	if (RC_ctrl->key.v & KEY_PRESSED_OFFSET_Q) {
		key_num_sum(1, &key_q);//抬起
//		Chassis_state=Chassis_UP_PUSH;
	}
	else {
		key_num_sum(0, &key_q);
	}
	
	if (RC_ctrl->key.v & KEY_PRESSED_OFFSET_E) {
		key_num_sum(1, &key_e);//伸出
//		Chassis_state=Chassis_UP_PUSH;
	}
	else {
		key_num_sum(0, &key_e);
	}
	
	if (RC_ctrl->key.v & KEY_PRESSED_OFFSET_R) {
		key_num_sum(1, &key_r);//底盘与云台是否独立
	}
	else {
		key_num_sum(0, &key_r);
	}
	
	if (RC_ctrl->key.v & KEY_PRESSED_OFFSET_V) {
		key_num_sum(1, &key_v);//空接
	}
	else {
		key_num_sum(0, &key_v);
	}

	
	if (RC_ctrl->key.v & KEY_PRESSED_OFFSET_G) {
		key_num_sum(1, &key_g);//放回
	}
	else {
		key_num_sum(0, &key_g);
	}
	
	if (RC_ctrl->key.v & KEY_PRESSED_OFFSET_R) {
		key_num_sum(1, &key_r);//地面
	}
	else {
		key_num_sum(0, &key_r);
	}
	
	if (RC_ctrl->key.v & KEY_PRESSED_OFFSET_G) {
		key_num_sum(1, &key_g);//小资源岛
	}
	else {
		key_num_sum(0, &key_g);
	}
	
	if (RC_ctrl->mouse.press_l) {
		key_num_sum(1, &mouse_l);//开关吸盘
	}
	else {
		key_num_sum(0, &mouse_l);
	}
	
	if (RC_ctrl->mouse.press_r) {
		key_num_sum(1, &mouse_r);//机械臂回正
	}
	else {
		key_num_sum(0, &mouse_r);
	}

	if (RC_ctrl->key.v & KEY_PRESSED_OFFSET_SHIFT) {
		key_long_press(1, &shift_pos);//正系数
	}
	else {
		key_long_press(0, &shift_pos);
	}

	if (RC_ctrl->key.v & KEY_PRESSED_OFFSET_CTRL) {
		key_long_press(1, &ctrl_neg);//负系数
	}
	else {
		key_long_press(0, &ctrl_neg);
	}
	
	if (RC_ctrl->key.v & KEY_PRESSED_OFFSET_B) {
		key_num_sum(1, &key_b);//放回
	}
	else {
		key_num_sum(0, &key_b);
	}
}

void Message_bag::ARMControl()
{
	if(mouse_l.key_count == 1 && RobotBehaviorPC.IsSuckerON==0)//吸盘
	{
		RobotBehaviorPC.IsSuckerON = 1;
		GPIO_SetBits(GPIOI,GPIO_Pin_2);
		key_clear(&mouse_l);
	}else if(mouse_l.key_count == 1 && RobotBehaviorPC.IsSuckerON==1)
	{
		RobotBehaviorPC.IsSuckerON = 0;
		GPIO_ResetBits(GPIOI,GPIO_Pin_2);
		key_clear(&mouse_l);
	}
	
//	if(mouse_r.key_count == 1 && RobotBehaviorPC.IsArmInitON==0)//机械臂回正开关
//	{
//		RobotBehaviorPC.IsArmInitON = 1;
//		RobotBehaviorPC.ActionGroup = Init;//回正
//		
//		key_clear(&mouse_r);
//	}else if(mouse_r.key_count == 1 && RobotBehaviorPC.IsArmInitON==1)
//	{
//		RobotBehaviorPC.IsArmInitON = 0;
//		RobotBehaviorPC.ActionGroup = PutBack;//放回
//		
//		key_clear(&mouse_r);
//	}
//	
//	if(key_v.key_count == 1)
//	{
//		RobotBehaviorPC.ActionGroup = AirCatch;//空接
//		
//		key_clear(&key_v);
//	}
//	else if(key_g.key_count == 1)
//	{
//		RobotBehaviorPC.ActionGroup = CatchGround;//地面
//		
//		key_clear(&key_g);
//	}
	
//	if(key_r.key_count == 1 && RobotBehaviorPC.IsFollowON==0)//跟随
//	{
//		RobotBehaviorPC.IsFollowON = 1;
//		Chassis_state = Chassis_FOLLOW;
//		key_clear(&key_r);
//	}else if(key_r.key_count == 1 && RobotBehaviorPC.IsFollowON==1)
//	{
//		RobotBehaviorPC.IsFollowON = 0;
//		Chassis_state = Chassis_NO_MOVE;
//		key_clear(&key_r);
//	}

	if(key_r.key_count == 1 && RobotBehaviorPC.SpeedLevel==0)//跟随
	{
		RobotBehaviorPC.SpeedLevel = 1;
		Chassis_state = Chassis_FOLLOW_L1;
		key_clear(&key_r);
	}else if(key_r.key_count == 1 && RobotBehaviorPC.SpeedLevel==1)
	{
		RobotBehaviorPC.SpeedLevel = 2;
		Chassis_state = Chassis_FOLLOW_L2;
		key_clear(&key_r);
  }else if(key_r.key_count == 1 && RobotBehaviorPC.SpeedLevel==2)
	{
		RobotBehaviorPC.SpeedLevel = 0;
		Chassis_state = Chassis_NO_MOVE;
		key_clear(&key_r);
	}

	if(key_b.key_count == 1)//开启图传，锁定底盘
	{
		RobotBehaviorPC.SpeedLevel = 0;
		Chassis_state = Chassis_NO_MOVE;
		key_clear(&key_b);
	}
	
	if ((switch_is_up(RC_ctrl->rc.s[S2_CHANNEL]) && switch_is_up(RC_ctrl->rc.s[S1_CHANNEL])))//全部朝上复位
	{
		System_Reset();
	}
}
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
	int32_t relative_ecd = ecd - offset_ecd;
	if (relative_ecd > 4096)
	{
		relative_ecd -= 8191;
	}
	else if (relative_ecd < -4096)
	{
		relative_ecd += 8191;
	}

	return (relative_ecd * (0.0439453f));
}

static void System_Reset(void)
{
	SCB->AIRCR = ((0x5FA << SCB_AIRCR_VECTKEY_Pos) |
		(SCB->AIRCR & SCB_AIRCR_PRIGROUP_Msk) |
		SCB_AIRCR_SYSRESETREQ_Msk);
}

void Message_bag::Message_Send(void)
{

   SendData(0x126,limit.dowm_limit,limit.up_limit,limit.back_limit,limit.front_limit,RobotBehaviorPC.IsSuckerON,RobotBehaviorPC.SpeedLevel);
}


void Message_bag::Message_Feedback(void)
{
	limit.dowm_limit=GPIO_ReadInputDataBit(GPIOH,GPIO_Pin_11);
  limit.up_limit  =GPIO_ReadInputDataBit(GPIOH,GPIO_Pin_12);
	limit.back_limit=GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_12);
  limit.front_limit  =GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_13);
}

void Message_bag::SendData(uint32_t StdId, int16_t data1, int16_t data2, int16_t data3, int16_t data4 ,int16_t data5,int16_t data6)
{
    CanTxMsg TxMessage;
    TxMessage.StdId = StdId;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = data1;
    TxMessage.Data[1] = data2;
    TxMessage.Data[2] = data3;
    TxMessage.Data[3] = data4;
    TxMessage.Data[4] = data5;
    TxMessage.Data[5] = data6;

    CAN_Transmit( CAN2, &TxMessage );
}

void Message_bag::SendData(uint32_t StdId, int16_t data1, int16_t data2, int16_t data3, int16_t data4 )
{
    CanTxMsg TxMessage;
    TxMessage.StdId = StdId;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = data1;
    TxMessage.Data[1] = data2;
    TxMessage.Data[2] = data3;
    TxMessage.Data[3] = data4;

    CAN_Transmit( CAN2, &TxMessage );
}

void Message_bag::SendData(uint32_t StdId, int16_t data1, int16_t data2, int16_t data3 )
{
    CanTxMsg TxMessage;
    TxMessage.StdId = StdId;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = data1;
    TxMessage.Data[1] = data2;
    TxMessage.Data[2] = data3;

    CAN_Transmit( CAN2, &TxMessage );
}

void Message_bag::SendData(uint32_t StdId, int16_t data1, int16_t data2 )
{
    CanTxMsg TxMessage;
    TxMessage.StdId = StdId;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = data1;
    TxMessage.Data[1] = data2;

    CAN_Transmit( CAN2, &TxMessage );
}

void Message_bag::SendData(uint32_t StdId, int16_t data1 )
{
    CanTxMsg TxMessage;
    TxMessage.StdId = StdId;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = data1;

    CAN_Transmit( CAN2, &TxMessage );
}
