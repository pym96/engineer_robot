#include "Message_Task.h"
//#include "app_usart7.h"
//#include "start_task.h"
#include "motor_control.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "RGB.h"
#define u8 uint8_t

uint8_t Message_flag = 0;

Message_bag message_bag;
//MOTOR_send cmd; 

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern TIM_HandleTypeDef htim1;

static void System_Reset(void);

void Message_task()
{
	message_bag.Messag_bag_Init();//初始化
//	cmd.id=0; 			//给电机控制指令结构体赋值
//	cmd.mode=1;
//	cmd.T=0;
//	cmd.W=0;
//	cmd.Pos=0;
//	cmd.K_P=0;
//	cmd.K_W=0.05;

	vTaskDelay(20);
	while (1)
	{
		Message_flag++;
		message_bag.Operate_to_robotstate();//遥控器模式控制
		message_bag.ARMControl();
//		message_bag.Message_Feedback();
		message_bag.Message_Send();
//    WS2812_SET_HSV(360,100,1,4);
		vTaskDelay(10);
	}
}

void Message_bag::Messag_bag_Init(void)
{
	RC_ctrl = get_remote_control_point();
	RobotBehaviorPC.IsArmInitON = 1;
	RobotBehaviorPC.ActionGroup = Init;//回正
	steering.steering_pitch=850;
	steering.steering_yaw  =2222;
	steering.steering_state=0;
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
			//抬升机构处理标志 ch0 ch1
			//右拨杆
			//底盘与抬升控制模式
			if (switch_is_up(RC_ctrl->rc.s[S2_CHANNEL]))
			{
				
			}
			else if (switch_is_mid(RC_ctrl->rc.s[S2_CHANNEL]))
			{
				
			}
			else if (switch_is_down(RC_ctrl->rc.s[S2_CHANNEL]))
			{

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
		//key_num_sum(1, &key_b);//
		key_num_sum(1, &key_b);
	}
	else {
		//key_num_sum(0, &key_b);
		key_num_sum(0, &key_b);
	}
}

void Message_bag::ARMControl()
{
	if(mouse_r.key_count == 1 && RobotBehaviorPC.IsArmInitON==0)//机械臂回正开关
	{
		RobotBehaviorPC.IsArmInitON = 1;
		RobotBehaviorPC.ActionGroup = Init;//回正
		
		key_clear(&mouse_r);
	}else if(mouse_r.key_count == 1 && RobotBehaviorPC.IsArmInitON==1)
	{
		RobotBehaviorPC.IsArmInitON = 0;
		RobotBehaviorPC.ActionGroup = PutBack;//放回
		
		key_clear(&mouse_r);
	}
	
	if(key_v.key_count == 1)
	{
		RobotBehaviorPC.ActionGroup = AirCatch;//空接
		
		key_clear(&key_v);
	}
	else if(key_g.key_count == 1)
	{
		RobotBehaviorPC.ActionGroup = CatchGround;//地面
		
		key_clear(&key_g);
	}
	
//	if(key_r.key_count == 1 && RobotBehaviorPC.IsFollowON==0)//跟随
//	{
//		RobotBehaviorPC.IsFollowON = 1;
//		key_clear(&key_r);
//	}else if(key_r.key_count == 1 && RobotBehaviorPC.IsFollowON==1)
//	{
//		RobotBehaviorPC.IsFollowON = 0;
//		key_clear(&key_r);
//	}
	
	if(key_r.key_count == 1)
	{
		RobotBehaviorPC.SpeedLevel++;
		key_clear(&key_r);
	}
	if ((switch_is_up(RC_ctrl->rc.s[S2_CHANNEL]) && switch_is_up(RC_ctrl->rc.s[S1_CHANNEL])))//全部朝上复位
	{
		System_Reset();
	}
}

static void System_Reset(void)
{
	SCB->AIRCR = ((0x5FA << SCB_AIRCR_VECTKEY_Pos) |
		(SCB->AIRCR & SCB_AIRCR_PRIGROUP_Msk) |
		SCB_AIRCR_SYSRESETREQ_Msk);
}

void Message_bag::Message_Send(void)
{
//  SendData(0x127,limit.front_limit,limit.back_limit);
  if(chassis.limit.up_limit==1)
		WS2812_SET_HSV(0,100,1,2);//red
	else
		WS2812_SET_HSV(120,100,1,2);//green
	
	
  if(chassis.limit.dowm_limit==1)
		WS2812_SET_HSV(0,100,1,3);//red
	else
		WS2812_SET_HSV(120,100,1,3);//green
	
	
  if(chassis.limit.front_limit==1)
		WS2812_SET_HSV(0,100,1,4);//red
	else
		WS2812_SET_HSV(120,100,1,4);//green
	
	
  if(chassis.limit.back_limit==1)
		WS2812_SET_HSV(0,100,1,5);//red
	else
		WS2812_SET_HSV(120,100,1,5);//green
	
	
  if(chassis.sucker_state==1)
	{
		WS2812_SET_HSV(120,100,1,1);//green
		 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6,GPIO_PIN_SET);
	}
	else
	{
		WS2812_SET_HSV(0,100,1,1);//red
		 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6,GPIO_PIN_RESET);

	}
	
	
  if(chassis.speed_level==0)
		WS2812_SET_HSV(0,100,1,6);//red
	else if(chassis.speed_level==1)
		WS2812_SET_HSV(240,100,1,6);//red
	else
		WS2812_SET_HSV(120,100,1,6);//red
	
  image_control();
}


void Message_bag::Message_Feedback(void)
{
//	limit.front_limit=HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_11);
//  limit.back_limit  =HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_13);
}


void Message_bag::SendData(uint32_t StdId, uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4 )
{
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t TxMailbox;  
	
    TxHeader.StdId = StdId;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 0x04;
    TxData[0] = data1;
    TxData[1] = data2;
    TxData[2] = data3;
    TxData[3] = data4;
	
  if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
  {

  }
}

void Message_bag::SendData(uint32_t StdId, uint8_t data1, uint8_t data2, uint8_t data3 )
{
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t TxMailbox;  
	
    TxHeader.StdId = StdId;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 0x03;
    TxData[0] = data1;
    TxData[1] = data2;
    TxData[2] = data3;
	
  if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
  {

  }
}

void Message_bag::SendData(uint32_t StdId, uint8_t data1, uint8_t data2 )
{
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t TxMailbox;  
	
    TxHeader.StdId = StdId;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 0x02;
    TxData[0] = data1;
    TxData[1] = data2;
	
  if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
  {

  }
}

void Message_bag::SendData(uint32_t StdId, uint8_t data1 )
{
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t TxMailbox;  
	
    TxHeader.StdId = StdId;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 0x08;
    TxData[0] = data1;

	
  if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
  {

  }
}

void Message_bag::image_control(void)
{
	
//	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, steering.steering_pitch);
//	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, steering.steering_yaw);
//	   steering.steering_yaw   +=RC_ctrl->mouse.x *(- 0.015);
//     steering.steering_pitch +=RC_ctrl->mouse.y *(- 0.01);
//		 steering.steering_pitch=constrain(steering.steering_pitch,500,1050);
//		 steering.steering_yaw=constrain(steering.steering_yaw,500,2500);
	
	if(ctrl_neg.key_count)
	{
		if(key_b.key_count)
		{
			steering.steering_state=2;
			key_clear(&key_b);
			key_clear(&ctrl_neg);
		}
	}
	else
	{
		if(key_b.key_count)
		{
			steering.steering_state++;
			if(steering.steering_state>=2)
				steering.steering_state=0;
			key_clear(&key_b);
		}
	}
	if(steering.steering_state==1)
	{
     steering.steering_yaw   +=RC_ctrl->mouse.x *(- 0.015);
     steering.steering_pitch +=RC_ctrl->mouse.y *(- 0.01);
		 steering.steering_pitch=constrain(steering.steering_pitch,500,1050);
		 steering.steering_yaw=constrain(steering.steering_yaw,500,2500);
	}
	
	if(steering.steering_state==2)
	{
	steering.steering_pitch=850;
	steering.steering_yaw  =2222;
	}

	if(	steering.steering_yaw  ==2222)	
		WS2812_SET_HSV(120,100,1,7);//green
	else
		WS2812_SET_HSV(0,100,1,7);//red

	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1,  steering.steering_pitch);
	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4,  steering.steering_yaw);
	
}

