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

static void System_Reset(void);

void Message_task()
{
	message_bag.Messag_bag_Init();//��ʼ��
//	cmd.id=0; 			//���������ָ��ṹ�帳ֵ
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
		message_bag.Operate_to_robotstate();//ң����ģʽ����
		message_bag.ARMControl();
		message_bag.Message_Feedback();
		message_bag.Message_Send();
//    WS2812_SET_HSV(360,100,1,4);
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
	
	//�жϳ���
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
*		�󲦸� 													�Ҳ���
*		�ϣ����̶��������ģʽ					�ϣ����̿���������һ �У���������������� �£����̹رն�������	   ң�У����̺͵��̷���
*		�У���е�������̿���ģʽ				�ϣ������̲ٿػ�е�� �У������̲ٿػ�е�� �£������̲ٿ�̧���Ƴ�   ң�У������е�ۻ�̧���Ƴ�����
*		�£����̶��������ģʽ					�ϣ����̿����������� �У����������������� �£����̹رն�������	   ң�У����̺͵��̷���
*/
void Message_bag::SolveLeftPlunk()
{
	static u8 LeftPlunkFlag = 0;
	//debug��ʾ����UIʹ��
	if(LeftPlunkFlag != switch_is_up(RC_ctrl->rc.s[S1_CHANNEL]))
	{
			RobotBehaviorRC.RobotBehaviorReset();
	}
	//�󲦸�
	if (switch_is_up(RC_ctrl->rc.s[S1_CHANNEL]))//���̶��������ģʽ	
	{
		RobotBehaviorRC.RobotMode = RobotChassisAction_m1;
		SolveRightPlunk();
	}
	else if (switch_is_mid(RC_ctrl->rc.s[S1_CHANNEL]))//��е�������̿���ģʽ
	{
		RobotBehaviorRC.RobotMode = RobotArm_m;
		SolveRightPlunk();
	}
	else if (switch_is_down(RC_ctrl->rc.s[S1_CHANNEL]))//������̧������ģʽ
	{
		//RobotBehaviorRC.RobotMode = RobotChassisAction_m2;
		RobotBehaviorRC.RobotMode = RobotChassis_m;
		SolveRightPlunk();
	}
	//�洢��һ�ε��󲦸���ֵ
	LeftPlunkFlag = RC_ctrl->rc.s[S2_CHANNEL];
};
void Message_bag::SolveRightPlunk()
{
	static u8 RightPlunkFlag = 0;
	//debug��ʾ����UIʹ��
	if(RightPlunkFlag != switch_is_up(RC_ctrl->rc.s[S2_CHANNEL]))
	{
			RobotBehaviorRC.RobotBehaviorReset();
	}
	
	switch(RobotBehaviorRC.RobotMode)
	{
		case RobotArm_m:
		{
			RobotBehaviorRC.IsArmON =1;
			//�Ҳ��� 
			//��е�������̿���ģʽ
			//���Ҳ���λ���м�ʱ��ر����̲���ȡ����������
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
				break;//��������
			}
			//�Ҳ���
			//���̶��������ģʽ
			if (switch_is_up(RC_ctrl->rc.s[S2_CHANNEL]))
			{			
				RobotBehaviorRC.ActionGroup = AirCatch;//�ս�
			}
			else if (switch_is_mid(RC_ctrl->rc.s[S2_CHANNEL]))
			{			
				RobotBehaviorRC.ActionGroup = Init;//��ʼ����
			}
			else if (switch_is_down(RC_ctrl->rc.s[S2_CHANNEL]))
			{
				RobotBehaviorRC.ActionGroup = PutBack;//�ſ�
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
				break;//��������
			}
			//�Ҳ���
			//���̶��������ģʽ
			if (switch_is_up(RC_ctrl->rc.s[S2_CHANNEL]))
			{
				RobotBehaviorRC.ActionGroup = CatchGround;//ץ����
			}
			else if (switch_is_mid(RC_ctrl->rc.s[S2_CHANNEL]))
			{
				RobotBehaviorRC.ActionGroup = Stop;//ֹͣ��ԭ��
			}
			else if (switch_is_down(RC_ctrl->rc.s[S2_CHANNEL]))
			{
				RobotBehaviorRC.ActionGroup = CatchMid;//ץС��Դ��
			}			
			break;
		}
		case RobotChassis_m:
		{
			RobotBehaviorRC.IsSuckerON=false;
			RobotBehaviorRC.IsChassisON = true;
			//̧�����������־ ch0 ch1
			//�Ҳ���
			//������̧������ģʽ
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
			//�������� ��λ
			System_Reset();
			break;
		}
	}
	
	//�洢��һ�ε��Ҳ�����ֵ
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
		key_num_sum(1, &key_q);//̧��
//		Chassis_state=Chassis_UP_PUSH;
	}
	else {
		key_num_sum(0, &key_q);
	}
	
	if (RC_ctrl->key.v & KEY_PRESSED_OFFSET_E) {
		key_num_sum(1, &key_e);//���
//		Chassis_state=Chassis_UP_PUSH;
	}
	else {
		key_num_sum(0, &key_e);
	}
	
	if (RC_ctrl->key.v & KEY_PRESSED_OFFSET_R) {
		key_num_sum(1, &key_r);//��������̨�Ƿ����
	}
	else {
		key_num_sum(0, &key_r);
	}
	
	if (RC_ctrl->key.v & KEY_PRESSED_OFFSET_V) {
		key_num_sum(1, &key_v);//�ս�
	}
	else {
		key_num_sum(0, &key_v);
	}

	
	if (RC_ctrl->key.v & KEY_PRESSED_OFFSET_G) {
		key_num_sum(1, &key_g);//�Ż�
	}
	else {
		key_num_sum(0, &key_g);
	}
	
	if (RC_ctrl->key.v & KEY_PRESSED_OFFSET_R) {
		key_num_sum(1, &key_r);//����
	}
	else {
		key_num_sum(0, &key_r);
	}
	
	if (RC_ctrl->key.v & KEY_PRESSED_OFFSET_G) {
		key_num_sum(1, &key_g);//С��Դ��
	}
	else {
		key_num_sum(0, &key_g);
	}
	
	if (RC_ctrl->mouse.press_l) {
		key_num_sum(1, &mouse_l);//��������
	}
	else {
		key_num_sum(0, &mouse_l);
	}
	
	if (RC_ctrl->mouse.press_r) {
		key_num_sum(1, &mouse_r);//��е�ۻ���
	}
	else {
		key_num_sum(0, &mouse_r);
	}

	if (RC_ctrl->key.v & KEY_PRESSED_OFFSET_SHIFT) {
		key_long_press(1, &shift_pos);//��ϵ��
	}
	else {
		key_long_press(0, &shift_pos);
	}

	if (RC_ctrl->key.v & KEY_PRESSED_OFFSET_CTRL) {
		key_long_press(1, &ctrl_neg);//��ϵ��
	}
	else {
		key_long_press(0, &ctrl_neg);
	}
	
	if (RC_ctrl->key.v & KEY_PRESSED_OFFSET_B) {
		//key_num_sum(1, &key_b);//
		key_long_press(1, &key_b);
	}
	else {
		//key_num_sum(0, &key_b);
		key_long_press(0, &key_b);
	}
}

void Message_bag::ARMControl()
{

	
	if(mouse_r.key_count == 1 && RobotBehaviorPC.IsArmInitON==0)//��е�ۻ�������
	{
		RobotBehaviorPC.IsArmInitON = 1;
		RobotBehaviorPC.ActionGroup = Init;//����
		
		key_clear(&mouse_r);
	}else if(mouse_r.key_count == 1 && RobotBehaviorPC.IsArmInitON==1)
	{
		RobotBehaviorPC.IsArmInitON = 0;
		RobotBehaviorPC.ActionGroup = PutBack;//�Ż�
		
		key_clear(&mouse_r);
	}
	
	if(key_v.key_count == 1)
	{
		RobotBehaviorPC.ActionGroup = AirCatch;//�ս�
		
		key_clear(&key_v);
	}
	else if(key_g.key_count == 1)
	{
		RobotBehaviorPC.ActionGroup = CatchGround;//����
		
		key_clear(&key_g);
	}
	
	if(key_r.key_count == 1 && RobotBehaviorPC.IsFollowON==0)//����
	{
		RobotBehaviorPC.IsFollowON = 1;
		key_clear(&key_r);
	}else if(key_r.key_count == 1 && RobotBehaviorPC.IsFollowON==1)
	{
		RobotBehaviorPC.IsFollowON = 0;
		key_clear(&key_r);
	}
	
	if ((switch_is_up(RC_ctrl->rc.s[S2_CHANNEL]) && switch_is_up(RC_ctrl->rc.s[S1_CHANNEL])))//ȫ�����ϸ�λ
	{
		System_Reset();
	}
}
//static float motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
//{
//	int32_t relative_ecd = ecd - offset_ecd;
//	if (relative_ecd > 4096)
//	{
//		relative_ecd -= 8191;
//	}
//	else if (relative_ecd < -4096)
//	{
//		relative_ecd += 8191;
//	}

//	return (relative_ecd * (0.0439453f));
//}

static void System_Reset(void)
{
	SCB->AIRCR = ((0x5FA << SCB_AIRCR_VECTKEY_Pos) |
		(SCB->AIRCR & SCB_AIRCR_PRIGROUP_Msk) |
		SCB_AIRCR_SYSRESETREQ_Msk);
}

void Message_bag::Message_Send(void)
{
   SendData(0x126,limit.front_limit,limit.back_limit);
}


void Message_bag::Message_Feedback(void)
{
	limit.front_limit=HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_11);
  limit.back_limit  =HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_13);
}


void Message_bag::SendData(uint32_t StdId, int16_t data1, int16_t data2, int16_t data3, int16_t data4 )
{
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t TxMailbox;  
	
    TxHeader.StdId = StdId;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 0x08;
    TxData[0] = data1;
    TxData[1] = data2;
    TxData[2] = data3;
    TxData[3] = data4;
	
  if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
  {

  }
}

void Message_bag::SendData(uint32_t StdId, int16_t data1, int16_t data2, int16_t data3 )
{
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t TxMailbox;  
	
    TxHeader.StdId = StdId;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 0x08;
    TxData[0] = data1;
    TxData[1] = data2;
    TxData[2] = data3;
	
  if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
  {

  }
}

void Message_bag::SendData(uint32_t StdId, int16_t data1, int16_t data2 )
{
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t TxMailbox;  
	
    TxHeader.StdId = StdId;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 0x08;
    TxData[0] = data1;
    TxData[1] = data2;
	
  if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
  {

  }
}

void Message_bag::SendData(uint32_t StdId, int16_t data1 )
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
	if(key_b.key_count)
	{
		
	}
	
}

