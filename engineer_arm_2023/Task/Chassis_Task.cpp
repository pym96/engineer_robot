#include "Chassis_Task.h"
#include "app_usart7.h"
#include "math.h"

Chassis_Ctrl Chassis_ctrl;

uint8_t Chassis_flag = 0;
int test=0;

void Chassis_Task(void* pvParameters)
{
	static BaseType_t err = pdFALSE;
	vTaskDelay(200);  /*空闲一段时间*/
	Chassis_ctrl.Chassis_Init();

	while (1)
	{ 
//		Chassis_flag++;
		Chassis_ctrl.Limit_power_chassis_output();//获取等级状态 以及速度限制
		Chassis_ctrl.Chassia_Motor_Feedback_Update();
		Chassis_ctrl.UP_PUSH_Motor_Feedback_Update();
		Chassis_ctrl.chassis_set_contorl();
		Chassis_ctrl.up_push_set_contorl();
		Chassis_ctrl.chassis_control_loop();
    Chassis_ctrl.Up_Push_loop();
		Chassis_ctrl.Chassis_Motor_ctrl();
    Chassis_ctrl.Up_Push_Move();		
		
    vTaskDelay(5);
	}
}

/**********************************************************************************/
//底盘速度环pid值
const static fp32 Motor_Speed_Pid[3] = { M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD };
//抬升推出pid值
const static fp32 UP_Speed_Pid[3] =   { 25000, 18, 0 };
const static fp32 PUSH_Speed_Pid[3] = { 15000, 0.5, 0 };
//底盘旋转环pid值
const static fp32 Chassis_Follow_Gimbal_Pid[3] = { CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD };
//底盘校准PID
const static fp32 chassis_angle_pid[3] = { CHASSIS_ANGLE_POSITION_PID_KP, CHASSIS_ANGLE_POSITION_PID_KI, CHASSIS_ANGLE_POSITION_PID_KD };
const static fp32 chassis_gyro_pid[3] = { CHASSIS_GYRO_SPEED_PID_KP, CHASSIS_GYRO_SPEED_PID_KI, CHASSIS_GYRO_SPEED_PID_KD };
const static fp32 chassis_x_order_filter[1] = { CHASSIS_ACCEL_X_NUM };
const static fp32 chassis_y_order_filter[1] = { CHASSIS_ACCEL_Y_NUM };
const static fp32 chassis_z_order_filter[1] = { CHASSIS_ACCEL_Z_NUM };
/**********************************************************************************/

void Chassis_Ctrl::Chassis_Init(void)
{
	uint8_t i = 0;
	uint8_t j = 0;
	
	Chassis_message = &message_bag;

	for (i = 0; i < 4; i++)//电机PID初始化 以及 电机数据结构指针获取
	{
		Motor[i].chassis_motor_measure = Motor_Ctrl.Chassis.Get_Motor_Measure_Pointer(i);
		gMotor[i].up_push_motor_measure= Motor_Ctrl.Up_Push.Get_Motor_Measure_Pointer(i);
		PID_Init(&Speed_Pid[i], PID_POSITION, Motor_Speed_Pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
//		PID_Init(&Up_Push_Pid[i], PID_POSITION, UP_PUSH_Speed_Pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
	}
	for(j =0; j< 4 ; j++)
	{
		if(j<=1)
		PID_Init(&Up_Push_Pid[j], PID_POSITION, PUSH_Speed_Pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
		else
		PID_Init(&Up_Push_Pid[j], PID_POSITION, UP_Speed_Pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);		
		
	}
	//初始化校准PID
	PID_Init(&chassis_setangle, PID_POSITION, chassis_angle_pid, CHASSIS_ANGLE_POSITION_PID_MAX_OUT, CHASSIS_ANGLE_POSITION_PID_MAX_IOUT);
	PID_Init(&chassis_setangle_gyro, PID_POSITION, chassis_gyro_pid, CHASSIS_GYRO_SPEED_PID_MAX_OUT, CHASSIS_GYRO_SPEED_PID_MAX_IOUT);
	//用一阶滤波代替斜波函数生成
	first_order_filter_init(&Filter_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
	first_order_filter_init(&Filter_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);
	first_order_filter_init(&Filter_vw, CHASSIS_CONTROL_TIME, chassis_z_order_filter);

	//最大（max） 与反方向最大（min） 速度
	Velocity.vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
	Velocity.vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;

	Velocity.vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
	Velocity.vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;

	//修正系数
	head_wheel = 0.2;
	tail_wheel = 0.205;

	Velocity.Speed_Set[0] = Velocity.Speed_Set[1] = Velocity.Speed_Set[2] = 0.2;//前进 后退 旋转速度
	level_abgle_set = 0.0;

	Chassis_message->Chassis_state = Chassis_NO_MOVE;

	Chassia_Motor_Feedback_Update();//速度更新 角度更新

}

void Chassis_Ctrl::Chassia_Motor_Feedback_Update(void)
{
	uint8_t i = 0;
	for (i = 0; i < 4; i++)
	{
		//更新电机速度，加速度是速度的PID微分
		Motor[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * Motor[i].chassis_motor_measure->speed_rpm;
		Motor[i].accel = Speed_Pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE;
	}

	Chassis_angle = (float)chassis_imu_angle->Angle[2] * 0.0000958737f;//角度

	if (ABS(Chassis_angle) < 0.0008f) {
		Chassis_angle = 0;
	}

	Chassis_gyor = (float)chassis_imu_gyro->w[2] * 0.0010652644f;//角速度

//更新底盘前进速度 x，平移速度y，旋转速度wz，坐标系为右手系
	Velocity.vx = (-Motor[0].speed + Motor[1].speed + Motor[2].speed - Motor[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
	Velocity.vy = (-Motor[0].speed - Motor[1].speed + Motor[2].speed + Motor[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
	Velocity.wz = (-Motor[0].speed - Motor[1].speed - Motor[2].speed - Motor[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;
}

//根据遥控模式选择底盘运行方式
void Chassis_Ctrl::chassis_behaviour_control_set(fp32* vx_set, fp32* vy_set, fp32* angle_set)
{
	if (Chassis_message->Chassis_state == Chassis_NO_MOVE)//无力状态
	{
		*vx_set = 0.0f;
		*vy_set = 0.0f;
		*angle_set = 0.0f;
	}
	else if (Chassis_message->Chassis_state == Chassis_FOLLOW_L1||Chassis_message->Chassis_state == Chassis_FOLLOW_L2)//跟随云台
	{
		Chassis_rc_to_control_vector(vx_set, vy_set);//将遥控值转换为底盘设定量
//		caclc_chassia_front_angle();//本函数用于设定angset，使底盘与云台朝向一致
//		*angle_set = chassis_setangle.out;
#if USE_RC==1	
		if ((Chassis_message->RC_ctrl->rc.ch[0] <-30||Chassis_message->RC_ctrl->rc.ch[0] >30)) 
		{
				*angle_set = - 5*RC_CH_STEP * Chassis_message->RC_ctrl->rc.ch[0];
		}else
		{
				*angle_set=0;
		}
#else
			*angle_set += Chassis_message->RC_ctrl->mouse.x *(- 0.005);
#endif 
	}
	else if (Chassis_message->Chassis_state == Chassis_SPIN) {
		Chassis_rc_to_control_vector(vx_set, vy_set);
		*angle_set = level_abgle_set;
	}
}

void Chassis_Ctrl::chassis_set_contorl(void)
{
	//设置速度
	fp32 vx_set = 0.0f, vy_set = 0.0f, angle_set = 0.0f;
	chassis_behaviour_control_set(&vx_set, &vy_set, &angle_set);//根据按键设定底盘VX,VY,W的速度

	if (Chassis_message->Chassis_state == Chassis_FOLLOW_L1)//跟随云台
	{
		//计算底盘角度偏移量
		fp32 chassis_wz = angle_set;
		Velocity.wz_set = -chassis_wz;
		Velocity.vy_set = fp32_constrain(vx_set, Velocity.vx_min_speed, Velocity.vx_max_speed);
		Velocity.vx_set = fp32_constrain(vy_set, Velocity.vy_min_speed, Velocity.vy_max_speed);
	}
	else if (Chassis_message->Chassis_state == Chassis_FOLLOW_L2)//跟随云台
	{
		//计算底盘角度偏移量
		fp32 chassis_wz = angle_set*2;
		Velocity.wz_set = -chassis_wz*1.15;
		Velocity.vy_set = fp32_constrain(vx_set*2, Velocity.vx_min_speed, Velocity.vx_max_speed);
		Velocity.vx_set = fp32_constrain(vy_set*2, Velocity.vy_min_speed, Velocity.vy_max_speed);
	}
	else if (Chassis_message->Chassis_state == Chassis_NO_MOVE)//无力状态
	{
		Velocity.vx_set = vx_set;
		Velocity.vy_set = vy_set;
		Velocity.wz_set = angle_set;
		Filter_vx.out = 0.0f;
		Filter_vy.out = 0.0f;
	}
	else if (Chassis_message->Chassis_state == Chassis_SPIN)//小陀螺模式
	{
		fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
		//chassis_motor.chassis_relative_angle=*(chassis_motor.chassis_yaw_motor_relative_angle);//相对云台角度
//旋转控制底盘速度方向，保证前进方向是云台方向，有利于运动平稳
		sin_yaw = sinf(-*relative_angle);
		cos_yaw = cosf(-*relative_angle);
		Velocity.vx_set = -sin_yaw * vx_set + cos_yaw * vy_set;
		Velocity.vy_set = cos_yaw * vx_set + sin_yaw * vy_set;

		Velocity.wz_set = angle_set;
		//速度限幅
		Velocity.vx_set = fp32_constrain(Velocity.vx_set, Velocity.vx_min_speed, Velocity.vx_max_speed);
		Velocity.vy_set = fp32_constrain(Velocity.vy_set, Velocity.vy_min_speed, Velocity.vy_max_speed);
	}
}

void Chassis_Ctrl::chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4])
{
	//旋转的时候， 由于云台靠前，所以是前面两轮 0 ，1 旋转的速度变慢， 后面两轮 2,3 旋转的速度变快
	wheel_speed[0] = vx_set - vy_set + wz_set * head_wheel;
	wheel_speed[1] = -vx_set - vy_set + wz_set * head_wheel;
	wheel_speed[2] = vx_set + vy_set + wz_set * tail_wheel;
	wheel_speed[3] = -vx_set + vy_set + wz_set * tail_wheel;
}
void Chassis_Ctrl::chassis_control_loop(void)
{
	fp32 max_vector = 0.0f, vector_rate = 0.0f;
	fp32 temp = 0.0f;
	fp32 wheel_speed[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
	float chassis_set[4] = { 0 };
	float chassis_get[4] = { 0 };
	float chassis_Tdout[4] = { 0 };
	uint8_t i = 0;
	//麦轮运动分解
	chassis_vector_to_mecanum_wheel_speed(Velocity.vx_set,
		Velocity.vy_set, Velocity.wz_set, wheel_speed);

	//计算轮子控制最大速度，并限制其最大速度
	for (i = 0; i < 4; i++)
	{
		Motor[i].speed_set = wheel_speed[i];
		temp = ABS(Motor[i].speed_set);
		if (max_vector < temp)
		{
			max_vector = temp;
		}
	}

	if (max_vector > MAX_WHEEL_SPEED)
	{
		vector_rate = MAX_WHEEL_SPEED / max_vector;
		for (i = 0; i < 4; i++)
		{
			Motor[i].speed_set *= vector_rate;
		}
	}

	//计算pid

	for (i = 0; i < 4; i++)
	{
		chassis_set[i] = Motor[i].speed_set;
		rc_deadline_limit(chassis_set[i], chassis_set[i], 0.10f);
		PID_Calc(&Speed_Pid[i], Motor[i].speed, chassis_set[i], 0);
	}

	//赋值电流值
	for (i = 0; i < 4; i++)
	{
		Motor[i].give_current = (int16_t)(Speed_Pid[i].out);
	}
}

void Chassis_Ctrl::Chassis_speed_change(void)
{
	if (Chassis_message->RC_ctrl->key.v & CHASSIS_SPEEDUP_KEY)//shift加速
	{
		++Velocity.Speed_up_sum;
	}
	else
	{
		Velocity.Speed_up_sum = 0;
	}
	if (Chassis_message->RC_ctrl->key.v & CHASSIS_SPEEDDOWN_KEY)//ctrl减速
	{
		++Velocity.Speed_down_sum;
	}
	else {
		Velocity.Speed_down_sum = 0;
	}
	//最多增加（减少）2
	if (Velocity.Speed_down_sum > 200)
	{
		Velocity.Speed_down_sum = 200;
	}
	if (Velocity.Speed_up_sum > 200)
	{
		Velocity.Speed_up_sum = 200;
	}
}

//遥控器的数据处理成底盘的前进vx速度，vy速度
void Chassis_Ctrl::Chassis_rc_to_control_vector(fp32* vx_set, fp32* vy_set)
{
	if (vx_set == NULL || vy_set == NULL) {
		return;
	}
	//遥控器原始通道值
	fp32 vx_set_channel, vy_set_channel;
#if USE_RC
	//前进后退
	if ((Chassis_message->RC_ctrl->rc.ch[3] != 0)) {
		if (Chassis_message->RC_ctrl->rc.ch[3] > 0)//前进
		{
			vy_set_channel = RC_CH_STEP * Chassis_message->RC_ctrl->rc.ch[3];
		}
		else if (Chassis_message->RC_ctrl->rc.ch[3] < 0)
		{
			vy_set_channel = RC_CH_STEP * Chassis_message->RC_ctrl->rc.ch[3];
		}
	}
	else {
		vy_set_channel = 0.0f;
		Filter_vy.out = 0.0f;
	}
	//平移
	if ((Chassis_message->RC_ctrl->rc.ch[2]) != 0) {
		if ((Chassis_message->RC_ctrl->rc.ch[2]) > 0)//右
		{
			vx_set_channel = -RC_CH_STEP * Chassis_message->RC_ctrl->rc.ch[2];
		}
		else if ((Chassis_message->RC_ctrl->rc.ch[2]) < 0)
		{
			vx_set_channel = -RC_CH_STEP * Chassis_message->RC_ctrl->rc.ch[2];
		}
	}
	else {
		vx_set_channel = 0.0f;
		Filter_vx.out = 0.0f;
	}
#else
	//Chassis_speed_change();//加速减速
//前进后退
	if ((Chassis_message->RC_ctrl->key.v & CHASSIS_FRONT_KEY) || (Chassis_message->RC_ctrl->key.v & CHASSIS_BACK_KEY)) {
		if (Chassis_message->RC_ctrl->key.v & CHASSIS_FRONT_KEY)
		{
			vy_set_channel = Velocity.vx_max_speed * Velocity.Speed_Set[0] * level_limit_speed_max + (Velocity.Speed_up_sum - Velocity.Speed_down_sum) * STEP_SPEED;
		}		//速度         = 最大速度 *方向设定*等级限幅 + shift（ctrl）增量
		else if (Chassis_message->RC_ctrl->key.v & CHASSIS_BACK_KEY)
		{
			vy_set_channel = Velocity.vx_min_speed * Velocity.Speed_Set[1] * level_limit_speed_max + (Velocity.Speed_up_sum - Velocity.Speed_down_sum) * STEP_SPEED;
		}
	}
	else {
		vy_set_channel = 0.0f;
		Filter_vy.out = 0.0f;
	}
	//平移
	if ((Chassis_message->RC_ctrl->key.v & CHASSIS_LEFT_KEY) || (Chassis_message->RC_ctrl->key.v & CHASSIS_RIGHT_KEY)) {
		if (Chassis_message->RC_ctrl->key.v & CHASSIS_LEFT_KEY)
		{
			vx_set_channel = Velocity.vx_max_speed * Velocity.Speed_Set[2] * level_limit_speed_max + (Velocity.Speed_up_sum - Velocity.Speed_down_sum) * STEP_SPEED;
		}
		else if (Chassis_message->RC_ctrl->key.v & CHASSIS_RIGHT_KEY)
		{
			vx_set_channel = Velocity.vx_min_speed * Velocity.Speed_Set[2] * level_limit_speed_max + (Velocity.Speed_up_sum - Velocity.Speed_down_sum) * STEP_SPEED;
		}
	}
	else {
		vx_set_channel = 0.0f;
		Filter_vx.out = 0.0f;
	}
#endif	
	//一阶低通滤波代替斜波作为底盘速度输入
	first_order_filter_cali(&Filter_vx, vx_set_channel);
	first_order_filter_cali(&Filter_vy, vy_set_channel);

	*vx_set = Filter_vx.out;
	*vy_set = Filter_vy.out;
}


//矫正模式，利用底盘陀螺仪校准角度
float no_move_zone = 0.17;
void Chassis_Ctrl::caclc_chassia_front_angle(void) {
	if (ABS(*relative_angle) < no_move_zone) {
		PID_clear(&chassis_setangle);
		PID_clear(&chassis_setangle_gyro);
	}
	else {
		PID_Calc(&chassis_setangle, *relative_angle, 0, 0);//设定值为当前的云台角度值，反馈值为当前底盘角度值
		PID_Calc(&chassis_setangle_gyro, -Chassis_gyor, chassis_setangle.out, 0);//底盘角速度环
	}
}
void Chassis_Ctrl::Limit_power_chassis_output(void) {
		level_limit_speed_max = 1.6;
		Velocity.Speed_Set[0] = 0.9; Velocity.Speed_Set[1] = 0.88; Velocity.Speed_Set[2] = 0.6;   level_abgle_set = 10.0;
}

void Chassis_Ctrl::Chassis_Motor_ctrl(void)
{
		Motor_Ctrl.Chassis.CAN_Chassis->SendData(Motor[0].give_current, Motor[1].give_current, Motor[2].give_current, Motor[3].give_current);
//	Motor_Ctrl.Chassis.CAN_Chassis->SendData(Motor[0].give_current, 0, 0, 0);
//		Motor_Ctrl.Chassis.CAN_Chassis->SendData(3000, 3000, 3000, 3000);
}

void Chassis_Ctrl::Up_Push_Move(void)
{
//	if(Chassis_message->Chassis_state == Chassis_NO_MOVE)
//	Motor_Ctrl.Up_Push.Can_Up_Push->SendData(0, 0, 0, 0);
//	else
	Motor_Ctrl.Up_Push.Can_Up_Push->SendData(gMotor[0].give_current, gMotor[1].give_current, gMotor[2].give_current, gMotor[3].give_current);	
}



void Chassis_Ctrl::Up_Push_loop(void)
{
	uint8_t i = 0;
	//计算pid

	for (i = 0; i < 4; i++)
	{
		rc_deadline_limit(gMotor[i].speed_set, gMotor[i].speed_set, 0.10f);
		PID_Calc(&Up_Push_Pid[i], gMotor[i].speed, gMotor[i].speed_set, 0);
	}

	//赋值电流值
	for (i = 0; i < 4; i++)
	{
		gMotor[i].give_current = (int16_t)(Up_Push_Pid[i].out);
	}
}



void Chassis_Ctrl::UP_PUSH_Motor_Feedback_Update(void)
{
	uint8_t i = 0;
	for (i = 0; i < 4; i++)
	{
		//更新电机速度，加速度是速度的PID微分
		gMotor[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * gMotor[i].up_push_motor_measure->speed_rpm;
		gMotor[i].accel = Up_Push_Pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE;
	}
	
}

void Chassis_Ctrl::up_push_set_contorl(void)
{
	uint8_t i=0;
	
//	if (Chassis_message->Chassis_state == Chassis_UP_PUSH||Chassis_message->Chassis_state == Chassis_FOLLOW)
//	{

//		if ((message_bag.RC_ctrl->rc.ch[1]) > 50 )//|| (message_bag.RC_ctrl->rc.ch[1]) > 50)//右
//		{
//			gMotor[0].speed_set = -RC_CH_STEP * message_bag.RC_ctrl->rc.ch[1];
//			gMotor[1].speed_set =  RC_CH_STEP * message_bag.RC_ctrl->rc.ch[1];
//		}
//		else if((message_bag.RC_ctrl->rc.ch[1]) < -50 )//|| (message_bag.RC_ctrl->rc.ch[1]) < -50)
//		{
//			gMotor[0].speed_set = -RC_CH_STEP * message_bag.RC_ctrl->rc.ch[1];
//			gMotor[1].speed_set =  RC_CH_STEP * message_bag.RC_ctrl->rc.ch[1];
//		}
//		else
//		{
//			gMotor[0].speed_set =0;
//			gMotor[1].speed_set =0;
//		}		
		
		
//		if ((Chassis_message->RC_ctrl->rc.ch[3]) > 50)//
//		{
//			gMotor[2].speed_set =   -RC_CH_STEP * Chassis_message->RC_ctrl->rc.ch[3];
//			gMotor[3].speed_set =    RC_CH_STEP * Chassis_message->RC_ctrl->rc.ch[3];
//		}
//		else if((Chassis_message->RC_ctrl->rc.ch[3]) < -50)
//		{
//			gMotor[2].speed_set =  -RC_CH_STEP * Chassis_message->RC_ctrl->rc.ch[3];
//			gMotor[3].speed_set =   RC_CH_STEP * Chassis_message->RC_ctrl->rc.ch[3];
//			
//		}
//		else
//		{
//			gMotor[2].speed_set =0;
//			gMotor[3].speed_set =0;
//		}
		
				if(Chassis_message->ctrl_neg.key_count)//反转
		{
			gMotor[0].speed_set =0.8*(message_bag.key_e.key_flag);
			gMotor[1].speed_set =-gMotor[0].speed_set;
			gMotor[2].speed_set =0.8*(message_bag.key_q.key_flag);
			gMotor[3].speed_set =-gMotor[2].speed_set;


		}
		else if(Chassis_message->shift_pos.key_count)//正转
		{
			gMotor[0].speed_set =-0.8*(message_bag.key_e.key_flag);
			gMotor[1].speed_set =-gMotor[0].speed_set;
			gMotor[2].speed_set =-0.8*(message_bag.key_q.key_flag);
			gMotor[3].speed_set =-gMotor[2].speed_set;

		}
		else//停止
		{
		  for(i = 0 ; i <4 ; i++ )
	   {
		  gMotor[i].speed_set=0;
	   }
		}	
		
		if(Chassis_message->limit.dowm_limit==1&&((Chassis_message->ctrl_neg.key_count)))//(Chassis_message->RC_ctrl->rc.ch[3]) < 0||
		{
			gMotor[2].speed_set = 0;
			gMotor[3].speed_set =0;			
		}

		if(Chassis_message->limit.up_limit==1&&((Chassis_message->shift_pos.key_count)))//(Chassis_message->RC_ctrl->rc.ch[3]) >   0||
		{
			gMotor[2].speed_set = 0;
			gMotor[3].speed_set =0;			
		}

		if(Chassis_message->limit.back_limit==1&&((Chassis_message->ctrl_neg.key_count)))//(Chassis_message->RC_ctrl->rc.ch[1]) < 0||
		{
			gMotor[0].speed_set = 0;
			gMotor[1].speed_set =0;			
		}

		if(Chassis_message->limit.front_limit==1&&((Chassis_message->shift_pos.key_count)))//(Chassis_message->RC_ctrl->rc.ch[1]) >   0||
		{
			gMotor[0].speed_set = 0;
			gMotor[1].speed_set =0;			
		}

	
		
//	}
//	else if(Chassis_message->Chassis_state == Chassis_FOLLOW)
//	{
//				if(Chassis_message->ctrl_neg.key_count)//反转
//		{
//			gMotor[0].speed_set =3*(message_bag.key_e.key_flag);
//			gMotor[1].speed_set =-gMotor[0].speed_set;
//			gMotor[2].speed_set =3*(message_bag.key_q.key_flag);
//			gMotor[3].speed_set =-gMotor[2].speed_set;


//		}
//		else if(Chassis_message->shift_pos.key_count)//正转
//		{
//			gMotor[0].speed_set =-1*(message_bag.key_e.key_flag);
//			gMotor[1].speed_set =-gMotor[0].speed_set;
//			gMotor[2].speed_set =-1*(message_bag.key_q.key_flag);
//			gMotor[3].speed_set =-gMotor[2].speed_set;

//		}
//		else//停止
//		{
//		  for(i = 0 ; i <4 ; i++ )
//	   {
//		  gMotor[i].speed_set=0;
//	   }
//		}		


}



























