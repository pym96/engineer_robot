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

//����ʼ����һ��ʱ��
#define CHASSIS_TASK_INIT_TIME 357

//ǰ���ң����ͨ������
#define CHASSIS_X_CHANNEL 3
//���ҵ�ң����ͨ������
#define CHASSIS_Y_CHANNEL 2
//������ģʽ�£�����ͨ��ң����������ת
#define CHASSIS_WZ_CHANNEL 1

//ѡ�����״̬ ����ͨ����
#define MODE_CHANNEL 0
//ң����ǰ��ҡ�ˣ�max 660��ת���ɳ���ǰ���ٶȣ�m/s���ı���
#define CHASSIS_VX_RC_SEN 0.003030303f//0.005f
//ң��������ҡ�ˣ�max 660��ת���ɳ��������ٶȣ�m/s���ı���
#define CHASSIS_VY_RC_SEN 0.003030303f//0.005f
//�������yawģʽ�£�ң������yawң�ˣ�max 660�����ӵ�����Ƕȵı���
#define CHASSIS_ANGLE_Z_RC_SEN 0.000002f
//��������̨��ʱ�� ң������yawң�ˣ�max 660��ת���ɳ�����ת�ٶȵı���
#define CHASSIS_WZ_RC_SEN 0.01f

//�����������ڵ�ͨ�˲�//ֵԽ�����Խ��
#define CHASSIS_ACCEL_X_NUM 0.3f//0.5f
#define CHASSIS_ACCEL_Y_NUM 0.3f//0.5f
#define CHASSIS_ACCEL_Z_NUM 0.15f
//ң��������
#define CHASSIS_RC_DEADLINE 10

#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f

#define MOTOR_DISTANCE_TO_CENTER 0.2f

//����������Ƽ�� 2ms
#define CHASSIS_CONTROL_TIME_MS 2
//����������Ƽ�� 0.002s
#define CHASSIS_CONTROL_TIME 0.002
//�����������Ƶ�ʣ���δʹ�������
#define CHASSIS_CONTROL_FREQUENCE 500.0f
//����3508���can���͵���ֵ
#define MAX_MOTOR_CAN_CURRENT 10000.0f


//����ǰ�����ҿ��ư���+���ټ�

//��������
#define POWER_FIRST 1 
//Ѫ������
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


//m3508ת���ɵ����ٶ�(m/s)�ı������������� ����Ϊ���ܻ������Ҫ��������
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

//���̵������ٶ�
#define MAX_WHEEL_SPEED 4.0f//10.0f//4.0
//�����˶��������ǰ���ٶ�
#define NORMAL_MAX_CHASSIS_SPEED_X 2.0f//10.0f//2.9
//�����˶��������ƽ���ٶ�
#define NORMAL_MAX_CHASSIS_SPEED_Y 2.0f//10.0f//2.9
//�����˶�����������ٶ�
#define NORMAL_MAX_CHASSIS_SPEED_Z 4.0f//10.0f//2.9

//����������ת�ٶȣ�����ǰ�������ֲ�ͬ�趨�ٶȵı�����Ȩ 0Ϊ�ڼ������ģ�����Ҫ����
#define CHASSIS_WZ_SET_SCALE 0.1f

//ҡ��ԭ�ز���ҡ�����Ƕ�(rad)
#define SWING_NO_MOVE_ANGLE 0.1f
//ҡ�ڹ��̵����˶����Ƕ�(rad)
#define SWING_MOVE_ANGLE 0.5f

//���̵���ٶȻ�PID
#define M3505_MOTOR_SPEED_PID_KP 18000.0f
#define M3505_MOTOR_SPEED_PID_KI 5.0f
#define M3505_MOTOR_SPEED_PID_KD 0.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

//������ת����PID
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
} Chassis_Motor_t;//���̽��ձ��������� �Լ� �趨����

typedef struct
{
	const motor_measure_t* up_push_motor_measure;
	fp32 accel;
	fp32 speed;
	fp32 speed_set;
	int16_t give_current;
} UP_PUSH_Motor_t;//̧���Ƴ����ձ��������� �Լ� �趨����

typedef struct {
	fp32 vx;      //�����ٶ� ǰ������ ǰΪ������λ m/s
	fp32 vy;      //�����ٶ� ���ҷ��� ��Ϊ��  ��λ m/s
	fp32 wz;      //������ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s
	fp32 vx_set;  //�����趨�ٶ� ǰ������ ǰΪ������λ m/s
	fp32 vy_set;  //�����趨�ٶ� ���ҷ��� ��Ϊ������λ m/s
	fp32 wz_set;  //�����趨��ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s

	fp32 vx_max_speed;  //ǰ����������ٶ� ��λm/s
	fp32 vx_min_speed;  //ǰ��������С�ٶ� ��λm/s
	fp32 vy_max_speed;  //���ҷ�������ٶ� ��λm/s
	fp32 vy_min_speed;  //���ҷ�����С�ٶ� ��λm/s

	fp32    Speed_Set[3];//�����ٶ� [0]ǰ���ٶ� [1]�����ٶ� [2]ƽ���ٶ�

	uint8_t Speed_up_sum;
	uint8_t Speed_down_sum;
} Chassis_Velocity_t;

class Chassis_Ctrl
{
public:
	Message_bag* Chassis_message;

	void Chassis_Init(void);//���̳�ʼ��
	void Chassia_Motor_Feedback_Update(void);//���̵�����ݸ���
	void Chassis_rc_to_control_vector(fp32* vx_set, fp32* vy_set);//ң�������ݴ���Ϊxyw�ٶ�
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

	//�˲�֮����ٶ�
	first_order_filter_type_t Filter_vx;
	first_order_filter_type_t Filter_vy;
	first_order_filter_type_t Filter_vw;

	Chassis_Velocity_t   Velocity;

	struct SGyro* chassis_imu_gyro;        //����������
	struct SAngle* chassis_imu_angle;       //����������

	fp32                        Chassis_angle;
	fp32                        Chassis_angle_set;
	fp32                        Chassis_gyor;
	fp32                        Chassis_gyor_set;
	const fp32* relative_angle;
	//�������
	fp32                        level_abgle_set;//С������ת�趨ֵ
	fp32                        level_limit_speed_max;
	//�����������ӵ���̨�ľ��벻һ�� ������Ҫһ������ϵ��
	float head_wheel;
	float tail_wheel;
};



#endif /* __CHASSIS_TASK_H */
