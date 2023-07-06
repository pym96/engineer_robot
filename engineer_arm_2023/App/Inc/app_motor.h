#ifndef __APP_MOTOR_H
#define __APP_MOTOR_H

#include "dev_system.h"
#include "Message_Task.h"

#define USE_PWM_CONTROL_FRIC

#define Chassis_Motor_Numbers      ( 4 )
#define Torque_Motor_Numbers      ( 4 )
/*dve_can*/
#define CAN_Gimbal_CAN             ( CAN1_Ctrl )
#define CAN_Chassis_CAN            ( CAN1_Ctrl )
#define CAN_Fric_CAN               ( CAN2_Ctrl )
#define CAN_UP_Push_CAN            ( CAN2_Ctrl )

#define CAN_Gimbal_StdId           ( CAN_GIMBAL_ALL_ID )
#define CAN_Chassis_StdId          ( CAN_CHASSIS_ALL_ID )
#define CAN_Fric_StdId             ( CAN_CHASSIS_ALL_ID )

#define Motor_Ctrl                 ( CAN_Cmd )//挂载电机的类

#ifdef USE_PWM_CONTROL_FRIC
#define LEFT_FRIC_PWM_PIN          PI6//PE13
#define RIGHT_FRIC_PWM_PIN         PI7//PE14
#define FRIC_MOTOR_STOP_DUTY_CYCLE 1000
#endif

#define JointNUM 4

//力矩电机的输出幅度
#define J0_PMIN -12.0f
#define J0_PMAX 12.0f
#define J0_VMIN -10.0f
#define J0_VMAX 10.0f

#define J1_PMIN -3.3f
#define J1_PMAX 3.3f
#define J1_VMIN -10.0f
#define J1_VMAX 10.0f
					 
#define J2_PMIN -1.8f
#define J2_PMAX 1.8f
#define J2_VMIN -10.0f
#define J2_VMAX 10.0f
					 
#define J3_PMIN -3.3f
#define J3_PMAX 3.3f
#define J3_VMIN -10.0f
#define J3_VMAX 10.0f

#define V_MIN -10.0f
#define V_MAX 10.0f
#define T_MIN -10.0f
#define T_MAX 10.0f

#ifndef MAXVALUE
	#define MAXVALUE 0
#endif

#ifndef MINVALUE
	#define MINVALUE 1
#endif

const float JointPLimit[JointNUM][2] = {{J0_PMAX,J0_PMIN},{J1_PMAX,J1_PMIN},{J2_PMAX,J2_PMIN},{J3_PMAX,J3_PMIN}};
const float JointVLimit[JointNUM][2] = {{J0_VMAX,J0_VMIN},{J1_VMAX,J1_VMIN},{J2_VMAX,J2_VMIN},{J3_VMAX,J3_VMIN}};

//底盘电机数据读取
#define get_motor_measure(ptr, rx_message)                                                     \
    {                                                                                          \
        (ptr)->last_ecd = (ptr)->ecd;                                                          \
        (ptr)->ecd = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);           \
        (ptr)->speed_rpm = (uint16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]);     \
        (ptr)->given_current = (uint16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]); \
        (ptr)->temperate = (rx_message)->Data[6];                                              \
    }
		
//云台电机数据读取
#define get_gimbal_motor_measuer(ptr, rx_message)                                              \
    {                                                                                          \
        (ptr)->last_ecd = (ptr)->ecd;                                                          \
        (ptr)->ecd = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);           \
        (ptr)->given_current = (uint16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]); \
        (ptr)->speed_rpm = (uint16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]);     \
        (ptr)->temperate = (rx_message)->Data[6];                                              \
    }
//力矩电机数据读取
#define get_torque_motor_measuer(ptr, rx_message)                                              \
    {                                                                                          \
        (ptr)->ID= ((rx_message)->Data[0]);                                        \
        (ptr)->POS.ldata = ((rx_message)->Data[1] << 8 | (rx_message)->Data[2]);           \
        (ptr)->VEl.ldata = ((rx_message)->Data[3] << 8 | (rx_message)->Data[4]>> 4); 			 \
        (ptr)->Torque.ldata = ((rx_message)->Data[4] << 4 | (rx_message)->Data[5]);        \
        (ptr)->temperature_MOS=(rx_message)->Data[6];                                        \
				(ptr)->temperature_Rotro =(rx_message)->Data[7];																			 \
    }

//处理限位开关数据		
#define get_limit_measuer(ptr, rx_message)                                              \
    {                                                                                          \
        (ptr)->limit.front_limit =   ((rx_message)->Data[0]) ;          \
        (ptr)->limit.back_limit  =   ((rx_message)->Data[1]) ;         \
    }
		 
typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
		CAN_3508_M1_ID = 0x201,
		CAN_3508_M2_ID = 0x202,
		CAN_3508_M3_ID = 0x203,
		CAN_3508_M4_ID = 0x204,
	
    CAN_UP_PUSH_ID =    0x200,
		CAN_3508_UP1_ID =   0x203,
		CAN_3508_UP2_ID =   0x204,
		CAN_3510_PUSH1_ID = 0x201,
		CAN_3510_PUSH2_ID = 0x202,
	
    CAN_GIMBAL_ALL_ID = 0x1FF,
	  CAN_CHASSIS_TO_GIMBAL=0x209,
		
		CAN_JOINT1_ID = 0x04,
		CAN_JOINT2_ID = 0x05,
		CAN_JOINT3_ID = 0x06,
		CAN_JOINT4_ID = 0x07,
	
} can_msg_id_e;

//rm电机统一数据结构体
typedef struct
{
  uint16_t ecd;
  int16_t speed_rpm;
  int16_t given_current;
  uint8_t temperate;
  int16_t last_ecd;
	uint8_t cnt;
	uint16_t offset;
	uint16_t angle_ecd;
	uint8_t mescnt;
} motor_measure_t;
//6020云台电机数据结构体
typedef struct
{
	int fdbPosition;        //电机的编码器反馈值
	int last_fdbPosition;   //电机上次的编码器反馈值
	int bias_position;      //机器人初始状态电机位置环设定值
	int real_current;         //实际电流
	int given_current;       //给定电流
	int round;              //电机转过的圈数
	int real_position;      //过零处理后的电机转子位置
	int last_real_position;//上次过零处理后的电机转子位置	
}gimbal_measure_t;

//力矩电机 速度位置模式
union int8_float
{
	float fdata;
	unsigned char ldata[8];
};
union int16_float
{
	float fdata;
	int Idata;
};

typedef struct
{
	uint8_t ID;
	uint8_t state;
	int16_float POS;
	int16_float VEl;
	int16_float Torque;
	float temperature_MOS;
	float temperature_Rotro;
}Torque_Motor_measure_t;

class Torque_Motor_Ctrl
{
	public:
		Torque_Motor_measure_t Torque_Motor_Measure[Torque_Motor_Numbers];
			
		float POS_[4];
		float VEl_[4];
		const Torque_Motor_measure_t *Get_Torque_Motor_Measure_Pointer( uint8_t i )
		{
			return &Torque_Motor_Measure[(i & 0x03)];
		}
};

class Chassis_Motor_Ctrl
{
public:	
  CANctrl *CAN_Chassis;
	motor_measure_t Chassis_Measure[Chassis_Motor_Numbers];
	
  Chassis_Motor_Ctrl() : CAN_Chassis( &CAN_Chassis_CAN ){}
  const motor_measure_t *Get_Motor_Measure_Pointer( uint8_t i )
	{
	  return &Chassis_Measure[(i & 0x03)];
	}
};

class Up_Push_Motor_Ctrl
{
	public:
		  CANctrl *Can_Up_Push;
	    motor_measure_t  Up_Push_Motor_Measure[4];
	
      Up_Push_Motor_Ctrl() : Can_Up_Push( &CAN_UP_Push_CAN ){}
		 	const motor_measure_t *Get_Motor_Measure_Pointer( uint8_t i )
		{
			return &Up_Push_Motor_Measure[(i & 0x03)];
		}
};

class Gimbal_Motor_Ctrl
{
public:
  CANctrl *CAN_Gimbal;
	motor_measure_t Yaw_Measure;
  //  motor_measure_t Motor_Pitch;
	
  Gimbal_Motor_Ctrl() : CAN_Gimbal( &CAN_Gimbal_CAN ){}
  const motor_measure_t *Get_Motor_Measure_Pointer( void )
	{
	  return &Yaw_Measure;
	}
};

class Fric_Motor_Ctrl
{
	
public:
#ifndef USE_PWM_CONTROL_FRIC
	CANctrl *CAN_Fric;
  motor_measure_t Fric_Measure;
	Fric_Motor_Ctrl() : CAN_Fric( &CAN_Fric_CAN ){}
  const motor_measure_t *Get_Motor_Measure_Pointer( void )
	{
	  return &Fric_Measure;
	}
#else
public:
	Fric_Motor_Ctrl() : Left_PWM_Pin( LEFT_FRIC_PWM_PIN ), Right_PWM_Pin( RIGHT_FRIC_PWM_PIN ){}
	void Fric_Motor_Init( void )
	{
	  Channl3=PWM_Init( Left_PWM_Pin,  ( F_CPU / 1000000 ), 100 ); //100HZ
		Channl4=PWM_Init( Right_PWM_Pin, ( F_CPU / 1000000 ), 100 ); //100HZ
	}
	
	void Fric_On( uint16_t Speed_Left,uint16_t Speed_Right )
	{
	  pwmWrite( Left_PWM_Pin,  Speed_Left );//ch3
		pwmWrite( Right_PWM_Pin, Speed_Right );//ch4
	}
	
	void Fric_Off( void )
	{
	  pwmWrite( Left_PWM_Pin,  FRIC_MOTOR_STOP_DUTY_CYCLE );
		pwmWrite( Right_PWM_Pin, FRIC_MOTOR_STOP_DUTY_CYCLE );
	}
private:
  uint8_t Left_PWM_Pin, Right_PWM_Pin;
  uint8_t Channl3,Channl4;
#endif
};

class CAN_Ctrl
{
public:
  Chassis_Motor_Ctrl Chassis;
  Gimbal_Motor_Ctrl  Gimbal;
  Fric_Motor_Ctrl    Fric;
	Torque_Motor_Ctrl    Arm;
  Up_Push_Motor_Ctrl  Up_Push;
  void CAN_CMD_RESET_ID( uint32_t StdId );
};
	
void CAN_ALL_Init( void );
void start_motor(uint16_t id);
void lock_motor(uint16_t id);
void setpoint_motor(uint16_t id);
void ctrl_motor_pv(uint16_t id ,float _pos, float _vel);

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

extern CAN_Ctrl CAN_Cmd;

#endif /* __APP_MOTOR_H */
