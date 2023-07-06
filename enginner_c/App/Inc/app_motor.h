#ifndef __APP_MOTOR_H
#define __APP_MOTOR_H

#include "main.h"
#include "Message_Task.h"


#define Torque_Motor_Numbers      ( 4 )


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
//#define get_limit_measuer(ptr, RxData)                                              \
//    {                                                                                          \
//        (ptr)->chassis.limit.dowm_limit =   (RxData[0]) ;          \
//        (ptr)->chassis.limit.up_limit  =   (RxData[1]) ;         \
//			  (ptr)->chassis.limit.back_limit  =   (RxData[2]) ;         \
//			  (ptr)->chassis.limit.front_limit  =   (RxData[3]) ;         \
//			  (ptr)->chassis.limit.sucker_state  =   (RxData[4]) ;         \
//			  (ptr)->chassis.limit.speedlevel  =   (RxData[5]) ;         \	
//    }
		 
typedef enum
{
		
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

typedef struct
{
	int8_t tem;
	int16_t iq;
	int16_t speed;
	int16_t encoder;
  int16_t circleAngle;
	int32_t accle;
	int64_t mutilAngle;
}MG_Motor_measure_t;

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






class CAN_Ctrl
{
public:

 void CAN_CMD_RESET_ID( uint32_t StdId );
};

typedef struct
{
	int32_t angle ;
	uint16_t vel; 
	int32_t last_angle;
	int32_t accle;
	u8 spinDirection;
	
}angle_control;

class MGmotor
{
	public:
	MG_Motor_measure_t MG_Motor_measure;
	angle_control angle_control;
	void motor_send(uint16_t _pos);
	void motor_start();
	void motor_MVP(float _pos,uint16_t _vel);
	void motor_cmd(uint8_t _cmd);
	void motor_test();
	void motor_accle(int32_t _accle);
	void motor_SPsend(uint16_t _pos,uint16_t _vel);
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
