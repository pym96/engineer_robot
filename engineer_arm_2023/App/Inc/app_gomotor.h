#ifndef _APP_GOMOTOR_H__
#define _APP_GOMOTOR_H__

#include "dev_system.h"

#define SATURATE(_IN, _MIN, _MAX) {\
 if (_IN < _MIN)\
 _IN = _MIN;\
 else if (_IN > _MAX)\
 _IN = _MAX;\
 } 

#define RS485_RE_Pin GPIO_Pin_7
#define RS485_RE_GPIO_Port GPIOI
#define RS485_DE_Pin GPIO_Pin_6
#define RS485_DE_GPIO_Port GPIOI
#define RS485_CTRLPin GPIO_Pin_6
#define RS485_CTRLGPIO_Port GPIOI
 
#define SET_485_DE_UP() GPIO_SetBits(RS485_DE_GPIO_Port, RS485_DE_Pin	)
#define SET_485_DE_DOWN() GPIO_ResetBits(RS485_DE_GPIO_Port, RS485_DE_Pin)

#define SET_485_RE_UP() GPIO_SetBits(RS485_RE_GPIO_Port, RS485_RE_Pin)
#define SET_485_RE_DOWN() GPIO_ResetBits(RS485_RE_GPIO_Port, RS485_RE_Pin)
 
#define SET_485_WRITE() GPIO_ResetBits(RS485_CTRLGPIO_Port,RS485_CTRLPin)
#define SET_485_READ()  GPIO_SetBits(RS485_CTRLGPIO_Port,RS485_CTRLPin)

/**
 * @brief 电机模式控制信息
 * 
 */
typedef struct
{
    uint8_t id     :4;      // 电机ID: 0,1...,13,14 15表示向所有电机广播数据(此时无返回)
    uint8_t status :3;      // 工作模式: 0.锁定 1.FOC闭环 2.编码器校准 3.保留
    uint8_t none   :1;      // 保留位
} RIS_Mode_t;   // 控制模式 1Byte

/**
 * @brief 电机状态控制信息
 * 
 */
typedef struct
{
    int16_t tor_des;        // 期望关节输出扭矩 unit: N.m      (q8)
    int16_t spd_des;        // 期望关节输出速度 unit: rad/s    (q8)
    int32_t pos_des;        // 期望关节输出位置 unit: rad      (q15)
    int16_t k_pos;          // 期望关节刚度系数 unit: -1.0-1.0 (q15)
    int16_t k_spd;          // 期望关节阻尼系数 unit: -1.0-1.0 (q15)
    
} RIS_Comd_t;   // 控制参数 12Byte

/**
 * @brief 电机状态反馈信息
 * 
 */
typedef struct
{
    int16_t  torque;        // 实际关节输出扭矩 unit: N.m     (q8)
    int16_t  speed;         // 实际关节输出速度 unit: rad/s   (q8)
    int32_t  pos;           // 实际关节输出位置 unit: rad     (q15)
    int8_t   temp;          // 电机温度: -128~127°C
    uint8_t  MError :3;     // 电机错误标识: 0.正常 1.过热 2.过流 3.过压 4.编码器故障 5-7.保留
    uint16_t force  :12;    // 足端气压传感器数据 12bit (0-4095)
    uint8_t  none   :1;     // 保留位
} RIS_Fbk_t;   // 状态数据 11Byte

typedef union
{
    int32_t     L;
    uint8_t     u8[4];
    uint16_t    u16[2];
    uint32_t    u32;
    float       F;
} COMData32;

typedef struct
{
    // 定义 数据包头
    unsigned char start[2]; // 包头
    unsigned char motorID;  // 电机ID  0,1,2,3 ...   0xBB 表示向所有电机广播（此时无返回）
    unsigned char reserved;
} COMHead;

typedef struct
{ // 以 4个字节一组排列 ，不然编译器会凑整
    // 定义 数据
    uint8_t mode;       // 当前关节模式
    uint8_t ReadBit;    // 电机控制参数修改     是否成功位
    int8_t Temp;        // 电机当前平均温度
    uint8_t MError;     // 电机错误 标识

    COMData32 Read;     // 读取的当前 电机 的控制数据
    int16_t T;          // 当前实际电机输出力矩       7 + 8 描述

    int16_t W;          // 当前实际电机速度（高速）   8 + 7 描述
    float LW;           // 当前实际电机速度（低速）

    int16_t W2;         // 当前实际关节速度（高速）   8 + 7 描述
    float LW2;          // 当前实际关节速度（低速）

    int16_t Acc;        // 电机转子加速度       15+0 描述  惯量较小
    int16_t OutAcc;     // 输出轴加速度         12+3 描述  惯量较大

    int32_t Pos;        // 当前电机位置（主控0点修正，电机关节还是以编码器0点为准）
    int32_t Pos2;       // 关节编码器位置(输出编码器)

    int16_t gyro[3];    // 电机驱动板6轴传感器数据
    int16_t acc[3];

    // 力传感器的数据
    int16_t Fgyro[3];
    int16_t Facc[3];
    int16_t Fmag[3];
    uint8_t Ftemp;      // 8位表示的温度  7位（-28~100度）  1位0.5度分辨率

    int16_t Force16;    // 力传感器高16位数据
    int8_t Force8;      // 力传感器低8位数据

    uint8_t FError;     //  足端传感器错误标识

    int8_t Res[1];      // 通讯 保留字节

} ServoComdV3; // 加上数据包的包头 和CRC 78字节（4+70+4）

typedef struct
{
    uint8_t head[2];    // 包头         2Byte
    RIS_Mode_t mode;    // 电机控制模式  1Byte
    RIS_Fbk_t   fbk;    // 电机反馈数据 11Byte
    uint16_t  CRC16;    // CRC          2Byte
} MotorData_t;  //返回数据

typedef struct
{
    uint8_t none[8];            // 保留

} LowHzMotorCmd;

typedef struct
{                               // 以 4个字节一组排列 ，不然编译器会凑整
                                // 定义 数据
    uint8_t mode;               // 关节模式选择
    uint8_t ModifyBit;          // 电机控制参数修改位
    uint8_t ReadBit;            // 电机控制参数发送位
    uint8_t reserved;

    COMData32 Modify;           // 电机参数修改 的数据
    //实际给FOC的指令力矩为：
    // K_P*delta_Pos + K_W*delta_W + T
    int16_t T;                  // 期望关节的输出力矩（电机本身的力矩）x256, 7 + 8 描述
    int16_t W;                  // 期望关节速度 （电机本身的速度） x128,       8 + 7描述
    int32_t Pos;                // 期望关节位置 x 16384/6.2832, 14位编码器（主控0点修正，电机关节还是以编码器0点为准）

    int16_t K_P;                // 关节刚度系数 x2048  4+11 描述
    int16_t K_W;                // 关节速度系数 x1024  5+10 描述

    uint8_t LowHzMotorCmdIndex; // 保留
    uint8_t LowHzMotorCmdByte;  // 保留

    COMData32 Res[1];           // 通讯 保留字节  用于实现别的一些通讯内容

} MasterComdV3; // 加上数据包的包头 和CRC 34字节

typedef struct
{
    // 定义 电机控制命令数据包
    uint8_t head[2];    // 包头         2Byte
    RIS_Mode_t mode;    // 电机控制模式  1Byte
    RIS_Comd_t comd;    // 电机期望数据 12Byte
    uint16_t   CRC16;   // CRC          2Byte
} ControlData_t;     //电机控制命令数据包

typedef struct
{
    // 定义 发送格式化数据
    ControlData_t motor_send_data;   //电机控制数据结构体
    int hex_len;                        //发送的16进制命令数组长度, 34
    long long send_time;                //发送该命令的时间, 微秒(us)
    // 待发送的各项数据
    unsigned short id;                  //电机ID，0代表全部电机
    unsigned short mode;                // 0:空闲, 5:开环转动, 10:闭环FOC控制
    //实际给FOC的指令力矩为：
    // K_P*delta_Pos + K_W*delta_W + T
    float T;                            //期望关节的输出力矩（电机本身的力矩）（Nm）
    float W;                            //期望关节速度（电机本身的速度）(rad/s)
    float Pos;                          //期望关节位置（rad）
    float K_P;                          //关节刚度系数
    float K_W;                          //关节速度系数
    COMData32 Res;                    // 通讯 保留字节  用于实现别的一些通讯内容
} MOTOR_send;

typedef struct
{
    // 定义 接收数据
    MotorData_t motor_recv_data;    //电机接收数据结构体，详见motor_msg.h
    int hex_len;                        //接收的16进制命令数组长度, 78
    long long resv_time;                //接收该命令的时间, 微秒(us)
    int correct;                        //接收数据是否完整（1完整，0不完整）
    //解读得出的电机数据
    unsigned char motor_id;             //电机ID
    unsigned char mode;                 // 0:空闲, 5:开环转动, 10:闭环FOC控制
    int Temp;                           //温度
    unsigned char MError;               //错误码
    float T;                            // 当前实际电机输出力矩
		float W;														// speed
    float Pos;                          // 当前电机位置（主控0点修正，电机关节还是以编码器0点为准）
		float footForce;												// 足端气压传感器数据 12bit (0-4095)

} MOTOR_recv;


bool GO_send(MOTOR_send *pData,USART_TypeDef *USARTx);

#endif
