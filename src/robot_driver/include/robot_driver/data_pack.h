#ifndef __ARM_DATA_H__
#define __ARM_DATA_H__

#include <stdint.h>

#pragma pack(1)
// Expect_Robstride 用于存储预期的力矩、位置、角速度以及PID参数
typedef enum
{
    RobStride_01,  //!<@brief 机器人类型1
    RobStride_02,  //!<@brief 机器人类型2
    RobStride_03,  //!<@brief 机器人类型3
    RobStride_04   //!<@brief 机器人类型4
} RobStrideType;
typedef struct
{
    float except_torque;  //!<@brief 期望力矩
    float except_pos;     //!<@brief 期望位置
    float except_omega;   //!<@brief 期望角速度
    float kp;             //!<@brief 比例增益
    float kd;             //!<@brief 微分增益
} Expect_Robstride;

// Expect_GM6020 用于存储 GM6020 电机的目标位置、目标速度以及PID参数
typedef struct
{
    float target_pos;     //!<@brief 目标位置
    float target_vel;     //!<@brief 目标速度
    float kp;             //!<@brief 比例增益
    float kd;             //!<@brief 微分增益
} Expect_GM6020;

// servo 用于表示伺服电机的上下限
typedef struct
{
    float up;               //!<@brief 上限
    float low;              //!<@brief 下限
} servo;

// target_pack_t 用于表示目标数据包，包含多个机器人控制模块的期望数据
typedef struct
{
    int pack_type;       //!<@brief 包类型
    int air_pump;
    servo servo1;        //!<@brief 伺服电机信息
    Expect_Robstride rob01;  //!<@brief RobStride 期望值
    Expect_GM6020 rob02;     //!<@brief GM6020 期望值
} target_pack_t;

typedef struct
{
    float rad;
    float omega;
    float torque;
    float iqf;
    float r;
    float temperature;
    float vbus;
    uint64_t error;
    uint8_t feedback;
}RobStrideState_t;

// GM6020_TypeDef 用于表示 GM6020 电机的反馈信息
typedef struct
{
    uint16_t MchanicalAngle;  //!<@brief 机械角度
    int16_t Speed;            //!<@brief 转速
    int16_t TorqueCurrent;    //!<@brief 转矩电流
    uint8_t temp;             //!<@brief 温度
    uint16_t LsatAngle;       //!<@brief 上一次的机械角度
    int16_t r;                //!<@brief 圈数
    int32_t Angle;            //!<@brief 连续化机械角度
    float Angle_DEG;          //!<@brief 连续化角度制角度
} GM6020_TypeDef;

// RobStrideMode 枚举类型定义了不同的控制模式
typedef enum
{
    RobStride_MotionControl,  //!<@brief 运动控制模式
    RobStride_Position,       //!<@brief 位置控制模式
    RobStride_Speed,          //!<@brief 速度控制模式
    RobStride_Torque,         //!<@brief 力矩控制模式
} RobStrideMode;

// RobStride_t 用于表示 RobStride 的控制参数和状态
typedef struct
{
    RobStrideType type;      //!<@brief 机器人类型
    RobStrideState_t state;  //!<@brief 机器人状态

} RobStride_t;

// state_pack_t 用于表示状态数据包，包含了伺服电机和 RobStride 的状态信息
typedef struct
{
    int pack_type;         //!<@brief 包类型
    servo servo2;          //!<@brief 伺服电机信息
    RobStride_t robstride01;  //!<@brief RobStride 控制信息
    GM6020_TypeDef GM6020;     //!<@brief GM6020 电机信息
} state_pack_t;

// RobStrideType 枚举定义了四个不同的机器人类型

#pragma pack()
#endif