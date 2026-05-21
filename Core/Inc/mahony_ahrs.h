#ifndef __MAHONY_AHRS_H
#define __MAHONY_AHRS_H

#include <stdint.h>

// 算法核心参数配置 (根据你的硬件实际情况调整)
#define Kp 8.0f          // 比例增益，控制加速度计收敛速度
#define Ki 0.25f        // 积分增益，用于消除陀螺仪零偏误差
#define SAMPLE_FREQ 200.0f // 传感器采样频率 (Hz)，例如 100Hz

// 姿态数据结构体
typedef struct {
    float q0, q1, q2, q3; // 四元数
    float roll;           // 横滚角 (度)
    float pitch;          // 俯仰角 (度)
    float yaw;            // 偏航角 (度)
} AHRS_Attitude_t;

extern AHRS_Attitude_t ahrs_data;

void Mahony_Init(void);
void Mahony_Update(float gx, float gy, float gz, float ax, float ay, float az);

#endif // __MAHONY_AHRS_H