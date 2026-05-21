#ifndef __EKF_IMU_H
#define __EKF_IMU_H

#include <math.h>

/* 定义常量 */
#define EKF_PI 3.1415926535f
#define EKF_RAD_TO_DEG (180.0f / EKF_PI)

/* EKF 结构体 */
typedef struct {
    float q[4];          // 状态向量: [w, x, y, z]
    float P[4][4];       // 状态协方差
    float Q[4][4];       // 过程噪声
    float R[3][3];       // 测量噪声
    float dt;            // 采样周期 (s)
} EKF6Axis_t;

/**
 * @brief 初始化 EKF 滤波器
 * @param var_gyro 陀螺仪过程噪声方差 (推荐 0.001 - 0.1)
 * @param var_acc  加速度计测量噪声方差 (推荐 0.01 - 0.5)
 */
void EKF_Init(EKF6Axis_t *ekf, float dt, float var_gyro, float var_acc);

/**
 * @brief 预测步：使用陀螺仪更新状态
 * @param gx, gy, gz 角速度 (单位: rad/s)
 */
void EKF_Predict(EKF6Axis_t *ekf, float gx, float gy, float gz);

/**
 * @brief 更新步：使用加速度计修正姿态
 * @param ax, ay, az 加速度 (单位: g 或 m/s2)
 */
void EKF_Update(EKF6Axis_t *ekf, float ax, float ay, float az);

/**
 * @brief 获取当前姿态角
 */
void EKF_GetEuler(EKF6Axis_t *ekf, float *roll, float *pitch, float *yaw);

#endif