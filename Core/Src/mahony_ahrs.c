#include "mahony_ahrs.h"
#include <math.h>
#include <stdint.h> // 必须包含，用于 int32_t

AHRS_Attitude_t ahrs_data = {0};

// 积分误差累计变量
static float eInt[3] = {0.0f, 0.0f, 0.0f};

/**
 * @brief 快速计算平方根倒数 (雷神之锤经典算法)
 * @note  已将 long 修改为 int32_t，彻底解决 64 位平台移植可能导致的崩溃问题
 */
static float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    int32_t i = *(int32_t*)&y; // 强制使用 32 位整型
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

/**
 * @brief 初始化四元数
 */
void Mahony_Init(void) {
    ahrs_data.q0 = 1.0f;
    ahrs_data.q1 = 0.0f;
    ahrs_data.q2 = 0.0f;
    ahrs_data.q3 = 0.0f;
}

/**
 * @brief Mahony 6轴姿态解算核心函数
 * @param gx, gy, gz : 陀螺仪数据 (必须是 rad/s)
 * @param ax, ay, az : 加速度计数据 (必须是 g 或 m/s^2，只需方向正确)
 */
void Mahony_Update(float gx, float gy, float gz, float ax, float ay, float az) {
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;
    float sinp; // 用于俯仰角计算限幅

    // 如果加速度计全为0，则忽略此次运算（防止除0异常）
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // 1. 归一化加速度计测量值
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // 2. 提取当前四元数的等效重力分量 (从机体坐标系转换到世界坐标系)
        halfvx = ahrs_data.q1 * ahrs_data.q3 - ahrs_data.q0 * ahrs_data.q2;
        halfvy = ahrs_data.q0 * ahrs_data.q1 + ahrs_data.q2 * ahrs_data.q3;
        halfvz = ahrs_data.q0 * ahrs_data.q0 - 0.5f + ahrs_data.q3 * ahrs_data.q3;

        // 3. 向量叉乘计算误差 (测量重力方向与估算重力方向的偏差)
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // 4. PI 控制器处理误差
        if(Ki > 0.0f) {
            eInt[0] += halfex * (2.0f * Ki * (1.0f / SAMPLE_FREQ));
            eInt[1] += halfey * (2.0f * Ki * (1.0f / SAMPLE_FREQ));
            eInt[2] += halfez * (2.0f * Ki * (1.0f / SAMPLE_FREQ));
            gx += eInt[0];
            gy += eInt[1];
            gz += eInt[2];
        } else {
            // [修复] 防止积分饱和 (Integral Windup)，Ki 降为 0 时必须清零积分项
            eInt[0] = 0.0f;
            eInt[1] = 0.0f;
            eInt[2] = 0.0f;
        }
        
        gx += halfex * (2.0f * Kp);
        gy += halfey * (2.0f * Kp);
        gz += halfez * (2.0f * Kp);
    }

    // 5. 积分陀螺仪数据，更新四元数
    gx *= (0.5f * (1.0f / SAMPLE_FREQ));
    gy *= (0.5f * (1.0f / SAMPLE_FREQ));
    gz *= (0.5f * (1.0f / SAMPLE_FREQ));
    qa = ahrs_data.q0;
    qb = ahrs_data.q1;
    qc = ahrs_data.q2;
    // [注]：q3最后更新，因此可以直接使用 ahrs_data.q3 参与前三项计算，省去局部变量 qd
    ahrs_data.q0 += (-qb * gx - qc * gy - ahrs_data.q3 * gz);
    ahrs_data.q1 += (qa * gx + qc * gz - ahrs_data.q3 * gy);
    ahrs_data.q2 += (qa * gy - qb * gz + ahrs_data.q3 * gx);
    ahrs_data.q3 += (qa * gz + qb * gy - qc * gx);

    // 6. 归一化四元数
    recipNorm = invSqrt(ahrs_data.q0 * ahrs_data.q0 + ahrs_data.q1 * ahrs_data.q1 + ahrs_data.q2 * ahrs_data.q2 + ahrs_data.q3 * ahrs_data.q3);
    ahrs_data.q0 *= recipNorm;
    ahrs_data.q1 *= recipNorm;
    ahrs_data.q2 *= recipNorm;
    ahrs_data.q3 *= recipNorm;

    // 7. 四元数转欧拉角 (ZYX 顺规，单位转换为度)
    ahrs_data.roll  = atan2f(2.0f * (ahrs_data.q0 * ahrs_data.q1 + ahrs_data.q2 * ahrs_data.q3), 1.0f - 2.0f * (ahrs_data.q1 * ahrs_data.q1 + ahrs_data.q2 * ahrs_data.q2)) * 57.29578f;
    
    // [修复] 严格限幅传入 asinf 的参数，防止因浮点运算误差超出 [-1.0, 1.0] 导致返回 NaN
    sinp = 2.0f * (ahrs_data.q0 * ahrs_data.q2 - ahrs_data.q3 * ahrs_data.q1);
    if (sinp > 1.0f) sinp = 1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    ahrs_data.pitch = asinf(sinp) * 57.29578f;
    
    ahrs_data.yaw   = atan2f(2.0f * (ahrs_data.q0 * ahrs_data.q3 + ahrs_data.q1 * ahrs_data.q2), 1.0f - 2.0f * (ahrs_data.q2 * ahrs_data.q2 + ahrs_data.q3 * ahrs_data.q3)) * 57.29578f;
}