#include "quaternion.h"
#include <math.h>

// 弧度转角度的宏
#define RAD_TO_DEG(x) ((x) * 57.29577951f)

Quaternion_t my_quat;
Euler_t my_euler;
/**
 * @brief 初始化为单位四元数 (代表无旋转)
 */
void Quat_Init(Quaternion_t *q) {
    q->w = 1.0f;
    q->x = 0.0f;
    q->y = 0.0f;
    q->z = 0.0f;
}

/**
 * @brief 四元数归一化 (防止浮点误差累积导致变形)
 */
void Quat_Normalize(Quaternion_t *q) {
    float norm = sqrtf(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
    if (norm == 0.0f) {
        q->w = 1.0f; q->x = 0.0f; q->y = 0.0f; q->z = 0.0f; // 错误保护
        return;
    }
    float invNorm = 1.0f / norm;
    q->w *= invNorm;
    q->x *= invNorm;
    q->y *= invNorm;
    q->z *= invNorm;
}

/**
 * @brief 四元数乘法 (用于连续旋转组合)
 * 注意：四元数乘法不满足交换律！ q1 * q2 != q2 * q1
 */
Quaternion_t Quat_Multiply(Quaternion_t q1, Quaternion_t q2) {
    Quaternion_t res;
    res.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
    res.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
    res.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
    res.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;
    return res;
}

/**
 * @brief 四元数转欧拉角 (ZYX 顺规)
 */
void Quat_ToEuler(Quaternion_t *q, Euler_t *euler) {
    // Roll (绕X轴)
    euler->roll = atan2f(2.0f * (q->w * q->x + q->y * q->z),
                         1.0f - 2.0f * (q->x * q->x + q->y * q->y));

    // Pitch (绕Y轴)
    float sinp = 2.0f * (q->w * q->y - q->z * q->x);
    if (fabsf(sinp) >= 1.0f)
        euler->pitch = copysignf(1.57079632f, sinp); // 万向锁保护 (赋为90度或-90度)
    else
        euler->pitch = asinf(sinp);

    // Yaw (绕Z轴)
    euler->yaw = atan2f(2.0f * (q->w * q->z + q->x * q->y),
                        1.0f - 2.0f * (q->y * q->y + q->z * q->z));

    // 弧度转度
    euler->roll  = RAD_TO_DEG(euler->roll);
    euler->pitch = RAD_TO_DEG(euler->pitch);
    euler->yaw   = RAD_TO_DEG(euler->yaw);
}


/**
 * @brief 纯陀螺仪积分更新四元数 (一阶毕卡求解)
 * @param q: 当前四元数指针
 * @param gx, gy, gz: 陀螺仪当前读数 (单位必须是 rad/s)
 * @param dt: 两次调用的时间间隔 (单位：秒)
 */
void Quat_UpdateByGyro(Quaternion_t *q, float gx, float gy, float gz, float dt) {
    // 预计算步长
    float half_dt = 0.5f * dt;

    // 暂存旧的四元数
    float qw = q->w, qx = q->x, qy = q->y, qz = q->z;


    q->w = qw - half_dt * (qx * gx + qy * gy + qz * gz);
    q->x = qx + half_dt * (qw * gx - qz * gy + qy * gz);
    q->y = qy + half_dt * (qz * gx + qw * gy - qx * gz);
    q->z = qz + half_dt * (-qy * gx + qx * gy + qw * gz);

    // 每次积分后必须重新归一化，防止向量长度发散
    Quat_Normalize(q);
	  Quat_ToEuler(q, &my_euler);
}
