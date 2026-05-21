#ifndef __QUATERNION_H__
#define __QUATERNION_H__

#ifdef __cplusplus
extern "C" {
#endif

    // 四元数结构体
    typedef struct {
        float w; // 实部 (标量)
        float x; // 虚部 i
        float y; // 虚部 j
        float z; // 虚部 k
    } Quaternion_t;

    // 欧拉角结构体 (单位：度)
    typedef struct {
        float roll;
        float pitch;
        float yaw;
    } Euler_t;
    // 基础数学操作
    void Quat_Init(Quaternion_t *q);
    void Quat_Normalize(Quaternion_t *q);
    Quaternion_t Quat_Multiply(Quaternion_t q1, Quaternion_t q2);
    extern  Quaternion_t my_quat;
    extern  Euler_t my_euler;
    // 核心转换与更新
    void Quat_ToEuler(Quaternion_t *q, Euler_t *euler);
    void Quat_UpdateByGyro(Quaternion_t *q, float gx, float gy, float gz, float dt);

#ifdef __cplusplus
}
#endif

#endif /* __QUATERNION_H__ */