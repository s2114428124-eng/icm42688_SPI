#include "ekf_imu.h"

static void normalize_q(float q[4]) {
    float norm = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (norm > 1e-6f) {
        float invNorm = 1.0f / norm;
        q[0] *= invNorm; q[1] *= invNorm; q[2] *= invNorm; q[3] *= invNorm;
    }
}

void EKF_Init(EKF6Axis_t *ekf, float dt, float var_gyro, float var_acc) {
    ekf->dt = dt;
    ekf->q[0] = 1.0f; ekf->q[1] = 0.0f; ekf->q[2] = 0.0f; ekf->q[3] = 0.0f;

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            ekf->P[i][j] = (i == j) ? 0.1f : 0.0f;
            ekf->Q[i][j] = (i == j) ? var_gyro : 0.0f;
        }
    }
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            ekf->R[i][j] = (i == j) ? var_acc : 0.0f;
        }
    }
}

void EKF_Predict(EKF6Axis_t *ekf, float gx, float gy, float gz) {
    float q_old[4];
    for (int i = 0; i < 4; i++) q_old[i] = ekf->q[i];

    float half_dt = 0.5f * ekf->dt;

    // 1. F 矩阵隐含在状态更新公式中
    ekf->q[0] += half_dt * (-q_old[1]*gx - q_old[2]*gy - q_old[3]*gz);
    ekf->q[1] += half_dt * ( q_old[0]*gx + q_old[2]*gz - q_old[3]*gy);
    ekf->q[2] += half_dt * ( q_old[0]*gy - q_old[1]*gz + q_old[3]*gx);
    ekf->q[3] += half_dt * ( q_old[0]*gz + q_old[1]*gy - q_old[2]*gx);
    normalize_q(ekf->q);

    // 2. 更新协方差 P = F*P*F' + Q
    // 这里简化 F 矩阵为线性化近似
    float F[4][4] = {
        {1.0f, -half_dt*gx, -half_dt*gy, -half_dt*gz},
        {half_dt*gx, 1.0f,  half_dt*gz, -half_dt*gy},
        {half_dt*gy, -half_dt*gz, 1.0f,  half_dt*gx},
        {half_dt*gz, half_dt*gy, -half_dt*gx, 1.0f}
    };

    float FP[4][4] = {0};
    for(int i=0; i<4; i++)
        for(int j=0; j<4; j++)
            for(int k=0; k<4; k++)
                FP[i][j] += F[i][k] * ekf->P[k][j];

    for(int i=0; i<4; i++) {
        for(int j=0; j<4; j++) {
            float tmp = 0;
            for(int k=0; k<4; k++) tmp += FP[i][k] * F[j][k];
            ekf->P[i][j] = tmp + ekf->Q[i][j];
        }
    }
}

void EKF_Update(EKF6Axis_t *ekf, float ax, float ay, float az) {
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm < 0.1f) return;
    ax /= norm; ay /= norm; az /= norm;

    float q0 = ekf->q[0], q1 = ekf->q[1], q2 = ekf->q[2], q3 = ekf->q[3];

    // 观测模型 h(x)
    float h[3] = {
        2.0f * (q1*q3 - q0*q2),
        2.0f * (q0*q1 + q2*q3),
        q0*q0 - q1*q1 - q2*q2 + q3*q3
    };

    // 雅可比矩阵 H
    float H[3][4] = {
        {-2.0f*q2,  2.0f*q3, -2.0f*q0,  2.0f*q1},
        { 2.0f*q1,  2.0f*q0,  2.0f*q3,  2.0f*q2},
        { 2.0f*q0, -2.0f*q1, -2.0f*q2,  2.0f*q3}
    };

    // 计算 S = HPH' + R
    float PHt[4][3] = {0};
    for(int i=0; i<4; i++)
        for(int j=0; j<3; j++)
            for(int k=0; k<4; k++) PHt[i][j] += ekf->P[i][k] * H[j][k];

    float S[3][3] = {0};
    for(int i=0; i<3; i++)
        for(int j=0; j<3; j++) {
            for(int k=0; k<4; k++) S[i][j] += H[i][k] * PHt[k][j];
            S[i][j] += ekf->R[i][j];
        }

    // 3x3 矩阵求逆 (伴随矩阵法)
    float det = S[0][0]*(S[1][1]*S[2][2] - S[1][2]*S[2][1]) -
                S[0][1]*(S[1][0]*S[2][2] - S[1][2]*S[2][0]) +
                S[0][2]*(S[1][0]*S[2][1] - S[1][1]*S[2][0]);
    if (fabsf(det) < 1e-8f) return;
    float invDet = 1.0f / det;
    float invS[3][3];
    invS[0][0] = (S[1][1]*S[2][2] - S[1][2]*S[2][1]) * invDet;
    invS[0][1] = (S[0][2]*S[2][1] - S[0][1]*S[2][2]) * invDet;
    invS[0][2] = (S[0][1]*S[1][2] - S[0][2]*S[1][1]) * invDet;
    invS[1][0] = (S[1][2]*S[2][0] - S[1][0]*S[2][2]) * invDet;
    invS[1][1] = (S[0][0]*S[2][2] - S[0][2]*S[2][0]) * invDet;
    invS[1][2] = (S[0][2]*S[1][0] - S[0][0]*S[1][2]) * invDet;
    invS[2][0] = (S[1][0]*S[2][1] - S[1][1]*S[2][0]) * invDet;
    invS[2][1] = (S[0][1]*S[2][0] - S[0][0]*S[2][1]) * invDet;
    invS[2][2] = (S[0][0]*S[1][1] - S[0][1]*S[1][0]) * invDet;

    // K = PH' * invS
    float K[4][3] = {0};
    for(int i=0; i<4; i++)
        for(int j=0; j<3; j++)
            for(int k=0; k<3; k++) K[i][j] += PHt[i][k] * invS[k][j];

    // 更新状态
    float dz[3] = {ax - h[0], ay - h[1], az - h[2]};
    for(int i=0; i<4; i++)
        ekf->q[i] += K[i][0]*dz[0] + K[i][1]*dz[1] + K[i][2]*dz[2];
    normalize_q(ekf->q);

    // P = (I - KH)P
    float KH[4][4] = {0};
    for(int i=0; i<4; i++)
        for(int j=0; j<4; j++)
            for(int k=0; k<3; k++) KH[i][j] += K[i][k] * H[k][j];

    float P_new[4][4] = {0};
    for(int i=0; i<4; i++)
        for(int j=0; j<4; j++) {
            float I_KH = ((i==j)?1.0f:0.0f) - KH[i][j];
            for(int k=0; k<4; k++) P_new[i][j] += I_KH * ekf->P[k][j];
        }
    for(int i=0; i<4; i++)
        for(int j=0; j<4; j++) ekf->P[i][j] = P_new[i][j];
}

void EKF_GetEuler(EKF6Axis_t *ekf, float *roll, float *pitch, float *yaw) {
    float q0 = ekf->q[0], q1 = ekf->q[1], q2 = ekf->q[2], q3 = ekf->q[3];
    *roll  = atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2)) * EKF_RAD_TO_DEG;
    *pitch = asinf(2.0f*(q0*q2 - q3*q1)) * EKF_RAD_TO_DEG;
    *yaw   = atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3)) * EKF_RAD_TO_DEG;
}