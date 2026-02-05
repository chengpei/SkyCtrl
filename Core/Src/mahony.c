#include "mahony.h"
#include <math.h>

// 四元数表示姿态
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

// Mahony 参数
static float Kp = 1.0f;
static float Ki = 0.0f;
static float integralFBx = 0, integralFBy = 0, integralFBz = 0;

/**
 * @brief 初始化 Mahony 滤波器
 */
void Mahony_Init(float kp, float ki)
{
    Kp = kp;
    Ki = ki;
    q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
    integralFBx = 0; integralFBy = 0; integralFBz = 0;
}

/**
 * @brief Mahony 滤波器更新
 */
void Mahony_Update(IMU_Data_t *imu, float dt, Attitude_t *att)
{
    float ax = imu->ax;
    float ay = imu->ay;
    float az = imu->az;
    float gx = imu->gx * (M_PI / 180.0f); // °/s -> rad/s
    float gy = imu->gy * (M_PI / 180.0f);
    float gz = imu->gz * (M_PI / 180.0f);

    // 1. 归一化加速度向量
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm == 0.0f) return; // 避免除零
    ax /= norm;
    ay /= norm;
    az /= norm;

    // 2. 估计重力方向
    float vx = 2*(q1*q3 - q0*q2);
    float vy = 2*(q0*q1 + q2*q3);
    float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    // 3. 误差计算 (加速度 - 重力向量)
    float ex = (ay*vz - az*vy);
    float ey = (az*vx - ax*vz);
    float ez = (ax*vy - ay*vx);

    // 4. 积分反馈
    if(Ki > 0.0f)
    {
        integralFBx += Ki*ex*dt;
        integralFBy += Ki*ey*dt;
        integralFBz += Ki*ez*dt;

        gx += integralFBx;
        gy += integralFBy;
        gz += integralFBz;
    }

    // 5. 比例反馈
    gx += Kp*ex;
    gy += Kp*ey;
    gz += Kp*ez;

    // 6. 四元数微分
    float qDot0 = 0.5f * (-q1*gx - q2*gy - q3*gz);
    float qDot1 = 0.5f * ( q0*gx + q2*gz - q3*gy);
    float qDot2 = 0.5f * ( q0*gy - q1*gz + q3*gx);
    float qDot3 = 0.5f * ( q0*gz + q1*gy - q2*gx);

    // 7. 积分更新四元数
    q0 += qDot0 * dt;
    q1 += qDot1 * dt;
    q2 += qDot2 * dt;
    q3 += qDot3 * dt;

    // 8. 四元数归一化
    norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 /= norm;
    q1 /= norm;
    q2 /= norm;
    q3 /= norm;

    // 9. 输出 Roll / Pitch / Yaw（单位：度）
    att->roll  = atan2f(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2)) * (180.0f / M_PI);
    att->pitch = asinf(2*(q0*q2 - q3*q1)) * (180.0f / M_PI);
    att->yaw   = atan2f(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3)) * (180.0f / M_PI);
}
