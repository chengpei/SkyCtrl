#include "mahony.h"
#include <math.h>

/* ================= 参数区 ================= */

#define DEG2RAD        0.01745329251f
#define RAD2DEG        57.2957795131f

#define ACC_TRUST_MIN  0.8f     // g
#define ACC_TRUST_MAX  1.2f     // g

#define INTEGRAL_LIMIT 0.1f     // 防止积分爆炸

/* ================= 四元数状态 ================= */

static float q0 = 1.0f;
static float q1 = 0.0f;
static float q2 = 0.0f;
static float q3 = 0.0f;

/* ================= Mahony 参数 ================= */

static float Kp = 2.0f;      // 推荐 2~5
static float Ki = 0.005f;    // 推荐 0.001~0.01

static float integralFBx = 0.0f;
static float integralFBy = 0.0f;
static float integralFBz = 0.0f;

/* ================= 初始化 ================= */

void Mahony_Init(float kp, float ki)
{
    Kp = kp;
    Ki = ki;

    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;

    integralFBx = 0.0f;
    integralFBy = 0.0f;
    integralFBz = 0.0f;
}

/* ================= 主更新函数 ================= */

void Mahony_Update(IMU_Data_t *imu, float dt, Attitude_t *att)
{
    float ax = imu->ax;
    float ay = imu->ay;
    float az = imu->az;

    float gx = imu->gx * DEG2RAD;
    float gy = imu->gy * DEG2RAD;
    float gz = imu->gz * DEG2RAD;

    float norm;
    float vx, vy, vz;
    float ex = 0.0f, ey = 0.0f, ez = 0.0f;

    /* ---------- 1. 计算加速度模长 ---------- */

    norm = sqrtf(ax*ax + ay*ay + az*az);

    if (norm > 0.0001f)
    {
        float acc_g = norm;  // 如果你单位已经是 g

        /* ---------- 2. 判断加速度是否可信 ---------- */

        if (acc_g > ACC_TRUST_MIN && acc_g < ACC_TRUST_MAX)
        {
            ax /= norm;
            ay /= norm;
            az /= norm;

            /* ---------- 3. 估计重力方向 ---------- */

            vx = 2.0f * (q1*q3 - q0*q2);
            vy = 2.0f * (q0*q1 + q2*q3);
            vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

            /* ---------- 4. 叉乘误差 ---------- */

            ex = (ay * vz - az * vy);
            ey = (az * vx - ax * vz);
            ez = (ax * vy - ay * vx);

            /* ---------- 5. 积分反馈（带限幅） ---------- */

            if (Ki > 0.0f)
            {
                integralFBx += Ki * ex * dt;
                integralFBy += Ki * ey * dt;
                integralFBz += Ki * ez * dt;

                if (integralFBx > INTEGRAL_LIMIT) integralFBx = INTEGRAL_LIMIT;
                if (integralFBx < -INTEGRAL_LIMIT) integralFBx = -INTEGRAL_LIMIT;

                if (integralFBy > INTEGRAL_LIMIT) integralFBy = INTEGRAL_LIMIT;
                if (integralFBy < -INTEGRAL_LIMIT) integralFBy = -INTEGRAL_LIMIT;

                if (integralFBz > INTEGRAL_LIMIT) integralFBz = INTEGRAL_LIMIT;
                if (integralFBz < -INTEGRAL_LIMIT) integralFBz = -INTEGRAL_LIMIT;

                gx += integralFBx;
                gy += integralFBy;
                gz += integralFBz;
            }

            /* ---------- 6. 比例反馈 ---------- */

            gx += Kp * ex;
            gy += Kp * ey;
            gz += Kp * ez;
        }
    }

    /* ---------- 7. 四元数微分 ---------- */

    float qDot0 = 0.5f * (-q1*gx - q2*gy - q3*gz);
    float qDot1 = 0.5f * ( q0*gx + q2*gz - q3*gy);
    float qDot2 = 0.5f * ( q0*gy - q1*gz + q3*gx);
    float qDot3 = 0.5f * ( q0*gz + q1*gy - q2*gx);

    q0 += qDot0 * dt;
    q1 += qDot1 * dt;
    q2 += qDot2 * dt;
    q3 += qDot3 * dt;

    /* ---------- 8. 四元数归一化（带保护） ---------- */

    norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);

    if (norm < 0.0001f)
    {
        q0 = 1.0f;
        q1 = q2 = q3 = 0.0f;
        return;
    }

    q0 /= norm;
    q1 /= norm;
    q2 /= norm;
    q3 /= norm;

    /* ---------- 9. 输出欧拉角 ---------- */

    float sinp = 2.0f * (q0*q2 - q3*q1);

    if (sinp > 1.0f)  sinp = 1.0f;
    if (sinp < -1.0f) sinp = -1.0f;

    att->roll  = atan2f(2.0f*(q0*q1 + q2*q3),
                        1.0f - 2.0f*(q1*q1 + q2*q2)) * RAD2DEG;

    att->pitch = asinf(sinp) * RAD2DEG;

    att->yaw   = atan2f(2.0f*(q0*q3 + q1*q2),
                        1.0f - 2.0f*(q2*q2 + q3*q3)) * RAD2DEG;
}
