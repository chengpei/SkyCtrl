#ifndef MAHONY_H
#define MAHONY_H

#include "imu.h"   // 你的 IMU_Data_t 定义

typedef struct
{
    float roll;   // 单位：度
    float pitch;  // 单位：度
    float yaw;    // 单位：度
} Attitude_t;

/**
 * @brief 初始化 Mahony 滤波器
 * @param Kp 误差比例系数（建议 0.5~2.0）
 * @param Ki 误差积分系数（一般设置 0~0.1，可先0）
 */
void Mahony_Init(float Kp, float Ki);

/**
 * @brief 更新姿态滤波器
 * @param imu 当前 IMU 校准后的 6 轴数据
 * @param dt  两次更新的时间间隔（秒），你的中断 2ms → dt = 0.002f
 * @param att 返回 Roll/Pitch/Yaw
 */
void Mahony_Update(IMU_Data_t *imu, float dt, Attitude_t *att);

#endif
