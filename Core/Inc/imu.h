#ifndef __IMU_H
#define __IMU_H

#include "stdint.h"

typedef struct {
    float ax, ay, az;   // 加速度 g
    float gx, gy, gz;   // 角速度 °/s
} IMU_Data_t;

/**
 * @brief  IMU 初始化（包含上电静止校准）
 */
void IMU_Init(void);

/**
 * @brief  更新 IMU 数据（建议周期调用，如 1kHz / 500Hz / 100Hz）
 * @param  imu 输出的 IMU 数据
 */
void IMU_Update(IMU_Data_t *imu);

#endif
