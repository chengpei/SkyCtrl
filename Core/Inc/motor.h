#ifndef __MOTOR_H
#define __MOTOR_H

#include "mahony.h"

/* ================= 配置区 ================= */

#define MOTOR_NUM          4

#define MOTOR_MIN_US       1000
#define MOTOR_MAX_US       2000

/* 初期调试用，别太高 */
#define MOTOR_LEVEL_1_US   1150
#define MOTOR_LEVEL_2_US   1250
#define MOTOR_LEVEL_3_US   1350

/* ESC 解锁时间（ms） */
#define MOTOR_ARM_DELAY_MS 3000

/* ================= 接口函数 ================= */

/**
 * @brief  初始化并启动所有电机 PWM
 */
void Motor_Init(void);

/**
 * @brief  ESC 解锁（最小油门保持一段时间）
 */
void Motor_Arm(void);

/**
 * @brief  设置单个电机油门（us）
 * @param  id: 1~4
 * @param  us: 1000~2000
 */
void Motor_SetPWM(uint8_t id, uint16_t us);

/**
 * @brief  设置所有电机油门（us）
 */
void Motor_SetAll(uint16_t us);

/**
 * @brief  停止所有电机（最小油门）
 */
void Motor_StopAll(void);

/**
 * @brief 根据 Roll/Pitch/Yaw 目标角度和姿态计算电机输出
 */
void MotorControl_Update(const Attitude_t *att, float roll_target, float pitch_target, float yaw_target, float base_throttle);

#endif
