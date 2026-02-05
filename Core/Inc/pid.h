#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float last_error;
    float output_min;  // 输出限制
    float output_max;
} PID_t;

/**
 * @brief 初始化 PID 控制器
 */
void PID_Init(PID_t *pid, float Kp, float Ki, float Kd, float out_min, float out_max);

/**
 * @brief 计算 PID 输出
 * @param pid PID 结构体指针
 * @param target 目标值
 * @param current 当前值
 * @param dt 时间间隔（秒）
 * @return PID 输出值
 */
float PID_Calc(PID_t *pid, float target, float current, float dt);

#endif
