#include "pid.h"

void PID_Init(PID_t *pid, float Kp, float Ki, float Kd, float out_min, float out_max)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->output_min = out_min;
    pid->output_max = out_max;
}

float PID_Calc(PID_t *pid, float target, float current, float dt)
{
    float error = target - current;
    pid->integral += error * dt;
    float derivative = (error - pid->last_error) / dt;
    pid->last_error = error;

    float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;

    // 限幅，避免 PWM 越界
    if (output > pid->output_max) output = pid->output_max;
    if (output < pid->output_min) output = pid->output_min;

    return output;
}
