#include "motor.h"
#include "stm32f1xx_hal.h"
#include "pid.h"

/* TIM3 由 CubeMX 生成 */
extern TIM_HandleTypeDef htim3;

// PID 对象
PID_t pid_roll, pid_pitch, pid_yaw;

// 四个电机当前 PWM
static float motor_pwm[MOTOR_NUM] = {MOTOR_MIN_US, MOTOR_MIN_US, MOTOR_MIN_US, MOTOR_MIN_US};

/* ================= 内部工具 ================= */

static inline float motor_limit(float us)
{
    if (us < MOTOR_MIN_US) return MOTOR_MIN_US;
    if (us > MOTOR_MAX_US) return MOTOR_MAX_US;
    return us;
}

/* ================= 接口实现 ================= */

void Motor_Init(void)
{
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    // PID 初始化，输出限制 ±500
    PID_Init(&pid_roll,  0.0f, 0.0f, 0.0f, -50, 50);
    PID_Init(&pid_pitch, 0.0f, 0.0f, 0.0f, -50, 50);
    PID_Init(&pid_yaw,   0.0f, 0.0f, 0.0f, -50, 50);

    Motor_StopAll();
}

void Motor_Arm(void)
{
    Motor_SetAll(MOTOR_MIN_US);
    HAL_Delay(MOTOR_ARM_DELAY_MS);
}

void Motor_SetPWM(uint8_t id, uint16_t us)
{
    us = motor_limit(us);

    switch (id)
    {
        case 1:
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, us);
            break;
        case 2:
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, us);
            break;
        case 3:
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, us);
            break;
        case 4:
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, us);
            break;
        default:
            /* 非法 ID，忽略 */
            break;
    }
}

void Motor_SetAll(uint16_t us)
{
    us = motor_limit(us);

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, us);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, us);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, us);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, us);
}

void Motor_StopAll(void)
{
    Motor_SetAll(MOTOR_MIN_US);
}

// 更新四个电机 PWM
void MotorControl_Update(const Attitude_t *att, float roll_target, float pitch_target, float yaw_target, float base_throttle)
{
    float dt = 0.002f; // 2ms 中断

    float roll_out  = PID_Calc(&pid_roll,  roll_target,  att->roll,  dt);
    float pitch_out = PID_Calc(&pid_pitch, pitch_target, att->pitch, dt);
    float yaw_out   = PID_Calc(&pid_yaw,   yaw_target,   att->yaw,   dt);

    // X 型四轴混控
    // M1 左前, M2 右前, M3 左后, M4 右后
    motor_pwm[0] = base_throttle + roll_out - pitch_out + yaw_out; // M1
    motor_pwm[1] = base_throttle - roll_out - pitch_out - yaw_out; // M2
    motor_pwm[2] = base_throttle + roll_out + pitch_out - yaw_out; // M3
    motor_pwm[3] = base_throttle - roll_out + pitch_out + yaw_out; // M4

    // 限幅
    for(int i=0; i<MOTOR_NUM; i++)
    {
        motor_pwm[i] = motor_limit(motor_pwm[i]);
        Motor_SetPWM(i + 1, motor_pwm[i]);
    }
}