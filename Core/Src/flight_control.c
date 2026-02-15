#include "flight_control.h"
#include "motor.h"
#include <string.h>

/* ================= 参数配置 ================= */

#define MAX_ROLL_DEG       15.0f
#define MAX_PITCH_DEG      15.0f
#define MAX_YAW_RATE_DEG   120.0f    // deg/s 速率控制

#define CONTROL_DT         0.002f    // 2ms

#define MOTOR_IDLE_US      1000
#define MOTOR_ARM_US       1050
#define MOTOR_MAX_US       2000

#define THROTTLE_RATE_US   500.0f    // 油门变化速度 us/s

/* ================= 内部变量 ================= */

static ControlInput_t  ctrl_input;
static FlightState_t   flight_state = FLIGHT_DISARMED;

/* 目标量 */
static float target_roll  = 0.0f;
static float target_pitch = 0.0f;
static float target_yaw_rate = 0.0f;
static float base_throttle = MOTOR_IDLE_US;

/* ================= 工具函数 ================= */

static inline float throttle_limit(float us)
{
    if (us < MOTOR_IDLE_US) return MOTOR_IDLE_US;
    if (us > MOTOR_MAX_US)  return MOTOR_MAX_US;
    return us;
}

/* ================= 初始化 ================= */

void FlightControl_Init(void)
{
    memset(&ctrl_input, 0, sizeof(ctrl_input));
    base_throttle = MOTOR_IDLE_US;
    flight_state  = FLIGHT_DISARMED;
}

/* ================= 输入接口 ================= */

void FlightControl_SetInput(const ControlInput_t *input)
{
    ctrl_input = *input;
}

/* ================= 解锁/上锁 ================= */

void FlightControl_Arm(void)
{
    base_throttle = MOTOR_IDLE_US;
    flight_state  = FLIGHT_ARMED;
}

void FlightControl_Disarm(void)
{
    Motor_StopAll();
    flight_state = FLIGHT_DISARMED;
}

/* ================= 核心更新 ================= */

void FlightControl_Update(const Attitude_t *att)
{
    if (flight_state == FLIGHT_DISARMED)
    {
        Motor_StopAll();
        return;
    }

    /* ===== 1. 摇杆映射 ===== */

    target_roll  = ctrl_input.stick_x * MAX_ROLL_DEG;
    target_pitch = ctrl_input.stick_y * MAX_PITCH_DEG;

    /* ---- Yaw 改为速率控制 ---- */
    if (ctrl_input.btn_yaw_cw)
        target_yaw_rate =  MAX_YAW_RATE_DEG;
    else if (ctrl_input.btn_yaw_ccw)
        target_yaw_rate = -MAX_YAW_RATE_DEG;
    else
        target_yaw_rate = 0.0f;

    /* ===== 2. 油门控制（无自动回中）===== */

    if (ctrl_input.btn_up)
        base_throttle += THROTTLE_RATE_US * CONTROL_DT;
    else if (ctrl_input.btn_down)
        base_throttle -= THROTTLE_RATE_US * CONTROL_DT;

    base_throttle = throttle_limit(base_throttle);

    /* ===== 3. 状态机管理 ===== */

    if (flight_state == FLIGHT_ARMED &&
        base_throttle > MOTOR_ARM_US)
    {
        flight_state = FLIGHT_FLYING;
    }

    if (flight_state == FLIGHT_FLYING &&
        base_throttle <= MOTOR_ARM_US)
    {
        flight_state = FLIGHT_ARMED;
    }

    /* ===== 4. 调用电机控制 =====
       注意：MotorControl_Update 内部应实现：
       - roll/pitch 角度外环
       - 角速度内环
       - yaw 速率 PID
    */
    if (flight_state == FLIGHT_FLYING)
    {
        MotorControl_Update(att,
                            target_roll,
                            target_pitch,
                            target_yaw_rate,
                            base_throttle);
    }
    else
    {
        Motor_SetAll(MOTOR_IDLE_US);  // 或 MOTOR_ARM_US
    }
}
