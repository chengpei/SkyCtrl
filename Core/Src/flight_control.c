#include "flight_control.h"
#include "motor.h"
#include <string.h>

/* ================= 参数配置 ================= */

#define MAX_ROLL_DEG     15.0f
#define MAX_PITCH_DEG    15.0f
#define YAW_RATE_DEG     30.0f     // 每秒最大偏航
#define CONTROL_DT       0.002f    // 2ms

#define HOVER_THROTTLE_US  1300  // 悬停油门
#define MOTOR_IDLE_US    1050
#define MOTOR_HOVER_US   1300

#define THROTTLE_RATE_US  400.0f   // 抬升下降速度 us / s
#define THROTTLE_RETURN_RATE  600.0f   // 回中速度

/* ================= 内部变量 ================= */

static ControlInput_t  ctrl_input;
static FlightState_t   flight_state = FLIGHT_DISARMED;

/* 飞控输出目标 */
static float    target_roll  = 0.0f;
static float    target_pitch = 0.0f;
static float    target_yaw   = 0.0f;
static float base_throttle = MOTOR_IDLE_US;

/* ================= 内部工具 ================= */

static inline float throttle_limit(float us)
{
    if (us < MOTOR_IDLE_US) return MOTOR_IDLE_US;
    if (us > MOTOR_MAX_US)  return MOTOR_MAX_US;
    return us;
}

static inline float yaw_wrap(float yaw)
{
    if (yaw > 180.0f) yaw -= 360.0f;
    if (yaw < -180.0f) yaw += 360.0f;
    return yaw;
}

/* ================= 接口实现 ================= */

void FlightControl_Init(void)
{
    memset(&ctrl_input, 0, sizeof(ctrl_input));
    base_throttle = MOTOR_IDLE_US;
    flight_state  = FLIGHT_DISARMED;
}

void FlightControl_SetInput(const ControlInput_t *input)
{
    ctrl_input = *input;   // 直接拷贝，解耦通信层
}

void FlightControl_Arm(void)
{
    base_throttle = MOTOR_IDLE_US;
    flight_state = FLIGHT_ARMED;
}

void FlightControl_Disarm(void)
{
    Motor_StopAll();
    flight_state = FLIGHT_DISARMED;
}

/* ================= 核心更新函数 ================= */

void FlightControl_Update(const Attitude_t *att)
{
    if (flight_state == FLIGHT_DISARMED)
    {
        Motor_StopAll();
        return;
    }

    /* ===== 1. 摇杆 → 姿态目标 ===== */
    target_roll  = ctrl_input.stick_x * MAX_ROLL_DEG;
    target_pitch = ctrl_input.stick_y * MAX_PITCH_DEG;

    /* ===== 2. 偏航控制 旋转是一个布尔值，旋转就以每秒30度的速度匀速转 ===== */
    if (ctrl_input.btn_yaw_cw)
        target_yaw += YAW_RATE_DEG * CONTROL_DT;
    else if (ctrl_input.btn_yaw_ccw)
        target_yaw -= YAW_RATE_DEG * CONTROL_DT;
    target_yaw = yaw_wrap(target_yaw);

    /* ===== 3. 推力控制 ===== */
    if (ctrl_input.btn_up) {
        base_throttle += THROTTLE_RATE_US * CONTROL_DT;
    } else if (ctrl_input.btn_down) {
        base_throttle -= THROTTLE_RATE_US * CONTROL_DT;
    } else {
        // 松手：自动回到悬停油门
        if (base_throttle < HOVER_THROTTLE_US)
            base_throttle += THROTTLE_RETURN_RATE * CONTROL_DT;
        else if (base_throttle > HOVER_THROTTLE_US)
            base_throttle -= THROTTLE_RETURN_RATE * CONTROL_DT;
    }
    base_throttle = throttle_limit(base_throttle);

    /* ===== 4. 进入飞行状态 ===== */
    if (flight_state == FLIGHT_ARMED &&
        base_throttle > MOTOR_IDLE_US + 50)
    {
        flight_state = FLIGHT_FLYING;
    }

    /* ===== 5. 调用底层电机控制 =====  */
    MotorControl_Update(att,
                        target_roll,
                        target_pitch,
                        target_yaw,
                        base_throttle);
}
