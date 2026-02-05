#ifndef __FLIGHT_CONTROL_H
#define __FLIGHT_CONTROL_H

#include <stdint.h>
#include "mahony.h"

/* ================= 控制输入 ================= */

typedef struct
{
    float stick_x;      // -1.0 ~ +1.0  左右（Roll）
    float stick_y;      // -1.0 ~ +1.0  前后（Pitch）

    uint8_t btn_up;         // 上升
    uint8_t btn_down;       // 下降
    uint8_t btn_yaw_cw;     // 顺时针
    uint8_t btn_yaw_ccw;    // 逆时针
} ControlInput_t;

/* ================= 飞行状态 ================= */

typedef enum
{
    FLIGHT_DISARMED = 0,
    FLIGHT_ARMED,
    FLIGHT_FLYING
} FlightState_t;

/* ================= 接口函数 ================= */

void FlightControl_Init(void);
void FlightControl_SetInput(const ControlInput_t *input);
void FlightControl_Update(const Attitude_t *att);

/* 状态控制 */
void FlightControl_Arm(void);
void FlightControl_Disarm(void);

#endif
