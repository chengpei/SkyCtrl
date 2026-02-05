#ifndef __MPU6050_H
#define __MPU6050_H

#include "stm32f1xx_hal.h"

/* ================= 配置区 ================= */

/* I2C 句柄（在 mpu6050.c 里 extern） */
extern I2C_HandleTypeDef hi2c1;

/* MPU6050 I2C 地址
 * GY-521 默认 ADO 接 GND → 0x68
 */
#define MPU6050_ADDR      (0x68 << 1)

/* ================= 数据结构 ================= */

typedef struct
{
    int16_t ax;
    int16_t ay;
    int16_t az;

    int16_t gx;
    int16_t gy;
    int16_t gz;
} MPU6050_RawData_t;

/* ================= 接口函数 ================= */

/* 初始化 MPU6050 */
uint8_t MPU6050_Init(void);

/* 读取 WHO_AM_I */
uint8_t MPU6050_ReadID(void);

/* 读取加速度 + 陀螺仪原始数据 */
uint8_t MPU6050_ReadRawData(MPU6050_RawData_t *data);

#endif
