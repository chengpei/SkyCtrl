#include "mpu6050.h"

/* MPU6050 寄存器地址 */
#define MPU6050_RA_WHO_AM_I     0x75
#define MPU6050_RA_PWR_MGMT_1  0x6B
#define MPU6050_RA_ACCEL_XOUT_H 0x3B
#define MPU6050_RA_GYRO_CONFIG  0x1B
#define MPU6050_RA_ACCEL_CONFIG 0x1C

/* 内部函数：写 1 字节 */
static HAL_StatusTypeDef MPU6050_WriteByte(uint8_t reg, uint8_t data)
{
    return HAL_I2C_Mem_Write(&hi2c1,
                             MPU6050_ADDR,
                             reg,
                             I2C_MEMADD_SIZE_8BIT,
                             &data,
                             1,
                             100);
}

/* 内部函数：读 N 字节 */
static HAL_StatusTypeDef MPU6050_ReadBytes(uint8_t reg, uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(&hi2c1,
                            MPU6050_ADDR,
                            reg,
                            I2C_MEMADD_SIZE_8BIT,
                            buf,
                            len,
                            100);
}

/* ================= 对外接口 ================= */

/**
 * @brief  初始化 MPU6050
 * @retval 0: 成功  1: 失败
 */
uint8_t MPU6050_Init(void)
{
    uint8_t id = MPU6050_ReadID();
    if (id != 0x68 && id != 0x69 && id != 0x70)
        return 0;

    /* 退出睡眠模式 */
    if (MPU6050_WriteByte(MPU6050_RA_PWR_MGMT_1, 0x00) != HAL_OK)
        return 0;

    HAL_Delay(10);

    /* 陀螺仪量程 ±2000 dps (11) */
    MPU6050_WriteByte(MPU6050_RA_GYRO_CONFIG, 0x18);

    /* 加速度量程 ±2g (00) */
    MPU6050_WriteByte(MPU6050_RA_ACCEL_CONFIG, 0x00);

    return 1;
}

/**
 * @brief  读取 WHO_AM_I
 */
uint8_t MPU6050_ReadID(void)
{
    uint8_t id = 0;
    MPU6050_ReadBytes(MPU6050_RA_WHO_AM_I, &id, 1);
    return id;
}

/**
 * @brief  读取加速度 & 陀螺仪原始数据
 */
uint8_t MPU6050_ReadRawData(MPU6050_RawData_t *data)
{
    uint8_t buf[14];

    if (MPU6050_ReadBytes(MPU6050_RA_ACCEL_XOUT_H, buf, 14) != HAL_OK)
        return 1;

    data->ax = (int16_t)(buf[0]  << 8 | buf[1]);
    data->ay = (int16_t)(buf[2]  << 8 | buf[3]);
    data->az = (int16_t)(buf[4]  << 8 | buf[5]);

    /* buf[6], buf[7] 是温度，可忽略 */

    data->gx = (int16_t)(buf[8]  << 8 | buf[9]);
    data->gy = (int16_t)(buf[10] << 8 | buf[11]);
    data->gz = (int16_t)(buf[12] << 8 | buf[13]);

    return 0;
}
