#include "imu.h"
#include "mpu6050.h"
#include "math.h"

#define IMU_CALIB_SAMPLES   1000

/* MPU6050 量程对应比例 */
#define ACC_SENS_2G        16384.0f     // LSB/g
#define GYRO_SENS_2000DPS  16.4f        // LSB/(°/s)

/* 零偏 */
static float ax_off = 0, ay_off = 0, az_off = 0;
static float gx_off = 0, gy_off = 0, gz_off = 0;

void IMU_Init(void)
{
    MPU6050_RawData_t raw;
    int32_t ax_sum = 0, ay_sum = 0, az_sum = 0;
    int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;

    /* 上电后保持静止！ */
    for (int i = 0; i < IMU_CALIB_SAMPLES; i++)
    {
        MPU6050_ReadRawData(&raw);

        ax_sum += raw.ax;
        ay_sum += raw.ay;
        az_sum += raw.az;

        gx_sum += raw.gx;
        gy_sum += raw.gy;
        gz_sum += raw.gz;

        HAL_Delay(2);   // ~500Hz 采样
    }

    ax_off = ax_sum / (float)IMU_CALIB_SAMPLES;
    ay_off = ay_sum / (float)IMU_CALIB_SAMPLES;
    az_off = az_sum / (float)IMU_CALIB_SAMPLES - ACC_SENS_2G;
    // Z 轴默认朝上，减去 1g

    gx_off = gx_sum / (float)IMU_CALIB_SAMPLES;
    gy_off = gy_sum / (float)IMU_CALIB_SAMPLES;
    gz_off = gz_sum / (float)IMU_CALIB_SAMPLES;
}

void IMU_Update(IMU_Data_t *imu)
{
    MPU6050_RawData_t raw;

    MPU6050_ReadRawData(&raw);

    /* 去零偏 */
    float ax = raw.ax - ax_off;
    float ay = raw.ay - ay_off;
    float az = raw.az - az_off;

    float gx = raw.gx - gx_off;
    float gy = raw.gy - gy_off;
    float gz = raw.gz - gz_off;

    /* 单位换算 */
    imu->ax = ax / ACC_SENS_2G;
    imu->ay = ay / ACC_SENS_2G;
    imu->az = az / ACC_SENS_2G;

    imu->gx = gx / GYRO_SENS_2000DPS;
    imu->gy = gy / GYRO_SENS_2000DPS;
    imu->gz = gz / GYRO_SENS_2000DPS;
}
