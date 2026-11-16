/*
 * mpu6050.h - MPU6050 I2C 驱动
 * 输出已经换算到机体系（见 config.h 的 IMU_BODY_* 宏）：
 *   gyro[3]  : deg/s，已减去上电校准的零偏
 *   accel[3] : g
 */
#ifndef FC_MPU6050_H
#define FC_MPU6050_H

#include <stdbool.h>
#include <stdint.h>
#include "config.h"

typedef struct {
    float gyro[3];    /* p, q, r  (deg/s) 机体系 */
    float accel[3];   /* ax, ay, az (g)  机体系 */
    float temp_c;
} imu_data_t;

bool mpu6050_init(I2C_HandleTypeDef *hi2c);

/* 上电陀螺零偏校准。飞机必须静止。返回 false 表示检测到在动或读取失败。 */
bool mpu6050_calibrate_gyro(void);

/* 读取一帧。返回 false 表示 I2C 失败（上一帧数据保留在 out 里不变）。 */
bool mpu6050_read(imu_data_t *out);

#endif /* FC_MPU6050_H */
