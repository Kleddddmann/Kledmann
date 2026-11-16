/*
 * attitude.h - 互补滤波姿态解算
 * 陀螺积分给出短期精确的角度，加速度计给出长期不漂移的参考，两者融合。
 */
#ifndef FC_ATTITUDE_H
#define FC_ATTITUDE_H

#include "mpu6050.h"

typedef struct {
    float roll_deg;    /* 正 = 右侧下沉 */
    float pitch_deg;   /* 正 = 机头抬起 */
    float yaw_deg;     /* 纯陀螺积分，会漂移，只用于显示 */
    float acc_norm_g;  /* 加速度模长，用于判断是否能信任加速度计 */
} attitude_t;

void attitude_init(attitude_t *att);

/* 用静止时的加速度计直接初始化角度，避免上电后慢慢收敛 */
void attitude_set_from_accel(attitude_t *att, const imu_data_t *imu);

void attitude_update(attitude_t *att, const imu_data_t *imu, float dt);

#endif /* FC_ATTITUDE_H */
