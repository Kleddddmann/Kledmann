#include "attitude.h"
#include <math.h>

#define DEG2RAD(x) ((x) * 0.017453292519943f)
#define RAD2DEG(x) ((x) * 57.29577951308232f)

void attitude_init(attitude_t *att)
{
    att->roll_deg = 0.0f;
    att->pitch_deg = 0.0f;
    att->yaw_deg = 0.0f;
    att->acc_norm_g = 1.0f;
}

/*
 * 机体系 NED：重力在机体系中的方向向量 g_b = -accel（静止时加速度计量到的是支撑力，朝上）
 * 水平时 g_b = (0, 0, 1)
 *   roll  = atan2(g_by, g_bz)
 *   pitch = atan2(-g_bx, sqrt(g_by^2 + g_bz^2))
 */
static void accel_to_angles(const imu_data_t *imu, float *roll, float *pitch)
{
    float gx = -imu->accel[0];
    float gy = -imu->accel[1];
    float gz = -imu->accel[2];
    *roll  = RAD2DEG(atan2f(gy, gz));
    *pitch = RAD2DEG(atan2f(-gx, sqrtf(gy * gy + gz * gz)));
}

void attitude_set_from_accel(attitude_t *att, const imu_data_t *imu)
{
    accel_to_angles(imu, &att->roll_deg, &att->pitch_deg);
    att->yaw_deg = 0.0f;
}

void attitude_update(attitude_t *att, const imu_data_t *imu, float dt)
{
    /* 1. 陀螺积分：机体角速度 (p,q,r) -> 欧拉角变化率 */
    float p = imu->gyro[0], q = imu->gyro[1], r = imu->gyro[2];
    float phi = DEG2RAD(att->roll_deg);
    float the = DEG2RAD(att->pitch_deg);

    /* pitch 接近 ±90° 时 tan 发散，限制一下。四轴正常飞行不会到这里 */
    if (the > DEG2RAD(85.0f))  the = DEG2RAD(85.0f);
    if (the < DEG2RAD(-85.0f)) the = DEG2RAD(-85.0f);

    float sp = sinf(phi), cp = cosf(phi);
    float tt = tanf(the), ct = cosf(the);

    float roll_rate  = p + q * sp * tt + r * cp * tt;
    float pitch_rate = q * cp - r * sp;
    float yaw_rate   = (q * sp + r * cp) / ct;

    float roll_gyro  = att->roll_deg  + roll_rate  * dt;
    float pitch_gyro = att->pitch_deg + pitch_rate * dt;
    att->yaw_deg    += yaw_rate * dt;
    if (att->yaw_deg > 180.0f)  att->yaw_deg -= 360.0f;
    if (att->yaw_deg < -180.0f) att->yaw_deg += 360.0f;

    /* 2. 加速度计参考：只有模长接近 1g（没有明显线加速度）时才融合 */
    float ax = imu->accel[0], ay = imu->accel[1], az = imu->accel[2];
    att->acc_norm_g = sqrtf(ax * ax + ay * ay + az * az);

    if (att->acc_norm_g > ATT_ACC_MIN_G && att->acc_norm_g < ATT_ACC_MAX_G) {
        float roll_acc, pitch_acc;
        accel_to_angles(imu, &roll_acc, &pitch_acc);

        /* 互补系数：alpha = tau / (tau + dt)，tau 越大越信陀螺 */
        float alpha = ATT_ACC_TAU_S / (ATT_ACC_TAU_S + dt);
        att->roll_deg  = alpha * roll_gyro  + (1.0f - alpha) * roll_acc;
        att->pitch_deg = alpha * pitch_gyro + (1.0f - alpha) * pitch_acc;
    } else {
        att->roll_deg  = roll_gyro;
        att->pitch_deg = pitch_gyro;
    }
}
