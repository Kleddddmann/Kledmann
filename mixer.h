/*
 * mixer.h - X 型四轴混控：油门 + 三轴力矩 -> 4 个电机
 */
#ifndef FC_MIXER_H
#define FC_MIXER_H

#include <stdint.h>

/*
 * throttle : 0..1
 * roll/pitch/yaw : -1..1 的归一化力矩（PID 输出）
 * out_us   : 4 个电机的 PWM 脉宽，范围 MOTOR_IDLE_US..MOTOR_MAX_US
 */
void mixer_run(float throttle, float roll, float pitch, float yaw, uint16_t out_us[4]);

#endif /* FC_MIXER_H */
