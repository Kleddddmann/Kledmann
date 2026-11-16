/*
 * flight.h - 飞控主逻辑：解锁状态机、模式切换、PID 级联、失控保护
 *
 * 两种飞行模式，由遥控器 CH_LEVEL 通道（AUX2）切换：
 *   LEVEL（开关 ON）：自动平衡。摇杆回中飞机自动回到水平，摇杆打多少就倾斜多少度。
 *                     角度环 -> 角速度环 两级 PID。
 *   RATE （开关 OFF）：手动/特技。摇杆控制旋转速度，松杆保持当前姿态，不会自动回平。
 *                     只有角速度环一级 PID。
 */
#ifndef FC_FLIGHT_H
#define FC_FLIGHT_H

#include <stdbool.h>
#include <stdint.h>
#include "attitude.h"
#include "mpu6050.h"
#include "pid.h"

typedef enum {
    FC_DISARMED = 0,   /* 上锁，电机不转 */
    FC_ARMED,          /* 解锁，正常飞行 */
    FC_FAILSAFE,       /* 失控保护，电机停止，需要重新解锁 */
} fc_arm_state_t;

typedef enum {
    FC_MODE_RATE = 0,  /* 手动速率模式（自动平衡关） */
    FC_MODE_LEVEL,     /* 自稳模式（自动平衡开） */
} fc_mode_t;

typedef struct {
    fc_arm_state_t arm_state;
    fc_mode_t      mode;
    bool           arm_switch;       /* 带迟滞后的开关状态 */
    bool           level_switch;
    bool           arm_switch_seen_off; /* 上电后解锁开关必须先经过 OFF 才允许解锁 */

    attitude_t     att;
    imu_data_t     imu;

    pid_t pid_rate_roll, pid_rate_pitch, pid_rate_yaw;
    pid_t pid_angle_roll, pid_angle_pitch;

    float roll_sp_dps, pitch_sp_dps, yaw_sp_dps;   /* 内环目标 */
    float roll_sp_deg, pitch_sp_deg;               /* 外环目标（仅 LEVEL 模式） */
    float torque[3];                               /* PID 输出 */
    float throttle;
    uint16_t motor_us[4];

    uint32_t loop_count;
    uint32_t imu_errors;
} fc_state_t;

void fc_init(fc_state_t *fc);

/* 校准后把姿态初始化为当前加速度计角度 */
void fc_set_initial_attitude(fc_state_t *fc, const imu_data_t *imu);

/* 每个控制周期调用一次。imu_ok=false 表示本周期 IMU 读取失败 */
void fc_update(fc_state_t *fc, bool imu_ok, float dt);

#endif /* FC_FLIGHT_H */
