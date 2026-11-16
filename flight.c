#include "flight.h"
#include "config.h"
#include "mixer.h"
#include "motors.h"
#include "rc.h"
#include <string.h>

static inline float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

void fc_init(fc_state_t *fc)
{
    memset(fc, 0, sizeof(*fc));
    fc->arm_state = FC_DISARMED;
    fc->mode = FC_MODE_RATE;
    attitude_init(&fc->att);

    pid_init(&fc->pid_rate_roll,  PID_RATE_ROLL_P,  PID_RATE_ROLL_I,  PID_RATE_ROLL_D,
             PID_RATE_I_LIMIT, PID_RATE_OUT_LIMIT, PID_RATE_D_CUTOFF_HZ);
    pid_init(&fc->pid_rate_pitch, PID_RATE_PITCH_P, PID_RATE_PITCH_I, PID_RATE_PITCH_D,
             PID_RATE_I_LIMIT, PID_RATE_OUT_LIMIT, PID_RATE_D_CUTOFF_HZ);
    pid_init(&fc->pid_rate_yaw,   PID_RATE_YAW_P,   PID_RATE_YAW_I,   PID_RATE_YAW_D,
             PID_RATE_I_LIMIT, PID_RATE_YAW_OUT_LIMIT, 0.0f);

    /* 外环输出是角速度目标，限幅到 MAX_RATE_DPS */
    pid_init(&fc->pid_angle_roll,  PID_ANGLE_P, PID_ANGLE_I, PID_ANGLE_D,
             MAX_RATE_DPS * 0.5f, MAX_RATE_DPS, 0.0f);
    pid_init(&fc->pid_angle_pitch, PID_ANGLE_P, PID_ANGLE_I, PID_ANGLE_D,
             MAX_RATE_DPS * 0.5f, MAX_RATE_DPS, 0.0f);

    for (int i = 0; i < 4; i++) fc->motor_us[i] = MOTOR_MIN_US;
}

void fc_set_initial_attitude(fc_state_t *fc, const imu_data_t *imu)
{
    attitude_set_from_accel(&fc->att, imu);
}

static void reset_all_pids(fc_state_t *fc)
{
    pid_reset(&fc->pid_rate_roll);
    pid_reset(&fc->pid_rate_pitch);
    pid_reset(&fc->pid_rate_yaw);
    pid_reset(&fc->pid_angle_roll);
    pid_reset(&fc->pid_angle_pitch);
}

/* ---------- 解锁状态机 ---------- */
static void update_arming(fc_state_t *fc)
{
    bool failsafe = rc_is_failsafe();
    fc->arm_switch = rc_switch(CH_ARM, fc->arm_switch);

    /* 安全：上电时解锁开关如果已经是 ON，必须先拨回 OFF 一次才能解锁 */
    if (!fc->arm_switch) fc->arm_switch_seen_off = true;

    switch (fc->arm_state) {
    case FC_DISARMED:
        if (failsafe) break;
        if (fc->arm_switch && fc->arm_switch_seen_off &&
            rc_throttle() <= ARM_THROTTLE_MAX) {
            reset_all_pids(fc);
            fc->arm_state = FC_ARMED;
        }
        break;

    case FC_ARMED:
        if (failsafe) {
            fc->arm_state = FC_FAILSAFE;
        } else if (!fc->arm_switch) {
            fc->arm_state = FC_DISARMED;
        }
        break;

    case FC_FAILSAFE:
        /* 信号恢复后要求开关先 OFF 再 ON 才能重新解锁，防止信号恢复瞬间电机突然转 */
        if (!failsafe && !fc->arm_switch) {
            fc->arm_state = FC_DISARMED;
        }
        break;
    }
}

/* ---------- 模式切换（自动平衡开关） ---------- */
static void update_mode(fc_state_t *fc)
{
    fc->level_switch = rc_switch(CH_LEVEL, fc->level_switch);

    fc_mode_t new_mode = fc->level_switch ? FC_MODE_LEVEL : FC_MODE_RATE;
    if (new_mode != fc->mode) {
        fc->mode = new_mode;
        /* 切换时清掉外环积分，避免带着旧的积分值进入新模式 */
        pid_reset(&fc->pid_angle_roll);
        pid_reset(&fc->pid_angle_pitch);
    }
}

/* ---------- 主更新 ---------- */
void fc_update(fc_state_t *fc, bool imu_ok, float dt)
{
    fc->loop_count++;
    if (!imu_ok) fc->imu_errors++;

    /* 1. 姿态解算（IMU 失败时沿用上一帧的 imu 数据继续积分，短暂丢帧问题不大） */
    attitude_update(&fc->att, &fc->imu, dt);

    /* 2. 解锁 / 模式 */
    update_arming(fc);
    update_mode(fc);

    /* 3. 未解锁：电机停，PID 清零，直接返回 */
    if (fc->arm_state != FC_ARMED) {
        reset_all_pids(fc);
        motors_stop();
        for (int i = 0; i < 4; i++) fc->motor_us[i] = MOTOR_MIN_US;
        fc->throttle = 0.0f;
        return;
    }

    /* 4. 读摇杆 */
    fc->throttle = rc_throttle();
    float stick_roll  = rc_stick(CH_ROLL);
    float stick_pitch = rc_stick(CH_PITCH);
    float stick_yaw   = rc_stick(CH_YAW);

    /* 5. 生成内环（角速度）目标 */
    if (fc->mode == FC_MODE_LEVEL) {
        /* 自稳：摇杆 -> 目标角度，角度环 -> 目标角速度 */
        fc->roll_sp_deg  = stick_roll  * MAX_ANGLE_DEG;
        fc->pitch_sp_deg = stick_pitch * MAX_ANGLE_DEG;
        fc->roll_sp_dps  = pid_update(&fc->pid_angle_roll,  fc->roll_sp_deg,  fc->att.roll_deg,  dt);
        fc->pitch_sp_dps = pid_update(&fc->pid_angle_pitch, fc->pitch_sp_deg, fc->att.pitch_deg, dt);
    } else {
        /* 速率：摇杆直接 -> 目标角速度 */
        fc->roll_sp_deg  = 0.0f;
        fc->pitch_sp_deg = 0.0f;
        fc->roll_sp_dps  = stick_roll  * MAX_RATE_DPS;
        fc->pitch_sp_dps = stick_pitch * MAX_RATE_DPS;
    }
    /* 偏航两种模式都是速率控制 */
    fc->yaw_sp_dps = stick_yaw * MAX_YAW_RATE_DPS;

    /* 6. 内环 PID：角速度误差 -> 归一化力矩 */
    fc->torque[0] = pid_update(&fc->pid_rate_roll,  fc->roll_sp_dps,  fc->imu.gyro[0], dt);
    fc->torque[1] = pid_update(&fc->pid_rate_pitch, fc->pitch_sp_dps, fc->imu.gyro[1], dt);
    fc->torque[2] = pid_update(&fc->pid_rate_yaw,   fc->yaw_sp_dps,   fc->imu.gyro[2], dt);

    /* 7. 油门很低（在地上）时：电机怠速、不输出力矩、积分清零，防止飞机在地上乱扭 */
    if (fc->throttle < IDLE_THROTTLE) {
        reset_all_pids(fc);
        for (int i = 0; i < 4; i++) fc->motor_us[i] = MOTOR_IDLE_US;
        motors_write_us(fc->motor_us);
        return;
    }

    /* 8. 混控 -> 电机 */
    mixer_run(fc->throttle, fc->torque[0], fc->torque[1], fc->torque[2], fc->motor_us);
    motors_write_us(fc->motor_us);
}
