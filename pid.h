/*
 * pid.h - 带积分限幅、D 项低通、测量值微分的 PID
 */
#ifndef FC_PID_H
#define FC_PID_H

#include <stdint.h>

typedef struct {
    float kp, ki, kd;
    float i_limit;        /* 积分项绝对值上限 */
    float out_limit;      /* 输出绝对值上限 */
    float d_cutoff_hz;    /* D 项一阶低通截止频率，<=0 不滤波 */

    float integ;          /* 积分状态 */
    float prev_meas;      /* 上一次测量值（用于测量值微分） */
    float d_state;        /* D 项滤波状态 */
    uint8_t initialised;
} pid_t;

void  pid_init(pid_t *p, float kp, float ki, float kd,
               float i_limit, float out_limit, float d_cutoff_hz);
void  pid_reset(pid_t *p);
float pid_update(pid_t *p, float setpoint, float meas, float dt);

#endif /* FC_PID_H */
