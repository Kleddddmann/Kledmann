#include "pid.h"

#define PID_PI 3.14159265358979f

static inline float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

void pid_init(pid_t *p, float kp, float ki, float kd,
              float i_limit, float out_limit, float d_cutoff_hz)
{
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
    p->i_limit = i_limit;
    p->out_limit = out_limit;
    p->d_cutoff_hz = d_cutoff_hz;
    pid_reset(p);
}

void pid_reset(pid_t *p)
{
    p->integ = 0.0f;
    p->prev_meas = 0.0f;
    p->d_state = 0.0f;
    p->initialised = 0;
}

float pid_update(pid_t *p, float setpoint, float meas, float dt)
{
    float err = setpoint - meas;

    /* P */
    float out = p->kp * err;

    /* I：带限幅防止饱和积累 */
    p->integ += p->ki * err * dt;
    p->integ = clampf(p->integ, -p->i_limit, p->i_limit);
    out += p->integ;

    /* D：对测量值微分而不是对误差微分，摇杆突然打满时不会“踢”一下 */
    float d_raw = 0.0f;
    if (p->initialised && dt > 0.0f) {
        d_raw = -(meas - p->prev_meas) / dt;
    }
    p->prev_meas = meas;
    p->initialised = 1;

    if (p->d_cutoff_hz > 0.0f) {
        float rc = 1.0f / (2.0f * PID_PI * p->d_cutoff_hz);
        float a = dt / (rc + dt);
        p->d_state += a * (d_raw - p->d_state);
    } else {
        p->d_state = d_raw;
    }
    out += p->kd * p->d_state;

    return clampf(out, -p->out_limit, p->out_limit);
}
