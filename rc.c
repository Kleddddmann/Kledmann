#include "rc.h"
#include <string.h>

static rc_state_t s_rc;

void rc_init(void)
{
    memset(&s_rc, 0, sizeof(s_rc));
    for (int i = 0; i < RC_MAX_CH; i++) s_rc.ch[i] = 1500;
    s_rc.ch[CH_THROTTLE] = 1000;
    s_rc.ch[CH_ARM] = 1000;
    s_rc.ch[CH_LEVEL] = 1000;
    s_rc.last_frame_ms = 0;
}

void rc_feed(const uint8_t *buf, uint16_t len)
{
    uint16_t tmp[RC_MAX_CH];
    for (uint16_t i = 0; i < len; i++) {
        bool got;
#if RC_PROTOCOL == RC_PROTOCOL_CRSF
        got = rc_crsf_feed_byte(buf[i], tmp);
#elif RC_PROTOCOL == RC_PROTOCOL_IBUS
        got = rc_ibus_feed_byte(buf[i], tmp);
#else
#error "RC_PROTOCOL 必须是 RC_PROTOCOL_CRSF 或 RC_PROTOCOL_IBUS"
#endif
        if (got) {
            /* 通道值合理性检查，防止解析错位 */
            bool sane = true;
            for (int c = 0; c < 4; c++) {
                if (tmp[c] < 800 || tmp[c] > 2200) { sane = false; break; }
            }
            if (!sane) { s_rc.frames_bad++; continue; }

            for (int c = 0; c < RC_MAX_CH; c++) {
                uint16_t v = tmp[c];
                if (v < 1000) v = 1000;
                if (v > 2000) v = 2000;
                s_rc.ch[c] = v;
            }
            s_rc.last_frame_ms = HAL_GetTick();
            s_rc.frames_ok++;
            s_rc.new_frame = true;
        }
    }
}

bool rc_is_failsafe(void)
{
    if (s_rc.frames_ok == 0) return true;   /* 从来没收到过 */
    return (HAL_GetTick() - s_rc.last_frame_ms) > RC_TIMEOUT_MS;
}

uint16_t rc_channel_us(uint8_t idx)
{
    if (idx >= RC_MAX_CH) return 1500;
    return s_rc.ch[idx];
}

static float apply_expo(float x, float expo)
{
    /* 常见的 expo 曲线：中位附近变柔，满舵保持 ±1 */
    return x * ((1.0f - expo) + expo * x * x);
}

float rc_stick(uint8_t idx)
{
    int32_t v = (int32_t)rc_channel_us(idx) - 1500;
    if (v > -STICK_DEADBAND_US && v < STICK_DEADBAND_US) return 0.0f;
    if (v > 0) v -= STICK_DEADBAND_US; else v += STICK_DEADBAND_US;
    float x = (float)v / (500.0f - STICK_DEADBAND_US);
    if (x > 1.0f) x = 1.0f;
    if (x < -1.0f) x = -1.0f;
    return apply_expo(x, STICK_EXPO);
}

float rc_throttle(void)
{
    float t = ((float)rc_channel_us(CH_THROTTLE) - 1000.0f) / 1000.0f;
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    return t;
}

bool rc_switch(uint8_t idx, bool current_state)
{
    uint16_t v = rc_channel_us(idx);
    if (current_state) {
        /* 当前是 ON，只有掉到 OFF 阈值以下才算 OFF */
        return v > SWITCH_OFF_US;
    } else {
        /* 当前是 OFF，只有超过 ON 阈值才算 ON */
        return v > SWITCH_ON_US;
    }
}

const rc_state_t *rc_get_state(void)
{
    return &s_rc;
}
