/*
 * rc.h - 接收机输入：CRSF / iBUS 解析 + 摇杆/开关读取 + 失控检测
 *
 * 用法：
 *   在 UART 接收回调里把收到的字节喂给 rc_feed()，
 *   主循环里用 rc_stick / rc_throttle / rc_switch / rc_is_failsafe 读取。
 */
#ifndef FC_RC_H
#define FC_RC_H

#include <stdbool.h>
#include <stdint.h>
#include "config.h"

typedef struct {
    uint16_t ch[RC_MAX_CH];      /* 每个通道 1000..2000 us */
    volatile uint32_t last_frame_ms;
    volatile uint32_t frames_ok;
    volatile uint32_t frames_bad;
    volatile bool     new_frame;
} rc_state_t;

void     rc_init(void);
void     rc_feed(const uint8_t *buf, uint16_t len);   /* 中断/DMA 回调中调用 */

bool     rc_is_failsafe(void);                        /* 超时没收到帧 */
uint16_t rc_channel_us(uint8_t idx);
float    rc_stick(uint8_t idx);                       /* -1..+1，带死区和 expo */
float    rc_throttle(void);                           /* 0..1 */
bool     rc_switch(uint8_t idx, bool current_state);  /* 带迟滞的两段开关 */
const rc_state_t *rc_get_state(void);

/* 协议解析器（内部使用，也可以单独拿去测试）
 * 每喂一个字节，解析出一帧完整通道数据时返回 true 并写入 ch_us[16] */
bool rc_crsf_feed_byte(uint8_t b, uint16_t ch_us[RC_MAX_CH]);
bool rc_ibus_feed_byte(uint8_t b, uint16_t ch_us[RC_MAX_CH]);

#endif /* FC_RC_H */
