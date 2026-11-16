/*
 * rc_ibus.c - 富斯 FlySky iBUS 协议解析
 *
 * 帧长 32 字节：[0x20][0x40][ch1 低][ch1 高]...[ch14 低][ch14 高][校验低][校验高]
 *   通道值直接就是 1000..2000
 *   校验 = 0xFFFF - 前 30 字节之和
 * 波特率 115200，8N1
 */
#include "rc.h"

#define IBUS_FRAME_LEN 32

bool rc_ibus_feed_byte(uint8_t b, uint16_t ch_us[RC_MAX_CH])
{
    static uint8_t buf[IBUS_FRAME_LEN];
    static uint8_t pos = 0;

    if (pos == 0) {
        if (b == 0x20) buf[pos++] = b;
        return false;
    }
    if (pos == 1) {
        if (b != 0x40) { pos = 0; return false; }
        buf[pos++] = b;
        return false;
    }

    buf[pos++] = b;
    if (pos < IBUS_FRAME_LEN) return false;
    pos = 0;

    uint16_t sum = 0xFFFF;
    for (int i = 0; i < IBUS_FRAME_LEN - 2; i++) sum -= buf[i];
    uint16_t rx_sum = (uint16_t)(buf[30] | (buf[31] << 8));
    if (sum != rx_sum) return false;

    for (int c = 0; c < 14; c++) {
        ch_us[c] = (uint16_t)(buf[2 + c * 2] | (buf[3 + c * 2] << 8));
    }
    ch_us[14] = 1500;
    ch_us[15] = 1500;
    return true;
}
