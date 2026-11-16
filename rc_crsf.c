/*
 * rc_crsf.c - CRSF (Crossfire / ExpressLRS) 协议解析
 *
 * 帧格式：[地址][长度][类型][载荷...][CRC8]
 *   地址：0xC8 = 飞控，0xEE = 发射模块
 *   长度：类型+载荷+CRC 的字节数
 *   CRC8：多项式 0xD5，覆盖 类型+载荷
 * RC 通道帧类型 0x16：22 字节载荷，16 个 11 位通道，取值 172..1811
 * 波特率 420000，8N1，不需要反相
 */
#include "rc.h"

#define CRSF_ADDR_FC         0xC8
#define CRSF_ADDR_TX         0xEE
#define CRSF_TYPE_RC         0x16
#define CRSF_MAX_FRAME       64

static uint8_t crc8_dvb_s2(uint8_t crc, uint8_t a)
{
    crc ^= a;
    for (int i = 0; i < 8; i++) {
        crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
    }
    return crc;
}

static void decode_channels(const uint8_t *p, uint16_t ch_us[RC_MAX_CH])
{
    uint32_t bits = 0;
    int bitcount = 0;
    int chi = 0;
    for (int i = 0; i < 22 && chi < 16; i++) {
        bits |= (uint32_t)p[i] << bitcount;
        bitcount += 8;
        while (bitcount >= 11 && chi < 16) {
            uint16_t raw = bits & 0x7FF;
            bits >>= 11;
            bitcount -= 11;
            /* 172..1811 -> 988..2012，和 Betaflight 一样的映射 */
            int32_t us = ((int32_t)raw - 992) * 5 / 8 + 1500;
            ch_us[chi++] = (uint16_t)us;
        }
    }
}

bool rc_crsf_feed_byte(uint8_t b, uint16_t ch_us[RC_MAX_CH])
{
    static uint8_t buf[CRSF_MAX_FRAME];
    static uint8_t pos = 0;
    static uint8_t frame_len = 0;   /* 长度字段的值 */

    if (pos == 0) {
        if (b == CRSF_ADDR_FC || b == CRSF_ADDR_TX) {
            buf[pos++] = b;
        }
        return false;
    }
    if (pos == 1) {
        /* 长度必须在 2..62 之间 */
        if (b < 2 || b > CRSF_MAX_FRAME - 2) { pos = 0; return false; }
        frame_len = b;
        buf[pos++] = b;
        return false;
    }

    buf[pos++] = b;
    if (pos < (uint8_t)(frame_len + 2)) return false;   /* 还没收齐 */

    /* 收齐一帧：校验 CRC（从类型字节到载荷末尾） */
    uint8_t crc = 0;
    for (int i = 2; i < frame_len + 1; i++) crc = crc8_dvb_s2(crc, buf[i]);
    bool ok = false;
    if (crc == buf[frame_len + 1]) {
        if (buf[2] == CRSF_TYPE_RC && frame_len == 24) {
            decode_channels(&buf[3], ch_us);
            ok = true;
        }
        /* 其他类型（链路统计 0x14 等）这里直接忽略 */
    }
    pos = 0;
    return ok;
}
