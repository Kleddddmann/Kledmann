#include "app.h"
#include "attitude.h"
#include "motors.h"
#include "mpu6050.h"
#include "rc.h"
#include <stdio.h>
#include <string.h>

static app_hw_t   s_hw;
static fc_state_t s_fc;

/* 接收机 UART DMA 环形缓冲 */
#define RC_DMA_BUF_LEN 128
static uint8_t  s_rc_dma_buf[RC_DMA_BUF_LEN];
static uint16_t s_rc_dma_old_pos = 0;

/* 调试串口发送缓冲 */
static char s_dbg_buf[160];

/* ---------- 周期级计时：用 DWT 周期计数器，比 SysTick 的 1ms 精细得多 ---------- */
static void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static inline uint32_t cycles_now(void)
{
    return DWT->CYCCNT;
}

/* ---------- LED ---------- */
static void led_set(uint8_t on)
{
    if (!s_hw.led_port) return;
    GPIO_PinState st = (on ^ s_hw.led_active_low) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(s_hw.led_port, s_hw.led_pin, st);
}

/*
 * LED 状态：
 *   校准中     : 快速双闪
 *   上锁/待机  : 1Hz 慢闪
 *   解锁-速率  : 常亮
 *   解锁-自稳  : 常亮，每秒短暂熄一下
 *   失控保护   : 5Hz 快闪
 */
static void led_update(void)
{
    uint32_t t = HAL_GetTick() % 1000;
    uint8_t on;
    switch (s_fc.arm_state) {
    case FC_ARMED:
        on = (s_fc.mode == FC_MODE_LEVEL) ? (t > 100) : 1;
        break;
    case FC_FAILSAFE:
        on = (t % 200) < 100;
        break;
    default:
        on = t < 100;
        break;
    }
    led_set(on);
}

/* ---------- 调试输出 ---------- */
static void debug_print(void)
{
#if DEBUG_PRINT_HZ > 0
    if (!s_hw.debug_uart) return;
    if (HAL_UART_GetState(s_hw.debug_uart) != HAL_UART_STATE_READY &&
        HAL_UART_GetState(s_hw.debug_uart) != HAL_UART_STATE_BUSY_RX) return;

    const char *arm = s_fc.arm_state == FC_ARMED ? "ARM" :
                      s_fc.arm_state == FC_FAILSAFE ? "FS " : "DIS";
    const char *mode = s_fc.mode == FC_MODE_LEVEL ? "LEVEL" : "RATE ";
    const rc_state_t *rc = rc_get_state();

    int n = snprintf(s_dbg_buf, sizeof(s_dbg_buf),
        "%s %s r=%6.1f p=%6.1f y=%6.1f | thr=%4u arm=%4u lvl=%4u | m=%4u %4u %4u %4u | rc=%lu/%lu imuerr=%lu\r\n",
        arm, mode,
        (double)s_fc.att.roll_deg, (double)s_fc.att.pitch_deg, (double)s_fc.att.yaw_deg,
        (unsigned)rc->ch[CH_THROTTLE], (unsigned)rc->ch[CH_ARM], (unsigned)rc->ch[CH_LEVEL],
        (unsigned)s_fc.motor_us[0], (unsigned)s_fc.motor_us[1],
        (unsigned)s_fc.motor_us[2], (unsigned)s_fc.motor_us[3],
        (unsigned long)rc->frames_ok, (unsigned long)rc->frames_bad,
        (unsigned long)s_fc.imu_errors);
    if (n > 0) {
        if (n > (int)sizeof(s_dbg_buf)) n = sizeof(s_dbg_buf);
        HAL_UART_Transmit_IT(s_hw.debug_uart, (uint8_t *)s_dbg_buf, (uint16_t)n);
    }
#endif
}

/* ---------- 接收机 UART：DMA 到 IDLE ---------- */
static void rc_uart_start(void)
{
    s_rc_dma_old_pos = 0;
    HAL_UARTEx_ReceiveToIdle_DMA(s_hw.rc_uart, s_rc_dma_buf, RC_DMA_BUF_LEN);
    /* 不需要半传输中断 */
    __HAL_DMA_DISABLE_IT(s_hw.rc_uart->hdmarx, DMA_IT_HT);
}

void app_uart_rx_event(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart != s_hw.rc_uart) return;

    /* size 是 DMA 当前写到的位置（环形模式下会回绕） */
    if (size != s_rc_dma_old_pos) {
        if (size > s_rc_dma_old_pos) {
            rc_feed(&s_rc_dma_buf[s_rc_dma_old_pos], size - s_rc_dma_old_pos);
        } else {
            rc_feed(&s_rc_dma_buf[s_rc_dma_old_pos], RC_DMA_BUF_LEN - s_rc_dma_old_pos);
            if (size > 0) rc_feed(&s_rc_dma_buf[0], size);
        }
        s_rc_dma_old_pos = size;
    }
    if (s_rc_dma_old_pos >= RC_DMA_BUF_LEN) s_rc_dma_old_pos = 0;

    /* 如果 DMA 不是环形模式（CubeMX 里没选 Circular），需要重新启动 */
    if (huart->hdmarx->Init.Mode != DMA_CIRCULAR) {
        rc_uart_start();
    }
}

/*
 * UART 出错（溢出/帧错误）时 HAL 会中止 DMA 接收，不管是不是环形模式，
 * 所以这里一律清标志并重新启动接收，否则接收机就"静音"了。
 */
void app_uart_rx_error(UART_HandleTypeDef *huart)
{
    if (huart != s_hw.rc_uart) return;
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    HAL_UART_AbortReceive(huart);
    rc_uart_start();
}

/* ---------- 初始化 ---------- */
void app_init(const app_hw_t *hw)
{
    s_hw = *hw;
    dwt_init();

    fc_init(&s_fc);
    rc_init();
    motors_init(s_hw.motor_tim);     /* 先把电调输出拉到最低，电调才会正常上电 */
    motors_stop();

    rc_uart_start();

    /* IMU 初始化，失败就一直快闪 */
    while (!mpu6050_init(s_hw.imu_i2c)) {
        led_set(1); HAL_Delay(50);
        led_set(0); HAL_Delay(50);
    }

    /* 陀螺零偏校准：飞机必须静止，动了就重来 */
    for (;;) {
        /* 双闪表示校准中 */
        led_set(1); HAL_Delay(60); led_set(0); HAL_Delay(60);
        led_set(1); HAL_Delay(60); led_set(0); HAL_Delay(300);
        if (mpu6050_calibrate_gyro()) break;
    }

    /* 用加速度计初始化姿态 */
    imu_data_t imu;
    if (mpu6050_read(&imu)) {
        s_fc.imu = imu;
        fc_set_initial_attitude(&s_fc, &imu);
    }
}

/* ---------- 主循环 ---------- */
void app_run(void)
{
    const uint32_t loop_cycles = (SystemCoreClock / LOOP_HZ);
    uint32_t next_loop = cycles_now();
    uint32_t last_loop = next_loop;
    uint32_t last_debug_ms = 0;

    for (;;) {
        /* 等到下一个周期 */
        while ((int32_t)(cycles_now() - next_loop) < 0) { }
        next_loop += loop_cycles;

        /* 实际 dt（防止某次循环超时导致积分错误） */
        uint32_t now = cycles_now();
        float dt = (float)(now - last_loop) / (float)SystemCoreClock;
        last_loop = now;
        if (dt > 4.0f * LOOP_DT) dt = 4.0f * LOOP_DT;   /* 异常长的间隔截断 */

        /* 读 IMU */
        bool imu_ok = mpu6050_read(&s_fc.imu);

        /* 飞控逻辑 + 电机输出 */
        fc_update(&s_fc, imu_ok, dt);

        /* 低优先级杂务 */
        led_update();
#if DEBUG_PRINT_HZ > 0
        uint32_t ms = HAL_GetTick();
        if (ms - last_debug_ms >= (1000 / DEBUG_PRINT_HZ)) {
            last_debug_ms = ms;
            debug_print();
        }
#endif
    }
}

const fc_state_t *app_get_fc(void)
{
    return &s_fc;
}
