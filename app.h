/*
 * app.h - 应用入口：把 IMU / 接收机 / 电机 / 飞控逻辑串起来
 *
 * 在 CubeMX 生成的 main.c 里：
 *   1. 所有 MX_xxx_Init() 之后调用 app_init(...)
 *   2. while(1) 里调用 app_run()（不会返回）
 *   3. 在 HAL_UARTEx_RxEventCallback 里调用 app_uart_rx_event(...)
 */
#ifndef FC_APP_H
#define FC_APP_H

#include <stdint.h>
#include "config.h"
#include "flight.h"

typedef struct {
    I2C_HandleTypeDef  *imu_i2c;      /* MPU6050 所在 I2C */
    UART_HandleTypeDef *rc_uart;      /* 接收机 UART（需要 DMA RX） */
    TIM_HandleTypeDef  *motor_tim;    /* 电机 PWM 定时器，CH1..CH4 */
    UART_HandleTypeDef *debug_uart;   /* 调试串口，可以为 NULL */
    GPIO_TypeDef       *led_port;     /* 状态 LED，可以为 NULL */
    uint16_t            led_pin;
    uint8_t             led_active_low;
} app_hw_t;

void app_init(const app_hw_t *hw);
void app_run(void);
void app_uart_rx_event(UART_HandleTypeDef *huart, uint16_t size);   /* HAL_UARTEx_RxEventCallback 里调用 */
void app_uart_rx_error(UART_HandleTypeDef *huart);                  /* HAL_UART_ErrorCallback 里调用 */

const fc_state_t *app_get_fc(void);

#endif /* FC_APP_H */
