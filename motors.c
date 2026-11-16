#include "motors.h"

static TIM_HandleTypeDef *s_tim;
static const uint32_t s_channels[4] = {
    TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4
};

/* 定时器输入时钟：F4 上当 APB 预分频 != 1 时，定时器时钟 = 2 x PCLK */
static uint32_t timer_clock_hz(void)
{
#if MOTOR_TIM_ON_APB2
    uint32_t clk = HAL_RCC_GetPCLK2Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE2) != RCC_CFGR_PPRE2_DIV1) clk *= 2;
#else
    uint32_t clk = HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1) clk *= 2;
#endif
    return clk;
}

void motors_init(TIM_HandleTypeDef *htim)
{
    s_tim = htim;

    /* 让计数器 1 tick = 1us，周期 = 1e6 / PWM 频率 */
    uint32_t psc = timer_clock_hz() / 1000000UL - 1;
    uint32_t arr = 1000000UL / MOTOR_PWM_HZ - 1;

    __HAL_TIM_SET_PRESCALER(s_tim, psc);
    __HAL_TIM_SET_AUTORELOAD(s_tim, arr);
    /* 产生一次更新事件让新的 PSC/ARR 立即生效 */
    s_tim->Instance->EGR = TIM_EGR_UG;

    for (int i = 0; i < 4; i++) {
        __HAL_TIM_SET_COMPARE(s_tim, s_channels[i], MOTOR_MIN_US);
        HAL_TIM_PWM_Start(s_tim, s_channels[i]);
    }
}

void motors_write_us(const uint16_t us[4])
{
    for (int i = 0; i < 4; i++) {
        uint16_t v = us[i];
        if (v < MOTOR_MIN_US) v = MOTOR_MIN_US;
        if (v > MOTOR_MAX_US) v = MOTOR_MAX_US;
        __HAL_TIM_SET_COMPARE(s_tim, s_channels[i], v);
    }
}

void motors_stop(void)
{
    uint16_t off[4] = { MOTOR_MIN_US, MOTOR_MIN_US, MOTOR_MIN_US, MOTOR_MIN_US };
    motors_write_us(off);
}
