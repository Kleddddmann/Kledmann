/*
 * motors.h - 4 路标准 PWM 电调输出（1000-2000us，400Hz）
 * 用一个定时器的 CH1..CH4，对应电机 1..4
 */
#ifndef FC_MOTORS_H
#define FC_MOTORS_H

#include <stdint.h>
#include "config.h"

void motors_init(TIM_HandleTypeDef *htim);
void motors_write_us(const uint16_t us[4]);
void motors_stop(void);                 /* 全部输出 MOTOR_MIN_US */

#endif /* FC_MOTORS_H */
