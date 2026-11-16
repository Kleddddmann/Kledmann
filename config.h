/*
 * config.h - 飞控所有可调参数集中在这里
 *
 * 坐标系约定（机体系，航空 NED 标准）：
 *   X 轴 = 机头方向   Roll  正 = 右侧下沉（向右滚转）
 *   Y 轴 = 机身右侧   Pitch 正 = 机头抬起
 *   Z 轴 = 向下       Yaw   正 = 从上往下看顺时针（机头向右转）
 */
#ifndef FC_CONFIG_H
#define FC_CONFIG_H

#include "stm32f4xx_hal.h"

/* ============================================================
 * 控制环
 * ============================================================ */
#define LOOP_HZ                 500          /* 主循环频率。MPU6050 走 I2C，500Hz 比较稳 */
#define LOOP_DT                 (1.0f / (float)LOOP_HZ)

/* ============================================================
 * 接收机协议（二选一）
 * ============================================================ */
#define RC_PROTOCOL_CRSF        1            /* ELRS / Crossfire, 420000 波特率 */
#define RC_PROTOCOL_IBUS        2            /* 富斯 FlySky iBUS, 115200 波特率 */
#define RC_PROTOCOL             RC_PROTOCOL_CRSF

#define RC_MAX_CH               16
#define RC_TIMEOUT_MS           500          /* 超过这个时间没收到有效帧 -> 失控保护 */

/* 通道映射，0-based，默认 AETR 顺序（大部分遥控器默认） */
#define CH_ROLL                 0            /* A: 副翼 */
#define CH_PITCH                1            /* E: 升降 */
#define CH_THROTTLE             2            /* T: 油门 */
#define CH_YAW                  3            /* R: 方向 */
#define CH_ARM                  4            /* AUX1: 解锁开关 */
#define CH_LEVEL                5            /* AUX2: 自动平衡（自稳）开关 <- 你要的那个开关 */

/* 开关判定带迟滞，避免在阈值附近抖动 */
#define SWITCH_ON_US            1700
#define SWITCH_OFF_US           1300

#define STICK_DEADBAND_US       15           /* 摇杆中位死区 */
#define STICK_EXPO              0.0f         /* 0 = 线性，0.3 左右手感更柔和 */

/* ============================================================
 * 电机 / 电调（标准 PWM 1000-2000us）
 * ============================================================ */
#define MOTOR_PWM_HZ            400          /* 400Hz 是标准 PWM 电调的上限，老电调用 50 */
#define MOTOR_MIN_US            1000         /* 停转 */
#define MOTOR_IDLE_US           1080         /* 解锁后怠速，太低电机会停转失去控制 */
#define MOTOR_MAX_US            2000
#define MOTOR_TIM_ON_APB2       0            /* TIM1/TIM8/TIM9-11 在 APB2 时设为 1；TIM2-5/12-14 设为 0 */

/* 电机顺序（Betaflight 标准 X 布局，从上往下看）：
 *      4(FL,顺时针)   2(FR,逆时针)
 *               \   /
 *                \ /
 *                / \
 *               /   \
 *      3(RL,逆时针)   1(RR,顺时针)
 * 如果你的桨是反过来装的（props out），把 MIXER_YAW_SIGN 改成 -1
 */
#define MIXER_YAW_SIGN          1.0f

/* ============================================================
 * IMU 安装方向：把传感器坐标系映射到机体系
 * 默认：MPU6050 平放，芯片丝印 X 箭头指向机头，芯片朝上
 *       传感器 X=前, Y=左, Z=上  ->  机体 X=前, Y=右, Z=下
 * 上电后自己验证（看 README 的检查步骤），不对就改这里的符号/轴
 * ============================================================ */
#define IMU_BODY_X(x, y, z)     ( (x))
#define IMU_BODY_Y(x, y, z)     (-(y))
#define IMU_BODY_Z(x, y, z)     (-(z))

/* MPU6050 量程 */
#define IMU_GYRO_RANGE_DPS      2000         /* 250 / 500 / 1000 / 2000 */
#define IMU_ACCEL_RANGE_G       8            /* 2 / 4 / 8 / 16 */
#define IMU_DLPF_CFG            2            /* 0=260Hz 1=184 2=94 3=44 4=21 5=10 6=5 (Hz) */
#define IMU_CAL_SAMPLES         1000         /* 上电零偏校准采样数，期间飞机必须静止 */
#define IMU_CAL_MAX_GYRO_DPS    5.0f         /* 校准时陀螺读数超过这个就认为在动，重来 */

/* 姿态互补滤波：时间常数越大越信陀螺，越小越信加速度计 */
#define ATT_ACC_TAU_S           1.5f
#define ATT_ACC_MIN_G           0.75f        /* 加速度模长不在这个范围内就不融合（飞机在加减速） */
#define ATT_ACC_MAX_G           1.25f

/* ============================================================
 * 飞行限制
 * ============================================================ */
#define MAX_ANGLE_DEG           35.0f        /* 自稳模式下摇杆满舵对应的最大倾角 */
#define MAX_RATE_DPS            300.0f       /* 速率模式下摇杆满舵对应的最大角速度，也是自稳模式外环输出上限 */
#define MAX_YAW_RATE_DPS        200.0f
#define ARM_THROTTLE_MAX        0.05f        /* 油门高于 5% 不允许解锁 */
#define IDLE_THROTTLE           0.05f        /* 油门低于这个值时电机怠速、积分清零 */

/* ============================================================
 * PID 默认参数 —— 一定要按你的机架重新调！见 README 调参章节
 *   内环（角速度环）：输入 deg/s 误差，输出 -1..1 的归一化力矩
 *   外环（角度环）  ：输入 deg 误差，输出 deg/s 的角速度目标
 * ============================================================ */
#define PID_RATE_ROLL_P         0.0025f
#define PID_RATE_ROLL_I         0.0040f
#define PID_RATE_ROLL_D         0.00006f

#define PID_RATE_PITCH_P        0.0025f
#define PID_RATE_PITCH_I        0.0040f
#define PID_RATE_PITCH_D        0.00006f

#define PID_RATE_YAW_P          0.0040f
#define PID_RATE_YAW_I          0.0100f
#define PID_RATE_YAW_D          0.0f

#define PID_RATE_I_LIMIT        0.30f        /* 积分项上限（归一化力矩） */
#define PID_RATE_OUT_LIMIT      0.50f        /* 单轴最大力矩，留一半给油门 */
#define PID_RATE_YAW_OUT_LIMIT  0.30f
#define PID_RATE_D_CUTOFF_HZ    80.0f        /* D 项低通，抑制陀螺噪声 */

#define PID_ANGLE_P             5.0f         /* 1° 误差 -> 5 deg/s 修正 */
#define PID_ANGLE_I             0.0f
#define PID_ANGLE_D             0.0f

/* ============================================================
 * 调试串口输出（0 关闭）
 * ============================================================ */
#define DEBUG_PRINT_HZ          10

#endif /* FC_CONFIG_H */
