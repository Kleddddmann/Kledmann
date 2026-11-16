#include "mpu6050.h"
#include <math.h>

/* ---- 寄存器 ---- */
#define MPU_ADDR            (0x68 << 1)   /* AD0 接地 = 0x68，接 VCC = 0x69 */
#define REG_SMPLRT_DIV      0x19
#define REG_CONFIG          0x1A
#define REG_GYRO_CONFIG     0x1B
#define REG_ACCEL_CONFIG    0x1C
#define REG_ACCEL_XOUT_H    0x3B
#define REG_PWR_MGMT_1      0x6B
#define REG_WHO_AM_I        0x75

#define I2C_TIMEOUT_MS      5

static I2C_HandleTypeDef *s_i2c;
static float s_gyro_scale;     /* LSB -> deg/s */
static float s_accel_scale;    /* LSB -> g */
static float s_gyro_bias[3];   /* 传感器坐标系下的零偏 (LSB) */

static bool write_reg(uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(s_i2c, MPU_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                             &val, 1, I2C_TIMEOUT_MS) == HAL_OK;
}

static bool read_regs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(s_i2c, MPU_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                            buf, len, I2C_TIMEOUT_MS) == HAL_OK;
}

/* 读 14 字节原始数据（传感器坐标系），不做任何换算 */
static bool read_raw(int16_t acc[3], int16_t *temp, int16_t gyr[3])
{
    uint8_t b[14];
    if (!read_regs(REG_ACCEL_XOUT_H, b, 14)) return false;
    acc[0] = (int16_t)((b[0] << 8) | b[1]);
    acc[1] = (int16_t)((b[2] << 8) | b[3]);
    acc[2] = (int16_t)((b[4] << 8) | b[5]);
    *temp  = (int16_t)((b[6] << 8) | b[7]);
    gyr[0] = (int16_t)((b[8] << 8) | b[9]);
    gyr[1] = (int16_t)((b[10] << 8) | b[11]);
    gyr[2] = (int16_t)((b[12] << 8) | b[13]);
    return true;
}

bool mpu6050_init(I2C_HandleTypeDef *hi2c)
{
    s_i2c = hi2c;

    uint8_t who = 0;
    if (!read_regs(REG_WHO_AM_I, &who, 1)) return false;
    if (who != 0x68 && who != 0x72 && who != 0x70) return false; /* 0x68 正品, 0x70/0x72 常见兼容片 */

    /* 复位，然后用 X 轴陀螺作为时钟源（比内部 RC 稳） */
    if (!write_reg(REG_PWR_MGMT_1, 0x80)) return false;
    HAL_Delay(100);
    if (!write_reg(REG_PWR_MGMT_1, 0x01)) return false;
    HAL_Delay(10);

    /* DLPF：启用后陀螺内部采样率 1kHz；SMPLRT_DIV=1 -> 输出 500Hz */
    if (!write_reg(REG_CONFIG, IMU_DLPF_CFG & 0x07)) return false;
    if (!write_reg(REG_SMPLRT_DIV, (uint8_t)((1000 / LOOP_HZ) - 1))) return false;

    /* 陀螺量程 */
    uint8_t gcfg;
    switch (IMU_GYRO_RANGE_DPS) {
        case 250:  gcfg = 0 << 3; s_gyro_scale = 1.0f / 131.0f; break;
        case 500:  gcfg = 1 << 3; s_gyro_scale = 1.0f / 65.5f;  break;
        case 1000: gcfg = 2 << 3; s_gyro_scale = 1.0f / 32.8f;  break;
        default:   gcfg = 3 << 3; s_gyro_scale = 1.0f / 16.4f;  break;
    }
    if (!write_reg(REG_GYRO_CONFIG, gcfg)) return false;

    /* 加速度计量程 */
    uint8_t acfg;
    switch (IMU_ACCEL_RANGE_G) {
        case 2:  acfg = 0 << 3; s_accel_scale = 1.0f / 16384.0f; break;
        case 4:  acfg = 1 << 3; s_accel_scale = 1.0f / 8192.0f;  break;
        case 16: acfg = 3 << 3; s_accel_scale = 1.0f / 2048.0f;  break;
        default: acfg = 2 << 3; s_accel_scale = 1.0f / 4096.0f;  break;
    }
    if (!write_reg(REG_ACCEL_CONFIG, acfg)) return false;

    s_gyro_bias[0] = s_gyro_bias[1] = s_gyro_bias[2] = 0.0f;
    HAL_Delay(50);
    return true;
}

bool mpu6050_calibrate_gyro(void)
{
    int32_t sum[3] = {0, 0, 0};
    int16_t acc[3], gyr[3], temp;
    int16_t gmin[3] = {32767, 32767, 32767};
    int16_t gmax[3] = {-32768, -32768, -32768};

    for (int i = 0; i < IMU_CAL_SAMPLES; i++) {
        if (!read_raw(acc, &temp, gyr)) return false;
        for (int a = 0; a < 3; a++) {
            sum[a] += gyr[a];
            if (gyr[a] < gmin[a]) gmin[a] = gyr[a];
            if (gyr[a] > gmax[a]) gmax[a] = gyr[a];
        }
        HAL_Delay(1000 / LOOP_HZ);
    }

    /* 校准期间如果某轴峰峰值超过阈值，说明飞机在动，拒绝这次校准 */
    float max_pp_lsb = IMU_CAL_MAX_GYRO_DPS / s_gyro_scale;
    for (int a = 0; a < 3; a++) {
        if ((float)(gmax[a] - gmin[a]) > max_pp_lsb) return false;
    }

    for (int a = 0; a < 3; a++) {
        s_gyro_bias[a] = (float)sum[a] / (float)IMU_CAL_SAMPLES;
    }
    return true;
}

bool mpu6050_read(imu_data_t *out)
{
    int16_t acc[3], gyr[3], temp;
    if (!read_raw(acc, &temp, gyr)) return false;

    /* 传感器坐标系下换算单位 */
    float gx = ((float)gyr[0] - s_gyro_bias[0]) * s_gyro_scale;
    float gy = ((float)gyr[1] - s_gyro_bias[1]) * s_gyro_scale;
    float gz = ((float)gyr[2] - s_gyro_bias[2]) * s_gyro_scale;
    float ax = (float)acc[0] * s_accel_scale;
    float ay = (float)acc[1] * s_accel_scale;
    float az = (float)acc[2] * s_accel_scale;

    /* 映射到机体系 */
    out->gyro[0]  = IMU_BODY_X(gx, gy, gz);
    out->gyro[1]  = IMU_BODY_Y(gx, gy, gz);
    out->gyro[2]  = IMU_BODY_Z(gx, gy, gz);
    out->accel[0] = IMU_BODY_X(ax, ay, az);
    out->accel[1] = IMU_BODY_Y(ax, ay, az);
    out->accel[2] = IMU_BODY_Z(ax, ay, az);
    out->temp_c   = (float)temp / 340.0f + 36.53f;
    return true;
}
