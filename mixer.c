#include "mixer.h"
#include "config.h"

/*
 * 混控表 {roll, pitch, yaw}，电机顺序见 config.h 的图：
 *   1 右后 (顺时针)   2 右前 (逆时针)   3 左后 (逆时针)   4 左前 (顺时针)
 *
 * Roll  正 = 向右滚 -> 右侧电机减速(-)，左侧加速(+)
 * Pitch 正 = 机头抬起 -> 前面电机加速(+)，后面减速(-)
 * Yaw   正 = 顺时针转 -> 逆时针转的电机加速(+)（反扭矩让机身顺时针转）
 */
static const float MIX[4][3] = {
    /*  roll   pitch   yaw   */
    { -1.0f, -1.0f, -1.0f },   /* M1 右后 CW  */
    { -1.0f, +1.0f, +1.0f },   /* M2 右前 CCW */
    { +1.0f, -1.0f, +1.0f },   /* M3 左后 CCW */
    { +1.0f, +1.0f, -1.0f },   /* M4 左前 CW  */
};

void mixer_run(float throttle, float roll, float pitch, float yaw, uint16_t out_us[4])
{
    float m[4];
    float mmin = 1e9f, mmax = -1e9f;

    yaw *= MIXER_YAW_SIGN;

    for (int i = 0; i < 4; i++) {
        m[i] = throttle + MIX[i][0] * roll + MIX[i][1] * pitch + MIX[i][2] * yaw;
        if (m[i] < mmin) mmin = m[i];
        if (m[i] > mmax) mmax = m[i];
    }

    /*
     * 饱和处理：整体平移，优先保住姿态控制（电机之间的差值）而不是绝对油门。
     * 满油门时整体下移，怠速时整体上移。
     */
    if (mmax > 1.0f) {
        float shift = mmax - 1.0f;
        for (int i = 0; i < 4; i++) m[i] -= shift;
    } else if (mmin < 0.0f) {
        float shift = -mmin;
        for (int i = 0; i < 4; i++) m[i] += shift;
    }

    float span = (float)(MOTOR_MAX_US - MOTOR_IDLE_US);
    for (int i = 0; i < 4; i++) {
        float v = m[i];
        if (v < 0.0f) v = 0.0f;
        if (v > 1.0f) v = 1.0f;
        out_us[i] = (uint16_t)(MOTOR_IDLE_US + v * span + 0.5f);
    }
}
