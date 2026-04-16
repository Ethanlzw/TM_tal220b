#include "loadcell.h"

// =========================
// 根据你的接线修改
// Nucleo-G431RB 上先用 D2 / D3 最方便
// HX711 DT  -> D2
// HX711 SCK -> D3
// =========================
const int LOADCELL_DOUT_PIN = D2;
const int LOADCELL_SCK_PIN  = D3;

// HX711对象
static HX711 scale;

// 校准因子：counts per N
// 这是临时占位值，后面必须根据你的实物标定修改
float calibration_factor_counts_per_N = 10000.0f;

// 保存offset，便于调试输出
static long g_offset = 0;

// 初始化
void loadcell_init()
{
    scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

    // 先设置比例因子
    // get_units() = get_value() / SCALE
    // 如果SCALE = counts_per_N，那么get_units()输出就是N
    scale.set_scale(calibration_factor_counts_per_N);
}

// 去皮
bool loadcell_tare(uint8_t times)
{
    if (!scale.wait_ready_timeout(1000)) {
        return false;
    }

    scale.tare(times);
    g_offset = scale.get_offset();
    return true;
}

// 设置校准因子
bool loadcell_set_calibration(float factor_counts_per_N)
{
    if (factor_counts_per_N == 0.0f) {
        return false;
    }

    calibration_factor_counts_per_N = factor_counts_per_N;
    scale.set_scale(calibration_factor_counts_per_N);
    return true;
}

// 是否就绪
bool loadcell_is_ready()
{
    return scale.is_ready();
}

// 读取原始ADC平均值
long loadcell_read_raw_average(uint8_t times)
{
    if (!scale.wait_ready_timeout(1000)) {
        return 0;
    }

    return scale.read_average(times);
}

// 读取张力，单位N
float loadcell_read_force_N(uint8_t times)
{
    if (!scale.wait_ready_timeout(1000)) {
        return NAN;
    }

    float force_N = scale.get_units(times);

    // 张力一般不允许负值，直接截断到0
    if (force_N < 0.0f) {
        force_N = 0.0f;
    }

    return force_N;
}

// 获取offset
long loadcell_get_offset()
{
    return g_offset;
}

// 获取校准因子
float loadcell_get_calibration()
{
    return calibration_factor_counts_per_N;
}