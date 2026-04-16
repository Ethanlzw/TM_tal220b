#include <Arduino.h>
#include <math.h>
#include "loadcell.h"

// =========================
// 用户参数
// =========================

// 一阶低通滤波参数，范围 0~1
// 数值越小越平滑
static const float FILTER_ALPHA = 0.20f;

// 输出周期
static const uint32_t OUTPUT_PERIOD_MS = 100;

// 是否在启动时自动去皮
static const bool AUTO_TARE_ON_STARTUP = true;

// 这里填你后面标定出来的值
// 当前先用占位值让程序跑起来
static const float INITIAL_CALIBRATION_COUNTS_PER_N = 10000.0f;

// =========================
// 运行时变量
// =========================
static float g_filtered_force_N = 0.0f;
static bool g_filter_initialized = false;
static uint32_t g_last_output_ms = 0;

// =========================
// 一阶低通滤波
// =========================
float low_pass_filter(float x)
{
    if (!g_filter_initialized) {
        g_filtered_force_N = x;
        g_filter_initialized = true;
    } else {
        g_filtered_force_N =
            FILTER_ALPHA * x +
            (1.0f - FILTER_ALPHA) * g_filtered_force_N;
    }

    return g_filtered_force_N;
}

// =========================
// VS Code serial-plotter 输出
// 格式要求：
// >name:value,name2:value,name3:value
// =========================
/*
void publish_force_plot(uint32_t time_ms, long raw_adc, float force_N, bool valid)
{
    (void)time_ms;  // 当前绘图不单独输出时间轴，插件会按采样顺序画图

    Serial.print(">");

    Serial.print("raw_adc:");
    Serial.print(raw_adc);

    Serial.print(",");

    Serial.print("force_N:");
    if (valid) {
        Serial.print(force_N, 3);
    } else {
        Serial.print(0.0f, 3);
    }

    Serial.print(",");

    Serial.print("valid:");
    Serial.println(valid ? 1 : 0);
}
*/
void publish_force_plot(uint32_t time_ms, long raw_adc, float force_N, bool valid)
{
    (void)time_ms;
    (void)raw_adc;

    Serial.print(">");

    Serial.print("force_N:");
    if (valid) {
        Serial.println(force_N, 3);
    } else {
        Serial.println(0.0f, 3);
    }
}
// =========================
// 初始化
// =========================
void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("=== Nucleo-G431RB + HX711 + TAL220B ===");
    Serial.println("System start");

    loadcell_init();

    if (!loadcell_set_calibration(INITIAL_CALIBRATION_COUNTS_PER_N)) {
        Serial.println("Calibration factor set failed");
    } else {
        Serial.println("Calibration factor set OK");
    }

    if (AUTO_TARE_ON_STARTUP) {
        Serial.println("Tare start, keep sensor unloaded...");
        bool tare_ok = loadcell_tare(15);

        if (tare_ok) {
            Serial.println("Tare success");
            Serial.print("Offset = ");
            Serial.println(loadcell_get_offset());
        } else {
            Serial.println("Tare failed: HX711 not ready");
        }
    }

    Serial.println("Plot output started");
    Serial.println("Format: >raw_adc:xxx,force_N:xxx,valid:x");
}

// =========================
// 主循环
// =========================
void loop()
{
    uint32_t now = millis();

    if ((now - g_last_output_ms) >= OUTPUT_PERIOD_MS) {
        g_last_output_ms = now;

        bool valid = loadcell_is_ready();

        if (!valid) {
            publish_force_plot(now, 0, NAN, false);
            return;
        }

        long raw_adc = loadcell_read_raw_average(5);
        float force_N = loadcell_read_force_N(5);

        if (isnan(force_N)) {
            publish_force_plot(now, raw_adc, NAN, false);
            return;
        }

        force_N = low_pass_filter(force_N);

        publish_force_plot(now, raw_adc, force_N, true);
    }
}