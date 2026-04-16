#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include "loadcell.h"

// =========================
// 可调参数
// =========================
static float    g_filter_alpha = 0.20f;      // 低通滤波系数 0~1，越小越平滑
static uint32_t g_output_period_ms = 100;    // 输出周期
static uint8_t  g_avg_samples = 5;           // 每次平均采样点数
static uint8_t  g_tare_samples = 15;         // 去皮采样点数
static float    g_deadband_N = 0.03f;        // 死区，小于该值直接置 0
static float    g_calibration_counts_per_N = 10000.0f; // 标定值，占位
static bool     g_auto_tare_on_startup = true;

// =========================
// 运行时变量
// =========================
static float    g_user_zero_N = 0.0f;        // 软件零点，装机后用来消除预紧力
static float    g_filtered_force_N = 0.0f;
static float    g_last_valid_force_N = 0.0f;
static bool     g_filter_initialized = false;
static uint32_t g_last_output_ms = 0;

// 串口命令缓冲
static char     g_cmd_buf[64];
static uint8_t  g_cmd_len = 0;

// =========================
// 工具函数
// =========================
static float clamp_float(float x, float xmin, float xmax)
{
    if (x < xmin) return xmin;
    if (x > xmax) return xmax;
    return x;
}

static void reset_filter()
{
    g_filter_initialized = false;
    g_filtered_force_N = 0.0f;
}

static float low_pass_filter(float x)
{
    g_filter_alpha = clamp_float(g_filter_alpha, 0.0f, 1.0f);

    if (!g_filter_initialized) {
        g_filtered_force_N = x;
        g_filter_initialized = true;
    } else {
        g_filtered_force_N =
            g_filter_alpha * x +
            (1.0f - g_filter_alpha) * g_filtered_force_N;
    }

    return g_filtered_force_N;
}

// 只输出单曲线，适配 VS Code serial-plotter
// 格式：>force_N:数值
static void publish_force_plot(float force_N)
{
    Serial.print(">force_N:");
    Serial.println(force_N, 4);
}

// 从 HX711 读一次平均值，并换算为未软件置零前的力
static bool read_force_once(long &raw_adc, float &force_N_unzeroed)
{
    if (!loadcell_is_ready()) {
        return false;
    }

    raw_adc = loadcell_read_raw_average(g_avg_samples);

    if (g_calibration_counts_per_N == 0.0f) {
        return false;
    }

    long offset = loadcell_get_offset();
    force_N_unzeroed = ((float)raw_adc - (float)offset) / g_calibration_counts_per_N;
    return true;
}

static void print_help()
{
    Serial.println();
    Serial.println("# Commands:");
    Serial.println("# h           : show help");
    Serial.println("# s           : show status");
    Serial.println("# t           : hardware tare, sensor must be unloaded");
    Serial.println("# z           : software zero, cancel current preload");
    Serial.println("# r           : clear software zero");
    Serial.println("# a 0.20      : set filter alpha");
    Serial.println("# d 0.03      : set deadband in N");
    Serial.println("# p 100       : set output period in ms");
    Serial.println("# n 5         : set averaging samples");
    Serial.println("# c 10000     : set calibration counts per N");
    Serial.println();
}

static void print_status()
{
    Serial.println();
    Serial.println("# Status:");
    Serial.print("# alpha               = "); Serial.println(g_filter_alpha, 4);
    Serial.print("# deadband_N          = "); Serial.println(g_deadband_N, 4);
    Serial.print("# output_period_ms    = "); Serial.println(g_output_period_ms);
    Serial.print("# avg_samples         = "); Serial.println(g_avg_samples);
    Serial.print("# tare_samples        = "); Serial.println(g_tare_samples);
    Serial.print("# calibration_cnt_per_N = "); Serial.println(g_calibration_counts_per_N, 4);
    Serial.print("# software_zero_N     = "); Serial.println(g_user_zero_N, 6);
    Serial.print("# hx711_offset        = "); Serial.println(loadcell_get_offset());
    Serial.println();
}

static char* ltrim(char *s)
{
    while (*s == ' ' || *s == '\t') {
        s++;
    }
    return s;
}

// =========================
// 命令处理
// =========================
static void handle_command(char *cmd)
{
    cmd = ltrim(cmd);

    if (strlen(cmd) == 0) {
        return;
    }

    if (strcmp(cmd, "h") == 0 || strcmp(cmd, "help") == 0) {
        print_help();
        return;
    }

    if (strcmp(cmd, "s") == 0) {
        print_status();
        return;
    }

    if (strcmp(cmd, "t") == 0) {
        Serial.println("# Tare start, keep sensor unloaded...");
        bool ok = loadcell_tare(g_tare_samples);
        if (ok) {
            g_user_zero_N = 0.0f;
            g_last_valid_force_N = 0.0f;
            reset_filter();
            Serial.print("# Tare OK, new offset = ");
            Serial.println(loadcell_get_offset());
        } else {
            Serial.println("# Tare failed: HX711 not ready");
        }
        return;
    }

    if (strcmp(cmd, "z") == 0) {
        long raw_adc = 0;
        float force_now = 0.0f;

        if (read_force_once(raw_adc, force_now)) {
            g_user_zero_N = force_now;
            g_last_valid_force_N = 0.0f;
            reset_filter();
            Serial.print("# Software zero set, zero_N = ");
            Serial.println(g_user_zero_N, 6);
        } else {
            Serial.println("# Zero failed: HX711 not ready");
        }
        return;
    }

    if (strcmp(cmd, "r") == 0) {
        g_user_zero_N = 0.0f;
        reset_filter();
        Serial.println("# Software zero cleared");
        return;
    }

    float fval = 0.0f;
    int ival = 0;

    if (sscanf(cmd, "a %f", &fval) == 1) {
        g_filter_alpha = clamp_float(fval, 0.0f, 1.0f);
        reset_filter();
        Serial.print("# alpha = ");
        Serial.println(g_filter_alpha, 4);
        return;
    }

    if (sscanf(cmd, "d %f", &fval) == 1) {
        if (fval < 0.0f) fval = 0.0f;
        g_deadband_N = fval;
        Serial.print("# deadband_N = ");
        Serial.println(g_deadband_N, 4);
        return;
    }

    if (sscanf(cmd, "p %d", &ival) == 1) {
        if (ival < 10) ival = 10;
        g_output_period_ms = (uint32_t)ival;
        Serial.print("# output_period_ms = ");
        Serial.println(g_output_period_ms);
        return;
    }

    if (sscanf(cmd, "n %d", &ival) == 1) {
        if (ival < 1) ival = 1;
        if (ival > 32) ival = 32;
        g_avg_samples = (uint8_t)ival;
        Serial.print("# avg_samples = ");
        Serial.println(g_avg_samples);
        return;
    }

    if (sscanf(cmd, "c %f", &fval) == 1) {
        if (fval <= 0.0f) {
            Serial.println("# Calibration must be > 0");
            return;
        }

        if (loadcell_set_calibration(fval)) {
            g_calibration_counts_per_N = fval;
            reset_filter();
            Serial.print("# calibration_counts_per_N = ");
            Serial.println(g_calibration_counts_per_N, 4);
        } else {
            Serial.println("# Calibration set failed");
        }
        return;
    }

    Serial.println("# Unknown command, type h for help");
}

static void service_serial_commands()
{
    while (Serial.available() > 0) {
        char ch = (char)Serial.read();

        if (ch == '\r') {
            continue;
        }

        if (ch == '\n') {
            g_cmd_buf[g_cmd_len] = '\0';
            handle_command(g_cmd_buf);
            g_cmd_len = 0;
        } else {
            if (g_cmd_len < sizeof(g_cmd_buf) - 1) {
                g_cmd_buf[g_cmd_len++] = ch;
            }
        }
    }
}

// =========================
// setup
// =========================
void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("=== Nucleo-G431RB + HX711 + TAL220B ===");
    Serial.println("# System start");

    loadcell_init();

    if (loadcell_set_calibration(g_calibration_counts_per_N)) {
        Serial.println("# Calibration factor set OK");
    } else {
        Serial.println("# Calibration factor set failed");
    }

    if (g_auto_tare_on_startup) {
        Serial.println("# Auto tare start, keep sensor unloaded...");
        bool tare_ok = loadcell_tare(g_tare_samples);

        if (tare_ok) {
            Serial.print("# Auto tare OK, offset = ");
            Serial.println(loadcell_get_offset());
        } else {
            Serial.println("# Auto tare failed: HX711 not ready");
        }
    }

    print_help();
    print_status();
    Serial.println("# Plot started");
}

// =========================
// loop
// =========================
void loop()
{
    service_serial_commands();

    uint32_t now = millis();
    if ((now - g_last_output_ms) < g_output_period_ms) {
        return;
    }
    g_last_output_ms = now;

    long raw_adc = 0;
    float force_unzeroed = 0.0f;

    if (!read_force_once(raw_adc, force_unzeroed)) {
        // 未就绪时保持上一有效值，避免掉到 0 形成锯齿
        publish_force_plot(g_last_valid_force_N);
        return;
    }

    // 软件置零，适合装机后消除预紧力
    float force_zeroed = force_unzeroed - g_user_zero_N;

    // 低通滤波
    float force_filtered = low_pass_filter(force_zeroed);

    // 死区，小扰动直接归零
    if (fabsf(force_filtered) < g_deadband_N) {
        force_filtered = 0.0f;
    }

    g_last_valid_force_N = force_filtered;
    publish_force_plot(force_filtered);
}