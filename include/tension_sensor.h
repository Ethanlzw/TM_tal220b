#pragma once
#include <Arduino.h>

// =========================
// 引脚配置
// 改成你的实际接线
// =========================
#ifndef TENSION_DOUT_PIN
#define TENSION_DOUT_PIN D2
#endif

#ifndef TENSION_SCK_PIN
#define TENSION_SCK_PIN D3
#endif

struct TensionData
{
    long raw_adc;               // HX711 原始 ADC
    float raw_force_N;          // 已减 offset、已乘方向，未软件置零，未滤波
    float force_N;              // 最终输出，已软件置零、已滤波、已死区处理
    bool valid;                 // 当前数据是否有效
    uint32_t timestamp_ms;      // 最近一次成功更新时间
};

// 初始化与高频更新
void tension_init();
bool tension_update();

// 数据读取
bool tension_is_valid();
float tension_get_force_N();
float tension_get_raw_force_N();
long  tension_get_raw_adc();
uint32_t tension_get_timestamp_ms();
TensionData tension_get_data();

// 去皮与置零
bool tension_hardware_tare(uint8_t samples = 10);
bool tension_software_zero();
void tension_clear_software_zero();

// 参数设置
bool tension_set_calibration(float counts_per_N);
float tension_get_calibration();

void tension_set_filter_alpha(float alpha);
float tension_get_filter_alpha();

void tension_set_deadband(float deadband_N);
float tension_get_deadband();

void tension_set_average_samples(uint8_t n);
uint8_t tension_get_average_samples();

void tension_set_direction(int8_t dir);   // +1 或 -1
int8_t tension_get_direction();

float tension_get_software_zero_N();
long  tension_get_offset();
void  tension_reset_filter();

// 调试绘图
void tension_set_plot_period_ms(uint32_t period_ms);
uint32_t tension_get_plot_period_ms();

void tension_set_plot_enabled(bool enabled);
bool tension_get_plot_enabled();

void tension_plot_update();               // 输出 >force_N:xxx

// 串口命令接口
void tension_serial_process();
void tension_print_help();
void tension_print_status();

// 可选电源控制
void tension_power_down();
void tension_power_up();