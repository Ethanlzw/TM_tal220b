#pragma once
#include <Arduino.h>

struct TensionData
{
    long raw_adc;               // HX711 原始 ADC
    float raw_force_N;          // 原始力值，已减硬件 offset，未软件置零，未滤波
    float force_N;              // 最终输出力值，已软件置零，已滤波，已死区处理
    bool valid;                 // 当前数据是否有效
    uint32_t timestamp_ms;      // 最近一次成功更新时间
};

void tension_init();
bool tension_update();          // 高频调用；有新样本时返回 true

bool tension_is_valid();
float tension_get_force_N();
float tension_get_raw_force_N();
long  tension_get_raw_adc();
uint32_t tension_get_timestamp_ms();
TensionData tension_get_data();

bool tension_hardware_tare(uint8_t samples);   // 空载时使用
bool tension_software_zero();                  // 装机后消除当前预紧力
void tension_clear_software_zero();

bool tension_set_calibration(float counts_per_N);
float tension_get_calibration();

void tension_set_filter_alpha(float alpha);    // 0~1，越大响应越快
float tension_get_filter_alpha();

void tension_set_deadband(float deadband_N);   // 小于死区直接归零
float tension_get_deadband();

void tension_set_average_samples(uint8_t n);   // 建议 1~4 用于快速响应
uint8_t tension_get_average_samples();

float tension_get_software_zero_N();
void tension_reset_filter();