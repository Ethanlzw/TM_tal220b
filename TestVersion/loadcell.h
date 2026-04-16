#ifndef LOADCELL_H
#define LOADCELL_H

#include <Arduino.h>
#include <HX711.h>

// 引脚定义
extern const int LOADCELL_DOUT_PIN;
extern const int LOADCELL_SCK_PIN;

// 校准因子
// 含义：counts per Newton
// 也就是：HX711原始计数 / 力(N)
extern float calibration_factor_counts_per_N;

// 对外接口
void loadcell_init();
bool loadcell_tare(uint8_t times = 10);
bool loadcell_set_calibration(float factor_counts_per_N);

// 读取接口
bool loadcell_is_ready();
long loadcell_read_raw_average(uint8_t times = 5);
float loadcell_read_force_N(uint8_t times = 5);

// 调试接口
long loadcell_get_offset();
float loadcell_get_calibration();

#endif