#pragma once
#include <Arduino.h>

// =========================
// 引脚配置
// 改成你的实际接线
// =========================
#ifndef LOADCELL_DOUT_PIN
#define LOADCELL_DOUT_PIN D2
#endif

#ifndef LOADCELL_SCK_PIN
#define LOADCELL_SCK_PIN D3
#endif

// =========================
// 基本接口
// =========================
void  loadcell_init();

bool  loadcell_is_ready();
bool  loadcell_wait_ready(uint32_t timeout_ms = 100);

long  loadcell_read_raw();
long  loadcell_read_raw_average(uint8_t samples = 1);

// =========================
// 去皮 / 标定 / 力值读取
// =========================
bool  loadcell_tare(uint8_t samples = 10);

long  loadcell_get_offset();
void  loadcell_set_offset(long offset);

bool  loadcell_set_calibration(float counts_per_N);
float loadcell_get_calibration();

float loadcell_read_force_N(uint8_t samples = 1);

// =========================
// 电源控制
// =========================
void  loadcell_power_down();
void  loadcell_power_up();