#pragma once

#include <Arduino.h>
#include <stdint.h>

// NUCLEO-G431RB Arduino-header mapping:
//   D3 -> PB3  HX711 DOUT
//   D2 -> PA10 HX711 SCK
#ifndef TENSION_DOUT_PIN
#define TENSION_DOUT_PIN D3
#endif

#ifndef TENSION_SCK_PIN
#define TENSION_SCK_PIN D2
#endif

struct TensionData {
    uint32_t sample_id;
    uint32_t timestamp_ms;

    int32_t raw_adc;
    float tare_offset_counts;
    float temp_offset_counts;
    float tracking_offset_counts;
    float zero_offset_counts;
    float net_counts;

    float counts_per_N_used;
    float raw_force_N;
    float filtered_force_N;
    float force_N;

    float temperature_C;

    bool valid;
    bool saturated;
    bool tared;
    bool temp_comp_enabled;
    bool zero_tracking_enabled;
    bool zero_tracking_applied;
};

void tension_init();
bool tension_tare(uint8_t samples = 64);
bool tension_update();

TensionData tension_get_data();
bool tension_is_valid();
float tension_get_force_N();

bool tension_set_calibration(float counts_per_N);
void tension_set_direction(int8_t dir);
void tension_set_filter_alpha(float alpha);
void tension_set_deadband(float deadband_N);
void tension_set_temperature(float temperature_C);

void tension_set_temperature_compensation(float ref_temperature_C,
                                          float zero_coeff_counts_per_C,
                                          float span_coeff_per_C);
void tension_enable_temperature_compensation(bool enable);

void tension_set_zero_tracking(float beta, float limit_N);
void tension_enable_zero_tracking(bool enable);
void tension_reset_zero_tracking();

float tension_get_calibration();
int8_t tension_get_direction();
float tension_get_filter_alpha();
float tension_get_deadband();
float tension_get_tare_offset_counts();
float tension_get_tracking_offset_counts();
bool tension_is_tared();
