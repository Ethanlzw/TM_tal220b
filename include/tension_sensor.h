#pragma once
#include <Arduino.h>

#ifndef TENSION_DOUT_PIN
#define TENSION_DOUT_PIN D3
#endif

#ifndef TENSION_SCK_PIN
#define TENSION_SCK_PIN D2
#endif

struct TensionData {
    long raw_adc;
    float force_raw_loadcell_N;
    float force_filtered_loadcell_N;
    float tether_tension_N;
    bool valid;
    bool saturated;
    uint32_t timestamp_ms;
};

void tension_init();
bool tension_tare(uint8_t samples = 16);
bool tension_update();

bool tension_is_valid();
float tension_get_tether_N();
float tension_get_force_N();
TensionData tension_get_data();
