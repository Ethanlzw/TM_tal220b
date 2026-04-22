#pragma once
#include <Arduino.h>

// ─── Pin defaults (override before #include if needed) ───────────────────────
#ifndef TENSION_DOUT_PIN
#define TENSION_DOUT_PIN D3
#endif
#ifndef TENSION_SCK_PIN
#define TENSION_SCK_PIN  D2
#endif

// ─── Data snapshot ───────────────────────────────────────────────────────────
struct TensionData {
    long     raw_adc;       // raw 24-bit signed value from HX711
    float    force_N;       // tared, filtered, deadbanded output
    bool     valid;         // false until first successful read after tare
    bool     saturated;     // true if HX711 output hit ±FS (overload)
    uint32_t timestamp_ms;
};

// ─── Lifecycle ────────────────────────────────────────────────────────────────
void tension_init();                    // configure pins, reset state
bool tension_tare(uint8_t samples = 16); // blocking; sets zero reference
bool tension_update();                  // non-blocking poll; call in loop()

// ─── Output ───────────────────────────────────────────────────────────────────
bool         tension_is_valid();
float        tension_get_force_N();
TensionData  tension_get_data();

// ─── Configuration (take effect on next update) ──────────────────────────────
bool   tension_set_calibration(float counts_per_N); // must be > 0
void   tension_set_direction(int8_t dir);           // +1 or -1
void   tension_set_filter_alpha(float alpha);       // 0.0–1.0 (1 = no filter)
void   tension_set_deadband(float deadband_N);      // ≥ 0

float  tension_get_calibration();
int8_t tension_get_direction();
float  tension_get_filter_alpha();
float  tension_get_deadband();
