#include "tension_sensor.h"
#include <limits.h>
#include <math.h>
#include <stdint.h>

// HX711 protocol: 24 data pulses + 1 gain/channel pulse = Channel A, gain 128.
static const uint8_t EXTRA_PULSES = 1;
static const long HX711_SATURATED_MIN = -8388608L;
static const long HX711_SATURATED_MAX = 8388607L;

// Key parameters to update after calibration and geometry verification.
static const float COUNTS_PER_N_LOADCELL = 44916.0f;
static const int8_t FORCE_DIRECTION = 1;
static const float FILTER_ALPHA = 0.20f;
static const float DEADBAND_LOADCELL_N = 0.02f;

// CAD-derived tether/load-cell geometry.
static const float WRAP_ANGLE_DEG = 161.99f;
static const float LOAD_AXIS_ANGLE_DEG = 12.03f;

static long s_tare_offset = 0;
static float s_filtered_loadcell_N = 0.0f;
static bool s_filter_ready = false;
static bool s_tared = false;

static TensionData s_data = {};

static inline bool hx711_ready()
{
    return digitalRead(TENSION_DOUT_PIN) == LOW;
}

static bool hx711_wait_ready(uint32_t timeout_ms)
{
    uint32_t t0 = millis();
    while (!hx711_ready()) {
        if ((millis() - t0) >= timeout_ms) {
            return false;
        }
    }
    return true;
}

static long hx711_read_one()
{
    if (!hx711_ready()) {
        return LONG_MIN;
    }

    uint32_t raw = 0;

    noInterrupts();
    for (uint8_t i = 0; i < 24; i++) {
        digitalWrite(TENSION_SCK_PIN, HIGH);
        delayMicroseconds(1);
        raw = (raw << 1) | (digitalRead(TENSION_DOUT_PIN) ? 1U : 0U);
        digitalWrite(TENSION_SCK_PIN, LOW);
        delayMicroseconds(1);
    }

    for (uint8_t i = 0; i < EXTRA_PULSES; i++) {
        digitalWrite(TENSION_SCK_PIN, HIGH);
        delayMicroseconds(1);
        digitalWrite(TENSION_SCK_PIN, LOW);
        delayMicroseconds(1);
    }
    interrupts();

    if (raw & 0x800000UL) {
        raw |= 0xFF000000UL;
    }

    return (long)(int32_t)raw;
}

static long hx711_average(uint8_t samples, uint32_t timeout_ms)
{
    if (samples == 0) {
        samples = 1;
    }

    int64_t sum = 0;
    uint8_t count = 0;

    for (uint8_t i = 0; i < samples; i++) {
        if (!hx711_wait_ready(timeout_ms)) {
            break;
        }

        long v = hx711_read_one();
        if (v == LONG_MIN) {
            continue;
        }

        sum += v;
        count++;
    }

    if (count == 0) {
        return LONG_MIN;
    }

    return (long)(sum / count);
}

static bool is_saturated(long raw)
{
    return (raw == HX711_SATURATED_MIN) || (raw == HX711_SATURATED_MAX);
}

static void reset_filter()
{
    s_filter_ready = false;
    s_filtered_loadcell_N = 0.0f;
}

static float geometry_factor()
{
    static const float DEG_TO_RAD_LOCAL = 0.01745329251994329577f;
    float wrap_rad = WRAP_ANGLE_DEG * DEG_TO_RAD_LOCAL;
    float axis_rad = LOAD_AXIS_ANGLE_DEG * DEG_TO_RAD_LOCAL;
    return 2.0f * sinf(0.5f * wrap_rad) * cosf(axis_rad);
}

static float adc_to_loadcell_force(long raw_adc)
{
    return (float)FORCE_DIRECTION *
           ((float)(raw_adc - s_tare_offset) / COUNTS_PER_N_LOADCELL);
}

static float apply_filter(float x)
{
    if (!s_filter_ready) {
        s_filtered_loadcell_N = x;
        s_filter_ready = true;
    } else {
        s_filtered_loadcell_N =
            FILTER_ALPHA * x + (1.0f - FILTER_ALPHA) * s_filtered_loadcell_N;
    }

    return s_filtered_loadcell_N;
}

void tension_init()
{
    pinMode(TENSION_DOUT_PIN, INPUT);
    pinMode(TENSION_SCK_PIN, OUTPUT);
    digitalWrite(TENSION_SCK_PIN, LOW);

    s_tare_offset = 0;
    s_tared = false;
    reset_filter();
    s_data = {};
}

bool tension_tare(uint8_t samples)
{
    long avg = hx711_average(samples, 30);
    if (avg == LONG_MIN) {
        return false;
    }

    s_tare_offset = avg;
    s_tared = true;
    reset_filter();
    s_data = {};
    return true;
}

bool tension_update()
{
    if (!s_tared || !hx711_ready()) {
        return false;
    }

    long raw = hx711_read_one();
    if (raw == LONG_MIN) {
        s_data.valid = false;
        return false;
    }

    float raw_loadcell_N = adc_to_loadcell_force(raw);
    if (isnan(raw_loadcell_N) || isinf(raw_loadcell_N)) {
        s_data.valid = false;
        return false;
    }

    float filtered_loadcell_N = apply_filter(raw_loadcell_N);
    if (fabsf(filtered_loadcell_N) < DEADBAND_LOADCELL_N) {
        filtered_loadcell_N = 0.0f;
    }

    float factor = geometry_factor();
    float tether_tension_N = filtered_loadcell_N / factor;

    s_data.raw_adc = raw;
    s_data.force_raw_loadcell_N = raw_loadcell_N;
    s_data.force_filtered_loadcell_N = filtered_loadcell_N;
    s_data.tether_tension_N = tether_tension_N;
    s_data.valid = true;
    s_data.saturated = is_saturated(raw);
    s_data.timestamp_ms = millis();

    return true;
}

bool tension_is_valid()
{
    return s_data.valid;
}

float tension_get_tether_N()
{
    return s_data.tether_tension_N;
}

float tension_get_force_N()
{
    return s_data.tether_tension_N;
}

TensionData tension_get_data()
{
    return s_data;
}
