#include "tension_sensor.h"
#include "loadcell.h"
#include <math.h>

// =========================
// 默认参数
// =========================
static float   s_calibration_counts_per_N = 10000.0f;
static float   s_filter_alpha             = 0.50f;   // 响应更快
static float   s_deadband_N               = 0.02f;
static uint8_t s_average_samples          = 1;       // 默认 1，优先响应速度

// =========================
// 内部状态
// =========================
static TensionData s_data = {0, 0.0f, 0.0f, false, 0};

static float s_software_zero_N = 0.0f;
static float s_filtered_force_N = 0.0f;
static bool  s_filter_initialized = false;

static float clamp_float(float x, float xmin, float xmax)
{
    if (x < xmin) return xmin;
    if (x > xmax) return xmax;
    return x;
}

void tension_reset_filter()
{
    s_filter_initialized = false;
    s_filtered_force_N = 0.0f;
}

static float low_pass_filter(float x)
{
    s_filter_alpha = clamp_float(s_filter_alpha, 0.0f, 1.0f);

    if (!s_filter_initialized) {
        s_filtered_force_N = x;
        s_filter_initialized = true;
    } else {
        s_filtered_force_N =
            s_filter_alpha * x +
            (1.0f - s_filter_alpha) * s_filtered_force_N;
    }

    return s_filtered_force_N;
}

void tension_init()
{
    loadcell_init();

    loadcell_set_calibration(s_calibration_counts_per_N);

    s_data.raw_adc = 0;
    s_data.raw_force_N = 0.0f;
    s_data.force_N = 0.0f;
    s_data.valid = false;
    s_data.timestamp_ms = 0;

    s_software_zero_N = 0.0f;
    tension_reset_filter();
}

bool tension_update()
{
    if (!loadcell_is_ready()) {
        return false;
    }

    if (s_average_samples < 1) {
        s_average_samples = 1;
    }

    long raw_adc = loadcell_read_raw_average(s_average_samples);
    long offset  = loadcell_get_offset();

    if (s_calibration_counts_per_N <= 0.0f) {
        s_data.valid = false;
        return false;
    }

    float raw_force_N = ((float)raw_adc - (float)offset) / s_calibration_counts_per_N;

    if (isnan(raw_force_N) || isinf(raw_force_N)) {
        s_data.valid = false;
        return false;
    }

    float zeroed_force_N = raw_force_N - s_software_zero_N;
    float filtered_force_N = low_pass_filter(zeroed_force_N);

    if (fabsf(filtered_force_N) < s_deadband_N) {
        filtered_force_N = 0.0f;
    }

    s_data.raw_adc = raw_adc;
    s_data.raw_force_N = raw_force_N;
    s_data.force_N = filtered_force_N;
    s_data.valid = true;
    s_data.timestamp_ms = millis();

    return true;
}

bool tension_is_valid()
{
    return s_data.valid;
}

float tension_get_force_N()
{
    return s_data.force_N;
}

float tension_get_raw_force_N()
{
    return s_data.raw_force_N;
}

long tension_get_raw_adc()
{
    return s_data.raw_adc;
}

uint32_t tension_get_timestamp_ms()
{
    return s_data.timestamp_ms;
}

TensionData tension_get_data()
{
    return s_data;
}

bool tension_hardware_tare(uint8_t samples)
{
    if (samples < 1) {
        samples = 1;
    }

    bool ok = loadcell_tare(samples);

    if (ok) {
        s_software_zero_N = 0.0f;
        s_data.raw_adc = 0;
        s_data.raw_force_N = 0.0f;
        s_data.force_N = 0.0f;
        s_data.valid = false;
        s_data.timestamp_ms = millis();
        tension_reset_filter();
    }

    return ok;
}

bool tension_software_zero()
{
    if (!loadcell_is_ready()) {
        return false;
    }

    long raw_adc = loadcell_read_raw_average(s_average_samples);
    long offset  = loadcell_get_offset();

    if (s_calibration_counts_per_N <= 0.0f) {
        return false;
    }

    float raw_force_N = ((float)raw_adc - (float)offset) / s_calibration_counts_per_N;

    if (isnan(raw_force_N) || isinf(raw_force_N)) {
        return false;
    }

    s_software_zero_N = raw_force_N;
    tension_reset_filter();

    return true;
}

void tension_clear_software_zero()
{
    s_software_zero_N = 0.0f;
    tension_reset_filter();
}

bool tension_set_calibration(float counts_per_N)
{
    if (counts_per_N <= 0.0f) {
        return false;
    }

    if (!loadcell_set_calibration(counts_per_N)) {
        return false;
    }

    s_calibration_counts_per_N = counts_per_N;
    tension_reset_filter();

    return true;
}

float tension_get_calibration()
{
    return s_calibration_counts_per_N;
}

void tension_set_filter_alpha(float alpha)
{
    s_filter_alpha = clamp_float(alpha, 0.0f, 1.0f);
    tension_reset_filter();
}

float tension_get_filter_alpha()
{
    return s_filter_alpha;
}

void tension_set_deadband(float deadband_N)
{
    if (deadband_N < 0.0f) {
        deadband_N = 0.0f;
    }

    s_deadband_N = deadband_N;
}

float tension_get_deadband()
{
    return s_deadband_N;
}

void tension_set_average_samples(uint8_t n)
{
    if (n < 1) {
        n = 1;
    }
    if (n > 32) {
        n = 32;
    }

    s_average_samples = n;
}

uint8_t tension_get_average_samples()
{
    return s_average_samples;
}

float tension_get_software_zero_N()
{
    return s_software_zero_N;
}