#include "tension_sensor.h"

#include <limits.h>
#include <math.h>
#include <stdint.h>

static const uint8_t EXTRA_PULSES = 1;
static const int32_t HX711_SATURATED_MIN = -8388608;
static const int32_t HX711_SATURATED_MAX = 8388607;

static float s_counts_per_N = 44916.0f;
static float s_filter_alpha = 0.20f;
static float s_deadband_N = 0.02f;
static int8_t s_direction = 1;

static float s_tare_offset_counts = 0.0f;
static float s_filtered_force_N = 0.0f;
static bool s_filter_ready = false;
static bool s_tared = false;

static float s_temperature_C = 25.0f;
static float s_ref_temperature_C = 25.0f;
static float s_zero_coeff_counts_per_C = 0.0f;
static float s_span_coeff_per_C = 0.0f;
static bool s_temp_comp_enabled = false;

static float s_tracking_offset_counts = 0.0f;
static float s_zero_tracking_beta = 0.001f;
static float s_zero_tracking_limit_N = 0.3f;
static bool s_zero_tracking_enabled = false;

static uint32_t s_sample_id = 0;
static TensionData s_data = {};

static inline bool hx711_ready()
{
    return digitalRead(TENSION_DOUT_PIN) == LOW;
}

static bool hx711_wait_ready(uint32_t timeout_ms)
{
    uint32_t t0 = millis();
    while (!hx711_ready()) {
        if ((uint32_t)(millis() - t0) >= timeout_ms) {
            return false;
        }
    }
    return true;
}

static int32_t hx711_read_one()
{
    if (!hx711_ready()) {
        return INT_MIN;
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

    if ((raw & 0x800000UL) != 0U) {
        raw |= 0xFF000000UL;
    }

    return (int32_t)raw;
}

static int32_t hx711_average(uint8_t samples, uint32_t timeout_ms)
{
    if (samples == 0U) {
        samples = 1U;
    }

    int64_t sum = 0;
    uint8_t count = 0;

    for (uint8_t i = 0; i < samples; i++) {
        if (!hx711_wait_ready(timeout_ms)) {
            break;
        }

        int32_t value = hx711_read_one();
        if (value == INT_MIN) {
            continue;
        }

        sum += value;
        count++;
    }

    if (count == 0U) {
        return INT_MIN;
    }

    return (int32_t)(sum / count);
}

static bool is_saturated(int32_t raw)
{
    return raw == HX711_SATURATED_MIN || raw == HX711_SATURATED_MAX;
}

static void reset_filter()
{
    s_filter_ready = false;
    s_filtered_force_N = 0.0f;
}

static float apply_filter(float x)
{
    if (!s_filter_ready) {
        s_filtered_force_N = x;
        s_filter_ready = true;
    } else {
        s_filtered_force_N = s_filter_alpha * x +
                             (1.0f - s_filter_alpha) * s_filtered_force_N;
    }
    return s_filtered_force_N;
}

static float temperature_delta_C()
{
    return s_temperature_C - s_ref_temperature_C;
}

static float temperature_zero_offset_counts()
{
    if (!s_temp_comp_enabled) {
        return 0.0f;
    }
    return s_zero_coeff_counts_per_C * temperature_delta_C();
}

static float effective_counts_per_N()
{
    float counts = s_counts_per_N;
    if (s_temp_comp_enabled) {
        counts *= (1.0f + s_span_coeff_per_C * temperature_delta_C());
    }

    if (!isfinite(counts) || counts <= 0.0f) {
        counts = 1.0f;
    }

    return counts;
}

void tension_init()
{
    pinMode(TENSION_DOUT_PIN, INPUT);
    pinMode(TENSION_SCK_PIN, OUTPUT);
    digitalWrite(TENSION_SCK_PIN, LOW);

    s_tare_offset_counts = 0.0f;
    s_tracking_offset_counts = 0.0f;
    s_tared = false;
    s_sample_id = 0;
    reset_filter();
    s_data = {};
}

bool tension_tare(uint8_t samples)
{
    int32_t avg = hx711_average(samples, 30);
    if (avg == INT_MIN) {
        return false;
    }

    s_tare_offset_counts = (float)avg;
    s_tracking_offset_counts = 0.0f;
    s_tared = true;
    reset_filter();
    s_data = {};
    s_data.tared = true;
    return true;
}

bool tension_update()
{
    if (!hx711_ready()) {
        return false;
    }

    int32_t raw = hx711_read_one();
    if (raw == INT_MIN) {
        s_data.valid = false;
        return false;
    }

    bool saturated = is_saturated(raw);
    float temp_offset = temperature_zero_offset_counts();
    float base_tare = s_tared ? s_tare_offset_counts : 0.0f;
    float zero_offset = base_tare + temp_offset + s_tracking_offset_counts;
    float net_counts = (float)raw - zero_offset;
    float counts_used = effective_counts_per_N();
    float raw_force = (float)s_direction * (net_counts / counts_used);

    if (!isfinite(raw_force)) {
        s_data.valid = false;
        return false;
    }

    bool tracking_applied = false;
    if (s_tared &&
        s_zero_tracking_enabled &&
        !saturated &&
        fabsf(raw_force) < s_zero_tracking_limit_N) {
        float correction = s_zero_tracking_beta * net_counts;
        s_tracking_offset_counts += correction;
        tracking_applied = true;

        zero_offset = base_tare + temp_offset + s_tracking_offset_counts;
        net_counts = (float)raw - zero_offset;
        raw_force = (float)s_direction * (net_counts / counts_used);
    }

    float filtered = apply_filter(raw_force);
    float deadbanded = filtered;
    if (fabsf(deadbanded) < s_deadband_N) {
        deadbanded = 0.0f;
    }

    s_sample_id++;

    s_data.sample_id = s_sample_id;
    s_data.timestamp_ms = millis();
    s_data.raw_adc = raw;
    s_data.tare_offset_counts = s_tare_offset_counts;
    s_data.temp_offset_counts = temp_offset;
    s_data.tracking_offset_counts = s_tracking_offset_counts;
    s_data.zero_offset_counts = zero_offset;
    s_data.net_counts = net_counts;
    s_data.counts_per_N_used = counts_used;
    s_data.raw_force_N = raw_force;
    s_data.filtered_force_N = filtered;
    s_data.force_N = deadbanded;
    s_data.temperature_C = s_temperature_C;
    s_data.valid = true;
    s_data.saturated = saturated;
    s_data.tared = s_tared;
    s_data.temp_comp_enabled = s_temp_comp_enabled;
    s_data.zero_tracking_enabled = s_zero_tracking_enabled;
    s_data.zero_tracking_applied = tracking_applied;

    return true;
}

TensionData tension_get_data()
{
    return s_data;
}

bool tension_is_valid()
{
    return s_data.valid;
}

float tension_get_force_N()
{
    return s_data.force_N;
}

bool tension_set_calibration(float counts_per_N)
{
    if (!isfinite(counts_per_N) || counts_per_N <= 0.0f) {
        return false;
    }
    s_counts_per_N = counts_per_N;
    reset_filter();
    return true;
}

void tension_set_direction(int8_t dir)
{
    s_direction = (dir >= 0) ? 1 : -1;
    reset_filter();
}

void tension_set_filter_alpha(float alpha)
{
    if (alpha < 0.0f) {
        alpha = 0.0f;
    }
    if (alpha > 1.0f) {
        alpha = 1.0f;
    }
    s_filter_alpha = alpha;
    reset_filter();
}

void tension_set_deadband(float deadband_N)
{
    s_deadband_N = (deadband_N >= 0.0f) ? deadband_N : 0.0f;
}

void tension_set_temperature(float temperature_C)
{
    if (isfinite(temperature_C)) {
        s_temperature_C = temperature_C;
    }
}

void tension_set_temperature_compensation(float ref_temperature_C,
                                          float zero_coeff_counts_per_C,
                                          float span_coeff_per_C)
{
    if (isfinite(ref_temperature_C)) {
        s_ref_temperature_C = ref_temperature_C;
    }
    if (isfinite(zero_coeff_counts_per_C)) {
        s_zero_coeff_counts_per_C = zero_coeff_counts_per_C;
    }
    if (isfinite(span_coeff_per_C)) {
        s_span_coeff_per_C = span_coeff_per_C;
    }
}

void tension_enable_temperature_compensation(bool enable)
{
    s_temp_comp_enabled = enable;
    reset_filter();
}

void tension_set_zero_tracking(float beta, float limit_N)
{
    if (isfinite(beta)) {
        if (beta < 0.0f) {
            beta = 0.0f;
        }
        if (beta > 0.1f) {
            beta = 0.1f;
        }
        s_zero_tracking_beta = beta;
    }

    if (isfinite(limit_N) && limit_N >= 0.0f) {
        s_zero_tracking_limit_N = limit_N;
    }
}

void tension_enable_zero_tracking(bool enable)
{
    s_zero_tracking_enabled = enable;
}

void tension_reset_zero_tracking()
{
    s_tracking_offset_counts = 0.0f;
}

float tension_get_calibration()
{
    return s_counts_per_N;
}

int8_t tension_get_direction()
{
    return s_direction;
}

float tension_get_filter_alpha()
{
    return s_filter_alpha;
}

float tension_get_deadband()
{
    return s_deadband_N;
}

float tension_get_tare_offset_counts()
{
    return s_tare_offset_counts;
}

float tension_get_tracking_offset_counts()
{
    return s_tracking_offset_counts;
}

bool tension_is_tared()
{
    return s_tared;
}
