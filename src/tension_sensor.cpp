#include "tension_sensor.h"
#include <math.h>
#include <stdint.h>

// =========================
// 默认参数
// =========================
static float   s_calibration_counts_per_N = 10000.0f;  // 占位值，后续改成真实标定值
static float   s_filter_alpha             = 0.60f;     // 越大响应越快
static float   s_deadband_N               = 0.02f;
static uint8_t s_average_samples          = 1;         // 默认优先响应速度
static int8_t  s_direction                = 1;         // +1 或 -1

// =========================
// 内部状态
// =========================
static long  s_offset = 0;                 // 硬件去皮 offset
static float s_software_zero_N = 0.0f;     // 软件零点
static float s_filtered_force_N = 0.0f;
static bool  s_filter_initialized = false;

static TensionData s_data = {0, 0.0f, 0.0f, false, 0};

// HX711 默认 A 通道，增益 128
static const uint8_t HX711_GAIN_PULSES = 1;

// =========================
// 工具函数
// =========================
static float clamp_float(float x, float xmin, float xmax)
{
    if (x < xmin) return xmin;
    if (x > xmax) return xmax;
    return x;
}

static inline void hx711_delay_us()
{
    delayMicroseconds(1);
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

// =========================
// HX711 底层驱动
// =========================
static bool hx711_is_ready()
{
    return (digitalRead(TENSION_DOUT_PIN) == LOW);
}

static bool hx711_wait_ready(uint32_t timeout_ms)
{
    uint32_t start = millis();

    while (!hx711_is_ready()) {
        if ((millis() - start) >= timeout_ms) {
            return false;
        }
    }

    return true;
}

static long hx711_read_raw_once()
{
    if (!hx711_is_ready()) {
        return 0;
    }

    uint32_t value = 0;

    noInterrupts();

    for (uint8_t i = 0; i < 24; i++) {
        digitalWrite(TENSION_SCK_PIN, HIGH);
        hx711_delay_us();

        value = (value << 1) | (digitalRead(TENSION_DOUT_PIN) ? 1U : 0U);

        digitalWrite(TENSION_SCK_PIN, LOW);
        hx711_delay_us();
    }

    // A 通道，128 增益
    for (uint8_t i = 0; i < HX711_GAIN_PULSES; i++) {
        digitalWrite(TENSION_SCK_PIN, HIGH);
        hx711_delay_us();
        digitalWrite(TENSION_SCK_PIN, LOW);
        hx711_delay_us();
    }

    interrupts();

    // 24 位有符号数扩展到 32 位
    if (value & 0x800000UL) {
        value |= 0xFF000000UL;
    }

    return (long)((int32_t)value);
}

static long hx711_read_raw_average(uint8_t samples)
{
    if (samples < 1) {
        samples = 1;
    }

    int64_t sum = 0;
    uint8_t count = 0;

    for (uint8_t i = 0; i < samples; i++) {
        if (!hx711_wait_ready(200)) {
            break;
        }

        sum += (int64_t)hx711_read_raw_once();
        count++;
    }

    if (count == 0) {
        return 0;
    }

    return (long)(sum / count);
}

static float raw_adc_to_force_N(long raw_adc)
{
    if (s_calibration_counts_per_N <= 0.0f) {
        return NAN;
    }

    float force_N =
        (float)s_direction *
        (((float)raw_adc - (float)s_offset) / s_calibration_counts_per_N);

    if (isnan(force_N) || isinf(force_N)) {
        return NAN;
    }

    return force_N;
}

// =========================
// 对外接口
// =========================
void tension_init()
{
    pinMode(TENSION_DOUT_PIN, INPUT);
    pinMode(TENSION_SCK_PIN, OUTPUT);
    digitalWrite(TENSION_SCK_PIN, LOW);

    s_offset = 0;
    s_software_zero_N = 0.0f;
    tension_reset_filter();

    s_data.raw_adc = 0;
    s_data.raw_force_N = 0.0f;
    s_data.force_N = 0.0f;
    s_data.valid = false;
    s_data.timestamp_ms = 0;
}

bool tension_update()
{
    if (!hx711_is_ready()) {
        return false;
    }

    if (s_average_samples < 1) {
        s_average_samples = 1;
    }

    long raw_adc = hx711_read_raw_average(s_average_samples);
    float raw_force_N = raw_adc_to_force_N(raw_adc);

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

    if (!hx711_wait_ready(500)) {
        return false;
    }

    s_offset = hx711_read_raw_average(samples);
    s_software_zero_N = 0.0f;
    tension_reset_filter();

    s_data.raw_adc = 0;
    s_data.raw_force_N = 0.0f;
    s_data.force_N = 0.0f;
    s_data.valid = false;
    s_data.timestamp_ms = millis();

    return true;
}

bool tension_software_zero()
{
    if (!hx711_is_ready()) {
        return false;
    }

    long raw_adc = hx711_read_raw_average(s_average_samples);
    float raw_force_N = raw_adc_to_force_N(raw_adc);

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
    if (counts_per_N <= 0.0f || isnan(counts_per_N) || isinf(counts_per_N)) {
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

void tension_set_direction(int8_t dir)
{
    s_direction = (dir >= 0) ? 1 : -1;
    tension_reset_filter();
}

int8_t tension_get_direction()
{
    return s_direction;
}

float tension_get_software_zero_N()
{
    return s_software_zero_N;
}

long tension_get_offset()
{
    return s_offset;
}

void tension_power_down()
{
    digitalWrite(TENSION_SCK_PIN, LOW);
    hx711_delay_us();
    digitalWrite(TENSION_SCK_PIN, HIGH);
    delayMicroseconds(70);
}

void tension_power_up()
{
    digitalWrite(TENSION_SCK_PIN, LOW);
    delayMicroseconds(10);
}