#include "loadcell.h"
#include <math.h>
#include <limits.h>

// =========================
// 内部状态
// =========================
static long  s_offset = 0;
static float s_calibration_counts_per_N = 10000.0f;  // 占位，后续换成真实标定值

// HX711 默认使用 A 通道，增益 128
// 读完 24 位后，再补 1 个时钟脉冲
static const uint8_t HX711_GAIN_PULSES = 1;

// 时钟脉冲宽度，1~2 us 足够
static inline void hx711_delay_us()
{
    delayMicroseconds(1);
}

void loadcell_init()
{
    pinMode(LOADCELL_DOUT_PIN, INPUT);
    pinMode(LOADCELL_SCK_PIN, OUTPUT);

    digitalWrite(LOADCELL_SCK_PIN, LOW);
}

bool loadcell_is_ready()
{
    return digitalRead(LOADCELL_DOUT_PIN) == LOW;
}

bool loadcell_wait_ready(uint32_t timeout_ms)
{
    uint32_t start = millis();

    while (!loadcell_is_ready()) {
        if ((millis() - start) >= timeout_ms) {
            return false;
        }
    }

    return true;
}

long loadcell_read_raw()
{
    if (!loadcell_is_ready()) {
        return 0;
    }

    uint32_t value = 0;

    noInterrupts();

    // 读取 24 位数据，MSB first
    for (uint8_t i = 0; i < 24; i++) {
        digitalWrite(LOADCELL_SCK_PIN, HIGH);
        hx711_delay_us();

        value = (value << 1) | (digitalRead(LOADCELL_DOUT_PIN) ? 1U : 0U);

        digitalWrite(LOADCELL_SCK_PIN, LOW);
        hx711_delay_us();
    }

    // 增益选择脉冲，A通道128增益
    for (uint8_t i = 0; i < HX711_GAIN_PULSES; i++) {
        digitalWrite(LOADCELL_SCK_PIN, HIGH);
        hx711_delay_us();
        digitalWrite(LOADCELL_SCK_PIN, LOW);
        hx711_delay_us();
    }

    interrupts();

    // 24 位补符号位，转成有符号 32 位
    if (value & 0x800000UL) {
        value |= 0xFF000000UL;
    }

    return (long)((int32_t)value);
}

long loadcell_read_raw_average(uint8_t samples)
{
    if (samples < 1) {
        samples = 1;
    }

    int64_t sum = 0;
    uint8_t count = 0;

    for (uint8_t i = 0; i < samples; i++) {
        if (!loadcell_wait_ready(200)) {
            break;
        }

        sum += (int64_t)loadcell_read_raw();
        count++;
    }

    if (count == 0) {
        return 0;
    }

    return (long)(sum / count);
}

bool loadcell_tare(uint8_t samples)
{
    if (samples < 1) {
        samples = 1;
    }

    if (!loadcell_wait_ready(500)) {
        return false;
    }

    s_offset = loadcell_read_raw_average(samples);
    return true;
}

long loadcell_get_offset()
{
    return s_offset;
}

void loadcell_set_offset(long offset)
{
    s_offset = offset;
}

bool loadcell_set_calibration(float counts_per_N)
{
    if (counts_per_N <= 0.0f || isnan(counts_per_N) || isinf(counts_per_N)) {
        return false;
    }

    s_calibration_counts_per_N = counts_per_N;
    return true;
}

float loadcell_get_calibration()
{
    return s_calibration_counts_per_N;
}

float loadcell_read_force_N(uint8_t samples)
{
    if (s_calibration_counts_per_N <= 0.0f) {
        return NAN;
    }

    long raw = loadcell_read_raw_average(samples);

    float force_N = ((float)raw - (float)s_offset) / s_calibration_counts_per_N;

    if (isnan(force_N) || isinf(force_N)) {
        return NAN;
    }

    return force_N;
}

void loadcell_power_down()
{
    digitalWrite(LOADCELL_SCK_PIN, LOW);
    hx711_delay_us();
    digitalWrite(LOADCELL_SCK_PIN, HIGH);
    delayMicroseconds(70);
}

void loadcell_power_up()
{
    digitalWrite(LOADCELL_SCK_PIN, LOW);
    delayMicroseconds(10);
}
