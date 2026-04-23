#include "tension_sensor.h"
#include <math.h>
#include <stdint.h>
#include <limits.h>

// ─── HX711 protocol constants ────────────────────────────────────────────────
// 25 total SCK pulses → Channel A, Gain 128 (datasheet Table 3)
static const uint8_t  EXTRA_PULSES     = 1;   // 24 data + 1 = 25 pulses total
static const long     HX711_SATURATED_MIN = (long)0xFF800000L; // 0x800000 sign-extended
static const long     HX711_SATURATED_MAX = 0x7FFFFFL;
static const uint32_t HX711_TARE_TIMEOUT_MS = 30;

// ─── Configuration ───────────────────────────────────────────────────────────
static float  s_cal_counts_per_N = 44916.0f; // placeholder – replace with real value
static float  s_filter_alpha     = 0.20f;
static float  s_deadband_N       = 0.02f;
static int8_t s_direction        = 1;

// ─── Runtime state ───────────────────────────────────────────────────────────
static long  s_tare_offset       = 0;
static float s_filtered_force_N  = 0.0f;
static bool  s_filter_ready      = false;
static bool  s_tared             = false;

static TensionData s_data = {};

// ─── HX711 low-level ─────────────────────────────────────────────────────────

static inline bool hx711_ready()
{
    return digitalRead(TENSION_DOUT_PIN) == LOW;
}

// Wait up to timeout_ms for DOUT LOW. Returns false on timeout.
static bool hx711_wait_ready(uint32_t timeout_ms)
{
    uint32_t t0 = millis();
    while (!hx711_ready()) {
        if ((millis() - t0) >= timeout_ms) return false;
        delayMicroseconds(50);
    }
    return true;
}

// Read one 24-bit sample. Returns LONG_MIN on failure.
// noInterrupts() window ≈ 24 × 2µs = ~48µs (well within safe range for Arduino).
static long hx711_read_one()
{
    if (!hx711_ready()) return LONG_MIN;

    uint32_t raw = 0;

    noInterrupts();
    for (uint8_t i = 0; i < 24; i++) {
        digitalWrite(TENSION_SCK_PIN, HIGH);
        delayMicroseconds(1);
        raw = (raw << 1) | (digitalRead(TENSION_DOUT_PIN) ? 1U : 0U);
        digitalWrite(TENSION_SCK_PIN, LOW);
        delayMicroseconds(1);
    }
    // Extra pulse(s) to set next conversion channel/gain (25 total → Ch.A Gain 128)
    for (uint8_t i = 0; i < EXTRA_PULSES; i++) {
        digitalWrite(TENSION_SCK_PIN, HIGH);
        delayMicroseconds(1);
        digitalWrite(TENSION_SCK_PIN, LOW);
        delayMicroseconds(1);
    }
    interrupts();

    // Sign-extend 24-bit two's complement → 32-bit
    if (raw & 0x800000UL) raw |= 0xFF000000UL;

    return (long)(int32_t)raw;
}

// Average `samples` readings. Each sample waits up to `timeout_ms` for DOUT.
// Returns LONG_MIN if no samples were collected.
static long hx711_average(uint8_t samples, uint32_t timeout_ms)
{
    if (samples == 0) samples = 1;
    int64_t sum = 0;
    uint8_t count = 0;
    long v_min = LONG_MAX;
    long v_max = LONG_MIN;

    for (uint8_t i = 0; i < samples; i++) {
        if (!hx711_wait_ready(timeout_ms)) break;
        long v = hx711_read_one();
        if (v == LONG_MIN) continue;
        sum += v;
        if (v < v_min) v_min = v;
        if (v > v_max) v_max = v;
        count++;
    }
    if (count == 0) return LONG_MIN;

    // For >=3 samples, trimmed mean (drop one min + one max) for better tare robustness.
    if (count >= 3) {
        sum -= v_min;
        sum -= v_max;
        count -= 2;
    }

    return (long)(sum / count);
}

static bool is_saturated(long raw)
{
    return (raw == HX711_SATURATED_MIN) || (raw == HX711_SATURATED_MAX);
}

// ─── Signal processing ───────────────────────────────────────────────────────

static float adc_to_force(long raw_adc)
{
    // s_cal_counts_per_N is always > 0 (enforced by setter)
    return (float)s_direction *
           ((float)(raw_adc - s_tare_offset) / s_cal_counts_per_N);
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

static void reset_filter()
{
    s_filter_ready = false;
    s_filtered_force_N = 0.0f;
}

// ─── Public API ──────────────────────────────────────────────────────────────

void tension_init()
{
    pinMode(TENSION_DOUT_PIN, INPUT);
    pinMode(TENSION_SCK_PIN,  OUTPUT);
    digitalWrite(TENSION_SCK_PIN, LOW);

    s_tare_offset = 0;
    s_tared       = false;
    reset_filter();
    s_data = {};
}

// Blocking tare: averages `samples` readings and stores as zero reference.
// HX711 at 80SPS → each sample up to 13ms. 16 samples ≈ 200ms total max.
bool tension_tare(uint8_t samples)
{
    long avg = hx711_average(samples, HX711_TARE_TIMEOUT_MS);
    if (avg == LONG_MIN) return false;

    s_tare_offset = avg;
    s_tared       = true;
    reset_filter();
    s_data = {};
    return true;
}

// Non-blocking update — call every loop iteration.
// Returns true and updates s_data only when a new sample is available.
bool tension_update()
{
    if (!s_tared)        return false;
    if (!hx711_ready())  return false;

    long raw = hx711_read_one();
    if (raw == LONG_MIN) {
        s_data.valid = false;
        return false;
    }

    float force = adc_to_force(raw);

    if (!isfinite(force)) {
        s_data.valid = false;
        return false;
    }

    float filtered = apply_filter(force);

    if (fabsf(filtered) < s_deadband_N) filtered = 0.0f;

    s_data.raw_adc      = raw;
    s_data.force_N      = filtered;
    s_data.valid        = true;
    s_data.saturated    = is_saturated(raw);
    s_data.timestamp_ms = millis();

    return true;
}

bool tension_is_valid()        { return s_data.valid; }
float tension_get_force_N()    { return s_data.force_N; }
TensionData tension_get_data() { return s_data; }

// ─── Configuration setters ───────────────────────────────────────────────────

bool tension_set_calibration(float counts_per_N)
{
    if (counts_per_N <= 0.0f || isnan(counts_per_N) || isinf(counts_per_N))
        return false;
    s_cal_counts_per_N = counts_per_N;
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
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    s_filter_alpha = alpha;
    reset_filter();
}

void tension_set_deadband(float deadband_N)
{
    if (deadband_N < 0.0f) deadband_N = 0.0f;
    s_deadband_N = deadband_N;
    // deadband change doesn't need filter reset – takes effect next update
}

float  tension_get_calibration()  { return s_cal_counts_per_N; }
int8_t tension_get_direction()    { return s_direction; }
float  tension_get_filter_alpha() { return s_filter_alpha; }
float  tension_get_deadband()     { return s_deadband_N; }
