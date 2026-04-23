#include <Arduino.h>
#include "tension_sensor.h"

static uint32_t g_last_print_ms = 0;
static bool g_last_saturated = false;

static const uint8_t  TARE_SAMPLES        = 16;
static const uint16_t TARE_RETRY_DELAY_MS = 20;
static const uint16_t PRINT_PERIOD_MS     = 100; // 10Hz

// Retry tare until success (non-fatal if HX711 is briefly not ready)
static void do_tare(uint8_t samples)
{
    Serial.print(F("tare..."));
    while (!tension_tare(samples)) {
        Serial.print(F(" retry"));
        delay(TARE_RETRY_DELAY_MS);
    }
    Serial.println(F(" ok"));
}

void setup()
{
    Serial.begin(115200);
    delay(100);

    tension_init();

    // Replace 10000.0 with real calibration value
    // Procedure: hang a known weight W_N (in Newtons), then:
    // counts_per_N = (raw_adc_loaded - raw_adc_tared) / W_N
    tension_set_calibration(44916.0f);

    // Set -1 if applying tension shows negative raw_force output
    tension_set_direction(1);

    // Low-pass filter: 0.20 ≈ moderate smoothing at 80 SPS
    // Increase toward 1.0 for faster response, decrease for more smoothing
    tension_set_filter_alpha(0.20f);

    // Suppress noise around zero (tune after calibration)
    tension_set_deadband(0.02f);

    // HX711 80SPS settling time = 50ms (datasheet); 100ms gives margin
    delay(100);
    Serial.println(F("HX711 tension sensor"));

    // Single tare with 16 samples (~200ms at 80SPS)
    do_tare(TARE_SAMPLES);

    Serial.println(F("ready"));

    // Output CSV headers
    Serial.println(F("timestamp_ms,raw_adc,force_N"));
}

void loop()
{
    if (tension_update()) {
        TensionData d = tension_get_data();

        // Overload warning
        if (d.saturated && !g_last_saturated) {
            Serial.println(F("WARNING: sensor saturated - input out of range!"));
        }
        g_last_saturated = d.saturated;

        // Print at 10 Hz
        uint32_t now = millis();
        if (now - g_last_print_ms >= PRINT_PERIOD_MS) {
            g_last_print_ms = now;
            Serial.print(d.timestamp_ms);
            Serial.print(',');
            Serial.print(d.raw_adc);
            Serial.print(',');
            Serial.println(d.force_N, 4);
        }

        // real-time feedback logic goes here 
        // Example: if (d.force_N > TENSION_LIMIT_N) { ... }
    }
}
