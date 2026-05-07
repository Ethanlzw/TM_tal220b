#include <Arduino.h>
#include "tension_sensor.h"

static const uint32_t OUTPUT_PERIOD_MS = 100;
static const uint8_t TARE_SAMPLES = 16;

static uint32_t g_last_print_ms = 0;

static void do_tare(uint8_t samples)
{
    Serial.print("tare...");
    while (!tension_tare(samples)) {
        Serial.print(" retry");
        delay(20);
    }
    Serial.println(" ok");
}

void setup()
{
    Serial.begin(115200);
    delay(100);

    tension_init();

    // HX711 80 SPS settling time is about 50 ms; 100 ms gives margin.
    delay(100);

    Serial.println("HX711 TAL220B tether tension sensor");
    do_tare(TARE_SAMPLES);
    Serial.println("ready");

    Serial.println("timestamp_ms,raw_adc,F_raw_loadcell_N,F_filter_loadcell_N,T_tether_N,saturated");
}

void loop()
{
    if (!tension_update()) {
        return;
    }

    TensionData d = tension_get_data();
    uint32_t now = millis();

    if (now - g_last_print_ms >= OUTPUT_PERIOD_MS) {
        g_last_print_ms = now;

        Serial.print(d.timestamp_ms);
        Serial.print(",");
        Serial.print(d.raw_adc);
        Serial.print(",");
        Serial.print(d.force_raw_loadcell_N, 5);
        Serial.print(",");
        Serial.print(d.force_filtered_loadcell_N, 5);
        Serial.print(",");
        Serial.print(d.tether_tension_N, 5);
        Serial.print(",");
        Serial.println(d.saturated ? 1 : 0);
    }
}
