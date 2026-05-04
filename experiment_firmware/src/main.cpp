#include <Arduino.h>
#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "tension_sensor.h"

static const float STANDARD_GRAVITY = 9.80665f;

static uint32_t g_last_print_ms = 0;
static uint32_t g_phase_start_ms = 0;
static uint16_t g_output_period_ms = 100;
static bool g_logging_enabled = true;

static uint16_t g_experiment_id = 0;
static uint16_t g_phase_id = 0;
static uint16_t g_event_id = 0;
static float g_load_mass_kg = 0.0f;
static float g_force_ref_N = 0.0f;
static uint8_t g_temperature_source = 0; // 0=manual/external thermometer

static char g_cmd_buffer[128];
static uint8_t g_cmd_len = 0;

static void print_help()
{
    Serial.println("# Commands:");
    Serial.println("#   HELP");
    Serial.println("#   HEADER");
    Serial.println("#   LOG,0|1 or START or STOP");
    Serial.println("#   PERIOD,<ms>");
    Serial.println("#   EXP,<id>");
    Serial.println("#   PHASE,<id>");
    Serial.println("#   MARK,<id>");
    Serial.println("#   MASS,<kg>");
    Serial.println("#   LOADN,<force_N>");
    Serial.println("#   TEMP,<degC>");
    Serial.println("#   TARE,<samples>");
    Serial.println("#   CAL,<counts_per_N>");
    Serial.println("#   DIR,1|-1");
    Serial.println("#   ALPHA,<0..1>");
    Serial.println("#   DEADBAND,<N>");
    Serial.println("#   TEMPCOMP,<0|1>,<ref_C>,<zero_counts_per_C>,<span_per_C>");
    Serial.println("#   ZT,<0|1>,<beta>,<limit_N>");
    Serial.println("#   ZTRST");
    Serial.println("#   STATUS");
}

static void print_header()
{
    Serial.println(
        "record_type,"
        "timestamp_ms,"
        "sample_id,"
        "experiment_id,"
        "phase_id,"
        "event_id,"
        "phase_time_ms,"
        "load_mass_kg,"
        "force_ref_N,"
        "temperature_C,"
        "temperature_source,"
        "raw_adc,"
        "tare_offset_counts,"
        "temp_offset_counts,"
        "tracking_offset_counts,"
        "zero_offset_counts,"
        "net_counts,"
        "counts_per_N_used,"
        "raw_force_N,"
        "filtered_force_N,"
        "force_N,"
        "valid,"
        "saturated,"
        "tared,"
        "temp_comp_enabled,"
        "zero_tracking_enabled,"
        "zero_tracking_applied,"
        "output_period_ms");
}

static void print_status()
{
    Serial.print("# STATUS logging=");
    Serial.print(g_logging_enabled ? 1 : 0);
    Serial.print(" period_ms=");
    Serial.print(g_output_period_ms);
    Serial.print(" experiment_id=");
    Serial.print(g_experiment_id);
    Serial.print(" phase_id=");
    Serial.print(g_phase_id);
    Serial.print(" event_id=");
    Serial.print(g_event_id);
    Serial.print(" mass_kg=");
    Serial.print(g_load_mass_kg, 6);
    Serial.print(" force_ref_N=");
    Serial.print(g_force_ref_N, 6);
    Serial.print(" cal_counts_per_N=");
    Serial.print(tension_get_calibration(), 6);
    Serial.print(" alpha=");
    Serial.print(tension_get_filter_alpha(), 6);
    Serial.print(" deadband_N=");
    Serial.print(tension_get_deadband(), 6);
    Serial.print(" tare_offset_counts=");
    Serial.print(tension_get_tare_offset_counts(), 3);
    Serial.print(" tracking_offset_counts=");
    Serial.println(tension_get_tracking_offset_counts(), 3);
}

static void print_data_row(const TensionData &d)
{
    Serial.print("DATA,");
    Serial.print(d.timestamp_ms);
    Serial.print(",");
    Serial.print(d.sample_id);
    Serial.print(",");
    Serial.print(g_experiment_id);
    Serial.print(",");
    Serial.print(g_phase_id);
    Serial.print(",");
    Serial.print(g_event_id);
    Serial.print(",");
    Serial.print((uint32_t)(d.timestamp_ms - g_phase_start_ms));
    Serial.print(",");
    Serial.print(g_load_mass_kg, 6);
    Serial.print(",");
    Serial.print(g_force_ref_N, 6);
    Serial.print(",");
    Serial.print(d.temperature_C, 3);
    Serial.print(",");
    Serial.print(g_temperature_source);
    Serial.print(",");
    Serial.print(d.raw_adc);
    Serial.print(",");
    Serial.print(d.tare_offset_counts, 3);
    Serial.print(",");
    Serial.print(d.temp_offset_counts, 3);
    Serial.print(",");
    Serial.print(d.tracking_offset_counts, 3);
    Serial.print(",");
    Serial.print(d.zero_offset_counts, 3);
    Serial.print(",");
    Serial.print(d.net_counts, 3);
    Serial.print(",");
    Serial.print(d.counts_per_N_used, 6);
    Serial.print(",");
    Serial.print(d.raw_force_N, 6);
    Serial.print(",");
    Serial.print(d.filtered_force_N, 6);
    Serial.print(",");
    Serial.print(d.force_N, 6);
    Serial.print(",");
    Serial.print(d.valid ? 1 : 0);
    Serial.print(",");
    Serial.print(d.saturated ? 1 : 0);
    Serial.print(",");
    Serial.print(d.tared ? 1 : 0);
    Serial.print(",");
    Serial.print(d.temp_comp_enabled ? 1 : 0);
    Serial.print(",");
    Serial.print(d.zero_tracking_enabled ? 1 : 0);
    Serial.print(",");
    Serial.print(d.zero_tracking_applied ? 1 : 0);
    Serial.print(",");
    Serial.println(g_output_period_ms);
}

static void normalize_command(char *line)
{
    for (char *p = line; *p != '\0'; ++p) {
        if (*p == ',' || *p == ';' || *p == '\t') {
            *p = ' ';
        }
    }
}

static char *next_token()
{
    return strtok(NULL, " ");
}

static float parse_float_arg(float fallback)
{
    char *tok = next_token();
    if (tok == NULL) {
        return fallback;
    }
    return atof(tok);
}

static long parse_long_arg(long fallback)
{
    char *tok = next_token();
    if (tok == NULL) {
        return fallback;
    }
    return atol(tok);
}

static void handle_command(char *line)
{
    normalize_command(line);

    char *cmd = strtok(line, " ");
    if (cmd == NULL) {
        return;
    }

    for (char *p = cmd; *p != '\0'; ++p) {
        *p = (char)toupper(*p);
    }

    if (strcmp(cmd, "HELP") == 0) {
        print_help();
    } else if (strcmp(cmd, "HEADER") == 0) {
        print_header();
    } else if (strcmp(cmd, "START") == 0) {
        g_logging_enabled = true;
        Serial.println("# LOG=1");
    } else if (strcmp(cmd, "STOP") == 0) {
        g_logging_enabled = false;
        Serial.println("# LOG=0");
    } else if (strcmp(cmd, "LOG") == 0) {
        g_logging_enabled = parse_long_arg(g_logging_enabled ? 1 : 0) != 0;
        Serial.print("# LOG=");
        Serial.println(g_logging_enabled ? 1 : 0);
    } else if (strcmp(cmd, "PERIOD") == 0) {
        long period = parse_long_arg(g_output_period_ms);
        if (period < 13) {
            period = 13; // HX711 80 SPS is about 12.5 ms.
        }
        if (period > 60000) {
            period = 60000;
        }
        g_output_period_ms = (uint16_t)period;
        Serial.print("# PERIOD=");
        Serial.println(g_output_period_ms);
    } else if (strcmp(cmd, "EXP") == 0) {
        g_experiment_id = (uint16_t)parse_long_arg(g_experiment_id);
        Serial.print("# EXP=");
        Serial.println(g_experiment_id);
    } else if (strcmp(cmd, "PHASE") == 0) {
        g_phase_id = (uint16_t)parse_long_arg(g_phase_id);
        g_phase_start_ms = millis();
        Serial.print("# PHASE=");
        Serial.println(g_phase_id);
    } else if (strcmp(cmd, "MARK") == 0) {
        g_event_id = (uint16_t)parse_long_arg(g_event_id);
        Serial.print("# MARK=");
        Serial.println(g_event_id);
    } else if (strcmp(cmd, "MASS") == 0) {
        g_load_mass_kg = parse_float_arg(g_load_mass_kg);
        g_force_ref_N = g_load_mass_kg * STANDARD_GRAVITY;
        Serial.print("# MASS_KG=");
        Serial.print(g_load_mass_kg, 6);
        Serial.print(" FORCE_REF_N=");
        Serial.println(g_force_ref_N, 6);
    } else if (strcmp(cmd, "LOADN") == 0) {
        g_force_ref_N = parse_float_arg(g_force_ref_N);
        g_load_mass_kg = g_force_ref_N / STANDARD_GRAVITY;
        Serial.print("# FORCE_REF_N=");
        Serial.print(g_force_ref_N, 6);
        Serial.print(" MASS_KG=");
        Serial.println(g_load_mass_kg, 6);
    } else if (strcmp(cmd, "TEMP") == 0) {
        float temperature_C = parse_float_arg(25.0f);
        tension_set_temperature(temperature_C);
        Serial.print("# TEMP_C=");
        Serial.println(temperature_C, 3);
    } else if (strcmp(cmd, "TARE") == 0) {
        uint8_t samples = (uint8_t)parse_long_arg(64);
        Serial.print("# TARE_START samples=");
        Serial.println(samples);
        bool ok = tension_tare(samples);
        Serial.print("# TARE_DONE ok=");
        Serial.print(ok ? 1 : 0);
        Serial.print(" tare_offset_counts=");
        Serial.println(tension_get_tare_offset_counts(), 3);
    } else if (strcmp(cmd, "CAL") == 0) {
        float counts_per_N = parse_float_arg(tension_get_calibration());
        bool ok = tension_set_calibration(counts_per_N);
        Serial.print("# CAL ok=");
        Serial.print(ok ? 1 : 0);
        Serial.print(" counts_per_N=");
        Serial.println(tension_get_calibration(), 6);
    } else if (strcmp(cmd, "DIR") == 0) {
        tension_set_direction((int8_t)parse_long_arg(tension_get_direction()));
        Serial.print("# DIR=");
        Serial.println(tension_get_direction());
    } else if (strcmp(cmd, "ALPHA") == 0) {
        tension_set_filter_alpha(parse_float_arg(tension_get_filter_alpha()));
        Serial.print("# ALPHA=");
        Serial.println(tension_get_filter_alpha(), 6);
    } else if (strcmp(cmd, "DEADBAND") == 0) {
        tension_set_deadband(parse_float_arg(tension_get_deadband()));
        Serial.print("# DEADBAND_N=");
        Serial.println(tension_get_deadband(), 6);
    } else if (strcmp(cmd, "TEMPCOMP") == 0) {
        bool enable = parse_long_arg(0) != 0;
        float ref_C = parse_float_arg(25.0f);
        float zero_coeff = parse_float_arg(0.0f);
        float span_coeff = parse_float_arg(0.0f);
        tension_set_temperature_compensation(ref_C, zero_coeff, span_coeff);
        tension_enable_temperature_compensation(enable);
        Serial.print("# TEMPCOMP enable=");
        Serial.print(enable ? 1 : 0);
        Serial.print(" ref_C=");
        Serial.print(ref_C, 3);
        Serial.print(" zero_counts_per_C=");
        Serial.print(zero_coeff, 6);
        Serial.print(" span_per_C=");
        Serial.println(span_coeff, 9);
    } else if (strcmp(cmd, "ZT") == 0) {
        bool enable = parse_long_arg(0) != 0;
        float beta = parse_float_arg(0.001f);
        float limit_N = parse_float_arg(0.3f);
        tension_set_zero_tracking(beta, limit_N);
        tension_enable_zero_tracking(enable);
        Serial.print("# ZERO_TRACKING enable=");
        Serial.print(enable ? 1 : 0);
        Serial.print(" beta=");
        Serial.print(beta, 6);
        Serial.print(" limit_N=");
        Serial.println(limit_N, 6);
    } else if (strcmp(cmd, "ZTRST") == 0) {
        tension_reset_zero_tracking();
        Serial.println("# ZERO_TRACKING_RESET");
    } else if (strcmp(cmd, "STATUS") == 0) {
        print_status();
    } else {
        Serial.print("# Unknown command: ");
        Serial.println(cmd);
    }
}

static void read_serial_commands()
{
    while (Serial.available() > 0) {
        char c = (char)Serial.read();
        if (c == '\r') {
            continue;
        }

        if (c == '\n') {
            g_cmd_buffer[g_cmd_len] = '\0';
            handle_command(g_cmd_buffer);
            g_cmd_len = 0;
        } else if (g_cmd_len < sizeof(g_cmd_buffer) - 1U) {
            g_cmd_buffer[g_cmd_len++] = c;
        } else {
            g_cmd_len = 0;
            Serial.println("# Command too long; buffer cleared");
        }
    }
}

void setup()
{
    Serial.begin(115200);
    delay(300);

    tension_init();
    tension_set_calibration(44916.0f);
    tension_set_direction(1);
    tension_set_filter_alpha(0.20f);
    tension_set_deadband(0.02f);
    tension_set_temperature(25.0f);

    g_phase_start_ms = millis();

    Serial.println("# TAL220B_HX711_EXPERIMENT_LOGGER");
    Serial.println("# Board target: NUCLEO-G431RB, HX711 DOUT=D3/PB3, SCK=D2/PA10");
    Serial.println("# Data rows start with DATA. Comment/status rows start with #.");
    print_header();
    print_help();
    print_status();
}

void loop()
{
    read_serial_commands();

    if (tension_update()) {
        TensionData d = tension_get_data();
        uint32_t now = millis();
        if (g_logging_enabled && (uint32_t)(now - g_last_print_ms) >= g_output_period_ms) {
            g_last_print_ms = now;
            print_data_row(d);
        }
    }
}
