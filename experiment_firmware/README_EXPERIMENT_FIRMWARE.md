# TAL220B + HX711 Experimental Firmware

This firmware is for the engineering experiments described in the drift, calibration, temperature, creep, noise, and dynamic-response plan.

It is intentionally separate from the normal runtime firmware. The normal project prints only:

```text
timestamp_ms,raw_adc,force_N
```

That is enough for a quick demonstration, but not enough for a defensible experiment report because it does not record reference load, experiment phase, temperature, tare offset, unfiltered force, filter settings, compensation state, saturation state, or sample ID.

## Build Target

The included `platformio.ini` targets:

```text
board = nucleo_g431rb
framework = arduino
```

The hardware is still STM32 NUCLEO-G431RB. This experimental logger uses Arduino-core serial/GPIO calls only because the current PlatformIO project is configured that way.

Default pins:

```text
HX711 DOUT = D3 = PB3
HX711 SCK  = D2 = PA10
```

## Output CSV Columns

The firmware prints one header and repeated `DATA` rows:

```text
record_type
timestamp_ms
sample_id
experiment_id
phase_id
event_id
phase_time_ms
load_mass_kg
force_ref_N
temperature_C
temperature_source
raw_adc
tare_offset_counts
temp_offset_counts
tracking_offset_counts
zero_offset_counts
net_counts
counts_per_N_used
raw_force_N
filtered_force_N
force_N
valid
saturated
tared
temp_comp_enabled
zero_tracking_enabled
zero_tracking_applied
output_period_ms
```

Rows beginning with `#` are status/comment rows and can be ignored during analysis.

## Serial Commands

Commands can use commas or spaces.

```text
HELP
HEADER
START
STOP
LOG,0
LOG,1
PERIOD,100
EXP,1
PHASE,2
MARK,10
MASS,1.5
LOADN,14.7
TEMP,25.6
TARE,64
CAL,44916
DIR,1
DIR,-1
ALPHA,0.2
DEADBAND,0.02
TEMPCOMP,1,25.0,0.0,0.0
ZT,1,0.001,0.3
ZT,0,0.001,0.3
ZTRST
STATUS
```

Suggested `phase_id` convention:

```text
0  setup / idle
1  warmup zero drift
2  tare after warmup
3  static calibration loading
4  static calibration unloading
5  temperature zero
6  temperature loaded
7  creep loaded
8  creep recovery unloaded
9  noise
10 dynamic response
```

## Save Data to Files

Use the Python logger:

```powershell
python tools\serial_csv_logger.py --port COM5 --out data\zero_drift_25C.csv --duration 3600
```

Send setup commands when opening the port:

```powershell
python tools\serial_csv_logger.py --port COM5 --out data\cal_loading_1kg.csv --command "EXP,2" --command "PHASE,3" --command "MASS,1.0"
```

If `pyserial` is missing:

```powershell
python -m pip install pyserial
```

## Experiment Examples

Zero drift:

```text
EXP,1
PHASE,1
TEMP,25.0
PERIOD,100
START
```

Calibration point:

```text
EXP,2
PHASE,3
TEMP,25.0
TARE,64
MASS,1.0
START
```

Creep:

```text
EXP,4
PHASE,7
TEMP,25.0
MASS,5.0
START
```

Temperature compensation test after coefficients are identified:

```text
TEMPCOMP,1,25.0,<zero_counts_per_C>,<span_per_C>
```

Zero tracking must stay disabled during loaded or flight-like tests. Only enable it during confirmed unloaded/slack-tether tests:

```text
ZT,1,0.001,0.3
```

Disable it before loaded tests:

```text
ZT,0,0.001,0.3
```
