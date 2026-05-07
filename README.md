# STM32 Tether Tension Firmware Description

## 1. Purpose

This firmware is used for tether tension measurement with an STM32 + HX711 + TAL220B system. Starting from the raw HX711 ADC count, the firmware converts the signal to load-cell force, applies low-pass filtering and deadband processing, then applies the pulley geometry correction to estimate the actual tether tension.

The processing chain is:

```text
raw_adc -> F_raw_loadcell_N -> F_filter_loadcell_N -> geometry correction -> T_tether_N
```

## 2. Program Logic

1. Initialize HX711 pins and internal state.
2. Perform one tare operation at startup and store the no-load ADC average as the zero offset.
3. Continuously read the raw HX711 ADC count `raw_adc`.
4. Convert ADC count to load-cell-axis force using the calibration factor:

```text
F_raw_loadcell_N = direction * (raw_adc - tare_offset) / counts_per_N
```

5. Apply a first-order low-pass filter to obtain `F_filter_loadcell_N`.
6. Apply a deadband near zero to reduce the effect of zero-load noise.
7. Convert the filtered load-cell force to actual tether tension using the pulley/load-cell geometry:

```text
T_tether_N = F_filter_loadcell_N / [2 * sin(wrap_angle / 2) * cos(load_axis_angle)]
```

With the current geometry:

```text
geometry_factor = 1.931968
T_tether_N ≈ 0.5176 * F_filter_loadcell_N
```

## 3. Key Parameters

The key parameters are located near the beginning of `src/tension_sensor.cpp`.

| Parameter | Current Value | Description |
|---|---:|---|
| `COUNTS_PER_N_LOADCELL` | `44916.0` | Load-cell calibration factor in counts/N |
| `FORCE_DIRECTION` | `1` | Force sign correction; change to `-1` if loading gives negative force |
| `FILTER_ALPHA` | `0.20` | First-order low-pass coefficient; larger is faster, smaller is smoother |
| `DEADBAND_LOADCELL_N` | `0.02` | Load-cell force deadband for suppressing near-zero noise |
| `WRAP_ANGLE_DEG` | `161.99` | Tether wrap angle in degrees |
| `LOAD_AXIS_ANGLE_DEG` | `12.03` | Angle between resultant force and load-cell sensing axis in degrees |

## 4. CSV Output

The serial CSV output is:

```text
timestamp_ms,raw_adc,F_raw_loadcell_N,F_filter_loadcell_N,T_tether_N,saturated
```

| Column | Meaning |
|---|---|
| `timestamp_ms` | Time since STM32 startup, in ms |
| `raw_adc` | Raw HX711 ADC count |
| `F_raw_loadcell_N` | Unfiltered load-cell-axis force |
| `F_filter_loadcell_N` | Filtered and deadbanded load-cell-axis force |
| `T_tether_N` | Geometry-corrected actual tether tension |
| `saturated` | HX711 full-scale saturation flag; `1` means saturated |

## 5. Notes

`COUNTS_PER_N_LOADCELL` should be determined from a static calibration test. If the calibration is performed directly on the load-cell axis, the geometry correction should be kept. If the complete pulley/tether mechanism is calibrated directly against actual tether tension, the geometry correction should not be applied again.

Runtime parameter setters have been removed. To change calibration, filtering, deadband, or geometry parameters, edit the constants in `src/tension_sensor.cpp` and rebuild/upload the firmware.


