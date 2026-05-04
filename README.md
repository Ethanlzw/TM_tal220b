# TM_tal220b
Tension Measurement, Nucleo_g431rb

## 功能更新

支持方向修正

支持硬件去皮

支持装机后软件置零

支持快速高频更新

## 文件更新

loadcell 文件集成到 tension_sensor

## 更新支持在线命令
tare  硬件去皮。传感器此时应该不受力，或者处于你定义的机械零点状态  

tare 20  硬件去皮。这里的 20 表示去皮时取 20 个样本做平均。

zero  软件置零

clearzero 清除软件零点

alpha 0.6  设置低通滤波系数。程序使用一阶低通滤波

dir -1  设置力的方向（负）

deadband 0.02  设置死区。当最终张力绝对值小于 0.02 N 时，直接输出 0。

avg 1  设置每次读取时的平均采样点数。读若干个样本后取平均，再作为本次结果。

cal 10000  设置标定系数 counts_per_N。calibration 就是每 1 N 对应多少 ADC counts。决定了 ADC 原始值怎么换算成牛顿

period 20  设置绘图输出周期，单位是毫秒

plot on  开启绘图输出。

plot off  关闭绘图输出。

status  查看信息

valid：当前数据是否有效  
raw_adc：HX711 原始 ADC 值
raw_force_N：原始力值  
force_N：最终输出张力  
offset：硬件去皮偏置  
zero_N：软件零点  
calibration：标定系数  
alpha：滤波系数  
deadband_N：死区  
avg_samples：平均采样点数  
direction：方向  
plot_enabled：是否输出绘图数据  
plot_period_ms：绘图输出周期  

help  显示所有可用命令列表


# TAL220B + HX711 Priority Experiment Protocol for Tension Measurement

## 1. Theoretical and Standards Basis

The TAL220B datasheet already provides parameters for temperature effects, creep, hysteresis, repeatability, and non-linearity. These parameters are valid theoretical references for experiment design and error analysis, but they cannot replace system-level testing.

The actual course project measurement system includes not only the TAL220B load cell, but also the HX711 module, STM32 power supply and firmware, wiring, mechanical fixture, tether attachment, and the real test environment. Therefore, the complete system must be tested under the actual installation conditions.

This priority experiment protocol is mainly based on:

- TAL220B datasheet: used to justify temperature coefficient, creep, hysteresis, repeatability, and non-linearity as error sources;
- HX711 datasheet: used to justify ADC sampling rate, gain, input range, and digital output behavior;
- OIML R 60: used as a reference for load-cell calibration, creep, and zero-return testing;
- NIST Handbook 44: used to justify that automatic zero tracking must be limited and must not be applied under real load;
- JCGM 100 / GUM: used for later measurement uncertainty analysis.

The factors with the largest impact on tethered UAV tension measurement:

```text
static calibration
zero drift
constant-load creep
creep recovery / zero return
dynamic response
```

Temperature effects should still be recorded, but for a room-temperature course project they do not need to be treated as independent multi-temperature experiments unless the temperature variation is significant or temperature compensation is required by the project.

## 2. Measurement Model

The raw HX711 output can be modeled as:

```text
raw(t) = zero0
       + k_zero_T * (T(t) - T_ref)
       + counts_per_N(T) * F(t)
       + creep(t)
       + noise(t)
```

where:

```text
raw(t)          raw HX711 ADC counts
zero0          zero-load offset at the reference temperature
k_zero_T       zero temperature coefficient, in counts/degC
T(t)           sensor temperature
T_ref          reference temperature
counts_per_N   calibration slope, in counts/N
F(t)           true tether tension
creep(t)       time-dependent creep term under constant load
noise(t)       random noise and electrical interference
```

The compensated tension estimate is:

```text
zero_offset(T) = zero_ref + k_zero_T * (T - T_ref)

counts_per_N(T) = counts_per_N_ref * (1 + k_span_T * (T - T_ref))

F_comp = direction * (raw - zero_offset(T)) / counts_per_N(T)
```

In the priority experiment protocol, `counts_per_N`, `zero0`, `creep(t)`, and `noise(t)` are the main quantities to be identified. The temperature terms `k_zero_T` and `k_span_T` are not treated as independent multi-temperature test targets, but temperature should still be recorded during each experiment to determine whether temperature variation affects the results.

## 3. Experiment A: 1-Hour Static Calibration, Linearity, Hysteresis, and Repeatability

### Purpose

Determine the actual calibration coefficient:

```text
counts_per_N
```

and estimate:

- linearity error;
- hysteresis;
- repeatability.

### Why This Experiment Is Retained

The calibration coefficient directly determines the force conversion result. If `counts_per_N` is incorrect, all subsequent drift, creep, and dynamic-response analysis will be based on an incorrect scale. Therefore, static calibration has the highest priority.

### Experiment Setup

Use known masses or traceable calibration weights.

Recommended load points:

```text
0 %, 10 %, 20 %, 40 %, 60 %, 80 % of selected safe full-scale load
```

### 1-Hour Loading Plan

A complete loading and unloading cycle can be arranged within one hour:

| Time Window | Operation |
|---:|---|
| 0-5 min | no load, tare, baseline |
| 5-10 min | 10 % load |
| 10-15 min | 20 % load |
| 15-20 min | 40 % load |
| 20-25 min | 60 % load |
| 25-30 min | 80 % load |
| 30-35 min | 60 % load |
| 35-40 min | 40 % load |
| 40-45 min | 20 % load |
| 45-50 min | 10 % load |
| 50-60 min | no-load zero return |

For each load point, use the final 60 s of that time window for averaging.

### Application to the Project

Write the measured calibration coefficient into:

```text
tension_set_calibration(counts_per_N)
```

or into the Simulink / STM32 parameter:

```text
p.countsPerN = measured counts_per_N
```

Hysteresis and repeatability should be treated as measurement uncertainty components rather than ignored.

## 4. Experiment B: 1-Hour Zero Drift Test

### Purpose

Quantify zero-load drift over one hour.

### Why This Experiment Is Retained

Zero drift directly causes non-zero displayed tension under no-load conditions and affects pre-flight zero checks and deadband selection. It is usually easier to observe in a course project than span temperature drift and has a more direct effect on tension decisions.

### Why It Is Needed

The TAL220B datasheet provides temperature coefficient and zero-related specifications, but the actual zero drift also depends on:

- HX711 module;
- STM32 power supply;
- wiring;
- mechanical mounting stress;
- temperature gradient;
- system warm-up behavior.

### Experiment Setup

```text
load condition: no load / slack tether
temperature: stable room temperature, record actual temperature
duration: 1 hour
output period: 100 ms
zero tracking: disabled
temperature compensation: disabled for baseline testing
tare: perform once before the test; do not repeat tare during the test
```

### Application to the Project

This experiment is used to determine:

- system warm-up time;
- pre-flight zero-check threshold;
- deadband parameter;
- whether slow zero tracking is needed during ground idle state.

## 5. Experiment C: 1-Hour Constant-Load Creep Test

### Purpose

Quantify how the sensor output changes with time under a constant load.

### Why This Experiment Is Retained

A tethered UAV may maintain a certain average tether tension for an extended period. The TAL220B datasheet gives a creep specification, but actual system creep is affected by the fixture, attachment geometry, load level, and mechanical stress history. Creep causes slow output change under constant tension, so it is a major error source that must be evaluated.

### Why It Is Needed

Actual system creep depends on:

- load magnitude;
- fixture stiffness;
- attachment geometry;
- mechanical stress history;
- temperature;
- loading method.

### Experiment Setup

Recommended load:

```text
60 %FS
```

Automatic zero tracking must be disabled during the creep test.

### Application to the Project

Automatic creep compensation during flight is not recommended unless the system has an independent method to distinguish creep from real slowly changing tether tension.

The creep result is mainly used for:

- uncertainty budget;
- safety margin;
- limitation statement for long-duration static measurements.

## 6. Experiment D: 1-Hour Creep Recovery / Zero Return Test

### Purpose

Quantify the zero-return error after sustained loading and unloading.

### Why This Experiment Is Retained

If the sensor does not quickly return to its original zero after being loaded, the system may show a zero error after flight or before the next takeoff. This directly affects pre-flight tare, ground zero checks, and the zero-tracking strategy.

### Experiment Setup

This experiment is performed immediately after the constant-load creep test:

- remove the load after the 1-hour loaded creep test;
- keep the sensor unloaded;
- do not perform tare;
- keep zero tracking disabled;
- continue logging for 1 hour.

### Application to the Project

If the zero-return error is significant, the project should:

- perform a post-flight zero check;
- not assume that zero immediately recovers after unloading;
- enable zero tracking only when the tether is confirmed to be slack on the ground.

## 7. Experiment E: Dynamic Response Test

### Purpose

Evaluate whether the system can capture transient tether tension changes.

### Why This Experiment Is Retained

The risk in tethered UAV operation is not only related to average tension, but also to short tension peaks. If filtering is too strong or the output period is too long, the system may fail to capture dangerous tension peaks in time. Therefore, dynamic response testing must be retained.

### Experiment Setup

- Apply a moderate known load;
- Apply small repeatable disturbances to the tether;
- Record using the shortest practical output period:

```text
PERIOD,13
```

- Test different filter coefficients:

```text
ALPHA,0.05
ALPHA,0.1
ALPHA,0.2
ALPHA,0.5
```

### Data Processing

For each filter setting, calculate:

```text
peak_force_N
rise_time_ms
settling_time_ms
delay_ms
noise_std_N
```

### Application to the Project

Use the dynamic response results to justify the selected filter parameter for tethered UAV tension monitoring.

