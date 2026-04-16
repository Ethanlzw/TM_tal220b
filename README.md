# TM_tal220b
Tension Measurement, Nucleo_g431rb

## version 1.0 

支持方向修正

支持硬件去皮

支持装机后软件置零

支持快速高频更新

## version 1.1 

loadcell 文件集成到 tension_sensor

## 更新支持在线命令
tare

tare 20

zero

clearzero

alpha 0.6

dir -1

deadband 0.02

avg 1

cal 10000

period 20

plot on

plot off

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


