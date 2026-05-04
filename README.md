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


### Application to the Project

Use the dynamic response results to justify the selected filter parameter for tethered UAV tension monitoring.

