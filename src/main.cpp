#include <Arduino.h>
#include "tension_sensor.h"

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println();

    tension_init();

    // 初始参数
    tension_set_direction(-1);          // 拉紧显示负值时设为 -1
    tension_set_calibration(10000.0f);  // 改成你的真实标定值
    tension_set_filter_alpha(0.60f);
    tension_set_deadband(0.02f);
    tension_set_average_samples(5);
    tension_set_plot_period_ms(20);
    tension_set_plot_enabled(true);

    Serial.println("# hardware tare start...");
    if (tension_hardware_tare(15)) {
        Serial.println("# hardware tare ok");
    } else {
        Serial.println("# hardware tare failed");
    }

    tension_print_help();
    tension_print_status();
}

void loop()
{
    // 高频更新
    tension_update();

    // 在线串口命令
    tension_serial_process();

    // 调试绘图输出
    tension_plot_update();

    // 这里可以直接放你的控制逻辑
    // if (tension_is_valid()) {
    //     float T = tension_get_force_N();
    //     if (T > 5.0f) {
    //         // do something
    //     }
    // }
}