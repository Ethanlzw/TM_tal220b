#include <Arduino.h>
#include "tension_sensor.h"

// 调试输出周期
static const uint32_t PRINT_PERIOD_MS = 20;
static uint32_t g_last_print_ms = 0;

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("=== Tension Sensor Demo ===");

    tension_init();

    // 推荐先做一次空载去皮
    Serial.println("Hardware tare...");
    if (tension_hardware_tare(15)) {
        Serial.println("Tare OK");
    } else {
        Serial.println("Tare failed");
    }

    // 参数可根据系统需求调整
    tension_set_calibration(10000.0f);   // 这里改成你的真实标定值
    tension_set_filter_alpha(0.60f);     // 越大响应越快
    tension_set_deadband(0.02f);         // 很小的抖动直接归零
    tension_set_average_samples(1);      // 响应优先，建议先 1

    // 如果装机后已有预紧力，可在机械系统稳定后调用一次
    // tension_software_zero();
}

void loop()
{
    // 高频调用，尽量不要加 delay
    bool updated = tension_update();

    // 其它程序可直接调用当前张力
    if (updated && tension_is_valid()) {
        float tension_N = tension_get_force_N();

        // 这里写你的控制逻辑
        // 例子：超过阈值触发保护
        if (tension_N > 5.0f) {
            // do something
        }
    }

    // 调试打印低频进行，避免拖慢系统
    uint32_t now = millis();
    if (now - g_last_print_ms >= PRINT_PERIOD_MS) {
        g_last_print_ms = now;

        if (tension_is_valid()) {
            TensionData d = tension_get_data();

            // 串口绘图格式，可被 VS Code serial-plotter 直接识别
            Serial.print(">force_N:");
            Serial.println(d.force_N, 4);

            // 如需普通文本调试，可改成下面这种
            // Serial.print("raw_adc=");
            // Serial.print(d.raw_adc);
            // Serial.print(", raw_force_N=");
            // Serial.print(d.raw_force_N, 4);
            // Serial.print(", force_N=");
            // Serial.println(d.force_N, 4);
        }
    }
}