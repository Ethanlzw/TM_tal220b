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

    // ========= 参数设置 =========
    tension_set_direction(-1);          // 如果拉紧时显示负值，就设为 -1
    tension_set_calibration(10000.0f);  // 改成你的真实标定值
    tension_set_filter_alpha(0.60f);    // 越大响应越快
    tension_set_deadband(0.02f);        // 小抖动归零
    tension_set_average_samples(1);     // 响应优先，建议先 1

    // ========= 初始去皮 =========
    Serial.println("Hardware tare...");
    if (tension_hardware_tare(15)) {
        Serial.println("Tare OK");
    } else {
        Serial.println("Tare failed");
    }

    // 如果装机后已有预紧力，可以在机械稳定后再执行一次：
    // tension_software_zero();
}

void loop()
{
    // 高频调用，不要加 delay
    bool updated = tension_update();

    // 你的控制逻辑可直接在这里调用张力值
    if (updated && tension_is_valid()) {
        float T = tension_get_force_N();

        // 示例：超过阈值做出反应
        if (T > 5.0f) {
            // do something
        }
    }

    // 调试输出低频进行，避免拖慢系统
    uint32_t now = millis();
    if (now - g_last_print_ms >= PRINT_PERIOD_MS) {
        g_last_print_ms = now;

        if (tension_is_valid()) {
            Serial.print(">force_N:");
            Serial.println(tension_get_force_N(), 4);
        }
    }
}