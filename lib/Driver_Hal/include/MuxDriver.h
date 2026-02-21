#pragma once
#include <Arduino.h>

/**
 * @brief CD4052 多路复用器驱动 (纯静态零成本)
 */
class MuxDriver {
public:
    // 定义通道别名，增加代码可读性
    enum class Channel : uint8_t {
        LeftGlove  = 0, // 对应 S1=0 (假设)
        RightGlove = 1, // 对应 S1=1
        None       = 0xFF
    };

    /**
     * @brief 初始化 Mux 引脚
     * @param pin_s1 选择引脚 GPIO 编号
     */
    static void init(int pin_s1) {
        pinMode(pin_s1, OUTPUT);
        digitalWrite(pin_s1, LOW); // 默认左手
    }

    /**
     * @brief 极速切换通道 (C++17 强制内联)
     */
    __attribute__((always_inline)) 
    static inline void select(int pin_s1, Channel ch) {
        if (ch == Channel::LeftGlove) {
            // 直接操作寄存器会更快，但为了兼容性先用 digitalWrite
            // 编译器优化级别 -O2 会将其优化得足够快
            digitalWrite(pin_s1, LOW);
        } else if (ch == Channel::RightGlove) {
            digitalWrite(pin_s1, HIGH);
        }
    }
};