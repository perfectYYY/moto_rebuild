#pragma once
#include <stdint.h>
#include <stddef.h>

// 命名空间封装，防止污染全局作用域
namespace Protocol {

    // ==============================================================================
    // 1. 协议常量
    // ==============================================================================
    constexpr uint16_t HEAD_MAGIC = 0xAA55; // 帧头魔数

    // 强类型枚举 (C++11/17)，禁止隐式转换
    enum class FrameType : uint8_t {
        StatusReport   = 0x01, // 手套 -> 终端 (上报)
        ControlCommand = 0x02, // 终端 -> 手套 (下发)
        Unknown        = 0xFF
    };

    // ==============================================================================
    // 2. 物理帧结构 (Physical Frame Layout)
    // 必须强制 1 字节对齐
    // ==============================================================================
    #pragma pack(push, 1)

    // 通用帧头 (4 Bytes)
    struct FrameHeader {
        uint16_t head;  // HEAD_MAGIC
        uint8_t  type;  // FrameType
        uint8_t  len;   // Payload Length
    };

    // --------------------------------------------------------
    // Type 0x01: 手套状态 Payload (37 Bytes)
    // --------------------------------------------------------
    struct GloveStatusPayload {
        uint8_t  side_id;           // 'L' or 'R'
        uint8_t  target_temp;       // 设定温度
        int16_t  current_temps[7];  // 7路当前温度 (原始值 * 100)
        uint16_t pwm_duty[7];       // 7路占空比 (0-1000)
        uint16_t voltage_mv;        // INA226 电压
        uint16_t current_ma;        // INA226 电流
        uint16_t power_mw;          // INA226 功率
        uint8_t  status_flags;      // 编码器及错误状态
    };

    // --------------------------------------------------------
    // Type 0x02: 控制指令 Payload (16 Bytes)
    // --------------------------------------------------------
    struct GloveControlPayload {
        uint8_t  update_mask;       // Bit0-6: PID更新, Bit7: 强制温度
        uint8_t  forced_temp;       // 强制设定温度
        uint8_t  pid_kp[7];         // Kp 参数
        uint8_t  power_limit[7];    // 功率限制
    };

    #pragma pack(pop)

    // ==============================================================================
    // 3. 编译期断言 (Static Assert)
    // 如果这些报错，说明编译器没有正确对齐结构体
    // ==============================================================================
    static_assert(sizeof(FrameHeader) == 4, "FrameHeader size error!");
    static_assert(sizeof(GloveStatusPayload) == 37, "GloveStatusPayload size error!");
    static_assert(sizeof(GloveControlPayload) == 16, "GloveControlPayload size error!");

    // ==============================================================================
    // 4. 辅助算法 (Sum8 Checksum)
    // constexpr 允许编译器在编译时进行优化测试
    // ==============================================================================
    [[nodiscard]] constexpr uint8_t calculateChecksum(const uint8_t* data, size_t len) {
        uint8_t sum = 0;
        for (size_t i = 0; i < len; ++i) {
            sum += data[i]; // 无符号数自然溢出
        }
        return sum;
    }

} // namespace Protocol