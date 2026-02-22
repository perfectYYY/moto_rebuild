#pragma once
#include <cstdint>
#include <array>
#include "Log.h"

/**
 * @brief TCA9548A I2C 多路复用器驱动
 * @details 8 通道 I2C 开关，用于连接多个同地址 I2C 设备
 */
class I2CMuxDriver {
public:
    // 通道枚举 (强类型)
    enum class Channel : uint8_t {
        Ch0 = 0,
        Ch1 = 1,
        Ch2 = 2,
        Ch3 = 3,
        Ch4 = 4,
        Ch5 = 5,
        Ch6 = 6,
        Ch7 = 7,
        None = 0xFF  // 所有通道关闭
    };
    
    explicit I2CMuxDriver(uint8_t i2cAddress = 0x70);
    
    /**
     * @brief 初始化
     */
    [[nodiscard]] bool init();
    
    /**
     * @brief 选择通道 (同一时间只能选一个)
     * @param ch 通道 (0-7 或 None)
     */
    void selectChannel(Channel ch);
    
    /**
     * @brief 获取当前通道
     */
    [[nodiscard]] Channel getCurrentChannel() const { return currentChannel_; }
    
    /**
     * @brief 禁用所有通道
     */
    void disableAll();
    
    /**
     * @brief 扫描所有通道上的设备
     */
    void scanAllChannels();

private:
    uint8_t address_;
    Channel currentChannel_{Channel::None};
    
    // 查找表：Channel -> 控制字节
    static constexpr std::array<uint8_t, 8> channelMask_ = {
        0x01, 0x02, 0x04, 0x08,  // Ch0-Ch3
        0x10, 0x20, 0x40, 0x80   // Ch4-Ch7
    };
};
