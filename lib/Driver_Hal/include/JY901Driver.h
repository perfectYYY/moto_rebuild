#pragma once
#include <cstdint>
#include <array>
#include "IImuDriver.h"
#include "Log.h"

/**
 * @brief JY901 九轴传感器驱动
 * @details 通过 I2C/Serial 读取 JY901 的姿态和解算数据
 * 
 * 支持两种数据获取模式：
 * 1. 直接读取姿态角 (Roll/Pitch/Yaw) - 推荐，使用 JY901 内部 9轴融合
 * 2. 读取原始传感器数据 - 用于本地解算
 */
class JY901Driver : public IImuDriver {
public:
    // JY901 寄存器定义
    struct Registers {
        static constexpr uint8_t SAVE = 0x00;
        static constexpr uint8_t CALSW = 0x01;
        static constexpr uint8_t RSW = 0x02;
        static constexpr uint8_t RRATE = 0x03;
        static constexpr uint8_t BAUD = 0x04;
        
        // 数据输出寄存器 (只读)
        static constexpr uint8_t AX = 0x34;   // 加速度 X
        static constexpr uint8_t AY = 0x35;   // 加速度 Y
        static constexpr uint8_t AZ = 0x36;   // 加速度 Z
        static constexpr uint8_t GX = 0x37;   // 陀螺仪 X
        static constexpr uint8_t GY = 0x38;   // 陀螺仪 Y
        static constexpr uint8_t GZ = 0x39;   // 陀螺仪 Z
        static constexpr uint8_t HX = 0x3A;   // 磁力计 X
        static constexpr uint8_t HY = 0x3B;   // 磁力计 Y
        static constexpr uint8_t HZ = 0x3C;   // 磁力计 Z
        static constexpr uint8_t ROLL = 0x3D; // 横滚角
        static constexpr uint8_t PITCH = 0x3E;// 俯仰角
        static constexpr uint8_t YAW = 0x3F;  // 航向角
        static constexpr uint8_t TEMP = 0x40; // 温度
    };
    
    // 配置结构
    struct Config {
        uint8_t i2cAddress = 0x50;     // 默认 I2C 地址
        uint8_t updateRate = 10;        // 输出频率 (0-10对应0.1-10Hz, 0x0B=20Hz, 0x0C=50Hz, 0x0D=100Hz, 0x0E=200Hz)
        bool useInternalFusion = true;  // 使用内部姿态解算
        
        Config() = default;
        Config(uint8_t addr, uint8_t rate, bool fusion) 
            : i2cAddress(addr), updateRate(rate), useInternalFusion(fusion) {}
    };
    
    JY901Driver();
    explicit JY901Driver(const Config& config);
    
    // IImuDriver 接口实现
    [[nodiscard]] bool init() override;
    [[nodiscard]] bool isOnline() const override { return online_; }
    [[nodiscard]] bool readRaw(ImuTypes::ImuRawData& data) override;
    void setSampleFrequency(float freqHz) override;
    [[nodiscard]] const char* getSensorName() const override { return "JY901"; }
    void startContinuousRead() override;
    void stopContinuousRead() override;
    
    // JY901 特有接口
    
    /**
     * @brief 读取 JY901 内部解算的姿态角
     * @return 欧拉角 (度)
     */
    [[nodiscard]] ImuTypes::EulerAngles readAttitude();
    
    /**
     * @brief 写入寄存器
     */
    bool writeRegister(uint8_t reg, uint8_t value);
    
    /**
     * @brief 读取寄存器
     */
    [[nodiscard]] uint8_t readRegister(uint8_t reg);
    
    /**
     * @brief 保存配置到 Flash
     */
    bool saveConfig();
    
    /**
     * @brief 校准传感器
     */
    bool calibrate();
    
    /**
     * @brief 设置 TCA9548A I2C 多路复用器通道
     * @param channel 通道号 (0-7), 0xFF 表示禁用
     */
    void setTCA9548AChannel(uint8_t channel) { tcaChannel_ = channel; }

private:
    Config config_;
    bool online_{false};
    uint8_t tcaChannel_ = 0xFF;  // TCA9548A 通道, 0xFF 表示不使用
    
    // 切换到 TCA9548A 通道
    void selectTCAChannel();
    
    // I2C 读写
    [[nodiscard]] bool writeByte(uint8_t reg, uint8_t data);
    [[nodiscard]] bool readBytes(uint8_t reg, uint8_t* buffer, size_t len);
    
    // 数据转换辅助
    [[nodiscard]] static int16_t parseInt16(const uint8_t* data);
    [[nodiscard]] static float parseFloat(const uint8_t* data);
};
