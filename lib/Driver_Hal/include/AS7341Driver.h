#pragma once
#include <cstdint>
#include <array>
#include "Log.h"

/**
 * @brief AS7341 光谱传感器驱动
 * @details 11 通道可见光光谱检测 (415nm-945nm)
 */
class AS7341Driver {
public:
    // I2C 寄存器地址
    struct Registers {
        static constexpr uint8_t ENABLE = 0x80;
        static constexpr uint8_t ATIME = 0x81;
        static constexpr uint8_t WTIME = 0x83;
        static constexpr uint8_t AILTL = 0x84;
        static constexpr uint8_t AILTH = 0x85;
        static constexpr uint8_t AIHTL = 0x86;
        static constexpr uint8_t AIHTH = 0x87;
        static constexpr uint8_t AUXID = 0x90;
        static constexpr uint8_t REVID = 0x91;
        static constexpr uint8_t ID = 0x92;
        static constexpr uint8_t STATUS = 0x93;
        static constexpr uint8_t ASTATUS = 0x94;
        static constexpr uint8_t CH0_DATA_L = 0x95;
        static constexpr uint8_t CONFIG = 0x70;
        static constexpr uint8_t LED = 0x74;
    };
    
    // 光谱通道数据结构
    struct SpectrumData {
        uint16_t f1_415nm;   // 紫光
        uint16_t f2_445nm;   // 蓝光
        uint16_t f3_480nm;   // 浅蓝
        uint16_t f4_515nm;   // 绿光
        uint16_t f5_555nm;   // 黄绿
        uint16_t f6_590nm;   // 黄光
        uint16_t f7_630nm;   // 橙光
        uint16_t f8_680nm;   // 红光
        uint16_t clear;      //  Clear
        uint16_t nir;        // 近红外
        uint16_t flicker;    // 闪烁检测
        uint32_t timestamp;
    };
    
    // 测量模式
    enum class Mode : uint8_t {
        Spm = 0x00,  // 单次测量
        Syns = 0x01, // 同步测量
        Syd = 0x03   // 连续测量
    };
    
    explicit AS7341Driver(uint8_t i2cAddress = 0x39);
    
    [[nodiscard]] bool init();
    [[nodiscard]] bool isOnline() const { return online_; }
    
    /**
     * @brief 设置测量模式
     */
    void setMode(Mode mode);
    
    /**
     * @brief 设置积分时间 (1-255, 每单位 2.78ms)
     */
    void setIntegrationTime(uint8_t atime);
    
    /**
     * @brief 设置 LED 电流 (0-255, 0=关闭)
     */
    void setLedCurrent(uint8_t current);
    
    /**
     * @brief 启动单次测量
     */
    void startMeasurement();
    
    /**
     * @brief 检查测量是否完成
     */
    [[nodiscard]] bool isDataReady();
    
    /**
     * @brief 读取光谱数据
     */
    [[nodiscard]] bool readData(SpectrumData& data);
    
    /**
     * @brief 获取设备 ID
     */
    [[nodiscard]] uint8_t getDeviceId();

private:
    uint8_t address_;
    bool online_{false};
    Mode currentMode_{Mode::Spm};
    
    [[nodiscard]] bool writeReg(uint8_t reg, uint8_t value);
    [[nodiscard]] uint8_t readReg(uint8_t reg);
    [[nodiscard]] uint16_t readReg16(uint8_t lowReg);
    [[nodiscard]] bool enableSpectralMeasurement(bool enable);
};
