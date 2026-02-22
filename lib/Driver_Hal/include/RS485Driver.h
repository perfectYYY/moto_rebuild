#pragma once
#include <cstdint>
#include <array>
#include "Log.h"

/**
 * @brief RS485 气象站驱动 (异步非阻塞)
 * @details 通过 Modbus 协议读取气象数据，采用状态机实现
 */
class RS485Driver {
public:
    // 气象数据结构
    struct WeatherData {
        float temperature{0.0f};    // 温度 (°C)
        float humidity{0.0f};       // 湿度 (%)
        float pressure{0.0f};       // 气压 (hPa)
        float windSpeed{0.0f};      // 风速 (m/s)
        float windDirection{0.0f};  // 风向 (度)
        float rainfall{0.0f};       // 雨量 (mm)
        uint32_t timestamp{0};      // 时间戳
    };
    
    struct Config {
        uint8_t deviceAddress = 0x01;  // 设备地址
        uint32_t baudRate = 9600;      // 波特率
        
        Config() = default;
        Config(uint8_t addr, uint32_t baud)
            : deviceAddress(addr), baudRate(baud) {}
    };
    
    enum class State : uint8_t {
        Idle = 0,           // 空闲
        Sending = 1,        // 发送命令中
        Waiting = 2,        // 等待响应
        Reading = 3,        // 读取数据中
        Complete = 4,       // 完成
        Error = 5           // 错误
    };
    
    RS485Driver();
    explicit RS485Driver(const Config& config);
    
    void attachSerial(HardwareSerial& serial) {
        hwSerial_ = &serial;
    }
    
    bool init();
    
    /**
     * @brief 启动读取所有数据 (非阻塞)
     * @return true 启动成功
     */
    bool startReadAll();
    
    /**
     * @brief 轮询更新状态机 (在 Tick 中调用)
     */
    void update();
    
    /**
     * @brief 检查是否完成
     */
    [[nodiscard]] bool isComplete() const { return state_ == State::Complete; }
    
    /**
     * @brief 检查是否出错
     */
    [[nodiscard]] bool hasError() const { return state_ == State::Error; }
    
    /**
     * @brief 获取结果数据
     */
    [[nodiscard]] const WeatherData& getData() const { return data_; }
    
    /**
     * @brief 重置状态机
     */
    void reset() { state_ = State::Idle; }

private:
    Config config_;
    HardwareSerial* hwSerial_ = nullptr;
    State state_{State::Idle};
    WeatherData data_{};
    
    // 内部状态
    uint32_t sendTime_{0};
    uint8_t currentParam_{0};  // 当前读取的参数索引
    std::array<uint8_t, 32> rxBuffer_{};
    size_t rxLen_{0};
    
    // Modbus 参数表
    static constexpr uint8_t PARAM_COUNT = 6;
    static constexpr std::array<uint16_t, PARAM_COUNT> REG_ADDRESSES = {
        0x0000,  // 温度
        0x0001,  // 湿度
        0x0002,  // 气压
        0x0003,  // 风速
        0x0004,  // 风向
        0x0005   // 雨量
    };
    
    [[nodiscard]] uint16_t calculateCRC(const uint8_t* data, size_t len);
    void sendCommand(uint16_t regAddr);
    bool tryParseResponse();
    float getParamValue(uint8_t index);
};
