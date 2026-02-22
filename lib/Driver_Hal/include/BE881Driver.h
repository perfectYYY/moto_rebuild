#pragma once
#include "IGnssDriver.h"
#include <array>
#include <functional>

/**
 * @brief BE-881 GNSS 模块驱动
 * @details 支持 GPS/GLONASS/北斗多模定位
 */
class BE881Driver : public IGnssDriver {
public:
    struct Config {
        uint32_t baudRate = 115200;
        uint16_t timeoutMs = 1000;
        
        Config() = default;
        Config(uint32_t baud, uint16_t timeout) : baudRate(baud), timeoutMs(timeout) {}
    };
    
    BE881Driver();
    explicit BE881Driver(const Config& config);
    
    void attachSerial(HardwareSerial& serial) {
        hwSerial_ = &serial;
    }
    
    // IGnssDriver 接口
    [[nodiscard]] bool init() override;
    [[nodiscard]] bool isOnline() const override { return online_; }
    void feedData(const uint8_t* data, size_t len) override;
    [[nodiscard]] Position getPosition() const override { return position_; }
    [[nodiscard]] Time getTime() const override { return time_; }
    [[nodiscard]] bool hasFix() const override { return position_.fixType > 0; }

private:
    Config config_;
    HardwareSerial* hwSerial_ = nullptr;
    bool online_{false};
    Position position_{};
    Time time_{};
    
    // NMEA 解析缓冲区
    static constexpr size_t NMEA_MAX_LEN = 128;
    std::array<char, NMEA_MAX_LEN> nmeaBuffer_{};
    size_t bufferPos_{0};
    
    // NMEA 语句解析
    void processNmeaLine(const char* line);
    [[nodiscard]] bool parseGGA(const char* sentence);
    [[nodiscard]] bool parseRMC(const char* sentence);
    [[nodiscard]] bool parseVTG(const char* sentence);
    
    // 工具函数
    [[nodiscard]] static double parseLatitude(const char* str, char hem);
    [[nodiscard]] static double parseLongitude(const char* str, char hem);
    [[nodiscard]] static double parseDecimal(const char* str);
    [[nodiscard]] static uint8_t parseHex(char c);
    [[nodiscard]] static bool verifyChecksum(const char* sentence);
};
