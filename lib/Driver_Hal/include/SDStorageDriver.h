#pragma once
#include "IStorageService.h"
#include <SD_MMC.h>
#include <FS.h>

/**
 * @brief SD 卡存储驱动
 * @details 使用 ESP32 SD_MMC 接口
 */
class SDStorageDriver : public IStorageService {
public:
    struct Config {
        uint8_t clkPin = 47;      // SD 卡时钟引脚 (与旧代码一致)
        uint8_t cmdPin = 48;      // SD 卡命令引脚 (与旧代码一致)
        uint8_t d0Pin = 21;       // SD 卡数据引脚 (与旧代码一致)
        bool use1BitMode = true;  // 使用 1-bit 模式 (与旧代码一致)
        
        Config() = default;
        Config(uint8_t clk, uint8_t cmd, uint8_t d0, bool bit1)
            : clkPin(clk), cmdPin(cmd), d0Pin(d0), use1BitMode(bit1) {}
    };
    
    SDStorageDriver();
    explicit SDStorageDriver(const Config& config);
    
    // IStorageService 接口
    [[nodiscard]] bool init() override;
    [[nodiscard]] bool writeRecord(const DataRecord& record) override;
    [[nodiscard]] bool writeRecords(const DataRecord* records, size_t count) override;
    void sync() override;
    [[nodiscard]] uint32_t getFreeSpaceMB() const override;
    [[nodiscard]] bool isOnline() const override { return online_; }
    bool createLogFile(uint32_t timestamp) override;
    void closeFile() override;

private:
    Config config_;
    bool online_{false};
    File currentFile_;
    
    static constexpr size_t BUFFER_SIZE = 4096;  // 4KB 缓冲区
    std::array<uint8_t, BUFFER_SIZE> buffer_;
    size_t bufferPos_{0};
    uint32_t sequenceCounter_{0};
    
    // 将记录序列化为二进制格式
    void serializeRecord(const DataRecord& record, uint8_t* out);
    
    // 刷新缓冲区到文件
    bool flushBuffer();
};
