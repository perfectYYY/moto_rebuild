#include "SDStorageDriver.h"
#include <cstring>
#include <ctime>

SDStorageDriver::SDStorageDriver() : config_(Config{}), bufferPos_(0), sequenceCounter_(0) {
    buffer_.fill(0);
}
SDStorageDriver::SDStorageDriver(const Config& config) : config_(config), bufferPos_(0), sequenceCounter_(0) {
    buffer_.fill(0);
}

bool SDStorageDriver::init() {
    // 配置 SD_MMC 引脚
    SD_MMC.setPins(config_.clkPin, config_.cmdPin, config_.d0Pin);
    
    // 初始化 SD 卡
    bool success = false;
    if (config_.use1BitMode) {
        success = SD_MMC.begin("/sdcard", true);  // 1-bit 模式
    } else {
        success = SD_MMC.begin("/sdcard");  // 4-bit 模式
    }
    
    if (!success) {
        LOG_E("SD", "SD card mount failed");
        online_ = false;
        return false;
    }
    
    uint8_t cardType = SD_MMC.cardType();
    if (cardType == CARD_NONE) {
        LOG_E("SD", "No SD card detected");
        online_ = false;
        return false;
    }
    
    const char* typeStr = "UNKNOWN";
    if (cardType == CARD_MMC) typeStr = "MMC";
    else if (cardType == CARD_SD) typeStr = "SD";
    else if (cardType == CARD_SDHC) typeStr = "SDHC";
    
    uint64_t sizeMB = SD_MMC.cardSize() / (1024 * 1024);
    LOG_I("SD", "Card type: %s, Size: %lluMB", typeStr, sizeMB);
    
    online_ = true;
    return true;
}

bool SDStorageDriver::writeRecord(const DataRecord& record) {
    if (!online_) {
        return false;
    }
    
    // 检查缓冲区空间
    if (bufferPos_ + sizeof(DataRecord) > BUFFER_SIZE) {
        if (!flushBuffer()) {
            return false;
        }
    }
    
    // 序列化到缓冲区
    serializeRecord(record, buffer_.data() + bufferPos_);
    bufferPos_ += sizeof(DataRecord);
    
    return true;
}

bool SDStorageDriver::writeRecords(const DataRecord* records, size_t count) {
    for (size_t i = 0; i < count; i++) {
        if (!writeRecord(records[i])) {
            return false;
        }
    }
    return true;
}

void SDStorageDriver::sync() {
    flushBuffer();
    if (currentFile_) {
        currentFile_.flush();
    }
}

uint32_t SDStorageDriver::getFreeSpaceMB() const {
    if (!online_) {
        return 0;
    }
    
    // ESP32 SD_MMC 使用 bytesAvailable() 获取剩余空间
    uint64_t freeBytes = SD_MMC.totalBytes() - SD_MMC.usedBytes();
    return static_cast<uint32_t>(freeBytes / (1024 * 1024));
}

bool SDStorageDriver::createLogFile(uint32_t timestamp) {
    if (!online_) {
        return false;
    }
    
    // 关闭旧文件
    closeFile();
    
    // 按时间戳命名: /sdcard/LOG_xxxxxxxx.bin
    char path[64];
    snprintf(path, sizeof(path), "/sdcard/LOG_%08u.bin", static_cast<unsigned int>(timestamp / 1000));
    
    currentFile_ = SD_MMC.open(path, FILE_WRITE);
    if (!currentFile_) {
        LOG_E("SD", "Failed to create file: %s", path);
        return false;
    }
    
    // 写入文件头
    const char header[] = "MOTO_LOG_v1.0";
    currentFile_.write(reinterpret_cast<const uint8_t*>(header), sizeof(header));
    
    LOG_I("SD", "Log file created: %s", path);
    sequenceCounter_ = 0;
    return true;
}

void SDStorageDriver::closeFile() {
    flushBuffer();
    if (currentFile_) {
        currentFile_.close();
        LOG_I("SD", "Log file closed");
    }
}

// ==============================================================================
// 私有方法
// ==============================================================================

void SDStorageDriver::serializeRecord(const DataRecord& record, uint8_t* out) {
    // 小端字节序序列化
    size_t pos = 0;
    
    auto writeU32 = [&](uint32_t v) {
        out[pos++] = static_cast<uint8_t>(v);
        out[pos++] = static_cast<uint8_t>(v >> 8);
        out[pos++] = static_cast<uint8_t>(v >> 16);
        out[pos++] = static_cast<uint8_t>(v >> 24);
    };
    
    auto writeF32 = [&](float v) {
        std::memcpy(&out[pos], &v, 4);
        pos += 4;
    };
    
    auto writeF64 = [&](double v) {
        std::memcpy(&out[pos], &v, 8);
        pos += 8;
    };
    
    writeU32(record.timestamp);
    writeU32(sequenceCounter_++);
    writeF32(record.roll);
    writeF32(record.pitch);
    writeF32(record.yaw);
    writeF32(record.speedKmh);
    writeF32(record.accelX);
    writeF32(record.accelY);
    writeF32(record.accelZ);
    out[pos++] = record.gloveSide;
    out[pos++] = record.gloveTemp;
    out[pos++] = static_cast<uint8_t>(record.gloveCurrent & 0xFF);
    out[pos++] = static_cast<uint8_t>(record.gloveCurrent >> 8);
    writeF32(record.temperature);
    writeF32(record.humidity);
    writeF32(record.pressure);
    writeF64(record.latitude);
    writeF64(record.longitude);
    writeF32(record.altitude);
    out[pos++] = record.satelliteCount;
    
    // 填充预留字段
    std::memcpy(&out[pos], record.reserved.data(), record.reserved.size());
}

bool SDStorageDriver::flushBuffer() {
    if (bufferPos_ == 0 || !currentFile_) {
        return true;
    }
    
    size_t written = currentFile_.write(buffer_.data(), bufferPos_);
    if (written != bufferPos_) {
        LOG_E("SD", "Write failed: %d/%d bytes", written, bufferPos_);
        return false;
    }
    
    bufferPos_ = 0;
    return true;
}
