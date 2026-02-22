#pragma once
#include <cstdint>
#include <cstddef>
#include <array>
#include <functional>
#include "Log.h"

/**
 * @brief 数据存储服务接口
 * @details 抽象 SD 卡/U 盘/Flash 等存储介质
 */
class IStorageService {
public:
    virtual ~IStorageService() = default;
    
    struct DataRecord {
        uint32_t timestamp;         // 时间戳 (ms)
        uint32_t sequenceNum;       // 序列号
        
        // IMU 数据
        float roll;
        float pitch;
        float yaw;
        float speedKmh;
        float accelX, accelY, accelZ;
        
        // 手套数据
        uint8_t gloveSide;          // 'L' or 'R'
        uint8_t gloveTemp;
        uint16_t gloveCurrent;
        
        // 传感器数据
        float temperature;
        float humidity;
        float pressure;
        
        // GNSS 数据
        double latitude;
        double longitude;
        float altitude;
        uint8_t satelliteCount;
        
        // 预留字段
        std::array<uint8_t, 16> reserved{};
    };
    static_assert(sizeof(DataRecord) == 96, "DataRecord size should be 96 bytes");
    
    /**
     * @brief 初始化存储介质
     */
    [[nodiscard]] virtual bool init() = 0;
    
    /**
     * @brief 写入一条记录
     */
    [[nodiscard]] virtual bool writeRecord(const DataRecord& record) = 0;
    
    /**
     * @brief 批量写入 (优化性能)
     */
    [[nodiscard]] virtual bool writeRecords(const DataRecord* records, size_t count) = 0;
    
    /**
     * @brief 同步缓冲区到介质
     */
    virtual void sync() = 0;
    
    /**
     * @brief 获取可用空间 (MB)
     */
    [[nodiscard]] virtual uint32_t getFreeSpaceMB() const = 0;
    
    /**
     * @brief 检查存储介质是否在线
     */
    [[nodiscard]] virtual bool isOnline() const = 0;
    
    /**
     * @brief 创建新日志文件 (按日期命名)
     */
    virtual bool createLogFile(uint32_t timestamp) = 0;
    
    /**
     * @brief 关闭当前文件
     */
    virtual void closeFile() = 0;
};
