#pragma once
#include <cstdint>
#include <array>
#include "Log.h"

/**
 * @brief GNSS 接收机驱动接口
 * @details 支持 NMEA 协议解析
 */
class IGnssDriver {
public:
    virtual ~IGnssDriver() = default;
    
    struct Position {
        double latitude{0.0};   // 纬度 (度)
        double longitude{0.0};  // 经度 (度)
        double altitude{0.0};   // 海拔 (米)
        float speed{0.0f};      // 速度 (m/s)
        float course{0.0f};     // 航向 (度)
        uint8_t satellites{0};  // 卫星数
        uint8_t fixType{0};     // 定位类型 (0=无效, 1=单点, 2=差分...)
        float hdop{99.9f};      // 水平精度因子
        uint32_t timestamp{0};  // 时间戳
    };
    
    struct Time {
        uint16_t year;
        uint8_t month;
        uint8_t day;
        uint8_t hour;
        uint8_t minute;
        uint8_t second;
    };
    
    [[nodiscard]] virtual bool init() = 0;
    [[nodiscard]] virtual bool isOnline() const = 0;
    
    /**
     * @brief 解析输入数据 (应在串口回调中调用)
     */
    virtual void feedData(const uint8_t* data, size_t len) = 0;
    
    /**
     * @brief 获取最新位置
     */
    [[nodiscard]] virtual Position getPosition() const = 0;
    
    /**
     * @brief 获取 UTC 时间
     */
    [[nodiscard]] virtual Time getTime() const = 0;
    
    /**
     * @brief 检查定位是否有效
     */
    [[nodiscard]] virtual bool hasFix() const = 0;
};
