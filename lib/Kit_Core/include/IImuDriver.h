#pragma once
#include <cstdint>
#include <cstddef>
#include "ImuTypes.h"

/**
 * @brief IMU 传感器驱动接口
 * @details 抽象 IMU 硬件操作，支持 JY901、MPU9250 等不同传感器
 */
class IImuDriver {
public:
    virtual ~IImuDriver() = default;
    
    /**
     * @brief 初始化传感器
     * @return true 初始化成功
     */
    [[nodiscard]] virtual bool init() = 0;
    
    /**
     * @brief 检查传感器是否在线
     */
    [[nodiscard]] virtual bool isOnline() const = 0;
    
    /**
     * @brief 读取原始数据
     * @param data 输出数据缓冲区
     * @return true 读取成功
     */
    [[nodiscard]] virtual bool readRaw(ImuTypes::ImuRawData& data) = 0;
    
    /**
     * @brief 设置采样频率
     * @param freqHz 频率 (Hz)
     */
    virtual void setSampleFrequency(float freqHz) = 0;
    
    /**
     * @brief 获取传感器ID/型号
     */
    [[nodiscard]] virtual const char* getSensorName() const = 0;
    
    /**
     * @brief 开始连续读取模式 (中断/DMA)
     */
    virtual void startContinuousRead() = 0;
    
    /**
     * @brief 停止连续读取
     */
    virtual void stopContinuousRead() = 0;
};
