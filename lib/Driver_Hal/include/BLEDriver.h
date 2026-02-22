#pragma once
#include <cstdint>
#include <array>
#include <functional>
#include "Log.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLECharacteristic.h>

/**
 * @brief BLE 通信驱动
 * @details 提供广播和 GATT 服务
 */
class BLEDriver : public BLEServerCallbacks, public BLECharacteristicCallbacks {
public:
    struct Config {
        const char* deviceName = "MotoTerminal";
        uint16_t broadcastInterval = 100;  // 广播间隔 (ms)
        uint8_t txPower = 3;               // 发射功率
        
        Config() = default;
        Config(const char* name, uint16_t interval, uint8_t power)
            : deviceName(name), broadcastInterval(interval), txPower(power) {}
    };
    
    // 数据回调类型
    using DataCallback = std::function<void(const uint8_t* data, size_t len)>;
    
    BLEDriver();
    explicit BLEDriver(const Config& config);
    
    [[nodiscard]] bool init();
    [[nodiscard]] bool isOnline() const { return online_; }
    
    /**
     * @brief 开始广播
     */
    void startAdvertising();
    
    /**
     * @brief 停止广播
     */
    void stopAdvertising();
    
    /**
     * @brief 发送数据到连接的设备
     */
    bool sendData(const uint8_t* data, size_t len);
    
    /**
     * @brief 设置接收回调
     */
    void onReceive(DataCallback cb) { receiveCallback_ = cb; }
    
    /**
     * @brief 检查是否有设备连接
     */
    [[nodiscard]] bool isConnected() const { return deviceConnected_; }
    
    /**
     * @brief 发送 IMU 数据
     * @param roll 横滚角 (度)
     * @param pitch 俯仰角 (度)
     * @param yaw 航向角 (度)
     * @param speedKmh 速度 (km/h)
     */
    bool sendImuData(float roll, float pitch, float yaw, float speedKmh);
    
    /**
     * @brief 发送系统状态
     * @param heap 空闲内存
     * @param tasks 任务数
     * @param uptime 运行时间(秒)
     */
    bool sendSystemStatus(uint32_t heap, uint32_t tasks, uint32_t uptime);
    
    // BLE 回调
    void onConnect(BLEServer* server) override;
    void onDisconnect(BLEServer* server) override;
    void onWrite(BLECharacteristic* characteristic) override;

private:
    Config config_;
    bool online_{false};
    bool deviceConnected_{false};
    DataCallback receiveCallback_;
    
    BLEServer* server_{nullptr};
    BLECharacteristic* txCharacteristic_{nullptr};
    BLECharacteristic* rxCharacteristic_{nullptr};
    
    // 服务和特征值 UUID
    static constexpr const char* SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b";
    static constexpr const char* TX_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8";  // 通知
    static constexpr const char* RX_UUID = "beb5483f-36e1-4688-b7f5-ea07361b26a8";  // 写入
};
