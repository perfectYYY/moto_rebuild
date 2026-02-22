#include "BLEDriver.h"
#include <BLE2902.h>

BLEDriver::BLEDriver() : config_(Config{}), online_(false), deviceConnected_(false), server_(nullptr), txCharacteristic_(nullptr), rxCharacteristic_(nullptr) {}
BLEDriver::BLEDriver(const Config& config) : config_(config), online_(false), deviceConnected_(false), server_(nullptr), txCharacteristic_(nullptr), rxCharacteristic_(nullptr) {}

bool BLEDriver::init() {
    BLEDevice::init(config_.deviceName);
    BLEDevice::setPower(static_cast<esp_power_level_t>(config_.txPower));
    
    // 创建服务器
    server_ = BLEDevice::createServer();
    server_->setCallbacks(this);
    
    // 创建服务
    BLEService* service = server_->createService(SERVICE_UUID);
    
    // 创建 TX 特征 (通知)
    txCharacteristic_ = service->createCharacteristic(
        TX_UUID,
        BLECharacteristic::PROPERTY_NOTIFY
    );
    // 使用静态分配代替 new
    static BLE2902 ble2902;
    txCharacteristic_->addDescriptor(&ble2902);
    
    // 创建 RX 特征 (写入)
    rxCharacteristic_ = service->createCharacteristic(
        RX_UUID,
        BLECharacteristic::PROPERTY_WRITE
    );
    rxCharacteristic_->setCallbacks(this);
    
    service->start();
    
    online_ = true;
    LOG_I("BLE", "Initialized, device name: %s", config_.deviceName);
    return true;
}

void BLEDriver::startAdvertising() {
    if (!online_) return;
    
    BLEAdvertising* advertising = BLEDevice::getAdvertising();
    advertising->addServiceUUID(SERVICE_UUID);
    advertising->setScanResponse(true);
    advertising->setMinPreferred(0x06);
    advertising->setMinPreferred(0x12);
    
    BLEDevice::startAdvertising();
    LOG_I("BLE", "Started advertising");
}

void BLEDriver::stopAdvertising() {
    if (!online_) return;
    
    BLEDevice::stopAdvertising();
    LOG_I("BLE", "Stopped advertising");
}

bool BLEDriver::sendData(const uint8_t* data, size_t len) {
    if (!deviceConnected_ || txCharacteristic_ == nullptr) {
        return false;
    }
    
    txCharacteristic_->setValue(const_cast<uint8_t*>(data), len);
    txCharacteristic_->notify();
    return true;
}

void BLEDriver::onConnect(BLEServer* server) {
    deviceConnected_ = true;
    LOG_I("BLE", "Device connected");
}

void BLEDriver::onDisconnect(BLEServer* server) {
    deviceConnected_ = false;
    LOG_I("BLE", "Device disconnected");
    
    // 重新广播
    delay(500);
    server->startAdvertising();
}

void BLEDriver::onWrite(BLECharacteristic* characteristic) {
    if (receiveCallback_ && characteristic == rxCharacteristic_) {
        std::string value = rxCharacteristic_->getValue();
        receiveCallback_(reinterpret_cast<const uint8_t*>(value.data()), value.length());
    }
}

// ==============================================================================
// IMU 数据发送 (JSON 格式，便于手机解析)
// ==============================================================================

bool BLEDriver::sendImuData(float roll, float pitch, float yaw, float speedKmh) {
    if (!deviceConnected_) {
        return false;
    }
    
    // 构造 JSON 格式数据
    char buffer[128];
    snprintf(buffer, sizeof(buffer),
             "{\"type\":\"imu\",\"r\":%.2f,\"p\":%.2f,\"y\":%.2f,\"s\":%.2f}",
             roll, pitch, yaw, speedKmh);

    txCharacteristic_->setValue(reinterpret_cast<uint8_t*>(buffer), strlen(buffer));
    txCharacteristic_->notify();
    return true;
}

bool BLEDriver::sendSystemStatus(uint32_t heap, uint32_t tasks, uint32_t uptime) {
    if (!deviceConnected_) {
        return false;
    }

    // 构造 JSON 格式数据
    char buffer[128];
    snprintf(buffer, sizeof(buffer),
             "{\"type\":\"status\",\"heap\":%lu,\"tasks\":%lu,\"up\":%lu}",
             heap, tasks, uptime);

    txCharacteristic_->setValue(reinterpret_cast<uint8_t*>(buffer), strlen(buffer));
    txCharacteristic_->notify();
    return true;
}
