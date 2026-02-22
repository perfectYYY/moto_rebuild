#include "JY901Driver.h"
#include <Wire.h>
#include <cstring>

JY901Driver::JY901Driver() : config_(Config{}), tcaChannel_(0xFF) {}
JY901Driver::JY901Driver(const Config& config) : config_(config), tcaChannel_(0xFF) {}

void JY901Driver::selectTCAChannel() {
    if (tcaChannel_ != 0xFF) {
        Wire.beginTransmission(0x70);
        Wire.write(1 << tcaChannel_);
        auto result = Wire.endTransmission();
        if (result != 0) {
            LOG_W("JY901", "TCA channel select failed: %d", result);
        }
    }
}

bool JY901Driver::init() {
    Wire.begin();
    
    // I2C 扫描 - 查找 JY901
    LOG_I("JY901", "Scanning I2C bus for JY901...");
    bool found = false;
    uint8_t foundAddr = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            LOG_I("JY901", "Found device at 0x%02X", addr);
            if (addr == config_.i2cAddress) {
                found = true;
                foundAddr = addr;
            }
        }
    }
    
    // 检测传感器是否在线
    Wire.beginTransmission(config_.i2cAddress);
    if (Wire.endTransmission() != 0) {
        LOG_E("JY901", "Sensor not detected at configured address 0x%02X", config_.i2cAddress);
        if (found) {
            LOG_W("JY901", "Found device at different address 0x%02X, please update config!", foundAddr);
        } else {
            LOG_E("JY901", "No I2C devices found! Check wiring (SDA/SCL/VCC/GND)");
        }
        online_ = false;
        return false;
    }
    
    // 配置输出速率
    setSampleFrequency(100.0f);  // 默认 100Hz
    
    // 配置输出内容
    if (config_.useInternalFusion) {
        // 使能姿态角输出
        writeRegister(Registers::RSW, 0x07);  // 使能 Roll/Pitch/Yaw
    }
    
    // 保存配置
    saveConfig();
    
    online_ = true;
    LOG_I("JY901", "Initialized at 0x%02X, %s mode", 
          config_.i2cAddress,
          config_.useInternalFusion ? "fusion" : "raw");
    
    return true;
}

void JY901Driver::setSampleFrequency(float freqHz) {
    uint8_t rate;
    if (freqHz >= 200.0f) {
        rate = 0x0E;  // 200Hz
    } else if (freqHz >= 100.0f) {
        rate = 0x0D;  // 100Hz
    } else if (freqHz >= 50.0f) {
        rate = 0x0C;  // 50Hz
    } else if (freqHz >= 20.0f) {
        rate = 0x0B;  // 20Hz
    } else {
        rate = static_cast<uint8_t>(freqHz / 10.0f);
        if (rate > 10) rate = 10;
    }
    
    writeRegister(Registers::RRATE, rate);
    LOG_I("JY901", "Sample rate set to %d", rate);
}

bool JY901Driver::readRaw(ImuTypes::ImuRawData& data) {
    if (!online_) {
        return false;
    }
    
    // 读取加速度 (AX, AY, AZ) - 每个寄存器 2 字节
    uint8_t buffer[6];
    if (!readBytes(Registers::AX, buffer, 6)) {
        return false;
    }
    
    data.accelX = parseInt16(buffer);
    data.accelY = parseInt16(buffer + 2);
    data.accelZ = parseInt16(buffer + 4);
    
    // 读取陀螺仪
    if (!readBytes(Registers::GX, buffer, 6)) {
        return false;
    }
    
    data.gyroX = parseInt16(buffer);
    data.gyroY = parseInt16(buffer + 2);
    data.gyroZ = parseInt16(buffer + 4);
    
    // 读取磁力计
    if (!readBytes(Registers::HX, buffer, 6)) {
        return false;
    }
    
    data.magX = parseInt16(buffer);
    data.magY = parseInt16(buffer + 2);
    data.magZ = parseInt16(buffer + 4);
    
    // 读取温度
    uint8_t tempBuf[2];
    if (readBytes(Registers::TEMP, tempBuf, 2)) {
        data.temperature = parseInt16(tempBuf);
    }
    
    // 时间戳
    data.timestamp = micros();
    
    return true;
}

ImuTypes::EulerAngles JY901Driver::readAttitude() {
    ImuTypes::EulerAngles angles{0, 0, 0};
    
    if (!online_) {
        return angles;
    }
    
    // 读取 Roll, Pitch, Yaw (每个 2 字节，单位 0.01度)
    uint8_t buffer[6];
    if (readBytes(Registers::ROLL, buffer, 6)) {
        angles.roll = parseInt16(buffer) * 0.01f;
        angles.pitch = parseInt16(buffer + 2) * 0.01f;
        angles.yaw = parseInt16(buffer + 4) * 0.01f;
    }
    
    return angles;
}

bool JY901Driver::writeRegister(uint8_t reg, uint8_t value) {
    return writeByte(reg, value);
}

uint8_t JY901Driver::readRegister(uint8_t reg) {
    uint8_t value = 0;
    readBytes(reg, &value, 1);
    return value;
}

bool JY901Driver::saveConfig() {
    return writeRegister(Registers::SAVE, 0x00);
}

bool JY901Driver::calibrate() {
    // 发送校准命令
    writeRegister(Registers::CALSW, 0x01);  // 加计校准
    delay(100);
    writeRegister(Registers::CALSW, 0x00);  // 停止校准
    
    LOG_I("JY901", "Calibration initiated");
    return true;
}

void JY901Driver::startContinuousRead() {
    // JY901 自动连续输出，无需额外操作
    LOG_I("JY901", "Continuous read mode active");
}

void JY901Driver::stopContinuousRead() {
    // JY901 始终输出，无法真正停止
    // 可以通过设置 RRATE=0 来降低频率
    writeRegister(Registers::RRATE, 0x00);
}

// ==============================================================================
// 私有方法
// ==============================================================================

bool JY901Driver::writeByte(uint8_t reg, uint8_t data) {
    selectTCAChannel();  // 切换 TCA9548A 通道
    Wire.beginTransmission(config_.i2cAddress);
    Wire.write(reg);
    Wire.write(data);
    return Wire.endTransmission() == 0;
}

bool JY901Driver::readBytes(uint8_t reg, uint8_t* buffer, size_t len) {
    selectTCAChannel();  // 切换 TCA9548A 通道
    Wire.beginTransmission(config_.i2cAddress);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) {
        return false;
    }
    
    size_t received = Wire.requestFrom(static_cast<int>(config_.i2cAddress), 
                                        static_cast<int>(len));
    if (received != len) {
        return false;
    }
    
    for (size_t i = 0; i < len; i++) {
        buffer[i] = static_cast<uint8_t>(Wire.read());
    }
    
    return true;
}

int16_t JY901Driver::parseInt16(const uint8_t* data) {
    // 小端字节序
    return static_cast<int16_t>(data[0] | (data[1] << 8));
}

float JY901Driver::parseFloat(const uint8_t* data) {
    // C++ 安全的内存拷贝法
    uint32_t u = (static_cast<uint32_t>(data[0])) |
                 (static_cast<uint32_t>(data[1]) << 8) |
                 (static_cast<uint32_t>(data[2]) << 16) |
                 (static_cast<uint32_t>(data[3]) << 24);
    float f;
    std::memcpy(&f, &u, sizeof(f));
    return f;
}
