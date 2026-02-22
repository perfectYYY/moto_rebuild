#include "AS7341Driver.h"
#include <Wire.h>

AS7341Driver::AS7341Driver(uint8_t i2cAddress) : address_(i2cAddress) {}

bool AS7341Driver::init() {
    Wire.begin();
    
    // 检测设备
    uint8_t id = getDeviceId();
    if (id != 0x24) {  // AS7341 默认 ID
        LOG_E("AS7341", "Device not found, ID=0x%02X", id);
        online_ = false;
        return false;
    }
    
    // 配置积分时间 (100ms)
    setIntegrationTime(36);  // 36 * 2.78ms ≈ 100ms
    
    // 禁用 LED
    setLedCurrent(0);
    
    // 启用光谱测量
    enableSpectralMeasurement(true);
    
    online_ = true;
    LOG_I("AS7341", "Initialized, ID=0x%02X", id);
    return true;
}

void AS7341Driver::setMode(Mode mode) {
    currentMode_ = mode;
    writeReg(Registers::CONFIG, static_cast<uint8_t>(mode));
}

void AS7341Driver::setIntegrationTime(uint8_t atime) {
    writeReg(Registers::ATIME, atime);
}

void AS7341Driver::setLedCurrent(uint8_t current) {
    if (current == 0) {
        writeReg(Registers::LED, 0x00);  // 关闭 LED
    } else {
        writeReg(Registers::LED, 0x80 | (current & 0x7F));  // 使能 + 电流值
    }
}

void AS7341Driver::startMeasurement() {
    if (currentMode_ == Mode::Spm) {
        // 触发单次测量
        uint8_t enable = readReg(Registers::ENABLE);
        writeReg(Registers::ENABLE, enable | 0x02);  // 设置 SPEREN
    }
}

bool AS7341Driver::isDataReady() {
    uint8_t status = readReg(Registers::STATUS);
    return (status & 0x01) != 0;  // AVALID bit
}

bool AS7341Driver::readData(SpectrumData& data) {
    if (!online_) {
        return false;
    }
    
    // 非阻塞检查：如果数据未就绪，立即返回 false
    // 业务层需要持续轮询
    if (!isDataReady()) {
        return false;
    }
    
    // 读取 6 个通道 (F1-F4 + Clear + NIR)
    // 注意：AS7341 使用 SMUX 配置来选择通道组
    // 这里简化处理，实际应根据配置读取
    
    data.f1_415nm = readReg16(Registers::CH0_DATA_L);
    data.f2_445nm = readReg16(Registers::CH0_DATA_L + 2);
    data.f3_480nm = readReg16(Registers::CH0_DATA_L + 4);
    data.f4_515nm = readReg16(Registers::CH0_DATA_L + 6);
    data.f5_555nm = readReg16(Registers::CH0_DATA_L + 8);
    data.f6_590nm = readReg16(Registers::CH0_DATA_L + 10);
    data.f7_630nm = 0;  // 需要切换 SMUX
    data.f8_680nm = 0;
    data.clear = 0;
    data.nir = 0;
    data.flicker = 0;
    data.timestamp = millis();
    
    return true;
}

uint8_t AS7341Driver::getDeviceId() {
    return readReg(Registers::ID);
}

// ==============================================================================
// 私有方法
// ==============================================================================

bool AS7341Driver::writeReg(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(address_);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

uint8_t AS7341Driver::readReg(uint8_t reg) {
    Wire.beginTransmission(address_);
    Wire.write(reg);
    Wire.endTransmission(false);
    
    Wire.requestFrom(static_cast<int>(address_), 1);
    if (Wire.available()) {
        return static_cast<uint8_t>(Wire.read());
    }
    return 0;
}

uint16_t AS7341Driver::readReg16(uint8_t lowReg) {
    uint16_t low = readReg(lowReg);
    uint16_t high = readReg(lowReg + 1);
    return low | (high << 8);
}

bool AS7341Driver::enableSpectralMeasurement(bool enable) {
    uint8_t val = enable ? 0x01 : 0x00;
    return writeReg(Registers::ENABLE, val);
}
