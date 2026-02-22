#include "I2CMuxDriver.h"
#include <Wire.h>

I2CMuxDriver::I2CMuxDriver(uint8_t i2cAddress) : address_(i2cAddress) {}

bool I2CMuxDriver::init() {
    Wire.begin();
    
    // 检测 TCA9548A 是否在线
    Wire.beginTransmission(address_);
    if (Wire.endTransmission() != 0) {
        LOG_E("I2CMux", "TCA9548A not detected at 0x%02X", address_);
        return false;
    }
    
    disableAll();
    LOG_I("I2CMux", "Initialized at 0x%02X", address_);
    return true;
}

void I2CMuxDriver::selectChannel(Channel ch) {
    uint8_t mask = 0x00;
    
    if (ch != Channel::None) {
        uint8_t idx = static_cast<uint8_t>(ch);
        if (idx < 8) {
            mask = channelMask_[idx];
        }
    }
    
    Wire.beginTransmission(address_);
    Wire.write(mask);
    Wire.endTransmission();
    
    currentChannel_ = ch;
    
    if (ch != Channel::None) {
        LOG_D("I2CMux", "Selected channel %d", static_cast<int>(ch));
    }
}

void I2CMuxDriver::disableAll() {
    selectChannel(Channel::None);
}

void I2CMuxDriver::scanAllChannels() {
    LOG_I("I2CMux", "Scanning all channels...");
    
    for (uint8_t ch = 0; ch < 8; ch++) {
        selectChannel(static_cast<Channel>(ch));
        delay(10);
        
        LOG_I("I2CMux", "Channel %d:", ch);
        
        // 扫描 I2C 地址 0x01-0x7F
        for (uint8_t addr = 1; addr < 127; addr++) {
            Wire.beginTransmission(addr);
            if (Wire.endTransmission() == 0) {
                LOG_I("I2CMux", "  Found device at 0x%02X", addr);
            }
        }
    }
    
    disableAll();
}
