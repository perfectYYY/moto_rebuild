#include "RS485Driver.h"

RS485Driver::RS485Driver() : config_(Config{}) {}
RS485Driver::RS485Driver(const Config& config) : config_(config) {}

bool RS485Driver::init() {
    if (hwSerial_ == nullptr) {
        LOG_E("RS485", "No serial attached");
        return false;
    }
    hwSerial_->begin(config_.baudRate);
    LOG_I("RS485", "Initialized at %d baud", config_.baudRate);
    return true;
}

bool RS485Driver::startReadAll() {
    if (state_ != State::Idle && state_ != State::Complete && state_ != State::Error) {
        return false;  // 正在处理中
    }
    
    state_ = State::Sending;
    currentParam_ = 0;
    data_ = WeatherData{};
    
    // 发送第一个参数请求
    sendCommand(REG_ADDRESSES[0]);
    sendTime_ = millis();
    state_ = State::Waiting;
    
    return true;
}

void RS485Driver::update() {
    switch (state_) {
        case State::Waiting: {
            // 检查是否超时 (100ms)
            if (millis() - sendTime_ > 100) {
                LOG_W("RS485", "Response timeout");
                state_ = State::Error;
                return;
            }
            
            // 检查是否有足够数据
            if (hwSerial_->available() >= 7) {
                state_ = State::Reading;
            }
            break;
        }
        
        case State::Reading: {
            // 读取响应
            rxLen_ = 0;
            while (hwSerial_->available() && rxLen_ < rxBuffer_.size()) {
                rxBuffer_[rxLen_++] = static_cast<uint8_t>(hwSerial_->read());
            }
            
            // 解析
            if (tryParseResponse()) {
                // 保存当前参数值
                float value = 0.0f;
                // 从响应中解析 (简化处理)
                if (rxLen_ >= 7) {
                    int16_t raw = static_cast<int16_t>((rxBuffer_[3] << 8) | rxBuffer_[4]);
                    value = raw / 10.0f;
                }
                
                // 根据当前参数索引保存
                switch (currentParam_) {
                    case 0: data_.temperature = value; break;
                    case 1: data_.humidity = value; break;
                    case 2: data_.pressure = value; break;
                    case 3: data_.windSpeed = value; break;
                    case 4: data_.windDirection = value; break;
                    case 5: data_.rainfall = value; break;
                }
                
                currentParam_++;
                
                if (currentParam_ >= PARAM_COUNT) {
                    // 所有参数读取完成
                    data_.timestamp = millis();
                    state_ = State::Complete;
                    LOG_D("RS485", "All params read complete");
                } else {
                    // 继续读取下一个参数
                    sendCommand(REG_ADDRESSES[currentParam_]);
                    sendTime_ = millis();
                    state_ = State::Waiting;
                }
            } else {
                LOG_W("RS485", "Parse failed");
                state_ = State::Error;
            }
            break;
        }
        
        default:
            break;
    }
}

void RS485Driver::sendCommand(uint16_t regAddr) {
    if (hwSerial_ == nullptr) return;
    
    // 清空接收缓冲区
    while (hwSerial_->available()) {
        hwSerial_->read();
    }
    
    // 构造 Modbus 请求帧
    uint8_t frame[8];
    frame[0] = config_.deviceAddress;
    frame[1] = 0x03;  // 读取保持寄存器
    frame[2] = static_cast<uint8_t>(regAddr >> 8);
    frame[3] = static_cast<uint8_t>(regAddr & 0xFF);
    frame[4] = 0x00;  // 读取 1 个寄存器
    frame[5] = 0x01;
    
    // 计算 CRC
    uint16_t crc = calculateCRC(frame, 6);
    frame[6] = static_cast<uint8_t>(crc & 0xFF);
    frame[7] = static_cast<uint8_t>(crc >> 8);
    
    hwSerial_->write(frame, 8);
    hwSerial_->flush();
}

bool RS485Driver::tryParseResponse() {
    if (rxLen_ < 7) return false;
    
    // 检查地址和功能码
    if (rxBuffer_[0] != config_.deviceAddress || rxBuffer_[1] != 0x03) {
        return false;
    }
    
    // 验证 CRC
    uint16_t receivedCRC = rxBuffer_[rxLen_ - 2] | (rxBuffer_[rxLen_ - 1] << 8);
    uint16_t calculatedCRC = calculateCRC(rxBuffer_.data(), rxLen_ - 2);
    
    return receivedCRC == calculatedCRC;
}

uint16_t RS485Driver::calculateCRC(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= static_cast<uint16_t>(data[i]);
        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}
