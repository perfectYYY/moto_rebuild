#include "BE881Driver.h"
#include <cstring>
#include <cstdlib>

BE881Driver::BE881Driver() : config_(Config{}), hwSerial_(nullptr), online_(false), bufferPos_(0) {
    nmeaBuffer_.fill(0);
}
BE881Driver::BE881Driver(const Config& config) : config_(config), hwSerial_(nullptr), online_(false), bufferPos_(0) {
    nmeaBuffer_.fill(0);
}

bool BE881Driver::init() {
    if (hwSerial_ == nullptr) {
        LOG_E("GNSS", "No serial attached");
        return false;
    }
    hwSerial_->begin(config_.baudRate);
    
    online_ = true;
    LOG_I("GNSS", "BE-881 initialized at %d baud", config_.baudRate);
    return true;
}

void BE881Driver::feedData(const uint8_t* data, size_t len) {
    (void)data; (void)len;
    // 从串口读取
    if (hwSerial_ == nullptr) return;
    
    while (hwSerial_->available()) {
        char c = static_cast<char>(hwSerial_->read());
        
        // 查找 NMEA 语句起始
        if (c == '$') {
            bufferPos_ = 0;
            nmeaBuffer_[bufferPos_++] = c;
        }
        else if (bufferPos_ > 0 && bufferPos_ < NMEA_MAX_LEN - 1) {
            nmeaBuffer_[bufferPos_++] = c;
            
            // 语句结束
            if (c == '\n' || c == '\r') {
                nmeaBuffer_[bufferPos_] = '\0';
                processNmeaLine(nmeaBuffer_.data());
                bufferPos_ = 0;
            }
        }
    }
}

void BE881Driver::processNmeaLine(const char* line) {
    // 验证校验和
    if (!verifyChecksum(line)) {
        LOG_W("GNSS", "Checksum error");
        return;
    }
    
    // 解析不同语句类型
    if (strstr(line, "GNGGA") != nullptr || strstr(line, "GPGGA") != nullptr) {
        parseGGA(line);
    }
    else if (strstr(line, "GNRMC") != nullptr || strstr(line, "GPRMC") != nullptr) {
        parseRMC(line);
    }
    else if (strstr(line, "GNVTG") != nullptr || strstr(line, "GPVTG") != nullptr) {
        parseVTG(line);
    }
}

bool BE881Driver::parseGGA(const char* sentence) {
    // 复制到可修改的缓冲区
    char buffer[128];
    strncpy(buffer, sentence, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';
    
    // $GNGGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh
    char* p = strchr(buffer, ',');
    if (p == nullptr) return false;
    p++;  // 跳过时间
    
    // 纬度
    char* comma = strchr(p, ',');
    if (comma == nullptr) return false;
    *comma = '\0';
    char latStr[16];
    strncpy(latStr, p, sizeof(latStr) - 1);
    latStr[sizeof(latStr) - 1] = '\0';
    p = comma + 1;
    
    // 纬度半球
    char latHem = *p;
    p += 2;  // 跳过逗号
    
    // 经度
    comma = strchr(p, ',');
    if (comma == nullptr) return false;
    *comma = '\0';
    char lonStr[16];
    strncpy(lonStr, p, sizeof(lonStr) - 1);
    lonStr[sizeof(lonStr) - 1] = '\0';
    p = comma + 1;
    
    // 经度半球
    char lonHem = *p;
    p += 2;
    
    // 定位状态
    comma = strchr(p, ',');
    if (comma == nullptr) return false;
    *comma = '\0';
    position_.fixType = static_cast<uint8_t>(atoi(p));
    p = comma + 1;
    
    // 卫星数
    comma = strchr(p, ',');
    if (comma == nullptr) return false;
    *comma = '\0';
    position_.satellites = static_cast<uint8_t>(atoi(p));
    p = comma + 1;
    
    // HDOP
    comma = strchr(p, ',');
    if (comma == nullptr) return false;
    *comma = '\0';
    position_.hdop = static_cast<float>(atof(p));
    p = comma + 1;
    
    // 海拔
    comma = strchr(p, ',');
    if (comma == nullptr) return false;
    *comma = '\0';
    position_.altitude = atof(p);
    
    // 转换坐标
    position_.latitude = parseLatitude(latStr, latHem);
    position_.longitude = parseLongitude(lonStr, lonHem);
    position_.timestamp = millis();
    
    return true;
}

bool BE881Driver::parseRMC(const char* sentence) {
    // 复制到可修改的缓冲区
    char buffer[128];
    strncpy(buffer, sentence, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';
    
    // $GNRMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,ddmmyy,x.x,a*hh
    char* p = strchr(buffer, ',');
    if (p == nullptr) return false;
    p++;  // 跳过时间
    
    // 状态 A/V
    char status = *p;
    p += 2;
    
    if (status != 'A') {
        position_.fixType = 0;  // 无效
        return false;
    }
    
    // 跳过纬度和经度 (已在 GGA 中解析)
    for (int i = 0; i < 4; i++) {
        p = strchr(p, ',');
        if (p == nullptr) return false;
        p++;
    }
    
    // 速度
    char* comma = strchr(p, ',');
    if (comma == nullptr) return false;
    *comma = '\0';
    position_.speed = static_cast<float>(atof(p)) * 0.514444f;  // 节 -> m/s
    p = comma + 1;
    
    // 航向
    comma = strchr(p, ',');
    if (comma == nullptr) return false;
    *comma = '\0';
    position_.course = static_cast<float>(atof(p));
    p = comma + 1;
    
    // 日期 ddmmyy
    comma = strchr(p, ',');
    if (comma == nullptr) return false;
    *comma = '\0';
    int date = atoi(p);
    time_.day = date / 10000;
    time_.month = (date / 100) % 100;
    time_.year = (date % 100) + 2000;
    
    return true;
}

bool BE881Driver::parseVTG(const char* sentence) {
    // 复制到可修改的缓冲区
    char buffer[128];
    strncpy(buffer, sentence, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';
    
    // $GNVTG,x.x,T,x.x,M,x.x,N,x.x,K*hh
    char* p = strchr(buffer, ',');
    if (p == nullptr) return false;
    p++;
    
    // 航向
    char* comma = strchr(p, ',');
    if (comma == nullptr) return false;
    *comma = '\0';
    position_.course = static_cast<float>(atof(p));
    
    return true;
}

// ==============================================================================
// 工具函数
// ==============================================================================

double BE881Driver::parseLatitude(const char* str, char hem) {
    double val = parseDecimal(str);
    int degrees = static_cast<int>(val / 100);
    double minutes = val - degrees * 100;
    double result = degrees + minutes / 60.0;
    return (hem == 'S') ? -result : result;
}

double BE881Driver::parseLongitude(const char* str, char hem) {
    double val = parseDecimal(str);
    int degrees = static_cast<int>(val / 100);
    double minutes = val - degrees * 100;
    double result = degrees + minutes / 60.0;
    return (hem == 'W') ? -result : result;
}

double BE881Driver::parseDecimal(const char* str) {
    return atof(str);
}

bool BE881Driver::verifyChecksum(const char* sentence) {
    const char* asterisk = strchr(sentence, '*');
    if (asterisk == nullptr) return false;
    
    uint8_t calcChecksum = 0;
    for (const char* p = sentence + 1; p < asterisk; p++) {
        calcChecksum ^= static_cast<uint8_t>(*p);
    }
    
    uint8_t sentChecksum = static_cast<uint8_t>(parseHex(asterisk[1]) << 4 | 
                                                  parseHex(asterisk[2]));
    return calcChecksum == sentChecksum;
}

uint8_t BE881Driver::parseHex(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return 0;
}
