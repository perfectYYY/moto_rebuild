#pragma once
#include <cstdint>
#include <cstddef>
#include <array>
#include <cmath>

namespace ImuTypes {

// ==============================================================================
// 1. 向量与四元数类型 (自然对齐，适合高速数学运算)
// ==============================================================================

struct Vector3 {
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};
    
    // 移除 constexpr，因为 std::sqrt 在 C++17 中不是 constexpr
    [[nodiscard]] inline float magnitude() const noexcept {
        return std::sqrt(x * x + y * y + z * z);
    }
    
    [[nodiscard]] constexpr Vector3 operator-(const Vector3& other) const noexcept {
        return Vector3{x - other.x, y - other.y, z - other.z};
    }
    
    [[nodiscard]] constexpr Vector3 operator*(float scalar) const noexcept {
        return Vector3{x * scalar, y * scalar, z * scalar};
    }
};

struct Quaternion {
    float w{1.0f};  // 标量部分
    float x{0.0f};  // i
    float y{0.0f};  // j
    float z{0.0f};  // k
    
    [[nodiscard]] constexpr Quaternion conjugate() const noexcept {
        return Quaternion{w, -x, -y, -z};
    }
    
    [[nodiscard]] constexpr Quaternion operator*(const Quaternion& q) const noexcept {
        return Quaternion{
            w * q.w - x * q.x - y * q.y - z * q.z,
            w * q.x + x * q.w + y * q.z - z * q.y,
            w * q.y - x * q.z + y * q.w + z * q.x,
            w * q.z + x * q.y - y * q.x + z * q.w
        };
    }
};

struct EulerAngles {
    float roll{0.0f};   // 横滚角 (度)
    float pitch{0.0f};  // 俯仰角 (度)
    float yaw{0.0f};    // 航向角 (度)
};

// ==============================================================================
// 2. IMU 原始数据包结构 (紧凑排列，用于物理通讯协议)
// ==============================================================================

#pragma pack(push, 1)
struct ImuRawData {
    int16_t accelX;     // 加速度 (原始值)
    int16_t accelY;
    int16_t accelZ;
    int16_t gyroX;      // 陀螺仪 (原始值)
    int16_t gyroY;
    int16_t gyroZ;
    int16_t magX;       // 磁力计 (原始值)
    int16_t magY;
    int16_t magZ;
    int16_t temperature;
    uint32_t timestamp; // 微秒时间戳
};
#pragma pack(pop) // 【关键修正】：尽早 pop，不要影响内部处理结构体

// 编译期断言修正：6 + 6 + 6 + 2 + 4 = 24 字节
static_assert(sizeof(ImuRawData) == 24, "ImuRawData packing error");

// ==============================================================================
// 3. IMU 内部处理数据结构 (利用编译器自然对齐，最高性能)
// ==============================================================================

struct ImuProcessedData {
    Vector3 accel;      // 加速度 (m/s²)
    Vector3 gyro;       // 角速度 (dps)
    Vector3 mag;        // 磁场 (uT)
    Vector3 linearAccel;// 去重力后线性加速度
    Vector3 velocity;   // 速度 (m/s)
    float temperature;
    uint32_t timestamp;
};

// ==============================================================================
// 4. 配置常量
// ==============================================================================

namespace Config {
    constexpr float DEFAULT_SAMPLE_FREQ = 100.0f;  // 默认 100Hz
    constexpr float DEFAULT_BETA = 0.05f;          // Madgwick 增益
    constexpr float GRAVITY = 9.80665f;            // 重力加速度
    constexpr float PI_F = 3.14159265359f;         // 避免与 Arduino PI 宏冲突
    constexpr float DEG_TO_RAD_F = PI_F / 180.0f;
    constexpr float RAD_TO_DEG_F = 180.0f / PI_F;
    constexpr float MAX_SPEED = 60.0f;             // 最大速度限幅 (m/s)
}

} // namespace ImuTypes