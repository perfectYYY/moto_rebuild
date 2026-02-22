#include "ImuManager.h"
#include <cstring>
#include <algorithm>

// ==============================================================================
// 查找表：FusionMode -> 更新函数 (替代 switch-case)
// ==============================================================================
const std::array<ImuManager::UpdateFunc, 4> ImuManager::updateTable_ = []{
    std::array<UpdateFunc, 4> tbl{};
    tbl.fill(nullptr);
    tbl[static_cast<size_t>(FusionMode::Jy901Direct)] = &ImuManager::updateJy901Direct;
    tbl[static_cast<size_t>(FusionMode::LocalMadgwick)] = &ImuManager::updateLocalMadgwick;
    tbl[static_cast<size_t>(FusionMode::Compare)] = &ImuManager::updateCompare;
    return tbl;
}();

// ==============================================================================
// 构造函数与初始化
// ==============================================================================

ImuManager::ImuManager() : driver_(nullptr), mode_(FusionMode::Jy901Direct), 
    stationaryCount_(0), movingCount_(0), isCalibrated_(false), 
    q_{1,0,0,0}, last_tick_time_(0), time_slot_(0),
    lastErrorLogTime_(0), errorCount_(0) {
    output_ = OutputData{};
    accelBias_ = ImuTypes::Vector3{};
    lpfAccel_ = ImuTypes::Vector3{};
}

void ImuManager::init(IImuDriver* driver) {
    init(driver, Config{});
}

void ImuManager::init(IImuDriver* driver, const Config& config) {
    driver_ = driver;
    config_ = config;
    
    if (driver_ != nullptr) {
        driver_->setSampleFrequency(config_.sampleFreq);
    }
    
    resetVelocity();
    resetPosition();
    
    LOG_I(getModuleName(), "IMU Manager initialized, mode=%d, freq=%.1fHz",
          static_cast<int>(mode_), config_.sampleFreq);
}

void ImuManager::setFusionMode(FusionMode mode) {
    mode_ = mode;
    LOG_I(getModuleName(), "Fusion mode set to %d", static_cast<int>(mode_));
}

// ==============================================================================
// 核心更新循环
// ==============================================================================

ImuManager::OutputData ImuManager::update() {
    if (driver_ == nullptr) {
        LOG_E(getModuleName(), "No driver attached!");
        return output_;
    }
    
    // 使用查找表分发 (替代 switch-case)
    const auto func = updateTable_[static_cast<size_t>(mode_)];
    if (func != nullptr) {
        (this->*func)();
    }
    
    return output_;
}

// ==============================================================================
// Mode 1: 直接使用 JY901 内部姿态 (推荐方案)
// ==============================================================================

void ImuManager::updateJy901Direct() {
    ImuTypes::ImuRawData raw{};
    if (!driver_->readRaw(raw)) {
        // 错误日志节流：每 5 秒只打印一次
        errorCount_++;
        uint32_t now = millis();
        if (now - lastErrorLogTime_ > 5000) {
            LOG_W(getModuleName(), "Failed to read IMU data (%lu errors in 5s)", errorCount_);
            lastErrorLogTime_ = now;
            errorCount_ = 0;
        }
        return;
    }
    
    // 假设 JY901 输出欧拉角在特定寄存器位置
    // 这里需要根据实际 JY901 数据格式解析
    // 临时使用原始数据计算
    
    // 转换原始数据到物理单位
    constexpr float ACCEL_SCALE = 9.80665f / 32768.0f;  // ±2g
    constexpr float GYRO_SCALE = 2000.0f / 32768.0f;    // ±2000dps
    
    ImuTypes::Vector3 accel{
        static_cast<float>(raw.accelX) * ACCEL_SCALE,
        static_cast<float>(raw.accelY) * ACCEL_SCALE,
        static_cast<float>(raw.accelZ) * ACCEL_SCALE
    };
    
    ImuTypes::Vector3 gyro{
        static_cast<float>(raw.gyroX) * GYRO_SCALE,
        static_cast<float>(raw.gyroY) * GYRO_SCALE,
        static_cast<float>(raw.gyroZ) * GYRO_SCALE
    };
    (void)gyro;  // 暂时未使用，避免警告
    
    // 读取 JY901 内部解算的姿态 (假设存储在 raw 的特定字段)
    // 实际实现需要根据 JY901 协议解析
    // 这里临时使用简单计算
    output_.attitude.roll = 0.0f;   // TODO: 从 JY901 读取
    output_.attitude.pitch = 0.0f;
    output_.attitude.yaw = 0.0f;
    
    // 使用姿态去除重力
    ImuTypes::Quaternion q = eulerToQuaternion(output_.attitude);
    output_.linearAccel = removeGravity(accel, q);
    
    // 速度更新
    const float dt = 1.0f / config_.sampleFreq;
    updateVelocity(output_.linearAccel, dt);
    applyZupt();
    
    output_.speedKmh = output_.velocity.magnitude() * 3.6f;
}

// ==============================================================================
// Mode 2: 本地 Madgwick 解算
// ==============================================================================

void ImuManager::updateLocalMadgwick() {
    ImuTypes::ImuRawData raw{};
    if (!driver_->readRaw(raw)) {
        // 错误日志节流
        errorCount_++;
        uint32_t now = millis();
        if (now - lastErrorLogTime_ > 5000) {
            LOG_W(getModuleName(), "Failed to read IMU data (%lu errors in 5s)", errorCount_);
            lastErrorLogTime_ = now;
            errorCount_ = 0;
        }
        return;
    }
    
    // 转换到物理单位
    constexpr float ACCEL_SCALE = 9.80665f / 32768.0f;
    constexpr float GYRO_SCALE = 2000.0f / 32768.0f;
    constexpr float DEG_TO_RAD_F = 3.14159265359f / 180.0f;
    
    ImuTypes::Vector3 accel{
        static_cast<float>(raw.accelX) * ACCEL_SCALE,
        static_cast<float>(raw.accelY) * ACCEL_SCALE,
        static_cast<float>(raw.accelZ) * ACCEL_SCALE
    };
    
    ImuTypes::Vector3 gyro{
        static_cast<float>(raw.gyroX) * GYRO_SCALE * DEG_TO_RAD_F,
        static_cast<float>(raw.gyroY) * GYRO_SCALE * DEG_TO_RAD_F,
        static_cast<float>(raw.gyroZ) * GYRO_SCALE * DEG_TO_RAD_F
    };
    
    const float dt = 1.0f / config_.sampleFreq;
    
    // 执行 Madgwick 更新
    madgwickUpdate(gyro, accel, dt);
    
    // 获取欧拉角
    output_.attitude = quaternionToEuler(q_);
    
    // 去除重力
    output_.linearAccel = removeGravity(accel, q_);
    
    // 速度更新
    updateVelocity(output_.linearAccel, dt);
    applyZupt();
    
    output_.speedKmh = output_.velocity.magnitude() * 3.6f;
}

// ==============================================================================
// Mode 3: 对比模式 (同时运行两种算法)
// ==============================================================================

void ImuManager::updateCompare() {
    // 保存当前状态
    ImuTypes::Quaternion qBackup = q_;
    OutputData outputBackup = output_;
    
    // 运行 Mode 1
    updateJy901Direct();
    OutputData jy901Output = output_;
    
    // 恢复状态运行 Mode 2
    q_ = qBackup;
    output_ = outputBackup;
    updateLocalMadgwick();
    OutputData madgwickOutput = output_;
    
    // 输出对比 (使用 JY901 作为主输出)
    output_ = jy901Output;
    
    LOG_D(getModuleName(), "Attitude diff: Roll=%.2f°, Pitch=%.2f°",
          jy901Output.attitude.roll - madgwickOutput.attitude.roll,
          jy901Output.attitude.pitch - madgwickOutput.attitude.pitch);
}

// ==============================================================================
// Madgwick AHRS 核心算法
// ==============================================================================

void ImuManager::madgwickUpdate(const ImuTypes::Vector3& gyro,
                                 const ImuTypes::Vector3& accel,
                                 float dt) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    
    float q0 = q_.w, q1 = q_.x, q2 = q_.y, q3 = q_.z;
    
    float ax = accel.x, ay = accel.y, az = accel.z;
    float gx = gyro.x, gy = gyro.y, gz = gyro.z;
    
    // 如果加速度不为零，使用梯度下降
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        // 归一化加速度
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;
        
        // 辅助变量
        float _2q0 = 2.0f * q0;
        float _2q1 = 2.0f * q1;
        float _2q2 = 2.0f * q2;
        float _2q3 = 2.0f * q3;
        float _4q0 = 4.0f * q0;
        float _4q1 = 4.0f * q1;
        float _4q2 = 4.0f * q2;
        float _8q1 = 8.0f * q1;
        float _8q2 = 8.0f * q2;
        float q0q0 = q0 * q0;
        float q1q1 = q1 * q1;
        float q2q2 = q2 * q2;
        float q3q3 = q3 * q3;
        
        // 梯度下降
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay 
             - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay 
             - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
        
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;
        
        // 计算四元数变化率
        qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz) - config_.beta * s0;
        qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy) - config_.beta * s1;
        qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx) - config_.beta * s2;
        qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx) - config_.beta * s3;
    } else {
        // 仅用陀螺仪
        qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
        qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
        qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
        qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);
    }
    
    // 积分
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;
    
    // 更新四元数
    q_.w = q0;
    q_.x = q1;
    q_.y = q2;
    q_.z = q3;
    
    // 归一化
    normalizeQuaternion(q_);
}

// ==============================================================================
// 速度解算与 ZUPT
// ==============================================================================

void ImuManager::updateVelocity(const ImuTypes::Vector3& linearAccel, float dt) {
    // 减去校准偏置
    ImuTypes::Vector3 corrected = linearAccel;
    if (isCalibrated_) {
        corrected.x -= accelBias_.x;
        corrected.y -= accelBias_.y;
        corrected.z -= accelBias_.z;
    }
    
    // 低通滤波 (alpha = 0.5)
    constexpr float ALPHA = 0.5f;
    lpfAccel_.x = ALPHA * corrected.x + (1.0f - ALPHA) * lpfAccel_.x;
    lpfAccel_.y = ALPHA * corrected.y + (1.0f - ALPHA) * lpfAccel_.y;
    lpfAccel_.z = ALPHA * corrected.z + (1.0f - ALPHA) * lpfAccel_.z;
    
    // 死区处理
    constexpr float DEADZONE = 0.08f;
    float ax = (std::abs(lpfAccel_.x) < DEADZONE) ? 0.0f : lpfAccel_.x;
    float ay = (std::abs(lpfAccel_.y) < DEADZONE) ? 0.0f : lpfAccel_.y;
    float az = (std::abs(lpfAccel_.z) < DEADZONE) ? 0.0f : lpfAccel_.z;
    
    // 速度积分
    output_.velocity.x += ax * dt;
    output_.velocity.y += ay * dt;
    output_.velocity.z += az * dt;
    
    // 速度衰减
    float speed = output_.velocity.magnitude();
    float decay = (speed < 1.0f) ? config_.decayIdle : config_.decayMoving;
    output_.velocity.x *= decay;
    output_.velocity.y *= decay;
    output_.velocity.z *= decay;
    
    // 速度限幅
    constexpr float MAX_SPEED = 60.0f;
    float currentSpeed = output_.velocity.magnitude();
    if (currentSpeed > MAX_SPEED) {
        float scale = MAX_SPEED / currentSpeed;
        output_.velocity = output_.velocity * scale;
    }
}

void ImuManager::applyZupt() {
    bool isStationary = detectStationary();
    
    if (isStationary) {
        stationaryCount_++;
        movingCount_ = 0;
        
        if (stationaryCount_ >= config_.zuptCount) {
            resetVelocity();
            stationaryCount_ = config_.zuptCount;
            output_.zuptActive = true;
            output_.motionState = MotionState::Stationary;
        }
    } else {
        stationaryCount_ = 0;
        movingCount_++;
        output_.zuptActive = false;
        output_.motionState = MotionState::Moving;
    }
}

[[nodiscard]] bool ImuManager::detectStationary() const {
    float accelMag = lpfAccel_.magnitude();
    float speed = output_.velocity.magnitude();
    
    return (accelMag < config_.stationaryAccThreshold) && 
           (speed < config_.stationaryVelThreshold);
}

void ImuManager::resetVelocity() {
    output_.velocity = ImuTypes::Vector3{0, 0, 0};
    lpfAccel_ = ImuTypes::Vector3{0, 0, 0};
    LOG_I(getModuleName(), "Velocity reset (ZUPT)");
}

void ImuManager::resetPosition() {
    // 位置重置 (如果需要位置追踪)
}

// ==============================================================================
// 校准
// ==============================================================================

void ImuManager::startCalibration() {
    accelBias_ = ImuTypes::Vector3{0, 0, 0};
    isCalibrated_ = false;
    LOG_I(getModuleName(), "Calibration started, keep device stationary");
}

uint16_t ImuManager::addCalibrationSample() {
    ImuTypes::ImuRawData raw{};
    if (!driver_->readRaw(raw)) {
        return 0;
    }
    
    constexpr float ACCEL_SCALE = 9.80665f / 32768.0f;
    accelBias_.x += static_cast<float>(raw.accelX) * ACCEL_SCALE;
    accelBias_.y += static_cast<float>(raw.accelY) * ACCEL_SCALE;
    accelBias_.z += static_cast<float>(raw.accelZ) * ACCEL_SCALE - 9.80665f;
    
    return 1;  // 返回样本计数
}

bool ImuManager::finishCalibration(uint16_t numSamples) {
    if (numSamples == 0) {
        LOG_E(getModuleName(), "Calibration failed: no samples");
        return false;
    }
    
    float inv = 1.0f / static_cast<float>(numSamples);
    accelBias_.x *= inv;
    accelBias_.y *= inv;
    accelBias_.z *= inv;
    isCalibrated_ = true;
    
    LOG_I(getModuleName(), "Calibration done: bias=(%.4f, %.4f, %.4f)",
          accelBias_.x, accelBias_.y, accelBias_.z);
    return true;
}

// ==============================================================================
// 四元数数学运算
// ==============================================================================

[[nodiscard]] ImuTypes::Vector3 ImuManager::removeGravity(
    const ImuTypes::Vector3& accel,
    const ImuTypes::Quaternion& q) const {
    
    // 重力向量 (世界坐标系)
    constexpr ImuTypes::Vector3 GRAVITY{0.0f, 0.0f, 9.80665f};
    
    // 将重力旋转到传感器坐标系
    ImuTypes::Vector3 gravityBody = rotateVectorByQuaternion(GRAVITY, q.conjugate());
    
    // 减去重力
    return accel - gravityBody;
}

[[nodiscard]] ImuTypes::Quaternion ImuManager::eulerToQuaternion(
    const ImuTypes::EulerAngles& angles) const {
    
    float roll = angles.roll * 0.01745329252f;  // DEG_TO_RAD
    float pitch = angles.pitch * 0.01745329252f;
    float yaw = angles.yaw * 0.01745329252f;
    
    float cy = std::cos(yaw * 0.5f);
    float sy = std::sin(yaw * 0.5f);
    float cp = std::cos(pitch * 0.5f);
    float sp = std::sin(pitch * 0.5f);
    float cr = std::cos(roll * 0.5f);
    float sr = std::sin(roll * 0.5f);
    
    ImuTypes::Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    
    return q;
}

[[nodiscard]] ImuTypes::EulerAngles ImuManager::quaternionToEuler(
    const ImuTypes::Quaternion& q) const {
    
    ImuTypes::EulerAngles angles;
    
    // Roll
    float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp) * 57.29577951f;  // RAD_TO_DEG
    
    // Pitch
    float sinp = 2.0f * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1.0f) {
        angles.pitch = std::copysign(90.0f, sinp);
    } else {
        angles.pitch = std::asin(sinp) * 57.29577951f;  // RAD_TO_DEG
    }
    
    // Yaw
    float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp) * 57.29577951f;  // RAD_TO_DEG
    
    return angles;
}

[[nodiscard]] ImuTypes::Vector3 ImuManager::rotateVectorByQuaternion(
    const ImuTypes::Vector3& v,
    const ImuTypes::Quaternion& q) const {
    
    // q * v * q*
    float ix = q.w * v.x + q.y * v.z - q.z * v.y;
    float iy = q.w * v.y + q.z * v.x - q.x * v.z;
    float iz = q.w * v.z + q.x * v.y - q.y * v.x;
    float iw = -q.x * v.x - q.y * v.y - q.z * v.z;
    
    ImuTypes::Vector3 result;
    result.x = ix * q.w + iw * (-q.x) + iy * (-q.z) - iz * (-q.y);
    result.y = iy * q.w + iw * (-q.y) + iz * (-q.x) - ix * (-q.z);
    result.z = iz * q.w + iw * (-q.z) + ix * (-q.y) - iy * (-q.x);
    
    return result;
}

[[nodiscard]] float ImuManager::invSqrt(float x) const {
    // 快速平方根倒数 (Quake III 算法)
    float halfx = 0.5f * x;
    float y = x;
    long i = *reinterpret_cast<long*>(&y);
    i = 0x5f3759df - (i >> 1);
    y = *reinterpret_cast<float*>(&i);
    y = y * (1.5f - (halfx * y * y));
    return y;
}

void ImuManager::normalizeQuaternion(ImuTypes::Quaternion& q) {
    float recipNorm = invSqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    q.w *= recipNorm;
    q.x *= recipNorm;
    q.y *= recipNorm;
    q.z *= recipNorm;
}
