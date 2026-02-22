#pragma once
#include <array>
#include <functional>
#include "IImuDriver.h"
#include "IImuFusion.h"
#include "Log.h"

/**
 * @brief IMU 业务管理器
 * @details 负责：
 * 1. 100Hz 姿态解算 (Madgwick/JY901)
 * 2. 速度积分与 ZUPT 零速修正
 * 3. 数据融合输出
 * 
 * 使用模式：
 * - Mode 1: 直接使用 JY901 内部姿态 (推荐，9轴融合)
 * - Mode 2: 本地 Madgwick 解算 (6轴)
 */
class ImuManager {
public:
    // 工作模式枚举
    enum class FusionMode : uint8_t {
        Jy901Direct = 1,    // 直接使用 JY901 输出
        LocalMadgwick = 2,  // 本地 Madgwick 解算
        Compare = 3         // 同时运行两种模式对比
    };
    
    // 运动状态
    enum class MotionState : uint8_t {
        Stationary = 0,     // 静止
        Moving = 1,         // 运动中
        Unknown = 2
    };
    
    struct Config {
        float sampleFreq = 100.0f;           // 采样频率
        float beta = 0.05f;                  // Madgwick 增益
        float decayIdle = 0.998f;            // 怠速衰减
        float decayMoving = 0.99995f;        // 行驶衰减
        float stationaryAccThreshold = 0.15f;// 静止判定加速度阈值
        float stationaryVelThreshold = 0.08f;// 静止判定速度阈值
        uint16_t zuptCount = 100;            // ZUPT 触发计数 (1秒@100Hz)
        
        Config() = default;
    };
    
    struct OutputData {
        ImuTypes::EulerAngles attitude;     // 姿态角 (度)
        ImuTypes::Vector3 velocity;         // 速度 (m/s)
        float speedKmh{0.0f};               // 速度 (km/h)
        ImuTypes::Vector3 linearAccel;      // 线性加速度
        MotionState motionState{MotionState::Unknown};
        bool zuptActive{false};             // ZUPT 是否生效
    };

    ImuManager();
    
    /**
     * @brief 初始化
     * @param driver IMU 驱动实例 (不拥有所有权)
     * @param config 配置参数
     */
    void init(IImuDriver* driver);
    void init(IImuDriver* driver, const Config& config);
    
    /**
     * @brief 设置融合模式
     */
    void setFusionMode(FusionMode mode);
    
    /**
     * @brief 周期性更新 (应在 100Hz 任务中调用)
     * @return 输出数据
     */
    [[nodiscard]] OutputData update();
    
    /**
     * @brief 重置速度和位置
     */
    void resetVelocity();
    void resetPosition();
    
    /**
     * @brief 开始加速度校准
     */
    void startCalibration();
    
    /**
     * @brief 添加校准样本
     * @return 当前样本数
     */
    uint16_t addCalibrationSample();
    
    /**
     * @brief 完成校准
     * @param numSamples 样本数
     * @return true 校准成功
     */
    bool finishCalibration(uint16_t numSamples);
    
    /**
     * @brief 获取当前输出数据
     */
    [[nodiscard]] const OutputData& getOutput() const { return output_; }
    
    /**
     * @brief 获取模块名称 (用于日志)
     */
    [[nodiscard]] static const char* getModuleName() { return "ImuMgr"; }

private:
    IImuDriver* driver_{nullptr};
    Config config_;
    FusionMode mode_{FusionMode::Jy901Direct};
    OutputData output_;
    
    // 内部状态
    ImuTypes::Vector3 accelBias_{0, 0, 0};
    ImuTypes::Vector3 lpfAccel_{0, 0, 0};
    uint16_t stationaryCount_{0};
    uint16_t movingCount_{0};
    bool isCalibrated_{false};
    
    // 本地 Madgwick 状态 (Mode 2/3 使用)
    ImuTypes::Quaternion q_{1, 0, 0, 0};
    
    // 调度状态机变量
    uint32_t last_tick_time_{0};
    uint8_t time_slot_{0};
    
    // 错误日志节流 (防止刷屏)
    uint32_t lastErrorLogTime_{0};
    uint32_t errorCount_{0};
    
    // 查找表：模式 -> 处理函数
    using UpdateFunc = void (ImuManager::*)();
    static const std::array<UpdateFunc, 4> updateTable_;
    
    // 不同模式的更新实现
    void updateJy901Direct();
    void updateLocalMadgwick();
    void updateCompare();
    
    // 辅助函数
    void updateVelocity(const ImuTypes::Vector3& linearAccel, float dt);
    void applyZupt();
    [[nodiscard]] bool detectStationary() const;
    void madgwickUpdate(const ImuTypes::Vector3& gyro, 
                        const ImuTypes::Vector3& accel, 
                        float dt);
    [[nodiscard]] ImuTypes::Quaternion eulerToQuaternion(
        const ImuTypes::EulerAngles& angles) const;
    [[nodiscard]] ImuTypes::EulerAngles quaternionToEuler(
        const ImuTypes::Quaternion& q) const;
    [[nodiscard]] ImuTypes::Vector3 rotateVectorByQuaternion(
        const ImuTypes::Vector3& v, 
        const ImuTypes::Quaternion& q) const;
    [[nodiscard]] ImuTypes::Vector3 removeGravity(
        const ImuTypes::Vector3& accel,
        const ImuTypes::Quaternion& q) const;
    [[nodiscard]] float invSqrt(float x) const;
    void normalizeQuaternion(ImuTypes::Quaternion& q);
};
