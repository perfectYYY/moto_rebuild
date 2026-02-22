#pragma once
#include <cstdint>
#include <cstddef>
#include "ImuTypes.h"

/**
 * @brief IMU 融合算法接口
 * @details 支持 Madgwick、Mahony 等不同融合算法
 */
class IImuFusion {
public:
    virtual ~IImuFusion() = default;
    
    /**
     * @brief 初始化融合算法
     * @param sampleFreq 采样频率 (Hz)
     */
    virtual void init(float sampleFreq) = 0;
    
    /**
     * @brief 重置状态
     */
    virtual void reset() = 0;
    
    /**
     * @brief 更新姿态估计 (核心算法)
     * @param gyro 陀螺仪数据 (dps)
     * @param accel 加速度数据 (m/s²)
     * @param dt 时间步长 (秒)
     */
    virtual void update(const ImuTypes::Vector3& gyro,
                        const ImuTypes::Vector3& accel,
                        float dt) = 0;
    
    /**
     * @brief 9轴更新 (含磁力计)
     */
    virtual void update9DoF(const ImuTypes::Vector3& gyro,
                            const ImuTypes::Vector3& accel,
                            const ImuTypes::Vector3& mag,
                            float dt) = 0;
    
    /**
     * @brief 获取当前四元数姿态
     */
    [[nodiscard]] virtual ImuTypes::Quaternion getQuaternion() const = 0;
    
    /**
     * @brief 获取欧拉角 (度)
     */
    [[nodiscard]] virtual ImuTypes::EulerAngles getEulerAngles() const = 0;
    
    /**
     * @brief 使用外部姿态直接设置 (如 JY901 内部解算结果)
     */
    virtual void setEulerAngles(const ImuTypes::EulerAngles& angles) = 0;
    
    /**
     * @brief 去除重力分量，得到线性加速度
     */
    [[nodiscard]] virtual ImuTypes::Vector3 removeGravity(
        const ImuTypes::Vector3& accel) const = 0;
    
    /**
     * @brief 设置算法参数
     */
    virtual void setBeta(float beta) = 0;
    virtual void setSampleFrequency(float freq) = 0;
};
