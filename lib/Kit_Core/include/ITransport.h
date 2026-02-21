#pragma once
#include <stdint.h>
#include <stddef.h>
#include <functional>

// ==============================================================================
// 传输层接口 (Interface)
// 任何通信驱动 (UART, CAN, BLE) 都必须继承并实现此接口
// ==============================================================================
class ITransport {
public:
    // 定义接收回调：void(数据指针, 长度)
    // 使用 std::function 实现现代 C++ 回调
    using ReceiveCallback = std::function<void(const uint8_t* data, size_t len)>;

    virtual ~ITransport() = default;

    /**
     * @brief 初始化硬件
     */
    virtual void init() = 0;

    /**
     * @brief 发送数据 (纯虚函数)
     * @param data 数据指针
     * @param len 长度
     * @return true 成功写入缓冲区
     */
    [[nodiscard]] virtual bool send(const uint8_t* data, size_t len) = 0;

    /**
     * @brief 设置接收回调
     * @details 业务层通过此函数订阅数据
     */
    virtual void setReceiveCallback(ReceiveCallback cb) = 0;
};