#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "ITransport.h"
#include "ProtocolDef.h"
#include "Log.h"

class UartTransport : public ITransport {
public:
    /**
     * @brief 构造函数
     * @param uart_num 硬件串口号 (0, 1, 2)
     * @param rx_pin RX GPIO
     * @param tx_pin TX GPIO
     */
    UartTransport(uint8_t uart_num, int rx_pin, int tx_pin);
    virtual ~UartTransport() = default;

    // --- ITransport 接口实现 ---
    void init() override;
    [[nodiscard]] bool send(const uint8_t* data, size_t len) override;
    void setReceiveCallback(ReceiveCallback cb) override;

    // --- 内部处理 ---
    // 供 FreeRTOS 任务调用的轮询函数
    void poll(); 

private:
    HardwareSerial serial_;
    int rx_pin_;
    int tx_pin_;
    ReceiveCallback callback_;
    
    // 接收任务句柄
    TaskHandle_t rx_task_handle_ = nullptr;

    // === 协议解析状态机变量 ===
    enum class ParseState {
        WaitHead1,  // 等待 0xAA
        WaitHead2,  // 等待 0x55
        WaitType,   // 读取 Type
        WaitLen,    // 读取 Len
        WaitPayload,// 读取 Payload
        WaitCheck   // 读取 Checksum
    } state_ = ParseState::WaitHead1;

    uint8_t buffer_[64];      // 接收缓冲区
    size_t  buf_idx_ = 0;     // 当前缓冲区索引
    size_t  expected_len_ = 0;// 期待的 Payload 长度
    Protocol::FrameHeader current_header_; // 暂存帧头

    // 静态任务入口 trampoline
    static void rxTaskTrampoline(void* arg);
};