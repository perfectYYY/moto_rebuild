#include "UartTransport.h"

// 构造函数：初始化成员，但不操作硬件
UartTransport::UartTransport(uint8_t uart_num, int rx_pin, int tx_pin)
    : serial_(uart_num), rx_pin_(rx_pin), tx_pin_(tx_pin), callback_(nullptr) {
}

void UartTransport::init() {
    LOG_I("UART", "Init Serial%d on Rx:%d Tx:%d", 1, rx_pin_, tx_pin_);

    // 启用 UART1 监听 GPIO18
    serial_.begin(115200, SERIAL_8N1, rx_pin_, tx_pin_);
    serial_.setRxBufferSize(256);

    // 启用接收任务
    xTaskCreatePinnedToCore(
        rxTaskTrampoline,
        "UartRxTask",
        4096,
        this,
        5,
        &rx_task_handle_,
        1
    );

    LOG_I("UART", "UART1 已启用，监听 GPIO%d (RX)", rx_pin_);
}

// 静态跳转函数
void UartTransport::rxTaskTrampoline(void* arg) {
    auto* transport = static_cast<UartTransport*>(arg);
    while (1) {
        transport->poll(); // 执行具体的轮询逻辑
        // 给系统一点喘息时间，但不要太长以免串口FIFO溢出
        // 115200bps -> ~11KB/s -> 1ms 约 11字节
        // vTaskDelay(1) 可能会有点慢，这里我们依赖 serial.available() 的非阻塞特性
        // 如果没有数据，yield 给其他任务
        if (!transport->serial_.available()) {
            vTaskDelay(pdMS_TO_TICKS(1)); 
        }
    }
}

// 发送实现
bool UartTransport::send(const uint8_t* data, size_t len) {
    if (len == 0) return false;
    // HardwareSerial::write 是阻塞还是缓冲取决于实现，通常有缓冲区
    size_t written = serial_.write(data, len);
    return (written == len);
}

void UartTransport::setReceiveCallback(ReceiveCallback cb) {
    callback_ = cb;
}

// 核心：协议解析状态机 (State Machine)
// 这种写法比递归或复杂 if-else 更快、更安全
void UartTransport::poll() {
    // 环形缓冲区保存最近的字节用于调试
    static uint8_t debug_buffer[64];
    static uint8_t debug_idx = 0;

    while (serial_.available()) {
        uint8_t byte = serial_.read();

        // 打印每个收到的原始字节
        LOG_I("UART", "RX GPIO%d: 0x%02X (%d)", rx_pin_, byte, byte);

        // 保存到调试缓冲区
        debug_buffer[debug_idx] = byte;
        debug_idx = (debug_idx + 1) % 64;

        switch (state_) {
            case ParseState::WaitHead1:
                // 0x55 是帧头第一个字节
                if (byte == 0x55) {
                    state_ = ParseState::WaitHead2;
                    LOG_I("UART", "→ WaitHead2");
                }
                break;

            case ParseState::WaitHead2:
                // 0xAA 是帧头第二个字节
                if (byte == 0xAA) {
                    state_ = ParseState::WaitType;
                    LOG_I("UART", "→ WaitType");
                } else if (byte == 0x55) {
                    // 仍然是 0x55，继续等待 0xAA
                    state_ = ParseState::WaitHead2;
                } else {
                    // 不是 0xAA，回到 WaitHead1
                    state_ = ParseState::WaitHead1;
                }
                break;

            case ParseState::WaitType:
                current_header_.type = byte;
                LOG_I("UART", "Type=0x%02X → WaitLen", byte);
                state_ = ParseState::WaitLen;
                break;

            case ParseState::WaitLen:
                current_header_.len = byte;
                LOG_I("UART", "Len=%d", byte);
                expected_len_ = byte;
                
                // 安全检查：如果长度太离谱，直接丢弃，防止缓冲区溢出
                if (expected_len_ > sizeof(buffer_) - 1) {
                    LOG_E("UART", "Payload too large: %d", expected_len_);
                    // 打印最近收到的字节
                    LOG_E("UART", "Recent bytes:");
                    for (int i = 0; i < 16; i++) {
                        uint8_t idx = (debug_idx + 56 + i) % 64;
                        LOG_E("UART", "  [%d] 0x%02X", i, debug_buffer[idx]);
                    }
                    state_ = ParseState::WaitHead1;
                } else if (expected_len_ == 0) {
                    state_ = ParseState::WaitCheck; // 长度为0直接跳去校验
                } else {
                    buf_idx_ = 0;
                    state_ = ParseState::WaitPayload;
                }
                break;

            case ParseState::WaitPayload:
                buffer_[buf_idx_++] = byte;
                if (buf_idx_ >= expected_len_) {
                    state_ = ParseState::WaitCheck;
                }
                break;

            case ParseState::WaitCheck:
                // 收到校验位，开始计算
                // 计算逻辑：Sum8(Head(2) + Type(1) + Len(1) + Payload(N))
                // 但是我们的 calculateChecksum 函数设计的是计算一段连续内存
                // 所以这里我们需要手动把 Header 里的部分加进去，或者重构一下 buffer
                
                // 方案：为了性能，我们手动累加 Header 部分
                uint8_t calc_sum = 0;
                calc_sum += (uint8_t)(Protocol::HEAD_MAGIC & 0xFF);
                calc_sum += (uint8_t)((Protocol::HEAD_MAGIC >> 8) & 0xFF);
                calc_sum += current_header_.type;
                calc_sum += current_header_.len;
                
                // 加上 Payload 部分
                for(size_t i=0; i<expected_len_; i++) {
                    calc_sum += buffer_[i];
                }

                if (calc_sum == byte) {
                    // 校验成功！触发回调
                    LOG_I("UART", "📥 收到完整帧! Type:%02X Len:%d", current_header_.type, expected_len_);
                    if (callback_) {
                        // 组装一个完整的 Header 传出去
                        Protocol::FrameHeader h = {
                            Protocol::HEAD_MAGIC,
                            current_header_.type,
                            current_header_.len
                        };
                        callback_(buffer_, expected_len_);
                    }
                } else {
                    LOG_E("UART", "CS Error: Exp %02X, Calc %02X", byte, calc_sum);
                    // 打印最近收到的字节
                    LOG_E("UART", "Recent bytes:");
                    for (int i = 0; i < 16; i++) {
                        uint8_t idx = (debug_idx + 56 + i) % 64;
                        LOG_E("UART", "  [%d] 0x%02X", i, debug_buffer[idx]);
                    }
                }
                
                // 无论成功失败，重置状态机
                state_ = ParseState::WaitHead1;
                break;
        }
    }
}