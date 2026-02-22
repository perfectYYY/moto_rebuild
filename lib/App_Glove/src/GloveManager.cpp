#include "GloveManager.h"
#include <string.h> // for memcpy

GloveManager::GloveManager() {
    // 安全起见，清零状态
    memset(&left_status_, 0, sizeof(left_status_));
    memset(&right_status_, 0, sizeof(right_status_));
}

void GloveManager::init(ITransport* transport, MuxControlCallback mux_cb) {
    transport_ = transport;
    mux_ctrl_ = mux_cb;
}

// 核心调度逻辑：20ms 时间片轮转 (TDM)
// 0ms  -> 切左手 -> 发请求
// 10ms -> 切右手 -> 发请求
void GloveManager::tick(uint32_t current_millis) {
    if (!transport_ || !mux_ctrl_) return;

    // 简单的 20ms 循环计数器
    uint32_t delta = current_millis - last_tick_time_;
    if (delta >= 20) {
        last_tick_time_ = current_millis;
        time_slot_ = 0; // 重置周期
    } else if (delta >= 10 && time_slot_ == 0) {
        time_slot_ = 10; // 进入下半周期
    } else {
        return; // 不需要动作
    }

    // 测试：一直使用通道1（右手套），不切换
    if (true) {
        mux_ctrl_(1); // 切右手
        sendControlRequest(false);
        return;
    }

    if (time_slot_ == 0) {
        // [T=0ms] 左手时隙
        mux_ctrl_(0); // 切左手
        // 稍微延时一点点让 Mux 稳定? 通常不需要，ESP32 GPIO 够快
        sendControlRequest(true);
    }
    else if (time_slot_ == 10) {
        // [T=10ms] 右手时隙
        mux_ctrl_(1); // 切右手
        sendControlRequest(false);
    }
}

void GloveManager::onPacketReceived(const Protocol::FrameHeader& header, const uint8_t* payload) {
    // 只处理状态上报包 (Type 0x01)
    if (header.type != static_cast<uint8_t>(Protocol::FrameType::StatusReport)) {
        return;
    }

    // 长度安全检查
    if (header.len != sizeof(Protocol::GloveStatusPayload)) {
        LOG_E("GLOVE", "Payload len mismatch! Exp:%d, Got:%d",
              sizeof(Protocol::GloveStatusPayload), header.len);
        return;
    }

    // 转换 Payload
    const auto* status = reinterpret_cast<const Protocol::GloveStatusPayload*>(payload);

    // 识别左右手并更新快照（每秒打印一次）
    static uint32_t last_log = 0;
    if (millis() - last_log > 1000) {
        if (status->side_id == 'L') {
            memcpy(&left_status_, status, sizeof(Protocol::GloveStatusPayload));
            LOG_I("GLOVE", "✅ 左手套 Temp: %.2f°C", left_status_.current_temps[0] / 100.0f);
        }
        else if (status->side_id == 'R') {
            memcpy(&right_status_, status, sizeof(Protocol::GloveStatusPayload));
            LOG_I("GLOVE", "✅ 右手套 Temp: %.2f°C", right_status_.current_temps[0] / 100.0f);
        } else {
            LOG_W("GLOVE", "Unknown Side ID: %c", status->side_id);
        }
        last_log = millis();
    }
}

void GloveManager::sendControlRequest(bool is_left) {
    // 构造控制包 (Type 0x02)
    // 目前先发空控制指令 (仅查询)
    Protocol::FrameHeader header;
    header.head = Protocol::HEAD_MAGIC;
    header.type = static_cast<uint8_t>(Protocol::FrameType::ControlCommand);
    header.len  = sizeof(Protocol::GloveControlPayload);

    Protocol::GloveControlPayload ctrl;
    memset(&ctrl, 0, sizeof(ctrl));
    // ctrl.update_mask = 0; // 不更新PID，只查状态

    // 序列化发送缓冲区
    // 结构: Head(4) + Payload(16) + Checksum(1) = 21 Bytes
    uint8_t tx_buf[32];
    size_t ptr = 0;

    // 1. 拷贝头
    memcpy(tx_buf + ptr, &header, sizeof(header));
    ptr += sizeof(header);

    // 2. 拷贝 Payload
    memcpy(tx_buf + ptr, &ctrl, sizeof(ctrl));
    ptr += sizeof(ctrl);

    // 3. 计算并追加校验和 (Sum8)
    // 范围: Head(从0xAA开始) + Payload
    tx_buf[ptr] = Protocol::calculateChecksum(tx_buf, ptr);
    ptr++;

    // 4. 发送
    transport_->send(tx_buf, ptr);
}