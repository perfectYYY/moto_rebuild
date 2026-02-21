#pragma once
#include <functional>
#include "IPacketHandler.h"
#include "ITransport.h"
#include "Log.h"

/**
 * @brief 手套业务管理器
 * @details 负责：
 * 1. 20ms 时分复用调度 (Left/Right切换)
 * 2. 解析 Type 0x01 状态包
 * 3. 发送 Type 0x02 控制包
 */
class GloveManager : public IPacketHandler {
public:
    // 定义 Mux 切换回调类型：void(channel_index)
    // 0:Left, 1:Right
    using MuxControlCallback = std::function<void(int)>;

    GloveManager();

    /**
     * @brief 初始化依赖
     * @param transport 传输层接口 (UART)
     * @param mux_cb Mux切换回调 (Lambda注入)
     */
    void init(ITransport* transport, MuxControlCallback mux_cb);

    /**
     * @brief 周期性调度 (应在 loop 中每 1ms 调用一次)
     * @details 内部维护 20ms 状态机
     */
    void tick(uint32_t current_millis);

    // --- IPacketHandler 接口实现 ---
    void onPacketReceived(const Protocol::FrameHeader& header, const uint8_t* payload) override;
    const char* getModuleName() const override { return "GloveMgr"; }

private:
    ITransport* transport_ = nullptr;
    MuxControlCallback mux_ctrl_ = nullptr;

    // 状态快照
    Protocol::GloveStatusPayload left_status_;
    Protocol::GloveStatusPayload right_status_;

    // 调度状态机变量
    uint32_t last_tick_time_ = 0;
    uint8_t time_slot_ = 0; // 0-9: Left, 10-19: Right

    // 内部函数：发送请求
    void sendControlRequest(bool is_left);
};