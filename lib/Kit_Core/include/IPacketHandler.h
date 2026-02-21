#pragma once
#include "ProtocolDef.h"

// ==============================================================================
// 包处理器接口 (Observer)
// 想要接收特定数据包的业务模块 (GloveManager, ScreenManager) 需继承此接口
// ==============================================================================
class IPacketHandler {
public:
    virtual ~IPacketHandler() = default;

    /**
     * @brief 当收到完整且通过校验的包时触发
     * @param header 帧头信息
     * @param payload 数据载荷 (指针仅在回调内有效)
     */
    virtual void onPacketReceived(const Protocol::FrameHeader& header, const uint8_t* payload) = 0;

    /**
     * @brief 获取模块名称 (用于调试)
     */
    virtual const char* getModuleName() const = 0;
};