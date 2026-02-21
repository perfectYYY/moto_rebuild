#pragma once
#include <array>
#include "IPacketHandler.h"
#include "Log.h"

/**
 * @brief 包分发器 (单例模式)
 * @details 负责根据 Type ID 将数据包路由到注册的 Handler
 */
class PacketDispatcher {
public:
    // 获取单例引用
    static PacketDispatcher& getInstance() {
        static PacketDispatcher instance;
        return instance;
    }

    /**
     * @brief 注册处理器
     * @param type 协议定义的 FrameType
     * @param handler 实现了 IPacketHandler 的对象指针
     */
    void registerHandler(Protocol::FrameType type, IPacketHandler* handler) {
        uint8_t index = static_cast<uint8_t>(type);
        if (handler) {
            handlers_[index] = handler;
            LOG_I("DISP", "Reg ID 0x%02X -> %s", index, handler->getModuleName());
        }
    }

    /**
     * @brief 路由分发函数
     * @details 该函数会被 UartTransport 的回调调用
     */
    void dispatch(const uint8_t* raw_data, size_t len) {
        // 1. 再次转换回 Header 结构 (零拷贝)
        if (len < sizeof(Protocol::FrameHeader)) return;
        
        const auto* header = reinterpret_cast<const Protocol::FrameHeader*>(raw_data);
        const uint8_t* payload = raw_data + sizeof(Protocol::FrameHeader);

        // 2. 查表分发 (O(1) 复杂度，无 Switch-Case)
        uint8_t type_idx = header->type;
        
        // 边界检查 & 空指针检查
        if (handlers_[type_idx] != nullptr) {
            handlers_[type_idx]->onPacketReceived(*header, payload);
        } else {
            LOG_D("DISP", "No handler for Type 0x%02X", type_idx);
        }
    }

private:
    PacketDispatcher() {
        handlers_.fill(nullptr); // 初始化清空
    }
    
    // 禁止拷贝
    PacketDispatcher(const PacketDispatcher&) = delete;
    PacketDispatcher& operator=(const PacketDispatcher&) = delete;

    // 查找表：索引即 Type ID
    // 空间换时间：256 * 4字节 = 1KB RAM，换取极致速度
    std::array<IPacketHandler*, 256> handlers_;
};