#include <Arduino.h>
#include "Log.h"
#include "MuxDriver.h"
#include "UartTransport.h"
#include "PacketDispatcher.h" // App_System
#include "GloveManager.h"     // App_Glove

// === 引脚定义 ===
#define PIN_UART_RX  18 
#define PIN_UART_TX  17
#define PIN_MUX_S1   16

// === 全局组件实例 ===
// 1. 硬件驱动 (Driver Layer)
UartTransport uartDriver(1, PIN_UART_RX, PIN_UART_TX);

// 2. 业务模块 (App Layer)
GloveManager gloveMgr;

void setup() {
    // 1. 基础日志
    Serial.begin(115200);
    // while(!Serial); // 调试时可以打开，生产环境不要阻塞
    LOG_I("SYS", "System Booting...");

    // 2. 初始化硬件
    MuxDriver::init(PIN_MUX_S1);
    uartDriver.init();

    // 3. 组装依赖 (Dependency Injection)
    
    // A. 连接 UART -> Dispatcher
    // 当 UART 收到包，扔给 Dispatcher
    uartDriver.setReceiveCallback([](const uint8_t* data, size_t len) {
        PacketDispatcher::getInstance().dispatch(data, len);
    });

    // B. 连接 Dispatcher -> GloveManager
    // 注册：Type 0x01 的包交给 GloveManager 处理
    PacketDispatcher::getInstance().registerHandler(
        Protocol::FrameType::StatusReport, 
        &gloveMgr
    );

    // C. 初始化 GloveManager
    // 注入：使用 uartDriver 发送，使用 MuxDriver 切换通道
    gloveMgr.init(&uartDriver, [](int channel) {
        if (channel == 0) MuxDriver::select(PIN_MUX_S1, MuxDriver::Channel::LeftGlove);
        else              MuxDriver::select(PIN_MUX_S1, MuxDriver::Channel::RightGlove);
    });

    LOG_I("SYS", "System Ready.");
}

void loop() {
    // 驱动业务层的 Ticker
    // GloveManager 会自动处理 20ms 的左右切换和发包
    gloveMgr.tick(millis());
    
    // 可以在这里加一点简单的延迟，或者干别的
    // 比如每秒打印一次心跳，证明主循环没死
    static uint32_t last_print = 0;
    if (millis() - last_print > 2000) {
        last_print = millis();
        LOG_I("SYS", "Heap: %d", ESP.getFreeHeap());
    }
}