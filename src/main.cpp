/**
 * @file main.cpp
 * @brief 摩托车数据终端 - 系统入口
 * @details 仅负责组件组装与依赖注入，禁止写业务逻辑
 * 
 * 任务分配 (FreeRTOS):
 * - Core 1, Priority 5: T_RealTime (100Hz IMU, 手套通信)
 * - Core 0, Priority 3: T_Sensors (5Hz AS7341, RS485)
 * - Core 0, Priority 2: T_Storage (SD卡写入)
 * - Core 0, Priority 2: T_GNSS (GNSS数据)
 * - Core 0, Priority 1: T_System (BLE, 屏幕刷新)
 */

#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <SD_MMC.h>
#include <BLEDevice.h>

// FreeRTOS (Arduino 已包含)
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

// 日志
#include "Log.h"

// 硬件驱动 (Driver Layer)
#include "MuxDriver.h"
#include "UartTransport.h"
#include "JY901Driver.h"
#include "I2CMuxDriver.h"
#include "RS485Driver.h"
#include "SDStorageDriver.h"
#include "BE881Driver.h"
#include "AS7341Driver.h"
#include "BLEDriver.h"

// 业务模块 (App Layer)
#include "PacketDispatcher.h"
#include "GloveManager.h"
#include "ImuManager.h"

// ==============================================================================
// 引脚定义
// ==============================================================================
constexpr int PIN_UART_RX = 18;
constexpr int PIN_UART_TX = 17;
constexpr int PIN_MUX_S1 = 16;
constexpr int PIN_I2C_SDA = 1;  // 修正：与旧代码一致
constexpr int PIN_I2C_SCL = 2;  // 修正：与旧代码一致
constexpr int PIN_SD_CLK = 47;  // 修正：与旧代码一致
constexpr int PIN_SD_CMD = 48;  // 修正：与旧代码一致
constexpr int PIN_SD_D0 = 21;   // 修正：与旧代码一致
constexpr int PIN_GNSS_RX = 4;
constexpr int PIN_GNSS_TX = 5;
constexpr int PIN_RS485_RX = 6;
constexpr int PIN_RS485_TX = 7;

// ==============================================================================
// 全局组件实例
// ==============================================================================

// 1. 传输层
UartTransport uartGlove(1, PIN_UART_RX, PIN_UART_TX);
HardwareSerial SerialGNSS(2);
HardwareSerial SerialRS485(2);  // 根据实际硬件调整

// 2. 硬件驱动
JY901Driver jy901Driver;
I2CMuxDriver i2cMux(0x70);
RS485Driver rs485Driver;
SDStorageDriver sdDriver;
BE881Driver gnssDriver;
AS7341Driver as7341Driver(0x39);
BLEDriver bleDriver;

// 3. 业务模块
GloveManager gloveMgr;
ImuManager imuMgr;

// 4. FreeRTOS 资源
QueueHandle_t imuDataQueue{nullptr};
QueueHandle_t sensorDataQueue{nullptr};
SemaphoreHandle_t i2cMutex{nullptr};

// ==============================================================================
// 函数前向声明
// ==============================================================================
void initHardware();
void initDependencies();
void createTasks();
void printSystemStatus();

// 任务函数
[[noreturn]] void taskRealTime(void* pvParameters);
[[noreturn]] void taskSensors(void* pvParameters);
[[noreturn]] void taskStorage(void* pvParameters);
[[noreturn]] void taskGNSS(void* pvParameters);
[[noreturn]] void taskSystem(void* pvParameters);

// ==============================================================================
// 入口函数
// ==============================================================================

void setup() {
    // 1. 基础日志
    Serial.begin(115200);
    delay(2000);  // 等待串口稳定
    
    LOG_I("SYS", "=================================");
    LOG_I("SYS", "Moto Terminal Rebuild v2.0");
    LOG_I("SYS", "C++17 Modern Architecture");
    LOG_I("SYS", "=================================");

    // 2. 初始化硬件
    initHardware();

    // 3. 组装依赖
    initDependencies();

    // 4. 创建队列和互斥锁
    imuDataQueue = xQueueCreate(10, sizeof(ImuManager::OutputData));
    sensorDataQueue = xQueueCreate(20, sizeof(IStorageService::DataRecord));
    i2cMutex = xSemaphoreCreateMutex();

    // 5. 创建 FreeRTOS 任务 (Arduino 已自动启动调度器)
    createTasks();

    LOG_I("SYS", "Setup complete, tasks created.");
}

void loop() {
    // Arduino 主循环 - 每 10 秒报告系统状态
    static uint32_t lastReport = 0;
    if (millis() - lastReport > 10000) {
        lastReport = millis();
        printSystemStatus();
    }
    
    delay(100);
}

// ==============================================================================
// 系统状态报告
// ==============================================================================

void printSystemStatus() {
    Serial.println("\n========================================");
    Serial.printf("[STATUS] Uptime: %lu seconds\n", millis() / 1000);
    Serial.printf("[STATUS] Free Heap: %d bytes\n", ESP.getFreeHeap());
    Serial.printf("[STATUS] Tasks: %d\n", uxTaskGetNumberOfTasks());
    Serial.printf("[STATUS] CPU Freq: %d MHz\n", ESP.getCpuFreqMHz());
    
    // 芯片信息
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    Serial.printf("[STATUS] Chip Cores: %d\n", chip_info.cores);
    
    // === I2C 设备扫描 (诊断用) ===
    Serial.println("\n--- I2C Scan ---");
    int foundCount = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.printf("  [I2C] Device found at 0x%02X", addr);
            // 标记常见设备
            if (addr == 0x39) Serial.print(" (AS7341?)");
            if (addr == 0x50) Serial.print(" (JY901?)");
            if (addr == 0x51) Serial.print(" (JY901 alt?)");
            if (addr == 0x70) Serial.print(" (TCA9548A?)");
            Serial.println();
            foundCount++;
        }
    }
    if (foundCount == 0) {
        Serial.println("  [I2C] No devices found!");
    }
    Serial.printf("  [I2C] Total: %d device(s)\n", foundCount);
    
    // 传感器状态
    Serial.println("\n--- Sensor Status ---");
    Serial.printf("  JY901 IMU: %s\n", jy901Driver.isOnline() ? "ONLINE" : "OFFLINE");
    Serial.printf("  SD Card: %s\n", sdDriver.isOnline() ? "ONLINE" : "OFFLINE");
    Serial.printf("  GNSS: %s\n", gnssDriver.hasFix() ? "HAS FIX" : "NO FIX");
    Serial.printf("  BLE: %s\n", bleDriver.isConnected() ? "CONNECTED" : "ADVERTISING");
    Serial.println("========================================\n");
    Serial.flush();
}

// ==============================================================================
// 串口命令处理
// ==============================================================================

void handleSerialCommands() {
    static String inputBuffer = "";
    
    while (Serial.available()) {
        char c = Serial.read();
        
        // 处理换行 (支持 \r 和 \n)
        if (c == '\r' || c == '\n') {
            if (inputBuffer.length() > 0) {
                String cmd = inputBuffer;
                cmd.trim();
                cmd.toUpperCase();
                
                LOG_I("CMD", "Received command: %s", inputBuffer.c_str());
                
                if (cmd == "RESET" || cmd == "REBOOT" || cmd == "RST") {
                    LOG_I("CMD", "System rebooting...");
                    Serial.flush();
                    delay(500);
                    ESP.restart();  // 软件复位
                }
                else if (cmd == "STATUS" || cmd == "STAT") {
                    LOG_I("CMD", "=== System Status ===");
                    LOG_I("CMD", "Heap: %d bytes", ESP.getFreeHeap());
                    LOG_I("CMD", "Tasks: %d", uxTaskGetNumberOfTasks());
                    LOG_I("CMD", "Uptime: %lu ms", millis());
                }
                else if (cmd == "HELP" || cmd == "?") {
                    Serial.println("\n=== Available Commands ===");
                    Serial.println("RESET  - Reboot the system");
                    Serial.println("STATUS - Show system status");
                    Serial.println("HELP   - Show this help");
                    Serial.println("========================\n");
                }
                else {
                    LOG_W("CMD", "Unknown command: %s (type HELP for list)", inputBuffer.c_str());
                }
                
                inputBuffer = "";
            }
        }
        else {
            // 添加字符到缓冲区
            if (inputBuffer.length() < 64) {
                inputBuffer += c;
            }
        }
    }
}

// ==============================================================================
// 硬件初始化
// ==============================================================================

void initHardware() {
    LOG_I("SYS", "Initializing hardware...");

    // I2C 总线
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(400000);  // 400kHz
    
    // ===== 扫描 TCA9548A 通道查找 JY901 =====
    LOG_I("SYS", "Scanning TCA9548A channels for JY901...");
    uint8_t jy901Channel = 0xFF;
    
    // 先检查 TCA9548A 是否存在
    Wire.beginTransmission(0x70);
    if (Wire.endTransmission() == 0) {
        LOG_I("SYS", "TCA9548A found at 0x70");
        
        // 扫描所有 8 个通道
        for (uint8_t ch = 0; ch < 8; ch++) {
            // 切换 TCA9548A 到通道 ch
            Wire.beginTransmission(0x70);
            Wire.write(1 << ch);
            Wire.endTransmission();
            delay(10);
            
            // 扫描该通道上的设备
            LOG_I("SYS", "  Scanning channel %d...", ch);
            for (uint8_t addr = 1; addr < 127; addr++) {
                if (addr == 0x70) continue; // 跳过 TCA9548A 本身
                Wire.beginTransmission(addr);
                if (Wire.endTransmission() == 0) {
                    LOG_I("SYS", "    Found device at 0x%02X", addr);
                    if (addr == 0x50 || addr == 0x51) {
                        jy901Channel = ch;
                        LOG_I("SYS", "    *** JY901 found at channel %d, addr 0x%02X ***", ch, addr);
                    }
                }
            }
        }
        
        // 重置 TCA9548A (关闭所有通道)
        Wire.beginTransmission(0x70);
        Wire.write(0x00);
        Wire.endTransmission();
    } else {
        LOG_W("SYS", "TCA9548A not found at 0x70");
    }

    // 1. 多路复用器 (手套切换)
    MuxDriver::init(PIN_MUX_S1);
    uartGlove.init();

    // 2. JY901 IMU (通过 TCA9548A 访问)
    if (jy901Channel != 0xFF) {
        // 配置 JY901 使用 TCA9548A 通道
        LOG_I("SYS", "Initializing JY901 via TCA9548A channel %d...", jy901Channel);
        
        // 在 JY901 驱动中设置通道 (需要修改驱动支持)
        // 临时方案：初始化前先切换通道
        Wire.beginTransmission(0x70);
        Wire.write(1 << jy901Channel);
        Wire.endTransmission();
        delay(10);
        
        // 设置 TCA9548A 通道
        jy901Driver.setTCA9548AChannel(jy901Channel);
        
        if (jy901Driver.init()) {
            jy901Driver.setSampleFrequency(100.0f);
            LOG_I("SYS", "JY901 ready on channel %d", jy901Channel);
        } else {
            LOG_W("SYS", "JY901 init failed on channel %d", jy901Channel);
        }
    } else {
        LOG_W("SYS", "JY901 not found on any TCA9548A channel");
    }

    // 3. I2C Mux (光谱传感器等)
    if (i2cMux.init()) {
        LOG_I("SYS", "I2C Mux ready");
    }

    // 4. AS7341 光谱传感器
    if (as7341Driver.init()) {
        as7341Driver.setIntegrationTime(36);  // 100ms
        LOG_I("SYS", "AS7341 ready");
    }

    // 5. RS485 气象站
    SerialRS485.begin(9600, SERIAL_8N1, PIN_RS485_RX, PIN_RS485_TX);
    rs485Driver.attachSerial(SerialRS485);
    if (rs485Driver.init()) {
        LOG_I("SYS", "RS485 ready");
    } else {
        LOG_W("SYS", "RS485 init failed");
    }

    // 6. SD 卡
    if (sdDriver.init()) {
        sdDriver.createLogFile(millis());
        LOG_I("SYS", "SD card ready");
    } else {
        LOG_W("SYS", "SD card not available");
    }

    // 7. GNSS
    SerialGNSS.begin(115200, SERIAL_8N1, PIN_GNSS_RX, PIN_GNSS_TX);
    gnssDriver.attachSerial(SerialGNSS);
    // 串口已在 attachSerial 中初始化
    if (gnssDriver.init()) {
        LOG_I("SYS", "GNSS ready");
    }

    // 8. BLE
    if (bleDriver.init()) {
        bleDriver.startAdvertising();
        LOG_I("SYS", "BLE ready");
    }

    LOG_I("SYS", "Hardware init complete");
}

// ==============================================================================
// 依赖注入
// ==============================================================================

void initDependencies() {
    LOG_I("SYS", "Wiring dependencies...");

    // 1. UART -> PacketDispatcher
    uartGlove.setReceiveCallback([](const uint8_t* data, size_t len) {
        PacketDispatcher::getInstance().dispatch(data, len);
    });

    // 2. Dispatcher -> GloveManager
    PacketDispatcher::getInstance().registerHandler(
        Protocol::FrameType::StatusReport,
        &gloveMgr
    );

    // 3. 初始化 GloveManager
    gloveMgr.init(&uartGlove, [](int channel) {
        if (channel == 0) {
            MuxDriver::select(PIN_MUX_S1, MuxDriver::Channel::LeftGlove);
        } else {
            MuxDriver::select(PIN_MUX_S1, MuxDriver::Channel::RightGlove);
        }
    });

    // 4. 初始化 IMU Manager
    ImuManager::Config imuConfig;
    imuConfig.sampleFreq = 100.0f;
    imuConfig.beta = 0.05f;
    imuConfig.decayIdle = 0.998f;
    imuConfig.decayMoving = 0.99995f;
    imuConfig.stationaryAccThreshold = 0.15f;
    imuConfig.stationaryVelThreshold = 0.08f;
    imuConfig.zuptCount = 100;
    
    imuMgr.init(&jy901Driver, imuConfig);
    imuMgr.setFusionMode(ImuManager::FusionMode::Jy901Direct);

    // 5. BLE 接收回调
    bleDriver.onReceive([](const uint8_t* data, size_t len) {
        LOG_I("BLE", "Received %d bytes", len);
        // TODO: 处理远程命令
    });

    LOG_I("SYS", "Dependencies wired");
}

// ==============================================================================
// 创建任务
// ==============================================================================

void createTasks() {
    // 任务 1: 实时任务 (100Hz IMU + 手套通信) - Core 1, Priority 5
    xTaskCreatePinnedToCore(
        taskRealTime,
        "T_RealTime",
        4096,
        nullptr,
        5,
        nullptr,
        1  // Core 1
    );

    // 任务 2: 传感器任务 (5Hz AS7341 + RS485) - Core 0, Priority 3
    xTaskCreatePinnedToCore(
        taskSensors,
        "T_Sensors",
        4096,
        nullptr,
        3,
        nullptr,
        0  // Core 0
    );

    // 任务 3: 存储任务 (SD卡写入) - Core 0, Priority 2
    xTaskCreatePinnedToCore(
        taskStorage,
        "T_Storage",
        4096,
        nullptr,
        2,
        nullptr,
        0  // Core 0
    );

    // 任务 4: GNSS 任务 - Core 0, Priority 2
    xTaskCreatePinnedToCore(
        taskGNSS,
        "T_GNSS",
        3072,
        nullptr,
        2,
        nullptr,
        0  // Core 0
    );

    // 任务 5: 系统任务 (BLE + 屏幕) - Core 0, Priority 1
    xTaskCreatePinnedToCore(
        taskSystem,
        "T_System",
        3072,
        nullptr,
        1,
        nullptr,
        0  // Core 0
    );

    LOG_I("SYS", "Tasks created");
}

// ==============================================================================
// 任务实现
// ==============================================================================

[[noreturn]] void taskRealTime(void* pvParameters) {
    LOG_I("TASK", "T_RealTime started on Core %d", xPortGetCoreID());

    constexpr TickType_t period = pdMS_TO_TICKS(10);  // 100Hz
    TickType_t lastWakeTime = xTaskGetTickCount();

    while (true) {
        // 1. IMU 更新 (100Hz)
        auto imuData = imuMgr.update();
        xQueueSend(imuDataQueue, &imuData, 0);  // 非阻塞发送

        // 2. 手套通信调度 (20Hz, 每 5 个周期执行一次)
        static uint8_t counter = 0;
        if (++counter >= 5) {
            counter = 0;
            gloveMgr.tick(millis());
        }

        // 精确延时
        vTaskDelayUntil(&lastWakeTime, period);
    }
}

[[noreturn]] void taskSensors(void* pvParameters) {
    LOG_I("TASK", "T_Sensors started on Core %d", xPortGetCoreID());

    // RS485 状态机状态
    bool rs485Reading = false;

    while (true) {
        // 1. AS7341 光谱读取 (5Hz) - 非阻塞
        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            AS7341Driver::SpectrumData spectrum{};
            // 先启动测量
            static bool as7341Measuring = false;
            static uint32_t as7341StartTime = 0;

            if (!as7341Measuring) {
                as7341Driver.startMeasurement();
                as7341Measuring = true;
                as7341StartTime = millis();
            } else if (millis() - as7341StartTime > 150) {
                // 150ms 后尝试读取
                if (as7341Driver.readData(spectrum)) {
                    LOG_D("SENSOR", "AS7341: F1=%d, F2=%d", spectrum.f1_415nm, spectrum.f2_445nm);
                }
                as7341Measuring = false;  // 重置状态
            }
            xSemaphoreGive(i2cMutex);
        }

        // 2. RS485 气象站 - 异步状态机
        if (!rs485Reading) {
            // 启动新的读取
            if (rs485Driver.startReadAll()) {
                rs485Reading = true;
            }
        } else {
            // 更新状态机
            rs485Driver.update();

            if (rs485Driver.isComplete()) {
                const auto& weather = rs485Driver.getData();
                LOG_D("SENSOR", "Weather: T=%.1fC, H=%.1f%%", weather.temperature, weather.humidity);
                rs485Driver.reset();
                rs485Reading = false;
            } else if (rs485Driver.hasError()) {
                LOG_W("SENSOR", "RS485 read error");
                rs485Driver.reset();
                rs485Reading = false;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));  // 50Hz 轮询
    }
}

[[noreturn]] void taskStorage(void* pvParameters) {
    LOG_I("TASK", "T_Storage started on Core %d", xPortGetCoreID());

    IStorageService::DataRecord record{};

    while (true) {
        // 从队列接收 IMU 数据
        if (xQueueReceive(imuDataQueue, &record, pdMS_TO_TICKS(100)) == pdTRUE) {
            // 填充其他传感器数据
            record.timestamp = millis();
            
            // 写入 SD 卡
            if (sdDriver.isOnline()) {
                (void)sdDriver.writeRecord(record);
            }
        }

        // 每 2 秒同步一次
        static uint32_t lastSync = 0;
        if (millis() - lastSync > 2000) {
            lastSync = millis();
            sdDriver.sync();
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

[[noreturn]] void taskGNSS(void* pvParameters) {
    LOG_I("TASK", "T_GNSS started on Core %d", xPortGetCoreID());

    while (true) {
        // 读取 GNSS 串口数据
        while (SerialGNSS.available()) {
            uint8_t data = SerialGNSS.read();
            gnssDriver.feedData(&data, 1);
        }

        // 获取位置信息
        if (gnssDriver.hasFix()) {
            auto pos = gnssDriver.getPosition();
            LOG_D("GNSS", "Lat=%.6f, Lon=%.6f, Spd=%.1fkm/h",
                  pos.latitude, pos.longitude, pos.speed * 3.6f);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

[[noreturn]] void taskSystem(void* pvParameters) {
    LOG_I("TASK", "T_System started on Core %d", xPortGetCoreID());

    while (true) {
        // 1. BLE 数据发送 (如果已连接)
        if (bleDriver.isConnected()) {
            static uint32_t lastBleSend = 0;
            if (millis() - lastBleSend > 200) {  // 5Hz 发送频率
                lastBleSend = millis();
                
                // 发送 IMU 数据
                auto& output = imuMgr.getOutput();
                bleDriver.sendImuData(
                    output.attitude.roll,
                    output.attitude.pitch,
                    output.attitude.yaw,
                    output.speedKmh
                );
            }
            
            // 每 5 秒发送一次系统状态
            static uint32_t lastStatusSend = 0;
            if (millis() - lastStatusSend > 5000) {
                lastStatusSend = millis();
                bleDriver.sendSystemStatus(
                    ESP.getFreeHeap(),
                    uxTaskGetNumberOfTasks(),
                    millis() / 1000
                );
            }
        }

        // 2. 系统状态打印 (每 10 秒一次，避免刷屏)
        static uint32_t lastPrint = 0;
        if (millis() - lastPrint > 10000) {
            lastPrint = millis();
            LOG_I("SYS", "Heap: %d, Tasks: %d",
                  ESP.getFreeHeap(), uxTaskGetNumberOfTasks());
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
