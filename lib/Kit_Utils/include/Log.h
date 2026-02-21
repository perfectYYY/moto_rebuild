#pragma once
#include <Arduino.h>

// ==============================================================================
// 1. 日志级别定义 (更新)
// ==============================================================================
#define LOG_LEVEL_NONE  0
#define LOG_LEVEL_ERROR 1
#define LOG_LEVEL_WARN  2  // 新增 Warning
#define LOG_LEVEL_INFO  3
#define LOG_LEVEL_DEBUG 4

// 默认级别
#ifndef ACTIVE_LOG_LEVEL
#define ACTIVE_LOG_LEVEL LOG_LEVEL_DEBUG
#endif

// ==============================================================================
// 2. 宏实现
// ==============================================================================

// DEBUG
#if ACTIVE_LOG_LEVEL >= LOG_LEVEL_DEBUG
    #define LOG_D(tag, format, ...) Serial.printf("[%lu][D][%s] " format "\r\n", millis(), tag, ##__VA_ARGS__)
#else
    #define LOG_D(tag, format, ...) do {} while(0)
#endif

// INFO
#if ACTIVE_LOG_LEVEL >= LOG_LEVEL_INFO
    #define LOG_I(tag, format, ...) Serial.printf("[%lu][I][%s] " format "\r\n", millis(), tag, ##__VA_ARGS__)
#else
    #define LOG_I(tag, format, ...) do {} while(0)
#endif

// WARN (新增)
#if ACTIVE_LOG_LEVEL >= LOG_LEVEL_WARN
    #define LOG_W(tag, format, ...) Serial.printf("[%lu][W][%s] " format "\r\n", millis(), tag, ##__VA_ARGS__)
#else
    #define LOG_W(tag, format, ...) do {} while(0)
#endif

// ERROR
#if ACTIVE_LOG_LEVEL >= LOG_LEVEL_ERROR
    #define LOG_E(tag, format, ...) Serial.printf("[%lu][E][%s] " format "\r\n", millis(), tag, ##__VA_ARGS__)
#else
    #define LOG_E(tag, format, ...) do {} while(0)
#endif