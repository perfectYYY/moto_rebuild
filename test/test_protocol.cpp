/**
 * @file test_protocol.cpp
 * @brief 协议层逻辑验证 (运行在 PC 本地)
 */

#include <unity.h>
#include <string.h>
#include <stdio.h>
#include "ProtocolDef.h"

// ==============================================================================
// 测试用例 1: 验证结构体大小与内存对齐
// ==============================================================================
void test_struct_packing(void) {
    // 验证 FrameHeader 是否严格为 4 字节
    TEST_ASSERT_EQUAL_INT(4, sizeof(Protocol::FrameHeader));

    // 验证 StatusPayload 是否严格为 37 字节
    TEST_ASSERT_EQUAL_INT(37, sizeof(Protocol::GloveStatusPayload));

    // 验证 ControlPayload 是否严格为 16 字节
    TEST_ASSERT_EQUAL_INT(16, sizeof(Protocol::GloveControlPayload));

    // 验证完整包的总长度计算
    size_t total_status_len = sizeof(Protocol::FrameHeader) + sizeof(Protocol::GloveStatusPayload) + 1; // +1 for checksum
    TEST_ASSERT_EQUAL_INT(42, total_status_len);
}

// ==============================================================================
// 测试用例 2: 验证校验和算法 (Sum8)
// ==============================================================================
void test_checksum_algorithm(void) {
    // Case A: 简单累加
    uint8_t data1[] = {0x01, 0x02, 0x03};
    uint8_t sum1 = Protocol::calculateChecksum(data1, 3);
    TEST_ASSERT_EQUAL_HEX8(0x06, sum1);

    // Case B: 溢出测试 (Sum8 是自然溢出的)
    // 200 + 100 = 300, 300 % 256 = 44 (0x2C)
    uint8_t data2[] = {200, 100};
    uint8_t sum2 = Protocol::calculateChecksum(data2, 2);
    TEST_ASSERT_EQUAL_HEX8(0x2C, sum2);

    // Case C: 实际帧模拟
    // Head(AA 55) + Type(01) + Len(00)
    // Sum = AA + 55 + 01 + 00 = FF + 01 = 00 (溢出)
    uint8_t frame[] = {0xAA, 0x55, 0x01, 0x00}; 
    uint8_t sum3 = Protocol::calculateChecksum(frame, 4);
    TEST_ASSERT_EQUAL_HEX8(0x00, sum3);
}

// ==============================================================================
// 测试用例 3: 模拟一次完整的数据封包 (Serialization)
// ==============================================================================
void test_packet_serialization(void) {
    // 1. 准备缓冲区
    uint8_t buffer[64] = {0};
    
    // 2. 填充帧头
    auto* header = reinterpret_cast<Protocol::FrameHeader*>(buffer);
    header->head = Protocol::HEAD_MAGIC; // 0xAA55 (注意小端序: 内存里是 55 AA)
    header->type = static_cast<uint8_t>(Protocol::FrameType::StatusReport); // 0x01
    header->len  = sizeof(Protocol::GloveStatusPayload); // 37

    // 3. 填充 Payload
    auto* payload = reinterpret_cast<Protocol::GloveStatusPayload*>(buffer + sizeof(Protocol::FrameHeader));
    payload->side_id = 'L'; // 0x4C
    payload->target_temp = 45;
    // 填充一个温度值测试大小端: 25.50度 -> 2550 -> 0x09F6
    // 小端序内存应为: F6 09
    payload->current_temps[0] = 2550; 

    // 4. 计算校验和
    // 范围: 从 0xAA 开始，到 Payload 结束
    size_t calc_len = sizeof(Protocol::FrameHeader) + sizeof(Protocol::GloveStatusPayload);
    uint8_t checksum = Protocol::calculateChecksum(buffer, calc_len);
    
    // 将校验和填入缓冲区末尾
    buffer[calc_len] = checksum;

    // --- 开始验证 ---
    
    // 验证帧头魔数 (小端序检查)
    TEST_ASSERT_EQUAL_HEX8(0x55, buffer[0]);
    TEST_ASSERT_EQUAL_HEX8(0xAA, buffer[1]);
    
    // 验证类型
    TEST_ASSERT_EQUAL_HEX8(0x01, buffer[2]);

    // 验证 Payload 数据
    TEST_ASSERT_EQUAL_HEX8('L', buffer[4]); // Side ID
    
    // 验证 int16_t 的小端序 (低字节在前)
    // current_temps[0] 偏移量 = 4(Header) + 1(Side) + 1(Target) = 6
    TEST_ASSERT_EQUAL_HEX8(0xF6, buffer[6]); 
    TEST_ASSERT_EQUAL_HEX8(0x09, buffer[7]);
}

// ==============================================================================
// 运行入口
// ==============================================================================
int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_struct_packing);
    RUN_TEST(test_checksum_algorithm);
    RUN_TEST(test_packet_serialization);
    return UNITY_END();
}