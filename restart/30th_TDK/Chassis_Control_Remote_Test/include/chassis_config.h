#pragma once
#include <stdint.h>

namespace Config {
// 順序固定為：前左、前右、後左、後右。保留原程式的輪向。
constexpr uint8_t NODE_ID[4] = {10, 11, 12, 13};
constexpr uint8_t CAN_BUS[4] = {1, 1, 2, 2};
constexpr uint32_t CAN_BAUD = 1000000; // CAN1 / CAN2：1 Mbps
constexpr uint32_t CRSF_BAUD = 420000;

// ODrive 速度單位為馬達 turns/s，不是 RPM，也不是底盤 m/s。
constexpr float MAX_SPEED = 90.0f;
constexpr int DEAD_ZONE = 30;
struct Channel {
    uint8_t number;
    int minimum, middle, maximum;
};
constexpr Channel FORWARD = {2, 1129, 1416, 1717};
constexpr Channel STRAFE  = {3, 1241, 1500, 1747};
constexpr Channel ROTATE  = {1, 1182, 1454, 1739};

constexpr uint32_t CENTER_HOLD_MS = 500;
constexpr uint32_t SEND_INTERVAL_MS = 15;
constexpr uint32_t HEARTBEAT_TIMEOUT_MS = 500;
constexpr uint32_t ARM_TIMEOUT_MS = 3000;
constexpr uint32_t IDLE_RETRY_MS = 250;
// 預設只印狀態切換。校正搖桿時可開啟定期通道／輪速輸出。
constexpr bool PRINT_STATUS = false;
constexpr uint32_t STATUS_INTERVAL_MS = 500;

constexpr bool validChannel(Channel c) {
    return c.number >= 1 && c.number <= 16 && c.minimum >= 900 &&
           c.maximum <= 2100 && c.minimum < c.middle - DEAD_ZONE &&
           c.maximum > c.middle + DEAD_ZONE;
}
static_assert(DEAD_ZONE >= 0 && validChannel(FORWARD) &&
              validChannel(STRAFE) && validChannel(ROTATE), "Invalid joystick calibration");
static_assert(MAX_SPEED > 0 && MAX_SPEED <= 10000, "Invalid speed limit");
static_assert(NODE_ID[0] < 63 && NODE_ID[1] < 63 && NODE_ID[2] < 63 && NODE_ID[3] < 63,
              "ODrive node IDs must be 0..62");
static_assert((CAN_BUS[0] == 1 || CAN_BUS[0] == 2) &&
              (CAN_BUS[1] == 1 || CAN_BUS[1] == 2) &&
              (CAN_BUS[2] == 1 || CAN_BUS[2] == 2) &&
              (CAN_BUS[3] == 1 || CAN_BUS[3] == 2), "CAN bus must be 1 or 2");
constexpr bool uniqueNode(uint8_t a, uint8_t b) {
    return CAN_BUS[a] != CAN_BUS[b] || NODE_ID[a] != NODE_ID[b];
}
static_assert(uniqueNode(0, 1) && uniqueNode(0, 2) && uniqueNode(0, 3) &&
              uniqueNode(1, 2) && uniqueNode(1, 3) && uniqueNode(2, 3), "Duplicate CAN node");
static_assert(CENTER_HOLD_MS > 0 && SEND_INTERVAL_MS > 0 &&
              HEARTBEAT_TIMEOUT_MS > 0 && ARM_TIMEOUT_MS > 0 &&
              IDLE_RETRY_MS > 0 && STATUS_INTERVAL_MS > 0, "Intervals must be positive");
} // namespace Config
