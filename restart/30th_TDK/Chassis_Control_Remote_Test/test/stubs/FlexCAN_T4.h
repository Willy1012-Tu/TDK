#pragma once
#include <stdint.h>
#include <deque>
#include <vector>
enum { CAN1, CAN2, RX_SIZE_256, TX_SIZE_16 };
struct CAN_message_t {
    uint32_t id = 0;
    uint8_t len = 0;
    struct { bool extended = false, remote = false; } flags;
    uint8_t buf[8] = {};
};
template<int, int, int> struct FlexCAN_T4 {
    std::deque<CAN_message_t> incoming;
    std::vector<CAN_message_t> sent;
    uint32_t baud = 0;
    bool fifo = false;
    int writes = 0, failAt = -1;
    void begin() {}
    void setBaudRate(uint32_t value) { baud = value; }
    void enableFIFO() { fifo = true; }
    void events() {}
    int write(const CAN_message_t &msg) {
        if (++writes == failAt) return 0;
        sent.push_back(msg);
        return 1;
    }
    bool read(CAN_message_t &msg) {
        if (incoming.empty()) return false;
        msg = incoming.front(); incoming.pop_front(); return true;
    }
};
