#pragma once
#include <stdint.h>
#include <string>
inline uint32_t testNow = 0;
inline uint32_t millis() { return testNow; }
struct MockSerial {
    bool connected = true;
    uint32_t baud = 0;
    void begin(uint32_t value) { baud = value; }
    explicit operator bool() const { return connected; }
    int availableForWrite() const { return 256; }
    void println(const char *) {}
    template<class... Args> void printf(const char *, Args...) {}
};
inline MockSerial Serial, Serial2;
