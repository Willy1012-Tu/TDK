#pragma once
#include "Arduino.h"
struct AlfredoCRSF {
    int channels[17] = {};
    bool linked = false;
    void begin(MockSerial &) {}
    void update() {}
    int getChannel(unsigned int number) const { return channels[number]; }
    bool isLinkUp() const { return linked; }
};
