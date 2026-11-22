#pragma once
#include <stdint.h>

namespace RcController {
struct Status {
    bool link, valid, neutral;
    int channels[16];
    int forward, strafe, rotate;
    int stage1TrimUs; // Effective CH4 offset; zero outside the eligible Stage 1 context.
    int32_t stage4TrimPulses; // Requested CH4 offset in General Stage4/6; legacy field name retained.
    int32_t stage2TrimPulses; // CH4 paired ID4/5 height offset in General Stage2.
    uint8_t selectedStage;
    uint32_t ageMs, rejectedActions;
    bool stage1Mode, processionMode; // CH14 General selects chassis settings only.
    uint8_t selectedProp, switchPosition; // 0=LOW, 1=MID, 2=HIGH, 255=invalid.
    bool lightStrobeEnabled; // Debounced General CH13; independent of mechanism/chassis interlocks.
};
void begin();
void poll();
Status status();
} // namespace RcController
