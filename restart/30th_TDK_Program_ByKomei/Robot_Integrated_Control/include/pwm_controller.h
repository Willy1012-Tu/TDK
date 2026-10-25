#pragma once
#include <stdint.h>

namespace PwmController {
struct Status {
  uint8_t pin, value, maxValue;
  uint32_t stepIntervalMs;
  bool strobeEnabled, rcControlled;
};
void begin();
void poll();
uint8_t currentValue();
// RC takes ownership: true starts full-brightness strobe, false holds output off.
void setRcStrobe(bool enabled);
// RAM-only breathing settings: peak 0..255 (0 turns it off), interval 1..1000 ms.
// An active RC strobe retains ownership; otherwise configuration resumes breathing.
bool configure(uint16_t maxValue, uint32_t stepIntervalMs);
Status status();
} // namespace PwmController
