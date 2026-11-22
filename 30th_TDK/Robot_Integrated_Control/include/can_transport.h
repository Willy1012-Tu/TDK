#pragma once
#include <stdint.h>
#include <FlexCAN_T4.h>

// One owner for both physical buses; callbacks are delivered from poll().
namespace CanTransport {
void begin();
void poll();
bool send(uint8_t bus, uint32_t id, const uint8_t *data, uint8_t length);
}
