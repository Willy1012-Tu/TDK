#pragma once
#include <stdint.h>
#include <FlexCAN_T4.h>

namespace ChassisController {
enum class Mode : uint8_t { Locked, Arming, Running };
struct AxisStatus {
    bool seen;
    uint32_t heartbeatAt, heartbeatAge, error;
    uint8_t state;
};
struct MotionSettings {
    bool rampEnabled;
    float acceleration, deceleration; // ODrive turns/s per second.
    float wheelbaseMm, trackWidthMm, icrXMm, icrLeftYMm, icrRightYMm; // +X forward, +Y right.
};
struct Status {
    Mode mode;
    const char *modeName, *reason;
    bool healthy, emergencyLatched;
    float wheel[4], targetWheel[4]; // Sent ramped commands and the latest mixed targets.
    MotionSettings motion;
    bool motionEditable, processionMode;
    AxisStatus axes[4];
    uint8_t directionStage; // Accepted pending selection takes effect before mechanism return completes.
};
void begin();
void poll();
void receive(uint8_t bus, const CAN_message_t &message);
void emergencyStop();
void clearErrors();
Status status();
MotionSettings defaultMotionSettings(uint8_t profile = 0); // 0 normal, 1 CH14 procession.
bool configureMotion(const MotionSettings &settings, uint8_t expectedProfile = 255);
 // RAM only, stationary/neutral; explicit profile rejects a changed-bank request.
} // namespace ChassisController
