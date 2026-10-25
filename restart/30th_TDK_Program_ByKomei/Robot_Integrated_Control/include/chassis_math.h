#pragma once
#include "system_config.h"
#include <math.h>
#include <cmath>

namespace ChassisMath {
inline bool channelValid(int value) {
    return value >= SystemConfig::Rc::CHANNEL_MIN_US &&
           value <= SystemConfig::Rc::CHANNEL_MAX_US;
}
inline bool centered(int value, SystemConfig::Chassis::Channel channel) {
    return value >= channel.middle - SystemConfig::Chassis::DEAD_ZONE &&
           value <= channel.middle + SystemConfig::Chassis::DEAD_ZONE;
}
inline float mapJoystick(int value, SystemConfig::Chassis::Channel channel) {
    using namespace SystemConfig::Chassis;
    if (value < channel.minimum) value = channel.minimum;
    if (value > channel.maximum) value = channel.maximum;
    if (centered(value, channel)) return 0.0f;
    if (value > channel.middle)
        return float(value - channel.middle - DEAD_ZONE) /
               float(channel.maximum - channel.middle - DEAD_ZONE);
    return -float(channel.middle - DEAD_ZONE - value) /
            float(channel.middle - DEAD_ZONE - channel.minimum);
}
struct IcrGeometry {
    float wheelbaseMm, trackWidthMm, xMm, yMm;
};
constexpr IcrGeometry DEFAULT_ICR = {SystemConfig::Chassis::WHEELBASE_MM,
    SystemConfig::Chassis::TRACK_WIDTH_MM, 0.0f, 0.0f};
// Pure math defaults stay centered. The controller selects the manual turn ICR
// after calibrated dead-zone mapping; centered sticks never select a side.
inline IcrGeometry turnIcr(float rotation, float wheelbase, float track,
                           float x, float leftY, float rightY) {
    if (!std::isfinite(rotation) || rotation == 0) return {wheelbase, track, 0, 0};
    return {wheelbase, track, x,
        rotation * SystemConfig::Chassis::ICR_LEFT_ROTATE_SIGN > 0 ? leftY : rightY};
}
inline bool validIcr(const IcrGeometry &g) {
    using namespace SystemConfig::Chassis;
    return std::isfinite(g.wheelbaseMm) && g.wheelbaseMm >= 1 && g.wheelbaseMm <= MAX_GEOMETRY_MM &&
           std::isfinite(g.trackWidthMm) && g.trackWidthMm >= 1 && g.trackWidthMm <= MAX_GEOMETRY_MM &&
           std::isfinite(g.xMm) && fabsf(g.xMm) <= MAX_ICR_OFFSET_MM &&
           std::isfinite(g.yMm) && fabsf(g.yMm) <= MAX_ICR_OFFSET_MM;
}
// Existing electrical wheel signs are retained. Body +X is forward, +Y right.
// Shift the rotational velocity field from chassis center to the chosen ICR.
// Normalized angular input at the center corresponds to rim speed / (half-length + half-width).
inline void rotationBasis(const IcrGeometry &g, float (&basis)[4]) {
    const float k = 0.5f * (g.wheelbaseMm + g.trackWidthMm);
    const float x = g.xMm / k, y = g.yMm / k;
    basis[0] = -1.0f + x - y;
    basis[1] = -1.0f + x + y;
    basis[2] =  1.0f + x + y;
    basis[3] =  1.0f + x - y;
}
inline void mixWheels(int forward, int strafe, int rotate, float (&wheel)[4],
                      uint8_t stage = 4, const IcrGeometry &icr = DEFAULT_ICR) {
    using namespace SystemConfig::Chassis;
    // Normalize each RC axis using its own center/range before mapping stages.
    const float raw_vx = mapJoystick(forward, FORWARD);
    const float raw_vy = mapJoystick(strafe, STRAFE);
    const float vw = mapJoystick(rotate, ROTATE);
    float vx = raw_vx, vy = raw_vy;
    if (stage == 1) {
        // Stage 1: rigid +90 degree rotation of the translation vector.
        vx = -raw_vy;
        vy = raw_vx;
    } else if (stage == 2) {
        // Stage 2: rigid 180 degree rotation; both translation axes reverse.
        vx = -raw_vx;
        vy = -raw_vy;
    }
    // Stages 3/4 (and unknown values) retain the original frame.
    // Manual rotation always uses the body rotation signs.
    if (!validIcr(icr)) { for (float &value : wheel) value = 0; return; }
    float basis[4]; rotationBasis(icr, basis);
    wheel[0] = -(vx + vy) + vw * basis[0];
    wheel[1] =  (vx - vy) + vw * basis[1];
    wheel[2] =  (vx - vy) + vw * basis[2];
    wheel[3] = -(vx + vy) + vw * basis[3];
    float peak = 1.0f;
    for (float value : wheel) peak = fmaxf(peak, fabsf(value));
    for (float &value : wheel) value *= JOYSTICK_MAX_SPEED / peak;
}
// Final defensive guard for outgoing wheel targets.
inline void clampWheelSpeeds(float (&wheel)[4]) {
    using namespace SystemConfig::Chassis;
    for (float &value : wheel) {
        if (!std::isfinite(value)) value = 0.0f;
        if (value > SAFETY_MAX_SPEED) value = SAFETY_MAX_SPEED;
        if (value < -SAFETY_MAX_SPEED) value = -SAFETY_MAX_SPEED;
    }
}
} // namespace ChassisMath
