#pragma once
#include <math.h>
#include <cmath>
#include <stdint.h>
namespace ChassisRamp {
// A common interpolation fraction keeps wheel vectors kinematically compatible.
// Reversal reaches zero before accelerating in the opposite direction.
inline void advance(float (&current)[4], const float (&target)[4], uint32_t elapsedMs,
                    float acceleration, float deceleration) {
    if (!std::isfinite(acceleration) || !std::isfinite(deceleration) ||
        acceleration <= 0 || deceleration <= 0) {
        for (float &value : current) value = 0;
        return;
    }
    const float seconds = float(elapsedMs) * 0.001f;
    float fraction = 1.0f;
    for (uint8_t i = 0; i < 4; ++i) {
        if (!std::isfinite(current[i]) || !std::isfinite(target[i])) {
            for (float &value : current) value = 0;
            return;
        }
        const float delta = fabsf(target[i] - current[i]);
        if (delta == 0) continue;
        float allowance;
        if (current[i] * target[i] < 0)
            allowance = fminf(deceleration * seconds, fabsf(current[i]));
        else
            allowance = (fabsf(target[i]) > fabsf(current[i]) ? acceleration : deceleration) * seconds;
        fraction = fminf(fraction, allowance / delta);
    }
    for (uint8_t i = 0; i < 4; ++i) {
        current[i] = fraction >= 1 ? target[i] : current[i] + (target[i] - current[i]) * fraction;
        if (fabsf(current[i]) < 0.00001f) current[i] = 0;
    }
}
} // namespace ChassisRamp
