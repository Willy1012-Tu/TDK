#pragma once
#include "chassis_config.h"
#include <math.h>

inline bool channelValid(int value) {
    return value >= 900 && value <= 2100;
}

inline bool centered(int value, Config::Channel channel) {
    return value >= channel.middle - Config::DEAD_ZONE &&
           value <= channel.middle + Config::DEAD_ZONE;
}

inline float mapJoystick(int value, Config::Channel channel) {
    if (value < channel.minimum) value = channel.minimum;
    if (value > channel.maximum) value = channel.maximum;
    if (centered(value, channel)) return 0.0f;
    if (value > channel.middle)
        return float(value - channel.middle - Config::DEAD_ZONE) /
               float(channel.maximum - channel.middle - Config::DEAD_ZONE);
    return -float(channel.middle - Config::DEAD_ZONE - value) /
            float(channel.middle - Config::DEAD_ZONE - channel.minimum);
}

inline void mixWheels(int forward, int strafe, int rotate, float (&wheel)[4]) {
    const float vx = mapJoystick(forward, Config::FORWARD);
    const float vy = mapJoystick(strafe, Config::STRAFE);
    const float vw = mapJoystick(rotate, Config::ROTATE);
    wheel[0] = -(vx + vy + vw);
    wheel[1] =  (vx - vy - vw);
    wheel[2] =  (vx - vy + vw);
    wheel[3] = -(vx + vy - vw);
    float peak = 1.0f;
    for (float value : wheel) peak = fmaxf(peak, fabsf(value));
    for (float &value : wheel) value *= Config::MAX_SPEED / peak;
}
