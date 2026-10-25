#include "pwm_controller.h"
#include "system_config.h"
#include <Arduino.h>

namespace {
namespace Config = SystemConfig::Pwm;
enum class Mode : uint8_t { Breathe, Off, Strobe };
Mode mode = Mode::Breathe;
bool rcControlled = false;
uint8_t pwmValue = 0, maximumValue = Config::MAX_VALUE;
int8_t stepDirection = 1;
uint32_t lastStepMs = 0, stepIntervalMs = Config::STEP_INTERVAL_MS;
} // namespace

namespace PwmController {
void begin() {
  mode = Mode::Breathe;
  rcControlled = false;
  pwmValue = 0;
  maximumValue = Config::MAX_VALUE;
  stepIntervalMs = Config::STEP_INTERVAL_MS;
  stepDirection = 1;
  pinMode(Config::PIN, OUTPUT);
  analogWriteResolution(Config::RESOLUTION_BITS);
  analogWrite(Config::PIN, pwmValue);
  lastStepMs = millis();
}
void poll() {
  if (mode == Mode::Off) return;
  const uint32_t now = millis();
  if (mode == Mode::Strobe) {
    if (uint32_t(now - lastStepMs) < Config::STROBE_INTERVAL_MS) return;
    // One toggle per poll; delayed calls never create a catch-up burst.
    lastStepMs = now;
    pwmValue = pwmValue ? 0 : Config::STROBE_VALUE;
    analogWrite(Config::PIN, pwmValue);
    return;
  }
  if (!maximumValue || uint32_t(now - lastStepMs) < stepIntervalMs) return;
  // One bounded update per poll; a delayed loop never creates a catch-up burst.
  lastStepMs = now;
  pwmValue = static_cast<uint8_t>(int(pwmValue) + int(stepDirection));
  analogWrite(Config::PIN, pwmValue);
  if (pwmValue == 0 || pwmValue == maximumValue)
    stepDirection = static_cast<int8_t>(-stepDirection);
}
uint8_t currentValue() { return pwmValue; }
void setRcStrobe(bool enabled) {
  const Mode nextMode = enabled ? Mode::Strobe : Mode::Off;
  if (rcControlled && mode == nextMode) return;
  mode = nextMode;
  rcControlled = true;
  pwmValue = enabled ? Config::STROBE_VALUE : 0;
  analogWrite(Config::PIN, pwmValue);
  lastStepMs = millis();
}
bool configure(uint16_t maxValue, uint32_t intervalMs) {
  if (maxValue > 255 || intervalMs < Config::MIN_STEP_INTERVAL_MS ||
      intervalMs > Config::MAX_STEP_INTERVAL_MS) return false;
  const bool settingsChanged = maximumValue != maxValue || stepIntervalMs != intervalMs;
  maximumValue = static_cast<uint8_t>(maxValue);
  stepIntervalMs = intervalMs;
  // Store breathing parameters without taking over a running RC strobe.
  if (mode == Mode::Strobe) return true;
  if (!settingsChanged && !rcControlled && mode == Mode::Breathe) return true;
  const uint8_t previousValue = pwmValue;
  mode = maximumValue ? Mode::Breathe : Mode::Off;
  rcControlled = false;
  if (pwmValue > maximumValue) pwmValue = maximumValue;
  if (!pwmValue) stepDirection = 1;
  else if (pwmValue == maximumValue) stepDirection = -1;
  if (pwmValue != previousValue || !maximumValue)
    analogWrite(Config::PIN, pwmValue);
  lastStepMs = millis();
  return true;
}
Status status() {
  return {Config::PIN, pwmValue, maximumValue, stepIntervalMs,
          mode == Mode::Strobe, rcControlled};
}
} // namespace PwmController
