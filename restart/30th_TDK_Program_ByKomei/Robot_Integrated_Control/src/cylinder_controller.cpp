#include "cylinder_controller.h"
#include "system_config.h"
#include <Arduino.h>
namespace {
namespace Config = SystemConfig::Cylinder;
using MechanismConfig::CylinderAction;
enum class Phase : uint8_t { Idle, Deadtime, Moving, Hold };
Phase phase = Phase::Idle;
CylinderAction action = CylinderAction::Stop, referenceAction = CylinderAction::Stop;
bool referenceKnown = false, fullyRetracted = false, in1High = false, in2High = false;
uint32_t since = 0, durationMs = 0;
void lowBoth() {
  digitalWrite(Config::IN1_PIN, LOW);
  digitalWrite(Config::IN2_PIN, LOW);
  in1High = in2High = false;
}
} // namespace
namespace CylinderController {
void preparePins() {
#if defined(__IMXRT1062__)
  // Unlike digitalWrite on INPUT, these clear the actual output latches.
  // Clear both BEFORE enabling output, including warm resets with retained latch bits.
  digitalWriteFast(Config::IN1_PIN, LOW);
  digitalWriteFast(Config::IN2_PIN, LOW);
#else
  digitalWrite(Config::IN1_PIN, LOW);
  digitalWrite(Config::IN2_PIN, LOW);
#endif
  pinMode(Config::IN1_PIN, OUTPUT);
  pinMode(Config::IN2_PIN, OUTPUT);
}
void begin() {
  preparePins();
  in1High = in2High = false;
  phase = Phase::Idle;
  action = referenceAction = CylinderAction::Stop;
  referenceKnown = fullyRetracted = false;
  durationMs = 0;
}
void stop() {
  if (phase == Phase::Deadtime || phase == Phase::Moving)
    referenceKnown = fullyRetracted = false;
  lowBoth();
  phase = Phase::Idle;
  action = CylinderAction::Stop;
  durationMs = 0;
}
void invalidateReference() { referenceKnown = fullyRetracted = false; }
bool start(CylinderAction requested, uint32_t ms) {
  if (static_cast<uint8_t>(requested) > 2 || ms > Config::MAX_DURATION_MS ||
      (requested != CylinderAction::Stop && !ms))
    return false;
  stop();
  action = requested;
  durationMs = ms;
  since = millis();
  if (requested == CylinderAction::Stop) {
    phase = ms ? Phase::Hold : Phase::Idle;
    return true;
  }
  referenceKnown = fullyRetracted = false;
  // Every new motion waits a complete LOW/LOW interval, including first travel.
  phase = Phase::Deadtime;
  return true;
}
void poll() {
  if (phase == Phase::Idle)
    return;
  const uint32_t elapsed = uint32_t(millis() - since);
  if (phase == Phase::Deadtime) {
    if (elapsed < Config::REVERSAL_DEADTIME_MS)
      return;
    if (action == CylinderAction::Extend) {
      digitalWrite(Config::IN1_PIN, HIGH);
      in1High = true;
    } else {
      digitalWrite(Config::IN2_PIN, HIGH);
      in2High = true;
    }
    since = millis(); // The full duration begins when outputs actually energize.
    phase = Phase::Moving;
    return;
  }
  if (elapsed < durationMs)
    return;
  if (phase == Phase::Moving) {
    referenceAction = action;
    referenceKnown = true;
    fullyRetracted = action == CylinderAction::Retract && durationMs >= Config::HOME_RETRACT_MS;
  }
  lowBoth();
  phase = Phase::Idle;
  action = CylinderAction::Stop;
}
Status status() {
  const bool active = phase != Phase::Idle;
  uint32_t remaining = 0;
  if (phase == Phase::Deadtime)
    remaining = durationMs;
  else if (active) {
    const uint32_t elapsed = uint32_t(millis() - since);
    remaining = elapsed >= durationMs ? 0 : durationMs - elapsed;
  }
  return {action, active, durationMs, remaining, in1High, in2High, referenceAction, referenceKnown, fullyRetracted};
}
} // namespace CylinderController

#if defined(__IMXRT1062__)
// Runs after GPIO clocks/RAM init, before USB startup waits and C++ constructors.
// Call hardware-only code: the driver runtime is initialized later by begin().
extern "C" void startup_middle_hook(void) {
  CylinderController::preparePins();
}
#endif
