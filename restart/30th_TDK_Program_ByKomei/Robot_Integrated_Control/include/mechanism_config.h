#pragma once
#include "system_config.h"
#include "cylinder_types.h"
#include <stddef.h>
#include <stdint.h>

namespace MechanismConfig {
constexpr int SERVO2_START_US = SystemConfig::Mechanism::SERVO_START_US[0];
constexpr int SERVO3_START_US = SystemConfig::Mechanism::SERVO_START_US[1];
constexpr int SERVO5_START_US = SystemConfig::Mechanism::SERVO_START_US[3];
constexpr int SERVO4_START_US = SystemConfig::Mechanism::SERVO_START_US[2];
constexpr int SERVO6_START_US = SystemConfig::Mechanism::SERVO_START_US[4];
constexpr int SERVO9_START_US = SystemConfig::Mechanism::SERVO_START_US[5];
constexpr int SERVO10_START_US = SystemConfig::Mechanism::SERVO_START_US[6];
constexpr int SERVO11_START_US = SystemConfig::Mechanism::SERVO_START_US[7];
constexpr int SERVO12_START_US = SystemConfig::Mechanism::SERVO_START_US[8];
constexpr uint8_t normalizedProp(uint8_t stage, uint8_t prop = 0) {
  return stage == 1 ? (prop ? prop : SystemConfig::Mechanism::DEFAULT_STAGE1_PROP) : 0;
}
constexpr bool stageEnabled(uint8_t stage) {
  return stage >= 1 && stage <= SystemConfig::Mechanism::STAGE_COUNT &&
         (stage != 3 || SystemConfig::Mechanism::STAGE3_ENABLED);
}
static_assert(stageEnabled(SystemConfig::Mechanism::DEFAULT_STAGE), "Default stage must be enabled");
constexpr bool validProfile(uint8_t stage, uint8_t prop = 0) {
  return stageEnabled(stage) &&
         (stage == 1 ? prop <= SystemConfig::Mechanism::STAGE1_PROP_COUNT : prop == 0);
}
// Both mechanisms share ID6 trim control, with independently configurable ranges.
constexpr bool heightTrimStage(uint8_t stage) {
  return stage == 4 || stage == SystemConfig::Rc::FRUIT_STAGE;
}
constexpr int32_t heightTrimMaxOffset(uint8_t stage) {
  return stage == SystemConfig::Rc::FRUIT_STAGE ? SystemConfig::Rc::FRUIT_TRIM_MAX_OFFSET_PULSES
         : stage == 4 ? SystemConfig::Rc::STAGE4_TRIM_MAX_OFFSET_PULSES : 0;
}
constexpr int32_t boundedMotorPosition(uint8_t stage, uint8_t node, int64_t position) {
  const int32_t low = stage == 2 && node == 4 ? 0
                      : stage == SystemConfig::Rc::FRUIT_STAGE && node == 6
                          ? SystemConfig::Fruit::POSITION_MIN : -8388608;
  const int32_t high = stage == 2 && node == 5 ? 0
                       : node == 6 ? SystemConfig::ID6_POSITION_MAX : 8388607;
  return position < low ? low : position > high ? high : static_cast<int32_t>(position);
}
// Five Stage 1 props occupy 0..4; normal stages 2..6 occupy 5..9.
constexpr uint8_t profileKey(uint8_t stage, uint8_t prop = 0) {
  return stage == 1 ? normalizedProp(stage, prop) - 1
                    : SystemConfig::Mechanism::STAGE1_PROP_COUNT + stage - 2;
}
constexpr bool activeMotor(uint8_t node) {
  for (uint8_t id : SystemConfig::Mechanism::MOTOR_IDS)
    if (id == node)
      return true;
  return false;
}
constexpr bool homingMotor(uint8_t node) {
  for (uint8_t id : SystemConfig::Mechanism::HOMING_IDS)
    if (id == node)
      return true;
  return false;
}
constexpr bool activeServo(uint8_t pin) {
  for (uint8_t configured : SystemConfig::Mechanism::SERVO_PINS)
    if (configured == pin)
      return true;
  return false;
}
enum class ActionKind : uint8_t { Stepper, Servo, Delay, WaitAll, Cylinder };
struct Action {
  ActionKind kind;
  int32_t position;
  uint16_t rpm;
  uint8_t acc, pin;
  int pulse;
  uint32_t ms;
  bool wait;
  uint16_t substep;
  uint8_t node = SystemConfig::Mechanism::MOTOR_ID;
  CylinderAction cylinderAct = CylinderAction::Stop;
};
constexpr Action stepperFor(uint8_t node, int32_t position, uint16_t rpm, uint8_t acc = 0,
                            bool wait = true) {
  return {ActionKind::Stepper, position, rpm, acc, 0, 0, 0, wait, 0, node};
}
constexpr Action stepper(int32_t position, uint16_t rpm, uint8_t acc = 0, bool wait = true) {
  return stepperFor(SystemConfig::Mechanism::MOTOR_ID, position, rpm, acc, wait);
}
constexpr Action servo(uint8_t pin, int pulse, uint32_t ms = 0, bool wait = true) {
  return {ActionKind::Servo, 0, 0, 0, pin, pulse, ms, wait, 0, 0};
}
constexpr Action cylinder(CylinderAction action, uint32_t ms, bool wait = true) {
  return {ActionKind::Cylinder, 0, 0, 0, 0, 0, ms, wait, 0, 0, action};
}
constexpr Action delay(uint32_t ms) {
  return {ActionKind::Delay, 0, 0, 0, 0, 0, ms, true, 0, 0};
}
constexpr Action waitAll() {
  return {ActionKind::WaitAll, 0, 0, 0, 0, 0, 0, true, 0, 0};
}
constexpr Action branch(Action action, uint16_t number) {
  action.substep = number;
  return action;
}
struct Step {
  const Action *actions;
  size_t count;
};
template <size_t N> constexpr Step step(const Action (&actions)[N]) {
  return {actions, N};
}
constexpr bool valid(const Action &a) {
  switch (a.kind) {
  case ActionKind::Stepper:
    return activeMotor(a.node) && a.position >= -8388608 &&
           a.position <= 8388607 && a.rpm >= 1 && a.rpm <= 3000;
  case ActionKind::Servo:
    return activeServo(a.pin) && a.pulse >= 500 && a.pulse <= 2500 &&
           a.ms <= 60000;
  case ActionKind::Delay:
    return a.ms <= 60000;
  case ActionKind::WaitAll:
    return true;
  case ActionKind::Cylinder:
    return static_cast<uint8_t>(a.cylinderAct) <= 2 &&
           a.ms <= SystemConfig::Cylinder::MAX_DURATION_MS &&
           (a.cylinderAct == CylinderAction::Stop || a.ms > 0);
  }
  return false;
}
constexpr Action HOME14[] = {stepper(0, 320, 0, false), servo(2, SERVO2_START_US, 0, false),
                             servo(5, SERVO5_START_US, 0, false), waitAll()};
constexpr Action HOME6[] = {
    stepper(0, SystemConfig::Fruit::RETURN_RPM, SystemConfig::Fruit::RETURN_ACC, false),
    servo(2, SystemConfig::Fruit::SERVO2_HOME_US, SystemConfig::Fruit::SERVO_HOME_DURATION_MS, false),
    servo(5, SystemConfig::Fruit::SERVO5_HOME_US, SystemConfig::Fruit::SERVO_HOME_DURATION_MS, false),
    waitAll()};
constexpr Action ENTRY2[] = {
    cylinder(CylinderAction::Retract, SystemConfig::Cylinder::HOME_RETRACT_MS, false),
    servo(9, SystemConfig::Stage2::SERVO9_ENTRY_US, 0, false),
    servo(4, SystemConfig::Stage2::SERVO4_ENTRY_US, 0, false),
    servo(3, SystemConfig::Stage2::SERVO3_ENTRY_US, 0, false),
    servo(6, SERVO6_START_US, 0, false), waitAll()};
constexpr Action HOME2[] = {stepperFor(4, 0, 320, 0, false),
                            stepperFor(5, 0, 320, 0, false),
                            servo(3, SERVO3_START_US, 0, false), cylinder(CylinderAction::Retract, SystemConfig::Cylinder::HOME_RETRACT_MS, false),
                            servo(4, SERVO4_START_US, 0, false),
                            servo(6, SERVO6_START_US, 0, false),
                            servo(9, SERVO9_START_US, 0, false), waitAll()};
// Stage 3 has no home switches. HOME returns to its stationary startup zero via FE.
constexpr Action HOME3[] = {stepperFor(4, 0, 320, 0, false),
                            stepperFor(5, 0, 320, 0, false),
                            servo(3, SERVO3_START_US, 0, false), waitAll()};
// The fifth-stage placeholder never touches hardware on manual HOME.
constexpr Action HOME5[] = {delay(0)};
constexpr Action STARTUP_HOME[] = {stepperFor(6, 0, 320, 0, false),
                                   servo(2, SERVO2_START_US, 0, false),
                                   servo(5, SERVO5_START_US, 0, false), waitAll()};
// Leaving stage 3 returns both axes to their stationary startup zero via FE.
// Never command the former -2000/+2000 park beyond the mechanism zero limits.
constexpr Action PARK3[] = {stepperFor(4, 0, 320, 0, false),
                            stepperFor(5, 0, 320, 0, false),
                            servo(3, SERVO3_START_US, 0, false), waitAll()};
template <size_t N> constexpr bool validTable(const Step (&steps)[N]) {
  static_assert(N >= 1 && N <= 254, "A profile must have 1..254 steps");
  for (const auto &s : steps) {
    if (!s.count)
      return false;
    for (size_t i = 0; i < s.count; ++i)
      if (!valid(s.actions[i]))
        return false;
  }
  return true;
}
} // namespace MechanismConfig
#include <stage1_steps.generated.h>
#include <stage2_steps.generated.h>
#include <stage3_steps.generated.h>
#include <stage4_steps.generated.h>
#include <stage5_steps.generated.h>
#include <stage6_steps.generated.h>
#include <stage1_basket.generated.h>
#include <stage1_vase.generated.h>
#include <stage1_tube.generated.h>
#include <stage1_chicken.generated.h>
#include <stage1_bird.generated.h>
namespace MechanismConfig {
static_assert(validTable(Stage1::STEPS) && validTable(Stage1Basket::STEPS) &&
                  validTable(Stage1Vase::STEPS) && validTable(Stage1Tube::STEPS) &&
                  validTable(Stage1Chicken::STEPS) && validTable(Stage1Bird::STEPS) &&
                  validTable(Stage2::STEPS) && validTable(Stage3::STEPS) &&
                  validTable(Stage4::STEPS) && validTable(Stage5::STEPS) && validTable(Stage6::STEPS),
              "Invalid YAML actuator or step count");
}
