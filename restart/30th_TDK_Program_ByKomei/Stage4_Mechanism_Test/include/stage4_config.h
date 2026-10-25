#pragma once
#include <stddef.h>
#include <stdint.h>
#include "test_config.h"

namespace Stage4 {
constexpr int SERVO2_START_US = 1150, SERVO5_START_US = 2100;
enum class ActionKind : uint8_t { Stepper, Servo, Delay, WaitAll };
struct Action {
  ActionKind kind;
  int32_t position;
  uint16_t rpm;
  uint8_t acc, pin;
  int pulse;
  uint32_t ms;
  bool wait;
  uint16_t substep;
  uint8_t node = TestConfig::MOTOR_ID;
};
constexpr Action stepperFor(uint8_t node, int32_t position, uint16_t rpm, uint8_t acc=0, bool wait=true) {
  return {ActionKind::Stepper, position, rpm, acc, 0, 0, 0, wait, 0, node};
}
// 省略 node 時仍使用 ID 6，保留舊單馬達設定介面。
constexpr Action stepper(int32_t position, uint16_t rpm, uint8_t acc=0, bool wait=true) {
  return stepperFor(TestConfig::MOTOR_ID, position, rpm, acc, wait);
}
constexpr Action servo(uint8_t pin, int pulse, uint32_t ms=0, bool wait=true) {
  return {ActionKind::Servo, 0, 0, 0, pin, pulse, ms, wait, 0, 0};
}
constexpr Action delay(uint32_t ms) { return {ActionKind::Delay, 0, 0, 0, 0, 0, ms, true, 0, 0}; }
constexpr Action waitAll() { return {ActionKind::WaitAll, 0, 0, 0, 0, 0, 0, true, 0, 0}; }
constexpr Action branch(Action a, uint16_t number) { a.substep=number; return a; }
struct Step { const Action *actions; size_t count; };
template<size_t N> constexpr Step step(const Action (&actions)[N]) { return {actions, N}; }
constexpr bool activeMotor(uint8_t node) {
  for(auto id:TestConfig::MOTOR_IDS) if(id==node) return true;
  return false;
}
constexpr bool activeServo(uint8_t pin) {
  for(auto enabled:TestConfig::SERVO_PINS) if(enabled==pin) return true;
  return false;
}
constexpr bool valid(const Action &a) {
  switch (a.kind) {
    case ActionKind::Stepper:
      return activeMotor(a.node) && a.position>=-8388607
        && a.position<=8388607 && a.rpm>=1 && a.rpm<=3000;
    case ActionKind::Servo:
      return activeServo(a.pin) && a.pulse>=500 && a.pulse<=2500 && a.ms<=60000;
    case ActionKind::Delay: return a.ms<=60000;
    case ActionKind::WaitAll: return true;
  }
  return false;
}
constexpr Action HOME[] = {
  stepper(0, TestConfig::HOME_RPM, TestConfig::HOME_ACC, false),
  servo(2, SERVO2_START_US, 0, false), servo(5, SERVO5_START_US, 0, false), waitAll()
};
constexpr Action HOME2[] = {
  stepperFor(7,0,TestConfig::HOME_RPM,TestConfig::HOME_ACC,false),
  servo(4,2000,0,false),servo(6,1400,0,false),servo(9,1800,0,false),waitAll()
};
constexpr Action HOME3[] = {delay(0)};
constexpr Action STARTUP_HOME[] = {
  stepperFor(6,0,TestConfig::HOME_RPM,TestConfig::HOME_ACC,false),
  stepperFor(7,0,TestConfig::HOME_RPM,TestConfig::HOME_ACC,false),waitAll()
};
constexpr bool validForStage(uint8_t stage,const Action &a) {
  return valid(a) && (stage!=3 || (a.kind==ActionKind::Delay && a.ms==0));
}
}
#include <stage4_steps.generated.h>
#include <stage3_steps.generated.h>
#include <stage2_steps.generated.h>
namespace Stage4 {
constexpr size_t STEP_COUNT = sizeof(STEPS)/sizeof(STEPS[0]);
constexpr size_t STAGE2_STEP_COUNT = sizeof(Stage2::STEPS)/sizeof(Stage2::STEPS[0]);
constexpr size_t STAGE3_STEP_COUNT = sizeof(Stage3::STEPS)/sizeof(Stage3::STEPS[0]);
template<size_t N> constexpr bool validTable(const Step (&steps)[N],uint8_t stage=4) {
  for (const auto &s:steps) {
    if (!s.count) return false;
    for (size_t i=0;i<s.count;++i) if (!validForStage(stage,s.actions[i])) return false;
  }
  return true;
}
static_assert(STEP_COUNT>=1 && STEP_COUNT<=254 && STAGE2_STEP_COUNT>=1 && STAGE2_STEP_COUNT<=254 && STAGE3_STEP_COUNT>=1 && STAGE3_STEP_COUNT<=254,
  "關卡須為 1..254 段");
static_assert(validTable(STEPS) && validTable(Stage3::STEPS,3) && validTable(Stage2::STEPS,2), "YAML 動作超出硬體設定範圍");
static_assert(REPEAT_FROM>=1 && REPEAT_FROM<=STEP_COUNT && Stage3::REPEAT_FROM>=1
  && Stage3::REPEAT_FROM<=STAGE3_STEP_COUNT && Stage2::REPEAT_FROM>=1
  && Stage2::REPEAT_FROM<=STAGE2_STEP_COUNT, "循環起點無效");
}
