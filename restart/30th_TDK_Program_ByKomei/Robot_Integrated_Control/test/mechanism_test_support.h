#pragma once
#include "can_transport.h"
#include "mechanism_controller.h"
#include "rc_controller.h"
#include "system_safety.h"
#include <Arduino.h>
#include <Servo.h>
#include <iostream>
#include <stdexcept>
#include <string>
#define CHECK(x)                                                                                   \
  do {                                                                                             \
    if (!(x))                                                                                      \
      throw std::runtime_error(std::to_string(__LINE__) + ": " + #x);                              \
  } while (0)
namespace MechanismSupport {
using namespace MechanismController;
inline size_t ackCursor = 0;
// Standalone mechanism tests inject the RC snapshot without linking the CRSF driver.
// Legacy cases remain outside Stage1 context; dedicated trim cases opt in.
inline RcController::Status trimRadio{};
inline void trimContext(int offset = 0) {
  trimRadio.link = trimRadio.valid = trimRadio.stage1Mode = true;
  trimRadio.stage1TrimUs = offset;
}
// Field-tuned YAML profiles are the motion source; native tests verify their
// dispatch, barriers, and actual completed states without freezing old tuning.
inline uint8_t propStepCount(uint8_t prop) {
    switch (prop) {
    case 1: return Stage1Basket::STEP_COUNT;
    case 2: return Stage1Vase::STEP_COUNT;
    case 3: return Stage1Tube::STEP_COUNT;
    case 4: return Stage1Chicken::STEP_COUNT;
    case 5: return Stage1Bird::STEP_COUNT;
    default: throw std::runtime_error("Invalid prop test profile");
    }
}
inline const MechanismConfig::Step &propStep(uint8_t prop, uint8_t step) {
    CHECK(step >= 1 && step <= propStepCount(prop));
    switch (prop) {
    case 1: return Stage1Basket::STEPS[step - 1];
    case 2: return Stage1Vase::STEPS[step - 1];
    case 3: return Stage1Tube::STEPS[step - 1];
    case 4: return Stage1Chicken::STEPS[step - 1];
    case 5: return Stage1Bird::STEPS[step - 1];
    default: throw std::runtime_error("Invalid prop test profile");
    }
}
inline MechanismConfig::Action propMotor(uint8_t prop, uint8_t step) {
    auto result = MechanismConfig::stepperFor(6, 0, 320);
    for (uint8_t n = 1; n <= step; ++n) {
        const auto &plan = propStep(prop, n);
        for (size_t i = 0; i < plan.count; ++i)
            if (plan.actions[i].kind == MechanismConfig::ActionKind::Stepper && plan.actions[i].node == 6)
                result = plan.actions[i];
    }
    return result;
}
inline int propPulse(uint8_t prop, uint8_t step, uint8_t pin) {
    int pulse = 1500;
    for (size_t i = 0; i < SystemConfig::Mechanism::SERVO_COUNT; ++i)
        if (SystemConfig::Mechanism::SERVO_PINS[i] == pin) pulse = SystemConfig::Mechanism::SERVO_START_US[i];
    for (uint8_t n = 1; n <= step; ++n) {
        const auto &plan = propStep(prop, n);
        for (size_t i = 0; i < plan.count; ++i)
            if (plan.actions[i].kind == MechanismConfig::ActionKind::Servo && plan.actions[i].pin == pin)
                pulse = plan.actions[i].pulse;
    }
    return pulse;
}

inline void pump(unsigned count = 8) {
  while (count--)
    poll();
}
inline void advance(uint32_t ms) {
  mockMillis += ms;
  pump();
}
inline void reply(uint8_t node, uint8_t command, uint8_t value, uint8_t bus = 1, bool bad = false) {
  CAN_message_t f{};
  f.id = node;
  f.len = 3;
  f.buf[0] = command;
  f.buf[1] = value;
  f.buf[2] = static_cast<uint8_t>(node + command + value + (bad ? 1 : 0));
  receive(bus, f);
}
inline void finish(uint8_t node = 6, uint8_t command = 0xFE) {
  reply(node, command, 1);
  reply(node, command, 2);
  pump();
}
inline void finishPair() {
  finish(4);
  finish(5);
}
inline int count(uint8_t command, int node = -1) {
  int result = 0;
  for (const auto &f : Mock::sent)
    if (f.msg.buf[0] == command && (node < 0 || f.msg.id == uint32_t(node)))
      ++result;
  return result;
}
inline int countEnable(uint8_t node) {
  int result = 0;
  for (const auto &f : Mock::sent)
    if (f.msg.id == node && f.msg.buf[0] == 0xF3 && f.msg.buf[1] == 1) ++result;
  return result;
}
inline CAN_message_t lastCurrent(uint8_t node) {
  for (auto i = Mock::sent.rbegin(); i != Mock::sent.rend(); ++i)
    if (i->msg.id == node && i->msg.buf[0] == 0x83) return i->msg;
  throw std::runtime_error("Expected current-setting command");
}
inline CAN_message_t lastMove(uint8_t node = 6) {
  for (auto i = Mock::sent.rbegin(); i != Mock::sent.rend(); ++i)
    if (i->msg.id == node && i->msg.buf[0] == 0xFE)
      return i->msg;
  throw std::runtime_error("Expected FE command");
}
inline int target(uint8_t node = 6) {
  auto f = lastMove(node);
  int32_t raw = (int32_t(f.buf[4]) << 16) | (int32_t(f.buf[5]) << 8) | f.buf[6];
  return (raw & 0x800000) ? raw - 0x1000000 : raw;
}
inline unsigned speed(uint8_t node = 6) {
  auto f = lastMove(node);
  return (unsigned(f.buf[1]) << 8) | f.buf[2];
}
inline void completePending(bool acknowledgeStartup = false) {
  for (unsigned round = 0; round < 4000; ++round) {
    while (ackCursor < Mock::sent.size()) {
      const auto f = Mock::sent[ackCursor++].msg;
      if (f.buf[0] == 0xFE || f.buf[0] == 0x91) {
        reply(uint8_t(f.id), f.buf[0], 1);
        reply(uint8_t(f.id), f.buf[0], 2);
      } else if (acknowledgeStartup && (f.buf[0] == 0x8C || f.buf[0] == 0x83 || f.buf[0] == 0x92 ||
                                     (f.buf[0] == 0xF3 && f.buf[1] == 1)))
        reply(uint8_t(f.id), f.buf[0], 1);
    }
    pump();
    if (std::string(status().phase) == "ready")
      return;
    if (std::string(status().phase) == "fault")
      throw std::runtime_error("Unexpected mechanism fault");
    advance(20);
  }
  throw std::runtime_error("Completion barrier never became ready");
}
inline uint16_t confirmedCurrent(uint8_t node) {
  const auto state = status();
  switch (node) {
    case 4: return state.current4Ma;
    case 5: return state.current5Ma;
    case 6: return state.current6Ma;
        default: throw std::runtime_error("Invalid current-settings test node");
  }
}
inline void startupSetup() {
  begin();
  CHECK(std::string(status().startupPhase) == "boot_delay");
  CHECK(status().positionKnownMask == 0 && count(0x91) == 0);
  CHECK(status().currentConfiguredMask == 0 && status().current4Ma == 0 && status().current5Ma == 0 && status().current6Ma == 0);
  advance(SystemConfig::Mechanism::STARTUP_DELAY_SECONDS * 1000U);
  CHECK(status().startupNode == 4 && status().startupCommand == 0x8C &&
        status().startupAttempts == 1);
}
inline void acknowledgeWiredSettings(uint8_t through = 6) {
  for (uint8_t node : SystemConfig::Mechanism::MOTOR_IDS) {
    if (node > through) return;
    CHECK(count(0x91) == 0 && count(0xFE) == 0);
    reply(node, 0x8C, 1); pump();
    const uint16_t requested = SystemConfig::Mechanism::startupCurrentMilliamps(node);
    if (requested) {
      CHECK(status().startupNode == node && status().startupCommand == 0x83 &&
            status().startupCurrentMa == requested && std::string(status().startupPhase) == "current_wait");
      CHECK(countEnable(node) == 0 && count(0x91) == 0 && count(0x92) == 0);
      CHECK((status().currentConfiguredMask & (1U << (node - 4))) == 0);
      reply(node, 0x83, 1); pump();
      CHECK((status().currentConfiguredMask & (1U << (node - 4))) != 0);
    }
    CHECK(count(0x91) == 0 && status().startupNode == node && status().startupCommand == 0xF3);
    reply(node, 0xF3, 1); pump();
  }
  CHECK(count(0x91) == 0 && count(0xFE) == 0 && count(0x92, 4) == 1 &&
        std::string(status().startupPhase) == "zero_wait" && status().positionKnownMask == 0);
}
inline void startupCurrentSetup(uint8_t node) {
  CHECK(node >= 4 && node <= 6);
  startupSetup(); acknowledgeWiredSettings(node - 1);
  CHECK(status().startupNode == node && status().startupCommand == 0x8C);
  reply(node, 0x8C, 1); pump();
  CHECK(status().startupNode == node && status().startupCommand == 0x83 &&
        std::string(status().startupPhase) == "current_wait" && countEnable(node) == 0);
}
inline void acknowledgeSettings() {
  acknowledgeWiredSettings(); reply(4,0x92,1); pump();
  CHECK(status().positionKnownMask == 1 && count(0x91) == 0 && count(0x92,5) == 1);
  reply(5,0x92,1); pump();
  CHECK(count(0x91) == 1 && count(0xFE) == 0 && status().positionKnownMask == 3 && count(0x91,7) == 0);
}
inline void ready() {
  begin(); advance(SystemConfig::Mechanism::STARTUP_DELAY_SECONDS * 1000U); completePending(true);
  CHECK(status().stage == 4 && status().progress == 0 && !status().previousAvailable);
  for (uint8_t node : SystemConfig::Mechanism::MOTOR_IDS)
    CHECK(count(0x8C, node) == 1 && count(0xF3, node) == 1 &&
          count(0x91, node) == (node >= 6 ? 1 : 0) && count(0x92, node) == (node <= 5 ? 1 : 0) &&
          count(0x83, node) == 1);
  CHECK(count(0xFE) == 0 && status().position6 == 0);
  CHECK(status().position4 == 0 && status().position5 == 0 && Mock::servoAttached[3]);
  CHECK(status().positionKnownMask == 7 && std::string(status().startupPhase) == "ready");
  CHECK(status().currentConfiguredMask == 7 && status().current4Ma == 3000 && status().current5Ma == 3000 && status().current6Ma == 5200);
  for (uint8_t i = 0; i < SystemConfig::Mechanism::SERVO_COUNT; ++i) {
    const uint8_t pin = SystemConfig::Mechanism::SERVO_PINS[i];
    CHECK(Mock::servo[pin] == SystemConfig::Mechanism::SERVO_START_US[i] && Mock::servoAttached[pin]);
  }
  CHECK(!status().cylinder.active && !status().cylinder.in1High && !status().cylinder.in2High && !status().cylinder.referenceKnown);
  Mock::sent.clear(); ackCursor = 0;
}
inline void nextStep() {
  CHECK(next());
  pump();
}
inline void finishStep() {
  nextStep();
  completePending();
}
inline void previousStep() {
  CHECK(previous());
  pump();
  completePending();
}
inline void select(uint8_t stage) {
  CHECK(selectStage(stage));
  pump();
  completePending();
  CHECK(status().stage == stage && status().progress == 0 && status().completed == 0 &&
        status().step == 0 && !status().previousAvailable &&
        status().prop == MechanismConfig::normalizedProp(stage) && !status().propComplete);
}
inline void upload(uint8_t stage, int32_t pos, uint8_t node = 6) {
  using namespace MechanismConfig;
  CHECK(planBegin(1, 1, stage, 1));
  CHECK(planAction(0, branch(stepperFor(node, pos, 320), 1)));
  CHECK(planStep(0, 0, 1));
  CHECK(planCommit());
  pump();
  completePending();
}
inline void selectProp(uint8_t prop) {
  CHECK(selectProfile(1, prop)); pump(); completePending();
  CHECK(status().stage == 1 && status().prop == prop && status().pendingProp == 0 &&
        status().progress == 0 && !status().propComplete && !status().previousAvailable);
}
inline void uploadProfile(uint8_t stage, uint8_t prop, int32_t pos, uint8_t node = 6) {
  using namespace MechanismConfig;
  CHECK(planBegin(1, 1, stage, 1, prop));
  CHECK(planAction(0, branch(stepperFor(node, pos, 320), 1)));
  CHECK(planStep(0, 0, 1)); CHECK(planCommit()); pump(); completePending();
  CHECK(status().stage == stage && status().prop == normalizedProp(stage, prop) &&
        status().completed == 0 && status().runtimePlan && !status().propComplete);
}
inline void cylinderReference(MechanismConfig::CylinderAction action) {
  CHECK(status().cylinder.referenceKnown && status().cylinder.referenceAction == action && !status().cylinder.active);
  CHECK(!status().cylinder.in1High && !status().cylinder.in2High);
}
inline void completeCylinder() { completePending(); }
inline MechanismConfig::CylinderAction stage2Reference(uint8_t step) {
    auto reference = MechanismConfig::CylinderAction::Retract; // HOME2 establishes the entry estimate.
    CHECK(step <= Stage2::STEP_COUNT);
    for (uint8_t n = 0; n < step; ++n)
        for (size_t i = 0; i < Stage2::STEPS[n].count; ++i) {
            const auto &action = Stage2::STEPS[n].actions[i];
            if (action.kind == MechanismConfig::ActionKind::Cylinder &&
                action.cylinderAct != MechanismConfig::CylinderAction::Stop) reference = action.cylinderAct;
        }
    return reference;
}
inline int stage2Pulse(uint8_t step, uint8_t pin) {
  int result = 0;
  for (uint8_t i = 0; i < SystemConfig::Mechanism::SERVO_COUNT; ++i)
    if (SystemConfig::Mechanism::SERVO_PINS[i] == pin) result = SystemConfig::Mechanism::SERVO_START_US[i];
  for (uint8_t n = 0; n < step; ++n)
    for (size_t i = 0; i < Stage2::STEPS[n].count; ++i) {
      const auto &a = Stage2::STEPS[n].actions[i];
      if (a.kind == MechanismConfig::ActionKind::Servo && a.pin == pin) result = a.pulse;
    }
  return result;
}
inline MechanismConfig::Action stage3Motor(uint8_t step, uint8_t node) {
  auto result = MechanismConfig::stepperFor(node, 0, 320);
  for (uint8_t n = 0; n < step; ++n)
    for (size_t i = 0; i < Stage3::STEPS[n].count; ++i) {
      const auto &action = Stage3::STEPS[n].actions[i];
      if (action.kind == MechanismConfig::ActionKind::Stepper && action.node == node) result = action;
    }
  return result;
}
inline int stage3Pulse(uint8_t step) {
  int result = 0;
  for (uint8_t i = 0; i < SystemConfig::Mechanism::SERVO_COUNT; ++i)
    if (SystemConfig::Mechanism::SERVO_PINS[i] == 3) result = SystemConfig::Mechanism::SERVO_START_US[i];
  for (uint8_t n = 0; n < step; ++n)
    for (size_t i = 0; i < Stage3::STEPS[n].count; ++i) {
      const auto &action = Stage3::STEPS[n].actions[i];
      if (action.kind == MechanismConfig::ActionKind::Servo && action.pin == 3) result = action.pulse;
    }
  return result;
}
inline MechanismConfig::Action stage4Motor(uint8_t step, bool firstAtStep = false) {
  auto result = MechanismConfig::stepperFor(6, 0, 320);
  for (uint8_t n = 0; n < step; ++n)
    for (size_t i = 0; i < Stage4::STEPS[n].count; ++i) {
      const auto &a = Stage4::STEPS[n].actions[i];
      if (a.kind == MechanismConfig::ActionKind::Stepper && a.node == 6) {
        result = a;
        if (firstAtStep && n + 1 == step) return result;
      }
    }
  return result;
}
inline void checkFrames() {
  bool in1 = false, in2 = false;
  for (const auto &write : Mock::gpioWrites) {
    if (write.pin == SystemConfig::Cylinder::IN1_PIN) in1 = write.value != LOW;
    if (write.pin == SystemConfig::Cylinder::IN2_PIN) in2 = write.value != LOW;
    CHECK(!(in1 && in2));
  }
  for (const auto &f : Mock::sent) {
    CHECK(f.bus == 1 && MechanismConfig::activeMotor(static_cast<uint8_t>(f.msg.id)) && !f.msg.flags.extended &&
          !f.msg.flags.remote);
    uint8_t checksum = uint8_t(f.msg.id);
    for (uint8_t i = 0; i < f.msg.len - 1; ++i)
      checksum += f.msg.buf[i];
    CHECK(checksum == f.msg.buf[f.msg.len - 1]);
    CHECK(f.msg.buf[0] != 0xFD && f.msg.buf[0] != 0x80);
    if (f.msg.buf[0] == 0x83) {
      CHECK(f.msg.id >= 4 && f.msg.id <= 6 && f.msg.len == 4);
      if (f.msg.id == 6) CHECK(f.msg.buf[1] == 0x14 && f.msg.buf[2] == 0x50 && f.msg.buf[3] == 0xED);
      else CHECK(f.msg.buf[1] == 0x0B && f.msg.buf[2] == 0xB8 && f.msg.buf[3] == uint8_t(0x46 + f.msg.id));
    }
    if (f.msg.buf[0] == 0x91) CHECK(f.msg.id == 6);
    if (f.msg.buf[0] == 0x92) CHECK((f.msg.id == 4 || f.msg.id == 5) && f.msg.len == 2);
  }
}
} // namespace MechanismSupport
