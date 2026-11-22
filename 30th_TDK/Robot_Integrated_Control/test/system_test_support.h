#pragma once
#include <Arduino.h>
#include <AlfredoCRSF.h>
#include <FlexCAN_T4.h>
#include <Servo.h>
#include "can_transport.h"
#include "chassis_controller.h"
#include "mechanism_controller.h"
#include "cylinder_controller.h"
#include "pwm_controller.h"
#include "rc_controller.h"
#include "system_config.h"
#include "system_safety.h"
#include <algorithm>
#include <cstdio>
#include <cmath>
#include "chassis_math.h"
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

void setup();
void loop();
#define CHECK(expr) do { if (!(expr)) throw std::runtime_error(std::to_string(__LINE__) + ": " + #expr); } while (0)

namespace SystemSupport {
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

inline size_t responseCursor = 0;
inline bool autoCompleteMoves = true, autoStartMoves = true, autoAckStartup = true, autoCompleteHomes = true;
inline uint8_t holdCurrentAckNode = 0; // Simulate one missing 0x83 reply, leaving other startup ACKs live.
inline uint8_t odriveState[4] = {1, 1, 1, 1};
inline uint8_t pendingMks[SystemConfig::Mechanism::MOTOR_COUNT] = {};
inline unsigned requestId = 10;
inline constexpr uint8_t ALL_STAGES[5] = {1, 2, 3, 4, 5};
inline AlfredoCRSF &radio() { return *AlfredoCRSF::instance; }
inline bool mechanismReady() { return std::string(MechanismController::status().phase) == "ready"; }
inline bool mechanismBusy() {
    const std::string phase = MechanismController::status().phase;
    return phase == "busy" || phase == "home_wait" || phase == "return_wait";
}
inline bool mksFrame(const Mock::Frame &frame) {
    return frame.bus == 1 && frame.msg.id >= 4 && frame.msg.id <= 6;
}
inline void incomingMks(uint8_t node, uint8_t command, uint8_t status, uint8_t bus = 1,
                        bool corrupt = false) {
    CAN_message_t message{};
    message.id = node; message.len = 3;
    message.buf[0] = command; message.buf[1] = status;
    message.buf[2] = static_cast<uint8_t>(node + command + status + (corrupt ? 1 : 0));
    Mock::incoming[bus].push_back(message);
}
inline CAN_message_t heartbeat(uint8_t index, uint8_t state, uint32_t error = 0) {
    CAN_message_t message{};
    message.id = (uint32_t(SystemConfig::Chassis::NODE_ID[index]) << 5) | 0x01;
    message.len = 8; memcpy(message.buf, &error, 4); message.buf[4] = state;
    return message;
}
inline void respondToTransmissions() {
    while (responseCursor < Mock::sent.size()) {
        const auto frame = Mock::sent[responseCursor++];
        if (mksFrame(frame)) {
            const auto &message = frame.msg;
            CHECK(message.id >= 4 && message.id <= 6); // Any attempted CAN ID7 traffic is a regression.
            if (message.buf[0] == 0x8C || message.buf[0] == 0x83 || message.buf[0] == 0x92 || (message.buf[0] == 0xF3 && message.buf[1] == 1)) {
                if (autoAckStartup && !(message.buf[0] == 0x83 && message.id == holdCurrentAckNode))
                    incomingMks(uint8_t(message.id), message.buf[0], 1);
            }
            else if (message.buf[0] == 0x91 || message.buf[0] == 0xFE) {
                const uint8_t node = uint8_t(message.id);
                pendingMks[node - 4] = message.buf[0];
                if (message.buf[0] == 0x91 || autoStartMoves)
                    incomingMks(node, message.buf[0], 1);
                if (message.buf[0] == 0x91 ? autoCompleteHomes : autoCompleteMoves) {
                    incomingMks(node, message.buf[0], 2);
                    pendingMks[node - 4] = 0;
                }
            }
        } else if ((frame.msg.id & 31) == 0x07 && frame.msg.len == 4) {
            uint32_t state = 0; memcpy(&state, frame.msg.buf, 4);
            for (uint8_t i = 0; i < 4; ++i)
                if (frame.bus == SystemConfig::Chassis::CAN_BUS[i] &&
                    (frame.msg.id >> 5) == SystemConfig::Chassis::NODE_ID[i])
                    odriveState[i] = static_cast<uint8_t>(state);
        }
    }
}
inline void tick(uint32_t milliseconds = 5, bool freshHeartbeats = true) {
    mockMillis += milliseconds;
    respondToTransmissions();
    if (freshHeartbeats) for (uint8_t i = 0; i < 4; ++i)
        Mock::incoming[SystemConfig::Chassis::CAN_BUS[i]].push_back(heartbeat(i, odriveState[i]));
    loop();
}
inline void advance(uint32_t milliseconds, bool freshHeartbeats = true) {
    while (milliseconds) {
        const uint32_t slice = std::min(milliseconds, uint32_t(5));
        tick(slice, freshHeartbeats); milliseconds -= slice;
    }
}
inline bool useMotionDefaults = false;
inline void initialize() {
    setup();
    if (!useMotionDefaults) {
        // Legacy whole-system cases assert instantaneous mixing independently.
        // The motion suite uses the same real loop with production ramp defaults.
        auto motion = ChassisController::defaultMotionSettings(); motion.rampEnabled = false;
    motion.icrLeftYMm = motion.icrRightYMm = 0; // Isolate legacy center-based mixing.
        CHECK(ChassisController::configureMotion(motion));
    }
    radio().linked = true;
    for (int &channel : radio().channels) channel = 1000;
    radio().channels[SystemConfig::Rc::STAGE1_TRIM_CHANNEL] = SystemConfig::Rc::STAGE1_TRIM_CENTER_US;
    radio().channels[SystemConfig::Chassis::FORWARD.number] = SystemConfig::Chassis::FORWARD.middle;
    radio().channels[SystemConfig::Chassis::STRAFE.number] = SystemConfig::Chassis::STRAFE.middle;
    radio().channels[SystemConfig::Chassis::ROTATE.number] = SystemConfig::Chassis::ROTATE.middle;
    radio().channels[SystemConfig::Rc::STAGE1_SWITCH_CHANNEL] = 1000;
    radio().channels[SystemConfig::Rc::STAGE_CHANNELS[2]] = 2000; // Normal Stage 4: CH12.
}
inline void readyRobot() {
    initialize();
    for (unsigned i = 0; i < 2000; ++i) { // Allow the concurrent 5-second startup retract.
        tick();
        CHECK(!SystemSafety::stopped());
        if (mechanismReady() && ChassisController::status().mode == ChassisController::Mode::Running &&
            RcController::status().selectedStage == 4) {
            advance(50); return;
        }
    }
    throw std::runtime_error("Whole robot failed to become ready");
}
inline unsigned mksCount(uint8_t command, int node = -1, int argument = -1,
                         size_t since = 0) {
    unsigned count = 0;
    for (size_t i = since; i < Mock::sent.size(); ++i) {
        const auto &frame = Mock::sent[i];
        if (mksFrame(frame) && frame.msg.buf[0] == command &&
            (node < 0 || frame.msg.id == uint32_t(node)) &&
            (argument < 0 || (frame.msg.len >= 3 && frame.msg.buf[1] == argument))) ++count;
    }
    return count;
}
inline unsigned odriveCount(uint8_t command, size_t since = 0) {
    unsigned count = 0;
    for (size_t i = since; i < Mock::sent.size(); ++i)
        if (!mksFrame(Mock::sent[i]) && (Mock::sent[i].msg.id & 31) == command) ++count;
    return count;
}
inline CAN_message_t lastMove(uint8_t node = 6) {
    for (auto i = Mock::sent.rbegin(); i != Mock::sent.rend(); ++i)
        if (mksFrame(*i) && i->msg.id == node && i->msg.buf[0] == 0xFE) return i->msg;
    throw std::runtime_error("No FE command for requested node");
}
inline int32_t target(uint8_t node = 6) {
    const auto message = lastMove(node);
    const int32_t value = (int32_t(message.buf[4]) << 16) | (int32_t(message.buf[5]) << 8) | message.buf[6];
    return value & 0x800000 ? value - 0x1000000 : value;
}
inline void finish(uint8_t node = 6, uint8_t command = 0xFE) {
    incomingMks(node, command, 1); incomingMks(node, command, 2);
    CHECK(node >= 4 && node <= 6); pendingMks[node - 4] = 0; tick();
}
inline void finishOutstanding() {
    respondToTransmissions();
    for (uint8_t i = 0; i < SystemConfig::Mechanism::MOTOR_COUNT; ++i) if (pendingMks[i]) {
        incomingMks(uint8_t(i + 4), pendingMks[i], 1);
        incomingMks(uint8_t(i + 4), pendingMks[i], 2); pendingMks[i] = 0;
    }
}
inline void completeMechanism() {
    autoCompleteMoves = true;
    finishOutstanding();
    for (unsigned i = 0; i < 14000; ++i) {
        tick(); CHECK(!SystemSafety::stopped());
        if (mechanismReady()) return;
    }
    throw std::runtime_error("Mechanism failed to complete");
}
inline unsigned queue(const std::string &command) {
    const unsigned id = requestId++;
    Serial.input += "@" + std::to_string(id) + " " + command + "\n";
    return id;
}
inline std::string lineFor(unsigned id) {
    const std::string prefix = "@" + std::to_string(id) + " ";
    const size_t start = Serial.output.find(prefix);
    if (start == std::string::npos) return {};
    const size_t end = Serial.output.find('\n', start);
    return end == std::string::npos ? std::string{} : Serial.output.substr(start, end - start);
}
inline std::string request(const std::string &command, bool accepted = true) {
    const unsigned id = queue(command);
    for (unsigned i = 0; i < 1000; ++i) {
        tick();
        const std::string reply = lineFor(id);
        if (!reply.empty()) {
            CHECK(reply.find(accepted ? " OK " : " ERR ") != std::string::npos);
            return reply;
        }
    }
    throw std::runtime_error("USB reply timed out: " + command);
}
inline void button(uint8_t channel, bool high) {
    radio().channels[channel] = high ? 2000 : 1000;
    tick(); advance(SystemConfig::Rc::DEBOUNCE_MS);
}
inline uint8_t normalStageChannel(uint8_t stage) {
    for (uint8_t i=0;i<4;++i)
        if (SystemConfig::Rc::STAGE_MAP[i] == stage) return SystemConfig::Rc::STAGE_CHANNELS[i];
    throw std::runtime_error("No normal RC channel for stage");
}
inline void stage1Switch(int value) {
    radio().channels[SystemConfig::Rc::STAGE1_SWITCH_CHANNEL] = value;
    tick(); advance(SystemConfig::Rc::DEBOUNCE_MS);
}
// Generate fresh selector edges in either mode without replaying a held prop key.
inline void selectRcStage(uint8_t stage) {
    CHECK(stage >= 1 && stage <= 5);
    for (uint8_t channel : SystemConfig::Rc::PROP_CHANNELS) radio().channels[channel] = 1000;
    radio().channels[SystemConfig::Rc::NEXT_CHANNEL] = 1000;
    radio().channels[SystemConfig::Rc::PREV_CHANNEL] = 1000;
    tick(); advance(SystemConfig::Rc::DEBOUNCE_MS);
    stage1Switch(stage == 1 ? 2000 : 1000);
    if (stage != 1) {
        // CH7 LOW establishes a new general context; release the old prop keys
        // for a complete stable interval before generating a fresh selector.
        advance(SystemConfig::Rc::DEBOUNCE_MS);
        button(normalStageChannel(stage),true);
    }
}
inline void propRcPulse(uint8_t prop) {
    CHECK(prop >= 1 && prop <= 5);
    const uint8_t channel = SystemConfig::Rc::PROP_CHANNELS[prop-1];
    button(channel,false); button(channel,true);
}
inline void nextRcPulse() {
    button(SystemConfig::Rc::NEXT_CHANNEL, false);
    button(SystemConfig::Rc::NEXT_CHANNEL, true);
}
inline int32_t stage4Position(uint8_t step) {
    const auto &profile = Stage4::STEPS[step - 1];
    for (size_t i = 0; i < profile.count; ++i)
        if (profile.actions[i].kind == MechanismConfig::ActionKind::Stepper && profile.actions[i].node == 6) return profile.actions[i].position;
    throw std::runtime_error("Stage4 lacks an ID6 action");
}
inline void assertCylinderGpioHistory() {
    uint8_t in1=LOW,in2=LOW;
    for (const auto &event : Mock::gpioWrites) {
        if (event.pin==SystemConfig::Cylinder::IN1_PIN) in1=event.value;
        if (event.pin==SystemConfig::Cylinder::IN2_PIN) in2=event.value;
        CHECK(!(in1==HIGH && in2==HIGH));
    }
    CHECK(Mock::gpioState[SystemConfig::Cylinder::IN1_PIN]==in1 && Mock::gpioState[SystemConfig::Cylinder::IN2_PIN]==in2);
}
inline void assertNoId7() {
    for (const auto &frame : Mock::sent) CHECK(!(frame.bus==1 && frame.msg.id==7));
}
inline void assertCylinderStopped() {
    const auto cylinder=CylinderController::status();
    CHECK(!cylinder.active && !cylinder.in1High && !cylinder.in2High);
    CHECK(Mock::gpioState[SystemConfig::Cylinder::IN1_PIN]==LOW && Mock::gpioState[SystemConfig::Cylinder::IN2_PIN]==LOW);
    assertCylinderGpioHistory();
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
    int pulse = 0;
    for (uint8_t i = 0; i < SystemConfig::Mechanism::SERVO_COUNT; ++i)
        if (SystemConfig::Mechanism::SERVO_PINS[i] == pin) pulse = SystemConfig::Mechanism::SERVO_START_US[i];
    // STEP history begins at the Stage2 entry pose, not global boot standby.
    if (pin == 9) pulse = SystemConfig::Stage2::SERVO9_ENTRY_US;
    if (pin == 4) pulse = SystemConfig::Stage2::SERVO4_ENTRY_US;
    if (pin == 3) pulse = SystemConfig::Stage2::SERVO3_ENTRY_US;
    for (uint8_t n = 0; n < step; ++n)
        for (size_t i = 0; i < Stage2::STEPS[n].count; ++i) {
            const auto &action = Stage2::STEPS[n].actions[i];
            if (action.kind == MechanismConfig::ActionKind::Servo && action.pin == pin) pulse = action.pulse;
        }
    return pulse;
}

inline bool floatClose(float a, float b) { return fabsf(a - b) < 0.001f; }

// Independent wheel-direction bases for the explicit stage matrices.
// Inputs here are already normalized with each RC channel's own calibration.
inline void expectedOrientedWheels(uint8_t stage, float forward, float strafe, float rotate,
                                   float correction, float (&wheel)[4]) {
    constexpr float forwardBasis[4][4] = {{-1,-1,-1,-1},{1,-1,-1,1},{-1,1,1,-1},{-1,1,1,-1}};
    constexpr float strafeBasis[4][4] = {{1,-1,-1,1},{1,1,1,1},{-1,-1,-1,-1},{-1,-1,-1,-1}};
    constexpr float rotationBasis[4] = {-1,-1,1,1};
    CHECK(stage >= 1 && stage <= 5);
    const uint8_t index = stage <= 4 ? static_cast<uint8_t>(stage-1) : 3;
    float peak = 1;
    for (uint8_t i = 0; i < 4; ++i) {
        wheel[i] = forward * forwardBasis[index][i] + strafe * strafeBasis[index][i] + rotate * rotationBasis[i];
        peak = std::max(peak, std::fabs(wheel[i]));
    }
    for (uint8_t i = 0; i < 4; ++i) {
        wheel[i] = wheel[i] * SystemConfig::Chassis::JOYSTICK_MAX_SPEED / peak + correction * rotationBasis[i];
        wheel[i] = std::max(-SystemConfig::Chassis::SAFETY_MAX_SPEED, std::min(SystemConfig::Chassis::SAFETY_MAX_SPEED,wheel[i]));
    }
}
inline void sticks(int forward, int strafe, int rotate) {
    radio().channels[SystemConfig::Chassis::FORWARD.number] = forward;
    radio().channels[SystemConfig::Chassis::STRAFE.number] = strafe;
    radio().channels[SystemConfig::Chassis::ROTATE.number] = rotate;
}
inline void assertOrientedSnapshot(uint8_t stage, float forward, float strafe, float rotate,
                                    float correction = 0) {
    const auto mechanism = MechanismController::status();
    CHECK((mechanism.pendingStage ? mechanism.pendingStage : mechanism.stage) == stage);
    const auto chassis = ChassisController::status();
    CHECK(chassis.directionStage == stage);
    CHECK(chassis.mode == ChassisController::Mode::Running && chassis.healthy);
    float expected[4]; expectedOrientedWheels(stage,forward,strafe,rotate,correction,expected);
    for (uint8_t i = 0; i < 4; ++i) {
        CHECK(std::isfinite(chassis.wheel[i]) && floatClose(chassis.wheel[i],expected[i]));
        CHECK(std::fabs(chassis.wheel[i]) <= SystemConfig::Chassis::SAFETY_MAX_SPEED);
    }
}
// Observe real CAN payloads emitted by the cooperative whole-robot loop.
inline void assertSteeringCadence(uint8_t stage, float forward, float strafe, float rotate,
                                  float correction = 0, uint32_t duration = 90) {
    float expected[4]; expectedOrientedWheels(stage,forward,strafe,rotate,correction,expected);
    size_t cursor = Mock::sent.size(); bool seen[4]{}; uint32_t sentAt[4]{}; unsigned count[4]{};
    const auto updatesAt = radio().updateCount;
    for (uint32_t elapsed = 0; elapsed < duration; elapsed += 5) {
        tick(); CHECK(!SystemSafety::stopped());
        while (cursor < Mock::sent.size()) {
            const auto frame = Mock::sent[cursor++];
            if (mksFrame(frame) || (frame.msg.id & 31) != 0x0D) continue;
            int axis = -1;
            for (uint8_t i = 0; i < 4; ++i)
                if ((frame.msg.id >> 5) == SystemConfig::Chassis::NODE_ID[i] && frame.bus == SystemConfig::Chassis::CAN_BUS[i]) axis = i;
            CHECK(axis >= 0 && frame.msg.len == 8 && !frame.msg.flags.extended && !frame.msg.flags.remote);
            float velocity = 0, torque = 1; memcpy(&velocity,frame.msg.buf,4); memcpy(&torque,frame.msg.buf+4,4);
            CHECK(std::isfinite(velocity) && floatClose(velocity,expected[axis]) && torque == 0);
            CHECK(std::fabs(velocity) <= SystemConfig::Chassis::SAFETY_MAX_SPEED);
            if (seen[axis]) CHECK(mockMillis - sentAt[axis] == SystemConfig::Chassis::SEND_INTERVAL_MS);
            seen[axis] = true; sentAt[axis] = mockMillis; ++count[axis];
        }
        const auto mechanism = MechanismController::status();
        CHECK((mechanism.pendingStage ? mechanism.pendingStage : mechanism.stage) == stage);
    }
    for (unsigned packets : count) CHECK(packets >= duration / SystemConfig::Chassis::SEND_INTERVAL_MS - 1);
    CHECK(radio().updateCount >= updatesAt + duration / 5);
    assertOrientedSnapshot(stage,forward,strafe,rotate,correction);
}


inline void assertJoystickOnly() {
    const auto rc = RcController::status();
    float expected[4]; ChassisMath::mixWheels(rc.forward, rc.strafe, rc.rotate, expected, MechanismController::status().stage);
    const auto chassis = ChassisController::status();
    for (uint8_t i = 0; i < 4; ++i) CHECK(floatClose(chassis.wheel[i], expected[i]));
}



inline void writeStatusFixture(const char *path, const std::string &wire) {
    const size_t begin = wire.find('{'); CHECK(begin != std::string::npos);
    FILE *file = fopen(path, "wb"); CHECK(file != nullptr);
    const std::string json = wire.substr(begin);
    CHECK(fwrite(json.data(), 1, json.size(), file) == json.size()); CHECK(fclose(file) == 0);
}

inline void assertEmergencyStopped(size_t since) {
    CHECK(SystemSafety::stopped());
    assertCylinderStopped();
    CHECK(ChassisController::status().mode == ChassisController::Mode::Locked);
    CHECK(ChassisController::status().emergencyLatched);
    CHECK(std::string(MechanismController::status().phase) == "fault");
    CHECK(mksCount(0xF7, -1, -1, since) == SystemConfig::Mechanism::MOTOR_COUNT);
    CHECK(mksCount(0xF3, -1, 0, since) == SystemConfig::Mechanism::MOTOR_COUNT);
    for (uint8_t pin : SystemConfig::Mechanism::SERVO_PINS) CHECK(!Mock::servoAttached[pin]);
    bool sawIdle[4]{};
    for (size_t i = since; i < Mock::sent.size(); ++i) {
        const auto &frame = Mock::sent[i];
        if (mksFrame(frame)) { CHECK((frame.msg.id >= 4 && frame.msg.id <= 6) && frame.msg.buf[0] != 0xFE); continue; }
        if ((frame.msg.id & 31) == 0x0D) {
            float velocity = 1; memcpy(&velocity, frame.msg.buf, 4); CHECK(velocity == 0);
        } else if ((frame.msg.id & 31) == 0x07) {
            uint32_t state = 0; memcpy(&state, frame.msg.buf, 4); CHECK(state == 1);
            for (uint8_t index = 0; index < 4; ++index)
                if ((frame.msg.id >> 5) == SystemConfig::Chassis::NODE_ID[index] &&
                    frame.bus == SystemConfig::Chassis::CAN_BUS[index]) sawIdle[index] = true;
        }
    }
    for (bool idle : sawIdle) CHECK(idle);
}
} // namespace SystemSupport
