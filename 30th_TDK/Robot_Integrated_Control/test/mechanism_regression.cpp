#include "mechanism_test_support.h"
#include <string.h>
#include <vector>
namespace Mock {
bool globalStopped = false;
unsigned globalStopCalls = 0;
} // namespace Mock
namespace CanTransport {
bool send(uint8_t bus, uint32_t id, const uint8_t *data, uint8_t length) {
  if (int(id) == Mock::rejectNode && data[0] == Mock::rejectCommand)
    return false;
  CAN_message_t f{};
  f.id = id;
  f.len = length;
  memcpy(f.buf, data, length);
  Mock::sent.push_back({bus, f});
  return true;
}
} // namespace CanTransport
namespace RcController {
Status status() { return MechanismSupport::trimRadio; }
} // namespace RcController
namespace SystemSafety {
bool stopped() { return Mock::globalStopped; }
void stop(const char *) {
  if (Mock::globalStopped)
    return;
  Mock::globalStopped = true;
  ++Mock::globalStopCalls;
  MechanismController::stop();
}
} // namespace SystemSafety
using namespace MechanismSupport;
using namespace MechanismConfig;

// Exercise the final PWM output separately from the untrimmed motion state.
bool trimOutputTest(const std::string &name) {
  if (name == "trim_hold_repeated_offsets_do_not_accumulate") {
    select(1); trimContext(150); pump();
    CHECK(status().pulse2Base == 1150 && status().pulse2 == 1300 && Mock::servo[2] == 1300);
    CHECK(status().stage1TrimAllowed && status().stage1TrimUs == 150);
    const size_t writes=Mock::servoWrites.size(); pump(100);
    CHECK(status().pulse2Base == 1150 && Mock::servo[2] == 1300 && Mock::servoWrites.size() == writes);
    trimRadio.stage1TrimUs=-150; pump();
    CHECK(status().pulse2Base == 1150 && status().pulse2 == 1000 && status().stage1TrimUs == -150);
    trimRadio.stage1TrimUs=0; pump(); CHECK(Mock::servo[2] == 1150);
  } else if (name == "trim_previous_restores_untrimmed_snapshot") {
    select(1); trimContext(100); finishStep();
    CHECK(status().pulse2Base == propPulse(2,1,2) && Mock::servo[2] == propPulse(2,1,2)+100);
    finishStep(); trimRadio.stage1TrimUs=-100; pump(); previousStep();
    CHECK(status().completed == 1 && status().pulse2Base == propPulse(2,1,2));
    CHECK(Mock::servo[2] == propPulse(2,1,2)-100);
    previousStep(); CHECK(status().pulse2Base == 1150 && Mock::servo[2] == 1050);
  } else if (name == "trim_runtime_ramp_keeps_base_trajectory") {
    select(1); trimContext(100);
    CHECK(planBegin(1,1,1,1,2)); CHECK(planAction(0,branch(servo(2,1950,1000),1)));
    CHECK(planStep(0,0,1)); CHECK(planCommit()); completePending();
    nextStep(); advance(400);
    CHECK(status().pulse2Base == 1470 && Mock::servo[2] == 1570);
    trimRadio.stage1TrimUs=-100; pump();
    CHECK(status().pulse2Base == 1470 && Mock::servo[2] == 1370);
    advance(600); CHECK(status().pulse2Base == 1950 && Mock::servo[2] == 1850 && status().propComplete);
  } else if (name == "trim_limits_preserve_base_and_report_effective_delta") {
    select(1); trimContext(150);
    CHECK(planBegin(2,2,1,1,2)); CHECK(planAction(0,branch(servo(2,2500),1)));
    CHECK(planAction(1,branch(servo(2,500),1))); CHECK(planStep(0,0,1)); CHECK(planStep(1,1,1));
    CHECK(planCommit()); completePending(); finishStep();
    CHECK(status().pulse2Base == 2500 && Mock::servo[2] == 2500 && status().stage1TrimUs == 0);
    trimRadio.stage1TrimUs=-150; pump(); CHECK(Mock::servo[2] == 2350);
    finishStep(); CHECK(status().pulse2Base == 500 && Mock::servo[2] == 500 && status().stage1TrimUs == 0);
    trimRadio.stage1TrimUs=150; pump(); CHECK(Mock::servo[2] == 650);
  } else if (name == "trim_context_pending_and_home_gates") {
    select(1); trimContext(150); pump(); CHECK(Mock::servo[2] == 1300);
    trimRadio.stage1Mode=false; pump(); CHECK(Mock::servo[2] == 1150 && !status().stage1TrimAllowed);
    trimRadio.stage1Mode=true; pump(); CHECK(Mock::servo[2] == 1300);
    CHECK(home()); pump(); CHECK(status().stage1TrimUs == 0 && Mock::servo[2] == 1150);
    completePending(); pump(50); CHECK(!status().stage1TrimAllowed && Mock::servo[2] == 1150);
    finishStep(); CHECK(status().stage1TrimAllowed && Mock::servo[2] == propPulse(2,1,2)+150);
    CHECK(selectStage(4)); pump(); CHECK(status().stage1TrimUs == 0 && !status().stage1TrimAllowed);
    completePending(); CHECK(status().stage == 4 && Mock::servo[2] == 1150);
    finishStep(); CHECK(Mock::servo[2] == 1500 && status().stage1TrimUs == 0);
  } else if (name == "trim_failsafe_zero_and_stop_no_pwm_restart") {
    select(1); trimContext(150); pump(); CHECK(Mock::servo[2] == 1300);
    trimRadio.link=false; pump(); CHECK(Mock::servo[2] == 1150 && status().stage1TrimUs == 0);
    trimRadio.link=true; trimRadio.valid=false; pump(); CHECK(Mock::servo[2] == 1150);
    trimRadio.valid=true; pump(); CHECK(Mock::servo[2] == 1300);
    SystemSafety::stop("trim test"); const size_t writes=Mock::servoWrites.size();
    trimRadio.stage1TrimUs=-150; advance(1000);
    CHECK(Mock::servoWrites.size() == writes && !Mock::servoAttached[2] && status().stage1TrimUs == 0);
  } else return false;
  return true;
}

// Selection regression cases live outside the legacy scenario chain to avoid
// the MSVC limit on deeply nested else-if statements.
bool selectionTest(const std::string &name) {
  if (name == "selection_known_retract_return_skips") {
    select(2); finishStep(); cylinderReference(CylinderAction::Retract);
    const uint32_t at = mockMillis; const size_t writes = Mock::gpioWrites.size(); CHECK(selectStage(4)); pump();
    CHECK(status().stage == 4 && std::string(status().phase) == "ready" && mockMillis == at && !status().cylinder.active);
    CHECK(Mock::servo[4] == 2000 && Mock::servo[6] == 1400 && Mock::servo[9] == 1100 && Mock::sent.empty());
    for (size_t i = writes; i < Mock::gpioWrites.size(); ++i) CHECK(Mock::gpioWrites[i].value == LOW);
    cylinderReference(CylinderAction::Retract);
  } else if (name == "selection_known_retract_entry_skips") {
    select(2); select(4); const uint32_t at = mockMillis; const size_t writes = Mock::gpioWrites.size(); CHECK(selectStage(2)); pump();
    CHECK(status().stage == 2 && std::string(status().phase) == "ready" && mockMillis == at && status().completed == 0 && !status().cylinder.active);
    for (size_t i = writes; i < Mock::gpioWrites.size(); ++i) CHECK(Mock::gpioWrites[i].value == LOW);
    cylinderReference(CylinderAction::Retract);
  } else if (name == "selection_unknown_retract_return_required") {
    select(2); CylinderController::invalidateReference(); CHECK(selectStage(4)); pump();
    CHECK(status().stage == 2 && status().pendingStage == 4 && status().cylinder.active && status().cylinder.durationMs == 5000);
    advance(50); advance(4999); CHECK(!next() && status().stage == 2); advance(1); CHECK(status().stage == 4); cylinderReference(CylinderAction::Retract);
  } else if (name == "selection_known_retract_manual_home_not_skipped") {
    select(2); CHECK(home()); pump(); CHECK(status().cylinder.active && status().cylinder.durationMs == 5000);
    advance(50); advance(4999); CHECK(!next() && std::string(status().phase) == "home_wait"); advance(1);
    CHECK(std::string(status().phase) == "ready"); cylinderReference(CylinderAction::Retract);
  } else if (name == "selection_known_retract_debug_not_skipped") {
    select(2); CHECK(setDebug(true)); CHECK(debugCylinder(CylinderAction::Retract,200)); pump(); CHECK(status().cylinder.active && status().cylinder.durationMs == 200);
    advance(50); advance(199); CHECK(status().cylinder.active && !setDebug(false)); advance(1); CHECK(setDebug(false)); cylinderReference(CylinderAction::Retract);
  } else if (name == "selection_cylinder_retarget_preserves_clock") {
    select(2); CHECK(setDebug(true)); CHECK(debugCylinder(CylinderAction::Extend,100)); completePending(); CHECK(setDebug(false));
    CHECK(selectStage(1)); pump(); advance(50); advance(1000); const auto state = status().cylinder; const size_t writes = Mock::gpioWrites.size();
    CHECK(state.action == CylinderAction::Retract && state.remainingMs == 4000 && state.in2High);
    CHECK(selectStage(3)); CHECK(selectStage(4)); CHECK(selectStage(5)); pump();
    CHECK(status().stage == 2 && status().pendingStage == 5 && status().cylinder.remainingMs == 4000 && Mock::gpioWrites.size() == writes);
    advance(3999); CHECK(status().stage == 2 && !next()); advance(1);
    CHECK(status().stage == 5 && std::string(status().phase) == "ready" && status().pendingStage == 0); cylinderReference(CylinderAction::Retract);
  } else if (name == "selection_entry_retarget_preserves_clock") {
    CHECK(selectStage(2)); pump(); advance(50); advance(1000); const size_t writes = Mock::gpioWrites.size();
    CHECK(selectStage(3)); CHECK(selectStage(4)); CHECK(selectStage(2)); pump();
    CHECK(status().stage == 2 && status().pendingStage == 2 && status().cylinder.remainingMs == 4000 && Mock::gpioWrites.size() == writes);
    advance(3999); CHECK(!next()); advance(1); CHECK(status().stage == 2 && std::string(status().phase) == "ready" && status().pendingStage == 0); cylinderReference(CylinderAction::Retract);
  } else if (name == "selection_mks_retarget_preserves_barrier") {
    select(3); finishStep(); CHECK(selectStage(1)); pump(); const int sent4 = count(0xFE,4), sent5 = count(0xFE,5);
    finish(4); CHECK(selectStage(2)); CHECK(selectStage(4)); pump();
    CHECK(status().stage == 3 && status().pendingStage == 4 && !next() && status().position5 != 0 && count(0xFE,4) == sent4 && count(0xFE,5) == sent5);
    finish(5); CHECK(status().stage == 4 && status().position4 == 0 && status().position5 == 0 && count(0xFE,4) == sent4 && count(0xFE,5) == sent5);
  } else if (name == "selection_mixed_return_waits_mks_after_cylinder") {
    CHECK(planBegin(1,3,4)); CHECK(planAction(0,branch(stepper(123,320,0,false),1))); CHECK(planAction(1,branch(cylinder(CylinderAction::Extend,100,false),1)));
    CHECK(planAction(2,branch(waitAll(),1))); CHECK(planStep(0,0,3)); CHECK(planCommit()); completePending(); finishStep();
    CHECK(selectStage(1)); pump(); reply(6,0xFE,1); advance(50); advance(5000); cylinderReference(CylinderAction::Retract);
    CHECK(status().stage == 4 && !next() && status().position6 == 123); const int sent = count(0xFE,6); CHECK(selectStage(3)); pump();
    CHECK(status().pendingStage == 3 && count(0xFE,6) == sent && !status().cylinder.active); reply(6,0xFE,2); pump();
    CHECK(status().stage == 3 && status().position6 == 0 && count(0xFE,6) == sent);
  } else if (name == "selection_return_ack_loss_fault_not_ready") {
    finishStep(); const int32_t confirmed = status().position6; CHECK(selectStage(3)); pump(); CHECK(selectStage(1)); CHECK(selectStage(2));
    advance(SystemConfig::Mechanism::START_TIMEOUT_MS);
    CHECK(std::string(status().phase) == "fault" && SystemSafety::stopped() && status().stage == 4 && status().pendingStage == 0 && status().position6 == confirmed && status().positionKnownMask == 3);
    finish(6); CHECK(status().position6 == confirmed && std::string(status().phase) == "fault" && !next());
  } else if (name == "selection_return_stop_keeps_unconfirmed_position") {
    finishStep(); const int32_t confirmed = status().position6; CHECK(selectStage(3)); pump(); reply(6,0xFE,1); SystemSafety::stop("return STOP");
    CHECK(!status().cylinder.active && !status().cylinder.in1High && !status().cylinder.in2High && status().position6 == confirmed && status().positionKnownMask == 3 && status().pendingStage == 0);
    finish(6); advance(60000); CHECK(std::string(status().phase) == "fault" && status().stage == 4 && status().position6 == confirmed && !selectStage(1));
  } else if (name == "selection_return_deadline_not_extended" || name == "selection_return_deadline_clock_wrap") {
    finishStep(); if (name == "selection_return_deadline_clock_wrap") mockMillis = UINT32_MAX - 1000;
    CHECK(selectStage(1)); mockMillis += SystemConfig::Mechanism::STAGE_RETURN_TIMEOUT_MS - 1; CHECK(selectStage(3)); ++mockMillis; pump();
    CHECK(std::string(status().phase) == "fault" && SystemSafety::stopped() && status().pendingStage == 0 && status().stage == 4);
    CHECK(std::string(status().notice) == "Stage selection return deadline exceeded" && count(0xFE,6) == 1 && count(0xF7) == 3);
  } else if (name == "selection_entry_deadline_stops_before_energizing") {
    CHECK(selectStage(2)); pump(); const size_t writes = Mock::gpioWrites.size(); mockMillis += SystemConfig::Mechanism::STAGE_RETURN_TIMEOUT_MS; poll();
    CHECK(std::string(status().phase) == "fault" && SystemSafety::stopped() && !status().cylinder.active && !status().cylinder.in1High && !status().cylinder.in2High);
    for (size_t i = writes; i < Mock::gpioWrites.size(); ++i) CHECK(Mock::gpioWrites[i].value == LOW);
  } else if (name == "selection_switchstop_deadline_stops") {
    CHECK(planBegin(1,2,4)); CHECK(planAction(0,branch(servo(5,1400,60000),1))); CHECK(planAction(1,branch(cylinder(CylinderAction::Extend,100),2)));
    CHECK(planStep(0,0,2)); CHECK(planCommit()); completePending(); nextStep(); CHECK(selectStage(3)); pump(); mockMillis += SystemConfig::Mechanism::STAGE_RETURN_TIMEOUT_MS; poll();
    CHECK(std::string(status().phase) == "fault" && status().pendingStage == 0 && status().stage == 4 && !status().cylinder.active && !Mock::servoAttached[5]);
    CHECK(std::string(status().notice) == "Stage selection return deadline exceeded");
  } else if (name == "selection_return_to_entry_deadline_shared") {
    finishStep(); CHECK(selectStage(2)); pump(); reply(6,0xFE,1); const uint32_t first = mockMillis; advance(1000); reply(6,0xFE,2); pump();
    CHECK(status().stage == 2 && std::string(status().phase) == "home_wait"); mockMillis = first + SystemConfig::Mechanism::STAGE_RETURN_TIMEOUT_MS; poll();
    CHECK(std::string(status().phase) == "fault" && status().pendingStage == 0 && !status().cylinder.active && !status().cylinder.in2High);
    CHECK(std::string(status().notice) == "Stage selection return deadline exceeded");
  } else if (name == "selection_ready_releases_old_deadline") {
    select(2); select(4); advance(SystemConfig::Mechanism::STAGE_RETURN_TIMEOUT_MS + 1);
    CHECK(std::string(status().phase) == "ready" && !SystemSafety::stopped()); CHECK(selectStage(2)); pump(); CHECK(std::string(status().phase) == "ready");
    CHECK(home()); pump(); advance(50); advance(5000); CHECK(std::string(status().phase) == "ready" && !SystemSafety::stopped());
  } else
    return false;
  return true;
}
// Added debug channels must remain independent, join completion barriers, and
// participate in the same STOP and previous-state handling as all older servos.
bool servoExpansionTest(const std::string &name) {
  if (name == "debug_extra_servo_endpoints") {
    CHECK(setDebug(true));
    CHECK(debugServo(10,0,0)); pump(); CHECK(Mock::servo[10] == 500);
    CHECK(debugServo(11,1800,0)); pump(); CHECK(Mock::servo[11] == 2500);
    CHECK(debugServo(12,450,0)); pump(); CHECK(Mock::servo[12] == 1000);
    CHECK(status().pulse10 == 500 && status().pulse11 == 2500 && status().pulse12 == 1000);
    for (uint8_t i=0;i<6;++i) CHECK(Mock::servo[SystemConfig::Mechanism::SERVO_PINS[i]] == SystemConfig::Mechanism::SERVO_START_US[i]);
    CHECK(!debugServo(10,1801,0) && !debugServo(11,900,60001) && !debugServo(13,900,0));
    CHECK(Mock::servo[10] == 500 && Mock::servo[11] == 2500 && Mock::servo[12] == 1000 && Mock::sent.empty());
    CHECK(setDebug(false));
  } else if (name == "debug_extra_servo_smooth_independent") {
    CHECK(setDebug(true));
    CHECK(debugServo(10,0,1000) && debugServo(11,1800,1500) && debugServo(12,0,2000)); pump();
    CHECK(Mock::servo[10] == 1500 && Mock::servo[11] == 1500 && Mock::servo[12] == 1500);
    advance(500);
    CHECK(Mock::servo[10] > 500 && Mock::servo[10] < 1500 && Mock::servo[11] > 1500 && Mock::servo[11] < 2500 && Mock::servo[12] > 500 && Mock::servo[12] < 1500);
    CHECK(!setDebug(false) && !next());
    advance(500); CHECK(Mock::servo[10] == 500 && Mock::servo[11] < 2500 && Mock::servo[12] > 500);
    advance(500); CHECK(Mock::servo[11] == 2500 && Mock::servo[12] > 500 && !setDebug(false));
    advance(500); CHECK(Mock::servo[12] == 500 && std::string(status().phase) == "ready" && setDebug(false) && Mock::sent.empty());
  } else if (name == "ram_thirteen_actuator_barrier_and_restore") {
    constexpr uint16_t total=SystemConfig::Mechanism::MOTOR_COUNT+SystemConfig::Mechanism::SERVO_COUNT+2;
    CHECK(planBegin(1,total,4)); uint16_t i=0;
    for (uint8_t node : SystemConfig::Mechanism::MOTOR_IDS) CHECK(planAction(i++,branch(stepperFor(node,123,320,0,false),1)));
    for (uint8_t pin : SystemConfig::Mechanism::SERVO_PINS) {
      const uint32_t ms=pin == 12 ? 1800 : pin == 11 ? 1200 : pin == 10 ? 800 : 100;
      CHECK(planAction(i++,branch(servo(pin,1700,ms,false),1)));
    }
    CHECK(planAction(i++,branch(cylinder(CylinderAction::Extend,1000,false),1)));
    CHECK(planAction(i,branch(waitAll(),1)) && planStep(0,0,total) && planCommit()); completePending(); nextStep();
    finish(4); finish(5); finish(6); advance(50); advance(1150);
    CHECK(!status().cylinder.active && Mock::servo[10] == 1700 && Mock::servo[11] == 1700 && Mock::servo[12] < 1700);
    CHECK(status().completed == 0 && !next() && !previous());
    advance(599); CHECK(status().completed == 0 && Mock::servo[12] < 1700); advance(1);
    CHECK(status().completed == 1 && std::string(status().phase) == "ready" && Mock::servo[12] == 1700);
    ackCursor=Mock::sent.size(); previousStep();
    for (uint8_t j=0;j<SystemConfig::Mechanism::SERVO_COUNT;++j) CHECK(Mock::servo[SystemConfig::Mechanism::SERVO_PINS[j]] == SystemConfig::Mechanism::SERVO_START_US[j]);
    CHECK(status().position4 == 0 && status().position5 == 0 && status().position6 == 0 && !status().cylinder.referenceKnown);
  } else if (name == "debug_nine_servo_stop_detaches_and_freezes") {
    CHECK(setDebug(true));
    for (uint8_t pin : SystemConfig::Mechanism::SERVO_PINS) CHECK(debugServo(pin,0,2000));
    advance(500); SystemSafety::stop("nine-servo STOP"); const size_t writes=Mock::servoWrites.size();
    for (uint8_t pin : SystemConfig::Mechanism::SERVO_PINS) CHECK(!Mock::servoAttached[pin] && !debugServo(pin,1800,0));
    advance(3000); CHECK(Mock::servoWrites.size() == writes && std::string(status().phase) == "fault" && count(0xF7) == 3);
  } else return false;
  return true;
}

void test(const std::string &name) {
  if (name == "servo_failure" || name == "servo_failure_pin12") {
    Mock::rejectServoPin = name == "servo_failure_pin12" ? 12 : 9;
    begin();
    CHECK(std::string(status().phase) == "fault" && Mock::globalStopCalls == 1);
    for (uint8_t pin : SystemConfig::Mechanism::SERVO_PINS)
      CHECK(!Mock::servoAttached[pin]);
    CHECK(!Mock::servoAttached[3]);
    checkFrames();
    return;
  }
  if (name == "startup_delayed_feedback") {
    begin(); advance(1000); CHECK(count(0x8C,4) == 1 && count(0x91) == 0);
    for (int i=0;i<3;++i) advance(500); CHECK(count(0x8C,4) == 4 && std::string(status().phase) == "booting");
    completePending(true); CHECK(status().positionKnownMask == 7 && count(0x91) == 1 && count(0xFE) == 0);
    CHECK(!status().cylinder.active && !status().cylinder.referenceKnown); checkFrames(); return;
  }
  if (name == "startup_delayed_enable") {
    startupSetup(); reply(4, 0x8C, 1); pump(); reply(4, 0x83, 1); pump();
    for (int i = 0; i < 4; ++i) advance(500);
    CHECK(count(0xF3, 4) == 5 && status().startupAttempts == 5 && count(0x91) == 0);
    completePending(true); CHECK(status().positionKnownMask == 7); checkFrames(); return;
  }
  if (name == "startup_ack_deadline") {
    startupSetup(); for (int i = 0; i < 19; ++i) advance(500); advance(499);
    CHECK(std::string(status().phase) == "booting" && status().startupAttempts == 20 && count(0x91) == 0);
    advance(1); CHECK(std::string(status().phase) == "fault" && Mock::globalStopCalls == 1);
    CHECK(std::string(status().notice).find("ID 4 0x8C attempt 20") != std::string::npos);
    CHECK(std::string(status().notice).find("10000 ms total") != std::string::npos);
    advance(10000); CHECK(count(0x8C, 4) == 20 && count(0x91) == 0 && !next()); checkFrames(); return;
  }
  if (name == "startup_all_ack_home_barrier") {
    startupSetup(); acknowledgeSettings(); CHECK(!next() && !home() && !selectProfile(1,5) && !triggerProp(1) && !exitStage1());
    reply(7,0x91,1); reply(7,0x91,2); reply(6,0x91,2); pump(); CHECK(status().positionKnownMask == 3 && count(0x91) == 1);
    finish(6,0x91); CHECK(status().positionKnownMask == 7 && std::string(status().phase) == "ready" && count(0xFE) == 0);
    CHECK(!status().cylinder.active && Mock::gpioState[SystemConfig::Cylinder::IN1_PIN] == LOW && Mock::gpioState[SystemConfig::Cylinder::IN2_PIN] == LOW); checkFrames(); return;
  }
  if (name == "startup_home_never_retried") {
    startupSetup(); acknowledgeSettings(); reply(6,0x91,1); advance(2000);
    CHECK(count(0x91) == 1 && std::string(status().phase) == "busy"); finish(6,0x91); CHECK(status().positionKnownMask == 7); checkFrames(); return;
  }
  if (name == "startup_feedback_rejected") {
    startupSetup(); reply(4, 0x8C, 0);
    CHECK(std::string(status().phase) == "fault" && Mock::globalStopCalls == 1);
    CHECK(std::string(status().notice).find("ID 4 0x8C attempt 1") != std::string::npos);
    advance(500); CHECK(count(0x8C) == 1 && count(0x91) == 0); checkFrames(); return;
  }
  if (name == "startup_enable_rejected") {
    startupSetup(); reply(4, 0x8C, 1); pump(); reply(4, 0x83, 1); pump(); reply(4, 0xF3, 0);
    CHECK(std::string(status().phase) == "fault" && Mock::globalStopCalls == 1);
    CHECK(std::string(status().notice).find("ID 4 0xF3 attempt 1") != std::string::npos);
    CHECK(count(0x91) == 0); checkFrames(); return;
  }
  if (name == "startup_id6_unwired_fault") {
    startupSetup(); acknowledgeWiredSettings(5); CHECK(status().startupNode == 6 && status().startupCommand == 0x8C);
    reply(7,0x8C,1); advance(10000); CHECK(std::string(status().phase) == "fault" && count(0x91) == 0 && count(0xF7) == 3);
    CHECK(std::string(status().notice).find("ID 6 0x8C") != std::string::npos); checkFrames(); return;
  }
  if (name == "startup_id6_enable_rejected") {
    startupSetup(); acknowledgeWiredSettings(5); reply(6,0x8C,1); pump(); reply(6,0x83,1); pump(); reply(6,0xF3,0);
    CHECK(std::string(status().phase) == "fault" && count(0x91) == 0 && std::string(status().notice).find("ID 6 0xF3") != std::string::npos); checkFrames(); return;
  }
  if (name == "startup_home_start_timeout") {
    startupSetup(); acknowledgeSettings(); advance(1500);
    CHECK(std::string(status().phase) == "fault" && status().positionKnownMask == 3 && count(0x91) == 1);
    CHECK(std::string(status().notice).find("ID 6 0x91") != std::string::npos); checkFrames(); return;
  }
  if (name == "startup_home_completion_timeout") {
    startupSetup(); acknowledgeSettings(); reply(6,0x91,1); advance(30000);
    CHECK(std::string(status().phase) == "fault" && status().positionKnownMask == 3 && count(0x91) == 1);
    CHECK(std::string(status().notice).find("completion ACK") != std::string::npos); checkFrames(); return;
  }
  if (name == "startup_partial_home_send_failure") {
    startupSetup(); acknowledgeWiredSettings(); reply(4,0x92,1); pump(); Mock::rejectNode=6; Mock::rejectCommand=0x91;
    reply(5,0x92,1); pump(); CHECK(count(0x91) == 0); advance(500);
    CHECK(std::string(status().phase) == "fault" && count(0xF7) == 3 && std::string(status().notice).find("CAN enqueue") != std::string::npos); checkFrames(); return;
  }
  if (name == "startup_stop_cancels_retry") {
    startupSetup(); advance(500); SystemSafety::stop("test startup STOP"); advance(10000);
    CHECK(count(0x8C) == 2 && count(0x91) == 0 && count(0xFE) == 0 && count(0xF7) == 3);
    for (uint8_t pin : SystemConfig::Mechanism::SERVO_PINS) CHECK(!Mock::servoAttached[pin]); checkFrames(); return;
  }
  if (name == "startup_retry_queue_late_ack") {
    startupSetup(); Mock::rejectNode = 4; Mock::rejectCommand = 0x8C; advance(500);
    CHECK(count(0x8C, 4) == 1 && std::string(status().startupPhase) == "feedback_queue");
    reply(4, 0x8C, 1); pump(); CHECK(status().startupCommand == 0x83 && countEnable(4) == 0);
    reply(4, 0x83, 1); pump(); CHECK(status().startupCommand == 0xF3 && countEnable(4) == 1);
    Mock::rejectNode = -1; completePending(true); CHECK(status().positionKnownMask == 7); checkFrames(); return;
  }
  if (name == "startup_enqueue_timeout") {
    Mock::rejectNode = 4; Mock::rejectCommand = 0x8C; begin(); advance(1000);
    CHECK(std::string(status().startupPhase) == "feedback_queue" && status().startupAttempts == 0);
    advance(500); CHECK(std::string(status().phase) == "fault" && count(0x91) == 0);
    CHECK(std::string(status().notice).find("CAN enqueue timeout") != std::string::npos); checkFrames(); return;
  }
  if (name == "startup_wrong_ack") {
    startupSetup(); reply(4, 0x8C, 1, 2); reply(4, 0x8C, 1, 1, true); reply(6, 0x8C, 1); reply(7, 0x8C, 1);
    reply(4, 0xF3, 1); reply(4, 0x8C, 2); pump(); CHECK(count(0xF3) == 0 && status().startupCommand == 0x8C);
    advance(500); CHECK(count(0x8C, 4) == 2); completePending(true); CHECK(status().positionKnownMask == 7); checkFrames(); return;
  }
  if (name == "startup_retry_clock_wrap") {
    mockMillis = UINT32_MAX - 1200; startupSetup(); advance(500); advance(500);
    CHECK(std::string(status().phase) == "booting" && count(0x8C, 4) == 3);
    completePending(true); CHECK(status().positionKnownMask == 7); checkFrames(); return;
  }
  if (name == "startup_stationary_zero_barrier") {
    startupSetup(); acknowledgeWiredSettings(); CHECK(!next() && !selectStage(3));
    reply(5,0x92,1); reply(4,0x92,2); reply(4,0x92,1,2); reply(4,0x92,1,1,true); pump();
    CHECK(status().positionKnownMask == 0 && count(0x91) == 0 && count(0xFE) == 0);
    reply(4,0x92,1); pump(); CHECK(status().positionKnownMask == 1 && status().startupNode == 5);
    reply(5,0x92,1); pump(); CHECK(status().positionKnownMask == 3 && count(0x91,4) == 0 && count(0x91,5) == 0);
    CHECK(count(0xFE) == 0); finish(6,0x91); finish(7,0x91); CHECK(status().positionKnownMask == 7); checkFrames(); return;
  }
  if (name == "startup_stationary_zero_rejected") {
    startupSetup(); acknowledgeWiredSettings(); reply(4,0x92,0);
    CHECK(std::string(status().phase) == "fault" && status().positionKnownMask == 0 && count(0x91) == 0 && count(0xFE) == 0);
    CHECK(std::string(status().notice).find("ID 4 0x92") != std::string::npos && count(0xF7) == 3); checkFrames(); return;
  }
  if (name == "startup_stationary_zero_timeout") {
    startupSetup(); acknowledgeWiredSettings(); reply(4,0x92,1); pump();
    for(int i=0;i<19;++i) advance(500); advance(500);
    CHECK(std::string(status().phase) == "fault" && status().positionKnownMask == 1 && count(0x91) == 0 && count(0xFE) == 0);
    CHECK(count(0x92,4) == 1 && count(0x92,5) == 20 && std::string(status().notice).find("ID 5 0x92 attempt 20") != std::string::npos); checkFrames(); return;
  }
  if (name == "startup_stationary_zero_retry_late_ack") {
    startupSetup(); acknowledgeWiredSettings(); Mock::rejectNode=4; Mock::rejectCommand=0x92; advance(500);
    CHECK(std::string(status().startupPhase) == "zero_queue" && count(0x92,4) == 1);
    reply(4,0x92,1); pump(); CHECK(status().positionKnownMask == 1 && status().startupNode == 5);
    Mock::rejectNode=-1; completePending(true); CHECK(status().positionKnownMask == 7); checkFrames(); return;
  }
  if (name == "startup_current_packet_sequence") {
    startupSetup(); const uint8_t nodes[]={4,5,6}, masks[]={1,3,7}; const uint16_t currents[]={3000,3000,5200};
    const uint8_t packets[3][4]={{0x83,0x0B,0xB8,0x4A},{0x83,0x0B,0xB8,0x4B},{0x83,0x14,0x50,0xED}};
    for (uint8_t i=0;i<3;++i) {
      const uint8_t node=nodes[i]; CHECK(status().startupNode == node && status().startupCommand == 0x8C);
      reply(node,0x8C,1); pump(); CHECK(status().startupCurrentMa == currents[i] && confirmedCurrent(node) == 0 && countEnable(node) == 0);
      CHECK(status().currentConfiguredMask == (i ? masks[i-1] : 0)); const auto frame=lastCurrent(node); CHECK(frame.len == 4 && memcmp(frame.buf,packets[i],4) == 0);
      reply(node,0x83,1); pump(); CHECK(status().currentConfiguredMask == masks[i] && confirmedCurrent(node) == currents[i]);
      CHECK(count(0x91) == 0 && count(0x92) == 0); reply(node,0xF3,1); pump();
    }
    reply(4,0x92,1); pump(); reply(5,0x92,1); pump(); finish(6,0x91);
    CHECK(status().positionKnownMask == 7 && count(0x91) == 1 && count(0x83) == 3 && count(0x83,7) == 0); checkFrames(); return;
  }
  if (name == "startup_current_id4_rejected" || name == "startup_current_id5_rejected") {
    const uint8_t node=name == "startup_current_id4_rejected" ? 4 : 5;
    startupCurrentSetup(node); reply(node,0x83,0);
    CHECK(std::string(status().phase) == "fault" && Mock::globalStopCalls == 1 && confirmedCurrent(node) == 0);
    CHECK(status().currentConfiguredMask == (node == 4 ? 0 : 1));
    CHECK(countEnable(node) == 0 && countEnable(6) == 0 && countEnable(7) == 0);
    CHECK(count(0x92) == 0 && count(0x91) == 0 && count(0xFE) == 0 && count(0xF7) == 3);
    const std::string expected="ID "+std::to_string(node)+" 0x83 3000 mA";
    CHECK(std::string(status().notice).find(expected) != std::string::npos);
    const int commandsAt=count(0x83); advance(11000); reply(node,0x83,1); pump();
    CHECK(count(0x83) == commandsAt && confirmedCurrent(node) == 0 && countEnable(node) == 0);
    checkFrames(); return;
  }
  if (name == "startup_current_id4_timeout" || name == "startup_current_id5_timeout") {
    const uint8_t node=name == "startup_current_id4_timeout" ? 4 : 5;
    startupCurrentSetup(node);
    for(int i=0;i<19;++i) advance(500); advance(499);
    CHECK(std::string(status().phase) == "booting" && status().startupAttempts == 20 && count(0x83,node) == 20);
    CHECK(confirmedCurrent(node) == 0 && countEnable(node) == 0 && count(0x92) == 0 && count(0x91) == 0);
    advance(1); CHECK(std::string(status().phase) == "fault" && Mock::globalStopCalls == 1);
    CHECK(status().currentConfiguredMask == (node == 4 ? 0 : 1) && confirmedCurrent(node) == 0);
    CHECK(std::string(status().notice).find("3000 mA") != std::string::npos && std::string(status().notice).find("10000 ms total") != std::string::npos);
    advance(10000); CHECK(count(0x83,node) == 20 && countEnable(node) == 0 && count(0x92) == 0 && count(0x91) == 0);
    checkFrames(); return;
  }
  if (name == "startup_current_lower_axes_wait_barrier") {
    startupCurrentSetup(4);
    reply(5,0x83,1); reply(4,0x83,2); reply(4,0x83,1,2); reply(4,0x83,1,1,true); pump();
    CHECK(status().currentConfiguredMask == 0 && status().current4Ma == 0 && status().current5Ma == 0 && countEnable(4) == 0);
    advance(500); CHECK(count(0x83,4) == 2 && count(0x92) == 0 && count(0x91) == 0);
    reply(4,0x83,1); pump(); CHECK(status().currentConfiguredMask == 1 && status().current4Ma == 3000 && countEnable(4) == 1);
    reply(4,0xF3,1); pump(); reply(5,0x8C,1); pump();
    CHECK(status().startupNode == 5 && status().startupCommand == 0x83 && status().current5Ma == 0 && countEnable(5) == 0);
    advance(1000); CHECK(count(0x83,5) == 2 && status().currentConfiguredMask == 1);
    CHECK(count(0x92) == 0 && count(0x91) == 0 && count(0xFE) == 0);
    reply(5,0x83,1); pump(); CHECK(status().currentConfiguredMask == 3 && status().current5Ma == 3000 && countEnable(5) == 1);
    completePending(true); CHECK(status().currentConfiguredMask == 7 && status().positionKnownMask == 7);
    checkFrames(); return;
  }
  if (name == "startup_current_delayed_ack") {
    startupCurrentSetup(6); for(int i=0;i<3;++i) advance(500); CHECK(count(0x83,6) == 4 && status().startupAttempts == 4);
    CHECK(status().current6Ma == 0 && status().currentConfiguredMask == 3 && countEnable(6) == 0);
    completePending(true); CHECK(status().currentConfiguredMask == 7 && status().current6Ma == 5200 && count(0x83,7) == 0 && count(0x91) == 1); checkFrames(); return;
  }
  if (name == "startup_current_id6_rejected") {
    startupCurrentSetup(6); reply(6,0x83,0);
    CHECK(std::string(status().phase) == "fault" && status().currentConfiguredMask == 3 && status().current6Ma == 0);
    CHECK(countEnable(6) == 0 && countEnable(7) == 0 && count(0x92) == 0 && count(0x91) == 0 && count(0xF7) == 3);
    CHECK(std::string(status().notice).find("ID 6 0x83") != std::string::npos); checkFrames(); return;
  }
  if (name == "startup_removed_id7_current_ignored") {
    startupCurrentSetup(6); reply(7,0x83,0); reply(7,0x83,1); pump();
    CHECK(std::string(status().phase) == "booting" && status().startupNode == 6 && status().currentConfiguredMask == 3 && countEnable(6) == 0);
    completePending(true); CHECK(status().currentConfiguredMask == 7 && count(0x83,7) == 0); checkFrames(); return;
  }
  if (name == "startup_current_ack_deadline") {
    startupCurrentSetup(6); for(int i=0;i<19;++i) advance(500); advance(499);
    CHECK(status().startupAttempts == 20 && count(0x83,6) == 20 && countEnable(6) == 0);
    advance(1); CHECK(std::string(status().phase) == "fault" && status().current6Ma == 0 && status().currentConfiguredMask == 3);
    CHECK(std::string(status().notice).find("ID 6 0x83") != std::string::npos && std::string(status().notice).find("attempt 20") != std::string::npos && std::string(status().notice).find("5200 mA") != std::string::npos && std::string(status().notice).find("10000 ms total") != std::string::npos);
    advance(10000); CHECK(count(0x83,6) == 20 && count(0x92) == 0 && count(0x91) == 0 && countEnable(6) == 0); checkFrames(); return;
  }
  if (name == "startup_removed_id7_all_feedback_ignored") {
    startupSetup(); reply(7,0x8C,1); reply(7,0x83,1); reply(7,0xF3,1); reply(7,0x92,1); reply(7,0x91,1); reply(7,0x91,2); pump();
    CHECK(status().startupNode == 4 && status().currentConfiguredMask == 0 && status().positionKnownMask == 0);
    completePending(true); CHECK(status().positionKnownMask == 7 && count(0x91,7) == 0); checkFrames(); return;
  }
  if (name == "startup_current_wrong_ack") {
    startupCurrentSetup(6); reply(7,0x83,1); reply(4,0x83,1); reply(6,0x83,1,2); reply(6,0x83,1,1,true);
    reply(6,0xF3,1); reply(6,0x83,2);
    CAN_message_t f{}; f.id=6; f.len=3; f.buf[0]=0x83; f.buf[1]=1; f.buf[2]=0x8A; f.flags.extended=true; receive(1,f);
    f.flags.extended=false; f.flags.remote=true; receive(1,f); f.flags.remote=false; f.len=2; receive(1,f); pump();
    CHECK(status().startupCommand == 0x83 && status().currentConfiguredMask == 3 && status().current6Ma == 0 && countEnable(6) == 0);
    advance(500); CHECK(count(0x83,6) == 2); completePending(true); CHECK(status().currentConfiguredMask == 7); checkFrames(); return;
  }
  if (name == "startup_current_retry_queue_late_ack") {
    startupCurrentSetup(6); Mock::rejectNode=6; Mock::rejectCommand=0x83; advance(500);
    CHECK(std::string(status().startupPhase) == "current_queue" && count(0x83,6) == 1 && status().current6Ma == 0);
    reply(6,0x83,1); pump(); CHECK(status().currentConfiguredMask == 7 && countEnable(6) == 1 && status().startupCommand == 0xF3);
    Mock::rejectNode=-1; completePending(true); CHECK(count(0x83,6) == 1 && status().currentConfiguredMask == 7); checkFrames(); return;
  }
  if (name == "startup_current_retry_fixed_deadline") {
    startupCurrentSetup(6); for(int i=0;i<18;++i) advance(500);
    Mock::rejectNode=6; Mock::rejectCommand=0x83; advance(500);
    CHECK(std::string(status().startupPhase) == "current_queue" && count(0x83,6) == 19);
    advance(499); CHECK(std::string(status().phase) == "booting"); advance(1);
    CHECK(std::string(status().phase) == "fault" && std::string(status().notice).find("10000 ms total") != std::string::npos);
    CHECK(status().currentConfiguredMask == 3 && countEnable(6) == 0 && count(0x91) == 0); checkFrames(); return;
  }
  if (name == "startup_current_enqueue_timeout") {
    startupSetup(); acknowledgeWiredSettings(5); Mock::rejectNode=6; Mock::rejectCommand=0x83; reply(6,0x8C,1); pump();
    CHECK(status().startupAttempts == 0 && count(0x83,6) == 0); advance(500);
    CHECK(std::string(status().phase) == "fault" && status().currentConfiguredMask == 3 && status().current6Ma == 0 && countEnable(6) == 0 && count(0x91) == 0); checkFrames(); return;
  }
  if (name == "startup_current_early_ack") {
    startupSetup(); acknowledgeWiredSettings(5); Mock::rejectNode=6; Mock::rejectCommand=0x83; reply(6,0x8C,1); pump();
    reply(6,0x83,1); pump(); CHECK(status().currentConfiguredMask == 3 && status().current6Ma == 0 && countEnable(6) == 0 && status().startupAttempts == 0);
    Mock::rejectNode=-1; advance(10); CHECK(count(0x83,6) == 1 && countEnable(6) == 0);
    completePending(true); CHECK(status().currentConfiguredMask == 7); checkFrames(); return;
  }
  if (name == "startup_current_late_ack_after_deadline") {
    startupCurrentSetup(6); mockMillis+=10000; reply(6,0x83,1); pump();
    CHECK(std::string(status().phase) == "fault" && status().current6Ma == 0 && status().currentConfiguredMask == 3);
    CHECK(countEnable(6) == 0 && count(0x91) == 0 && count(0x92) == 0); checkFrames(); return;
  }
  if (name == "startup_current_stop") {
    startupCurrentSetup(6); advance(500); SystemSafety::stop("current STOP"); advance(10000);
    CHECK(std::string(status().phase) == "fault" && count(0x83,6) == 2 && countEnable(6) == 0 && count(0xF7) == 3 && status().currentConfiguredMask == 3);
    CHECK(!status().cylinder.active && !status().cylinder.in1High && !status().cylinder.in2High && !next()); checkFrames(); return;
  }
  if (name == "startup_current_clock_wrap") {
    mockMillis=UINT32_MAX-1200; startupCurrentSetup(6); advance(500); advance(500);
    CHECK(std::string(status().phase) == "booting" && count(0x83,6) == 3 && mockMillis<1000);
    completePending(true); CHECK(status().currentConfiguredMask == 7 && status().positionKnownMask == 7); checkFrames(); return;
  }
  if (name == "cylinder_driver_startup_low") {
    CylinderController::begin(); const auto c=CylinderController::status();
    CHECK(!c.active && !c.in1High && !c.in2High && !c.referenceKnown && c.action == CylinderAction::Stop);
    CHECK(Mock::gpioMode[SystemConfig::Cylinder::IN1_PIN] == OUTPUT && Mock::gpioMode[SystemConfig::Cylinder::IN2_PIN] == OUTPUT && Mock::gpioState[SystemConfig::Cylinder::IN1_PIN] == LOW && Mock::gpioState[SystemConfig::Cylinder::IN2_PIN] == LOW);
    CHECK(Mock::gpioWrites.size() == 2 && Mock::gpioModes.size() == 2); checkFrames(); return;
  }
  if (name == "cylinder_driver_deadtime_full_duration") {
    CylinderController::begin(); CHECK(CylinderController::start(CylinderAction::Extend,1000));
    mockMillis+=49; CylinderController::poll(); CHECK(!CylinderController::status().in1High && CylinderController::status().remainingMs == 1000);
    ++mockMillis; CylinderController::poll(); CHECK(CylinderController::status().in1High && !CylinderController::status().in2High);
    mockMillis+=999; CylinderController::poll(); CHECK(CylinderController::status().active && CylinderController::status().remainingMs == 1);
    ++mockMillis; CylinderController::poll(); const auto c=CylinderController::status(); CHECK(!c.active && !c.in1High && !c.in2High && c.referenceKnown && c.referenceAction == CylinderAction::Extend); checkFrames(); return;
  }
  if (name == "cylinder_driver_late_poll_does_not_shorten_travel") {
    CylinderController::begin(); CHECK(CylinderController::start(CylinderAction::Retract,100)); mockMillis+=1000; CylinderController::poll();
    CHECK(CylinderController::status().in2High && CylinderController::status().remainingMs == 100); mockMillis+=99; CylinderController::poll(); CHECK(CylinderController::status().active);
    ++mockMillis; CylinderController::poll(); CHECK(CylinderController::status().referenceKnown && !CylinderController::status().active); checkFrames(); return;
  }
  if (name == "cylinder_driver_reverse_low_deadtime") {
    CylinderController::begin(); CHECK(CylinderController::start(CylinderAction::Extend,1000)); mockMillis+=50; CylinderController::poll(); mockMillis+=100;
    CHECK(CylinderController::start(CylinderAction::Retract,500)); CHECK(!CylinderController::status().in1High && !CylinderController::status().in2High && !CylinderController::status().referenceKnown);
    mockMillis+=49; CylinderController::poll(); CHECK(!CylinderController::status().in2High); ++mockMillis; CylinderController::poll(); CHECK(CylinderController::status().in2High && !CylinderController::status().in1High);
    mockMillis+=500; CylinderController::poll(); CHECK(!CylinderController::status().active && CylinderController::status().referenceAction == CylinderAction::Retract); checkFrames(); return;
  }
  if (name == "cylinder_driver_stop_cancels_deadtime" || name == "cylinder_driver_stop_cancels_motion") {
    CylinderController::begin(); CHECK(CylinderController::start(CylinderAction::Extend,1000));
    if(name == "cylinder_driver_stop_cancels_motion") { mockMillis+=50; CylinderController::poll(); CHECK(CylinderController::status().in1High); }
    CylinderController::stop(); CHECK(!CylinderController::status().active && !CylinderController::status().in1High && !CylinderController::status().in2High && !CylinderController::status().referenceKnown);
    mockMillis+=10000; CylinderController::poll(); CHECK(!CylinderController::status().active && Mock::gpioState[SystemConfig::Cylinder::IN1_PIN] == LOW && Mock::gpioState[SystemConfig::Cylinder::IN2_PIN] == LOW); checkFrames(); return;
  }
  if (name == "cylinder_driver_invalid_requests") {
    CylinderController::begin(); CHECK(!CylinderController::start(CylinderAction::Extend,0) && !CylinderController::start(CylinderAction::Retract,60001));
    CHECK(!CylinderController::start(static_cast<CylinderAction>(3),100)); CHECK(CylinderController::start(CylinderAction::Stop,0));
    CHECK(!CylinderController::status().active && Mock::gpioState[SystemConfig::Cylinder::IN1_PIN] == LOW && Mock::gpioState[SystemConfig::Cylinder::IN2_PIN] == LOW); checkFrames(); return;
  }
  if (name == "cylinder_driver_timed_stop_and_reference") {
    CylinderController::begin(); CHECK(CylinderController::start(CylinderAction::Extend,10)); mockMillis+=50; CylinderController::poll(); mockMillis+=10; CylinderController::poll();
    CHECK(CylinderController::status().referenceKnown); CHECK(CylinderController::start(CylinderAction::Stop,100)); CHECK(CylinderController::status().active && !CylinderController::status().in1High && !CylinderController::status().in2High);
    mockMillis+=99; CylinderController::poll(); CHECK(CylinderController::status().active); ++mockMillis; CylinderController::poll();
    CHECK(!CylinderController::status().active && CylinderController::status().referenceKnown); CylinderController::stop(); CHECK(CylinderController::status().referenceKnown);
    CylinderController::invalidateReference(); CHECK(!CylinderController::status().referenceKnown); checkFrames(); return;
  }
  if (name == "cylinder_driver_clock_wrap") {
    CylinderController::begin(); mockMillis=UINT32_MAX-25; CHECK(CylinderController::start(CylinderAction::Retract,100));
    mockMillis+=49; CylinderController::poll(); CHECK(!CylinderController::status().in2High); ++mockMillis; CylinderController::poll(); CHECK(CylinderController::status().in2High);
    mockMillis+=100; CylinderController::poll(); CHECK(!CylinderController::status().active && CylinderController::status().referenceKnown); checkFrames(); return;
  }
  if (name == "cylinder_driver_maximum_duration") {
    CylinderController::begin(); CHECK(CylinderController::start(CylinderAction::Extend,60000)); mockMillis+=50; CylinderController::poll();
    mockMillis+=59999; CylinderController::poll(); CHECK(CylinderController::status().active); ++mockMillis; CylinderController::poll(); CHECK(CylinderController::status().referenceKnown && !CylinderController::status().active); checkFrames(); return;
  }
  ready();
  if (trimOutputTest(name) || selectionTest(name) || servoExpansionTest(name)) { checkFrames(); return; }
  if (name == "startup") {
    CHECK(count(0xFE) == 0 && count(0x91) == 0 && !previous() && status().positionKnownMask == 7);
    constexpr uint8_t expectedPins[] = {2, 3, 4, 5, 6, 9, 10, 11, 12};
    constexpr int expectedPulse[] = {1150, 2250, 2000, 2100, 1400, 1100, 1500, 1500, 1500};
    static_assert(SystemConfig::Mechanism::SERVO_COUNT == 9, "Nine standby servo channels are required");
    for (uint8_t i = 0; i < SystemConfig::Mechanism::SERVO_COUNT; ++i) {
      CHECK(SystemConfig::Mechanism::SERVO_PINS[i] == expectedPins[i] && SystemConfig::Mechanism::SERVO_START_US[i] == expectedPulse[i]);
      CHECK(Mock::servoAttached[expectedPins[i]] && Mock::servo[expectedPins[i]] == expectedPulse[i]);
    }
  } else if (name == "startup_zero_target_is_known") {
    upload(4, 0); finishStep();
    CHECK(count(0xFE, 6) == 0 && status().positionKnownMask == 7);
    finishStep(); CHECK(count(0xFE, 6) == 0);
  } else if (name == "stage2") {
    select(2); cylinderReference(CylinderAction::Retract); CHECK(count(0xFE) == 0 && count(0x91) == 0);
    CHECK(status().steps == Stage2::STEP_COUNT && Stage2::STEP_COUNT == 6);
    for (uint8_t step=1;step<=Stage2::STEP_COUNT;++step) {
      finishStep(); CHECK(status().completed == step);
      for (uint8_t pin : {uint8_t(4),uint8_t(6),uint8_t(9)}) CHECK(Mock::servo[pin] == stage2Pulse(step,pin));
      cylinderReference(stage2Reference(step));
    }
    finishStep(); CHECK(status().completed == Stage2::REPEAT_FROM && status().progress == Stage2::STEP_COUNT+1);
    cylinderReference(stage2Reference(6));
    CHECK(Mock::servo[4] == stage2Pulse(2,4) && Mock::servo[9] == stage2Pulse(2,9) && Mock::servo[6] == stage2Pulse(6,6) && Mock::sent.empty());
  } else if (name == "stage2_parallel") {
    select(2); Mock::servoWrites.clear(); nextStep();
    CHECK(!status().cylinder.active && status().completed == 1);
    CHECK(Mock::servoWrites.size() == 2 && Mock::servoWrites[0].pin == 9 && Mock::servoWrites[1].pin == 4);
    CHECK(Mock::servoWrites[0].pulse == stage2Pulse(1,9) && Mock::servoWrites[1].pulse == stage2Pulse(1,4)); cylinderReference(CylinderAction::Retract);
    for (uint8_t step=2;step<=5;++step) finishStep();
    Mock::servoWrites.clear(); nextStep(); CHECK(!next() && !previous());
    advance(999); CHECK(status().completed == 5 && Mock::servo[4] != stage2Pulse(6,4) && Mock::servo[9] != stage2Pulse(6,9));
    advance(1); CHECK(status().completed == 6 && Mock::servo[4] == stage2Pulse(6,4) && Mock::servo[9] == stage2Pulse(6,9));
    cylinderReference(stage2Reference(6));
  } else if (name == "previous_stage2") {
    select(2); for(uint8_t step=1;step<=Stage2::STEP_COUNT;++step) finishStep();
    for(uint8_t remaining=Stage2::STEP_COUNT-1;remaining>=1;--remaining) {
      previousStep(); CHECK(status().completed == remaining);
      for(uint8_t pin : {uint8_t(4),uint8_t(6),uint8_t(9)}) CHECK(Mock::servo[pin] == stage2Pulse(remaining,pin));
      cylinderReference(stage2Reference(remaining));
    }
    previousStep(); CHECK(status().progress == 0 && !previous() && Mock::servo[4] == 2000 && Mock::servo[6] == 1400 && Mock::servo[9] == 1100); cylinderReference(CylinderAction::Retract);
    CHECK(Mock::sent.empty());
  } else if (name == "previous_stage2_many_loops") {
    select(2); std::vector<MechanismController::Status> history{status()};
    for(int i=0;i<304;++i) { finishStep(); history.push_back(status()); }
    for(int i=0;i<300;++i) { previousStep(); const auto &expected=history[size_t(303-i)]; CHECK(status().progress == expected.progress && status().completed == expected.completed);
      CHECK(status().cylinder.referenceAction == expected.cylinder.referenceAction && Mock::servo[4] == expected.pulse4 && Mock::servo[6] == expected.pulse6 && Mock::servo[9] == expected.pulse9); }
    CHECK(status().progress == 4 && status().completed == 4 && Mock::sent.empty());
  } else if (name == "stage2_home") {
    select(2); finishStep(); CHECK(home()); pump(); CHECK(status().cylinder.active && status().cylinder.action == CylinderAction::Retract && std::string(status().phase) == "home_wait");
    CHECK(Mock::servo[4] == 2000 && Mock::servo[6] == 1400 && Mock::servo[9] == 1100 && !next()); completePending();
    CHECK(status().completed == 0 && status().progress == 0 && !previous() && Mock::sent.empty()); cylinderReference(CylinderAction::Retract);
  } else if (name == "stage3_cycle") {
    select(3); CHECK(status().steps == 4 && Mock::servo[3] == stage3Pulse(0));
    for (uint8_t step : {uint8_t(1),uint8_t(2),uint8_t(3),uint8_t(4),uint8_t(2)}) {
      finishStep(); CHECK(status().completed == step && status().position4 == stage3Motor(step,4).position && status().position5 == stage3Motor(step,5).position);
      CHECK(Mock::servo[3] == stage3Pulse(step) && speed(4) == stage3Motor(step,4).rpm && lastMove(4).buf[3] == stage3Motor(step,4).acc);
    }
    CHECK(status().progress == 5 && count(0x91) == 0 && count(0x92) == 0 && count(0xFE,6) == 0 && count(0xFE,7) == 0);
  } else if (name == "stage3_pair_barrier") {
    select(3); nextStep(); CHECK(!next() && !previous()); finish(4);
    CHECK(status().position4 == stage3Motor(1,4).position && status().position5 == 0 && !next());
    reply(5,0xFE,2); pump(); CHECK(std::string(status().phase) == "busy"); finish(5);
    completePending(); // A configured servo ramp must also finish before Ready.
    CHECK(status().position5 == stage3Motor(1,5).position && status().completed == 1 && std::string(status().phase) == "ready");
  } else if (name == "stage3_previous") {
    select(3); for (int i=0;i<7;++i) finishStep();
    previousStep(); CHECK(status().completed == 3 && status().position4 == stage3Motor(3,4).position && status().position5 == stage3Motor(3,5).position && Mock::servo[3] == stage3Pulse(3));
    for (int i=0;i<5;++i) previousStep();
    CHECK(status().completed == 1 && status().position4 == stage3Motor(1,4).position && status().position5 == stage3Motor(1,5).position && Mock::servo[3] == stage3Pulse(1));
    previousStep(); CHECK(status().progress == 0 && status().position4 == 0 && status().position5 == 0 && Mock::servo[3] == stage3Pulse(0));
    CHECK(count(0x91) == 0 && count(0x92) == 0 && status().position6 == 0);
  } else if (name == "stage3_home_fe") {
    select(3); finishStep(); CHECK(home()); pump();
    CHECK(target(4) == 0 && target(5) == 0 && Mock::servo[3] == 2250 && count(0x91) == 0 && count(0x92) == 0);
    finish(4); CHECK(!next()); finish(5); CHECK(status().progress == 0 && status().position4 == 0 && status().position5 == 0);
    finishStep(); CHECK(status().completed == 1 && Mock::servo[3] == stage3Pulse(1));
  } else if (name == "stage3_park_barrier") {
    select(3); finishStep(); CHECK(selectStage(4)); pump();
    CHECK(status().stage == 3 && status().pendingStage == 4 && target(4) == 0 && target(5) == 0 && Mock::servo[3] == 2250);
    CHECK(speed(4) == 320 && speed(5) == 320 && lastMove(4).buf[3] == 0 && lastMove(5).buf[3] == 0);
    const uint8_t prefix[] = {0xFE, 0x01, 0x40, 0x00, 0x00, 0x00, 0x00};
    for (uint8_t node : {uint8_t(4), uint8_t(5)}) {
      const auto frame = lastMove(node);
      CHECK(frame.id == node && frame.len == 8 && memcmp(frame.buf, prefix, sizeof(prefix)) == 0);
      CHECK(frame.buf[7] == (node == 4 ? 0x43 : 0x44));
    }
    CHECK(std::string(status().phase) == "return_wait" && status().step == 0 && status().completed == 0 && status().progress == 0);
    // A completion before acceptance is ignored; one finished motor cannot activate the destination.
    reply(4,0xFE,2); pump(); CHECK(status().stage == 3 && status().position4 == stage3Motor(1,4).position);
    finish(4); CHECK(status().stage == 3 && status().pendingStage == 4 && status().position4 == 0 && !next() && !previous() && !runAll() && count(0x91) == 0);
    reply(5,0xFE,1); advance(500); CHECK(status().stage == 3 && status().pendingStage == 4 && !next()); reply(5,0xFE,2); pump();
    CHECK(status().stage == 4 && status().pendingStage == 0 && std::string(status().phase) == "ready" && status().progress == 0);
    CHECK(status().position4 == 0 && status().position5 == 0 && count(0xFE,4) == 2 && count(0xFE,5) == 2 && count(0x91) == 0 && count(0xFE,6) == 0 && count(0xFE,7) == 0);
  } else if (name == "stage3_switch_busy_to_stage2") {
    select(3); nextStep(); CHECK(selectStage(2)); pump(); CHECK(std::string(status().phase) == "busy" && status().pendingStage == 2 && !status().cylinder.active);
    finish(4); CHECK(!next()); finish(5);
    for (const auto &a : Stage3::YAML_STEP_1) if (a.kind == ActionKind::Servo) advance(a.ms);
    CHECK(std::string(status().phase) == "return_wait" && target(4) == 0 && target(5) == 0);
    finish(4); CHECK(status().stage == 3); finish(5); CHECK(status().stage == 2 && std::string(status().phase) == "home_wait" && status().cylinder.active && !next());
    ackCursor=Mock::sent.size(); completePending(); CHECK(status().completed == 0 && count(0x91) == 0); cylinderReference(CylinderAction::Retract); finishStep(); cylinderReference(stage2Reference(1));
  } else if (name == "stage3_park_retarget") {
    select(3); finishStep(); CHECK(selectStage(4)); pump(); CHECK(selectStage(1));
    CHECK(status().stage == 3 && status().pendingStage == 1 && std::string(status().phase) == "return_wait");
    finishPair(); CHECK(status().stage == 1 && status().pendingStage == 0 && status().progress == 0 && std::string(status().phase) == "ready");
    CHECK(target(4) == 0 && target(5) == 0 && count(0x91) == 0 && count(0xFE,6) == 0);
  } else if (name == "stage3_park0_reentry_stage1" || name == "stage3_park0_reentry_stage2") {
    const uint8_t destination=name == "stage3_park0_reentry_stage1" ? 1 : 2;
    select(3); finishStep(); CHECK(selectStage(destination)); pump(); CHECK(target(4) == 0 && target(5) == 0 && lastMove(4).buf[3] == 0 && lastMove(5).buf[3] == 0);
    finish(5); CHECK(status().stage == 3 && !next()); finish(4); ackCursor=Mock::sent.size(); completePending();
    CHECK(status().stage == destination && status().completed == 0 && count(0x91) == 0); finishStep(); select(3);
    CHECK(status().position4 == 0 && status().position5 == 0); finishStep(); previousStep(); CHECK(status().position4 == 0 && status().position5 == 0 && Mock::servo[3] == 2250 && !previous());
  } else if (name == "stage3_partial_pair_failure") {
    select(3); Mock::rejectNode=5; Mock::rejectCommand=0xFE; nextStep();
    CHECK(std::string(status().phase) == "fault" && count(0xFE,4) == 1 && count(0xFE,5) == 0 && count(0xF7) == 3);
    CHECK(std::string(status().notice).find("ID 5 0xFE") != std::string::npos);
  } else if (name == "stage3_park_partial_failure") {
    select(3); finishStep(); Mock::rejectNode=5; Mock::rejectCommand=0xFE; CHECK(selectStage(4)); pump();
    CHECK(std::string(status().phase) == "fault" && status().stage == 3 && count(0x91) == 0 && count(0xF7) == 3);
  } else if (name == "stage3_runtime_pair") {
    CHECK(planBegin(1,4,3)); CHECK(planAction(0,branch(stepperFor(4,-123,320,2,false),1)));
    CHECK(planAction(1,branch(stepperFor(5,123,320,2,false),1))); CHECK(planAction(2,branch(servo(3,1700,0,false),1)));
    CHECK(planAction(3,branch(waitAll(),1))); CHECK(planStep(0,0,4)); CHECK(planCommit()); completePending(); finishStep();
    CHECK(status().runtimePlan && status().position4 == -123 && status().position5 == 123 && Mock::servo[3] == 1700);
  } else if (name == "stage3_debug_fe_only") {
    select(3); CHECK(setDebug(true)); CHECK(!debugStepper(0,320,0,true,4)); CHECK(!debugStepper(0,320,0,true,5));
    CHECK(debugStepper(-321,320,0,false,4)); CHECK(debugStepper(321,320,0,false,5)); CHECK(debugServo(3,900,0));
    completePending(); CHECK(status().position4 == -321 && status().position5 == 321 && Mock::servo[3] == 1500 && count(0x91) == 0);
    CHECK(setDebug(false));
  } else if (name == "active_hardware_validation") {
    for (uint8_t node : SystemConfig::Mechanism::MOTOR_IDS) CHECK(valid(stepperFor(node,0,320)));
    for (uint8_t pin : SystemConfig::Mechanism::SERVO_PINS) CHECK(valid(servo(pin,1500)));
    CHECK(!valid(stepperFor(8,0,320)) && !valid(servo(8,1500)));
    CHECK(planBegin(1,1,4)); CHECK(!planAction(0,branch(stepperFor(8,0,320),1))); CHECK(!planAction(0,branch(servo(8,1500),1))); planAbort();
  } else if (name == "stage1") {
    select(1); CHECK(status().prop == 2 && status().steps == 3);
    nextStep(); CHECK(target() == 8000 && speed() == 1000 && lastMove().buf[3] == 230);
    CHECK(Mock::servo[5] == 2056 && Mock::servo[2] == 1833); completePending();
    finishStep(); CHECK(Mock::servo[2] == 2111);
    nextStep(); CHECK(target() == propMotor(2,3).position && lastMove().buf[3] == 245); completePending();
    CHECK(Mock::servo[2] == 1389 && status().completed == 3 && status().propComplete);
    const size_t frames=Mock::sent.size(); CHECK(!next()); pump(); CHECK(Mock::sent.size() == frames && status().progress == 3);
  } else if (name == "stage4") {
    nextStep();
    CHECK(target() == stage4Motor(1).position && Mock::servo[2] == 1500 && Mock::servo[5] == 1400);
    completePending();
    finishStep();
    CHECK(Mock::servo[2] == 1120);
    nextStep();
    CHECK(target() == stage4Motor(3).position && lastMove().buf[3] == 5);
    finish();
    advance(1000);
    CHECK(Mock::servo[5] > 1400 && Mock::servo[5] < 1550 && !next());
    advance(1000);
    CHECK(std::string(status().phase) == "ready" && Mock::servo[5] == 1550);
    nextStep();
    CHECK(target() == stage4Motor(4, true).position && speed() == stage4Motor(4, true).rpm);
    finish();
    CHECK(target() == stage4Motor(4).position && speed() == 320 && Mock::servo[5] == 1800);
    finish();
    finishStep();
    CHECK(Mock::servo[2] == 1500);
    finishStep();
    CHECK(status().position6 == 0 && Mock::servo[2] == 1150 && Mock::servo[5] == 2100);
  } else if (name == "previous_stage1") {
    select(1); for (int i = 0; i < 3; ++i) finishStep();
    CHECK(status().propComplete && status().position6 == propMotor(2,3).position);
    previousStep(); CHECK(status().completed == 2 && status().position6 == 8000 && Mock::servo[2] == 2111 && !status().propComplete);
    previousStep(); CHECK(status().completed == 1 && Mock::servo[2] == 1833);
    previousStep(); CHECK(status().position6 == 0 && Mock::servo[5] == 2100 && Mock::servo[2] == 1150 && !previous());
  } else if (name == "previous_stage4_launch") {
    for (int i = 0; i < 4; ++i)
      finishStep();
    previousStep();
    CHECK(target() == stage4Motor(3).position && speed() == 320);
    CHECK(Mock::servo[5] == 1550 && Mock::servo[2] == 1120 && status().completed == 3);
    previousStep();
    CHECK(status().position6 == stage4Motor(1).position && Mock::servo[5] == 1400);
    previousStep();
    CHECK(Mock::servo[2] == 1500);
    previousStep();
    CHECK(status().position6 == 0 && Mock::servo[5] == 2100);
  } else if (name == "previous_wrap") {
    for (int i = 0; i < 7; ++i)
      finishStep();
    previousStep();
    CHECK(status().progress == 6 && status().completed == 6 && status().position6 == 0);
    previousStep();
    CHECK(status().completed == 5 && status().position6 == stage4Motor(4).position && Mock::servo[2] == 1500 &&
          Mock::servo[5] == 1800);
  } else if (name == "previous_session_entry") {
    for (int i = 0; i < 3; ++i) finishStep();
    CHECK(status().position6 == stage4Motor(3).position && Mock::servo[2] == 1120 && Mock::servo[5] == 1550);
    select(1); CHECK(status().position6 == 0 && Mock::servo[2] == 1150 && Mock::servo[5] == 2100 && !previous());
    finishStep(); previousStep();
    CHECK(status().position6 == 0 && Mock::servo[2] == 1150 && Mock::servo[5] == 2100 && status().progress == 0 && !previous());
  } else if (name == "previous_busy") {
    nextStep();
    CHECK(!previous() && !next());
    completePending();
    CHECK(setDebug(true));
    CHECK(!previous());
    CHECK(setDebug(false));
    CHECK(!previous());

  } else if (name == "wrong_feedback") {
    nextStep();
    reply(6, 0xFE, 1, 2);
    reply(6, 0xFE, 2, 2);
    reply(6, 0xFE, 1, 1, true);
    reply(6, 0xFE, 2);
    pump();
    CHECK(std::string(status().phase) == "busy");
    reply(6, 0xFE, 1);
    reply(6, 0xFE, 2);
    pump();
    CHECK(std::string(status().phase) == "ready");
  } else if (name == "late_feedback") {
    nextStep();
    mockMillis += 1500;
    reply(6, 0xFE, 1);
    CHECK(Mock::globalStopCalls == 1);

  } else if (name == "motion_never_retried") {
    CHECK(home()); pump(); CHECK(count(0x91) == 1); reply(6, 0x91, 1); advance(2000);
    CHECK(count(0x91) == 1 && status().positionKnownMask == 7); finish(6, 0x91);
    CHECK(status().positionKnownMask == 7); nextStep(); reply(6, 0xFE, 1); advance(2000);
    CHECK(count(0xFE) == 1 && std::string(status().phase) == "busy"); finish();
  } else if (name == "manual_home_start_timeout") {
    CHECK(home()); pump(); advance(1500);
    CHECK(std::string(status().phase) == "fault" && count(0x91) == 1 && status().positionKnownMask == 3);
    CHECK(std::string(status().notice).find("ID 6 0x91") != std::string::npos);
    CHECK(std::string(status().notice).find("start ACK") != std::string::npos);
  } else if (name == "manual_home_completion_timeout") {
    CHECK(home()); pump(); reply(6, 0x91, 1); advance(30000);
    CHECK(std::string(status().phase) == "fault" && count(0x91) == 1);
    CHECK(std::string(status().notice).find("ID 6 0x91") != std::string::npos);
    CHECK(std::string(status().notice).find("completion ACK") != std::string::npos);
  } else if (name == "motion_start_timeout") {
    nextStep(); advance(1500);
    CHECK(std::string(status().phase) == "fault" && count(0xFE) == 1 && count(0xF7) == 3);
    CHECK(std::string(status().notice).find("ID 6 0xFE") != std::string::npos);
    CHECK(std::string(status().notice).find("start ACK") != std::string::npos);
  } else if (name == "motion_completion_timeout") {
    nextStep(); reply(6, 0xFE, 1); advance(30000);
    CHECK(std::string(status().phase) == "fault" && count(0xFE) == 1 && count(0xF7) == 3);
    CHECK(std::string(status().notice).find("completion ACK") != std::string::npos);
  } else if (name == "motion_enqueue_timeout") {
    Mock::rejectNode = 6; Mock::rejectCommand = 0xFE; nextStep(); advance(500);
    CHECK(std::string(status().phase) == "fault" && count(0xFE) == 0 && count(0xF7) == 3);
    CHECK(Mock::globalStopCalls == 1);
  } else if (name == "motion_rejected_limit") {
    nextStep(); reply(6, 0xFE, 1); reply(6, 0xFE, 5);
    CHECK(std::string(status().phase) == "fault" && Mock::globalStopCalls == 1);
    CHECK(status().positionKnownMask == 3 && count(0xF7) == 3);
  } else if (name == "stop_global") {
    for (int i = 0; i < 2; ++i) finishStep(); nextStep(); advance(1000); SystemSafety::stop("test");
    CHECK(count(0xF7) == 3 && Mock::globalStopCalls == 1); const int pulse = Mock::servo[5]; advance(3000);
    for (uint8_t pin : SystemConfig::Mechanism::SERVO_PINS) CHECK(!Mock::servoAttached[pin]);
    CHECK(pulse == Mock::servo[5] && !next() && !home() && !runAll() && !planBegin(1, 1));
  } else if (name == "stop_retry") {
    Mock::rejectNode=6; Mock::rejectCommand=0xF7; SystemSafety::stop("stop retry"); CHECK(count(0xF7) == 2);
    Mock::rejectNode=-1; advance(10); CHECK(count(0xF7) == 3 && Mock::globalStopCalls == 1);
  } else if (name == "ram_all_profiles_retained") {
    upload(1, 101); upload(2, 102);
    CHECK(planBegin(1, 1, 3)); CHECK(planAction(0, branch(delay(0), 1)));
    CHECK(planStep(0, 0, 1)); CHECK(planCommit()); completePending(); upload(4, 104);
    CHECK(status().planVersion == 4);
    for (uint8_t stage = 1; stage <= 4; ++stage) {
      select(stage); CHECK(status().runtimePlan && status().steps == 1); finishStep();
      if (stage != 3) CHECK(status().position6 == 100 + stage);
    }
    upload(1, 501); CHECK(status().planVersion == 5); finishStep(); CHECK(status().position6 == 501);
    select(4); finishStep(); CHECK(status().position6 == 104);
  } else if (name == "ram_transaction_validation") {
    upload(4, 123); CHECK(planBegin(1, 2, 4)); CHECK(planAction(0, branch(stepper(10, 320), 1)));
    CHECK(!planCommit()); planAbort(); CHECK(status().runtimePlan && status().planVersion == 1);
    finishStep(); CHECK(status().position6 == 123);
  } else if (name == "ram_parallel_duplicate") {
    CHECK(planBegin(1, 3, 2));
    CHECK(planAction(0, branch(servo(4, 1800, 2000, false), 1)));
    CHECK(planAction(1, branch(servo(4, 1300, 2000, false), 1)));
    CHECK(planAction(2, branch(waitAll(), 1))); CHECK(planStep(0, 0, 3)); CHECK(!planCommit());
    planAbort(); CHECK(status().stage == 4);
  } else if (name == "ram_parallel_nine_servos") {
    CHECK(planBegin(1, SystemConfig::Mechanism::SERVO_COUNT + 1, 2)); uint16_t i = 0;
    for (uint8_t pin : SystemConfig::Mechanism::SERVO_PINS)
      CHECK(planAction(i++, branch(servo(pin, 1700, 1000, false), 1)));
    CHECK(planAction(i, branch(waitAll(), 1))); CHECK(planStep(0, 0, SystemConfig::Mechanism::SERVO_COUNT + 1)); CHECK(planCommit()); completePending(); nextStep();
    advance(500); CHECK(std::string(status().phase) == "busy" && !next()); advance(500);
    CHECK(std::string(status().phase) == "ready" && status().progress == 1);
    for (uint8_t pin : SystemConfig::Mechanism::SERVO_PINS) CHECK(Mock::servo[pin] == 1700);
    previousStep();
    for (uint8_t j = 0; j < SystemConfig::Mechanism::SERVO_COUNT; ++j)
      CHECK(Mock::servo[SystemConfig::Mechanism::SERVO_PINS[j]] == SystemConfig::Mechanism::SERVO_START_US[j]);
  } else if (name == "ram_timeout") {
    upload(4, 123); CHECK(planBegin(1, 1, 1)); advance(10000);
    CHECK(!status().uploading && status().stage == 4 && !planCommit()); finishStep(); CHECK(status().position6 == 123);
  } else if (name == "ram_commit_from_standby") {
    select(3); finishStep(); upload(1,321);
    CHECK(status().stage == 1 && status().planVersion == 1 && count(0xFE,4) == 2 && count(0xFE,5) == 2 && Mock::servo[3] == 2250);
    CHECK(status().position4 == 0 && status().position5 == 0 && status().completed == 0 && count(0x91) == 0 && count(0xFE,6) == 0);
    finishStep(); CHECK(status().position6 == 321);
  } else if (name == "ram_abort_from_standby") {
    select(3); const size_t sentAt = Mock::sent.size(); CHECK(planBegin(1, 1, 1)); CHECK(planAction(0, branch(stepper(321, 320), 1)));
    CHECK(planStep(0, 0, 1)); planAbort(); CHECK(!planCommit());
    CHECK(status().stage == 3 && status().planVersion == 0 && !status().runtimePlan && Mock::sent.size() == sentAt);
  } else if (name == "home_selected") {
    select(3); finishStep(); CHECK(home()); pump(); CHECK(count(0x91) == 0 && target(4) == 0 && target(5) == 0); completePending();
    select(2); CHECK(home()); pump(); CHECK(status().cylinder.active && count(0x91) == 0); completePending();
    select(1); CHECK(home()); pump(); CHECK(count(0x91,6) == 1); completePending(); CHECK(status().progress == 0 && !previous() && count(0x91,4) == 0 && count(0x91,5) == 0);
  } else if (name == "debug_wired_axes") {
    CHECK(setDebug(true)); CHECK(debugStepper(-123,320,0,false,4) && debugStepper(123,320,0,false,5));
    CHECK(!debugStepper(0,320,0,true,4) && !debugStepper(0,320,0,true,5)); CHECK(debugStepper(-456,320,0,false,6));
    for(uint8_t pin : SystemConfig::Mechanism::SERVO_PINS) CHECK(debugServo(pin,900,1000));
    CHECK(debugCylinder(CylinderAction::Extend,1000)); completePending(); CHECK(status().positionKnownMask == 7 && status().position6 == -456);
    for(uint8_t pin : SystemConfig::Mechanism::SERVO_PINS) CHECK(Mock::servo[pin] == 1500); cylinderReference(CylinderAction::Extend); CHECK(setDebug(false));
  } else if (name == "debug_home_id6") {
    CHECK(setDebug(true)); CHECK(debugStepper(0, 320, 0, true, 6));
    CHECK(count(0x91) == 1 && count(0xFE) == 0); completePending(); CHECK(status().positionKnownMask == 7);
  } else if (name == "clock_wrap") {
    mockMillis = UINT32_MAX - 500; nextStep(); reply(6, 0xFE, 1); advance(600); reply(6, 0xFE, 2); pump();
    CHECK(status().position6 == stage4Motor(1).position && std::string(status().phase) == "ready");
  } else if (name == "run_once") {
    select(2); CHECK(runAll()); pump(); completePending();
    CHECK(status().completed == Stage2::STEP_COUNT && status().progress == Stage2::STEP_COUNT && Mock::servo[6] == stage2Pulse(Stage2::STEP_COUNT,6));
    cylinderReference(stage2Reference(Stage2::STEP_COUNT)); advance(500); CHECK(status().progress == Stage2::STEP_COUNT);
    finishStep(); CHECK(status().completed == Stage2::REPEAT_FROM && Mock::sent.empty());
  } else if (name == "stage2_selection_idempotent") {
    select(2); const size_t writes=Mock::gpioWrites.size(); CHECK(selectStage(2)); pump(); CHECK(Mock::gpioWrites.size() == writes && status().completed == 0);
    nextStep(); advance(50); CHECK(selectStage(4)); CHECK(!status().cylinder.active && !status().cylinder.in1High && !status().cylinder.in2High); CHECK(selectStage(2));
    completePending(); CHECK(status().stage == 2 && status().completed == 0 && status().progress == 0 && Mock::sent.empty()); cylinderReference(CylinderAction::Retract);
  } else if (name == "stage2_reentry") {
    select(2); finishStep(); select(4); cylinderReference(CylinderAction::Retract); CHECK(!status().cylinder.active);
    CHECK(selectStage(2)); pump(); CHECK(status().stage == 2 && std::string(status().phase) == "ready" && !status().cylinder.active); completePending();
    CHECK(status().completed == 0 && !previous() && Mock::servo[9] == 1100 && Mock::sent.empty()); cylinderReference(CylinderAction::Retract);
  } else if (name == "stage2_auto_entry_stop") {
    CHECK(selectStage(2)); pump(); advance(50); CHECK(status().cylinder.in2High); SystemSafety::stop("entry stop");
    CHECK(!status().cylinder.active && !status().cylinder.in1High && !status().cylinder.in2High && !status().cylinder.referenceKnown); advance(30000);
    CHECK(std::string(status().phase) == "fault" && count(0xF7) == 3 && !next() && !selectStage(4));
  } else if (name == "ram_stage2_auto_activation") {
    CHECK(planBegin(1,1,2)); CHECK(planAction(0,branch(servo(4,1800),1))); CHECK(planStep(0,0,1)); CHECK(planCommit()); pump();
    CHECK(status().stage == 2 && std::string(status().phase) == "home_wait" && status().runtimePlan && status().planVersion == 1 && !next()); completePending();
    finishStep(); CHECK(Mock::servo[4] == 1800); CHECK(planBegin(1,1,2)); CHECK(planAction(0,branch(servo(9,1700),1))); CHECK(planStep(0,0,1)); CHECK(planCommit()); completePending();
    CHECK(status().completed == 0 && status().planVersion == 2 && Mock::servo[4] == 2000); finishStep(); CHECK(Mock::servo[9] == 1700 && Mock::sent.empty());
  } else if (name == "ram_parallel_thirteen_axes") {
    constexpr uint16_t total=SystemConfig::Mechanism::MOTOR_COUNT+SystemConfig::Mechanism::SERVO_COUNT+2;
    CHECK(planBegin(1,total,4)); uint16_t i=0; for(uint8_t node : SystemConfig::Mechanism::MOTOR_IDS) CHECK(planAction(i++,branch(stepperFor(node,-1234,320,0,false),1)));
    for(uint8_t pin : SystemConfig::Mechanism::SERVO_PINS) CHECK(planAction(i++,branch(servo(pin,1700,1000,false),1)));
    CHECK(planAction(i++,branch(cylinder(CylinderAction::Extend,1000,false),1))); CHECK(planAction(i,branch(waitAll(),1))); CHECK(planStep(0,0,total)); CHECK(planCommit()); completePending(); nextStep();
    finish(4); finish(5); finish(6); advance(50); advance(999); CHECK(!next() && status().completed == 0); advance(1); CHECK(status().completed == 1); cylinderReference(CylinderAction::Extend);
    ackCursor=Mock::sent.size(); previousStep(); CHECK(status().position4 == 0 && status().position5 == 0 && status().position6 == 0 && !status().cylinder.referenceKnown);
    for(uint8_t j=0;j<SystemConfig::Mechanism::SERVO_COUNT;++j) CHECK(Mock::servo[SystemConfig::Mechanism::SERVO_PINS[j]] == SystemConfig::Mechanism::SERVO_START_US[j]);
  } else if (name == "ram_parallel_high_bit_duplicate") {
    CHECK(planBegin(1,3,4)); CHECK(planAction(0,branch(servo(12,1700,1000,false),1)));
    CHECK(planAction(1,branch(servo(12,1800,1000,false),1))); CHECK(planAction(2,branch(waitAll(),1)));
    CHECK(planStep(0,0,3)); CHECK(!planCommit()); planAbort(); CHECK(status().planVersion == 0);
  } else if (name == "ram_parallel_high_bit_missing_barrier") {
    CHECK(planBegin(1,1,4)); CHECK(planAction(0,branch(servo(12,1700,1000,false),1)));
    CHECK(planStep(0,0,1)); CHECK(!planCommit()); planAbort(); CHECK(status().planVersion == 0);
  } else if (name == "ram_stage3_commit_waits_for_park") {
    select(3); finishStep(); CHECK(planBegin(1,1,4)); CHECK(planAction(0,branch(stepper(123,320),1))); CHECK(planStep(0,0,1)); CHECK(planCommit()); pump();
    CHECK(status().stage == 3 && status().planVersion == 0 && !next()); finish(4); CHECK(status().stage == 3); finish(5);
    CHECK(status().stage == 4 && status().planVersion == 1 && status().runtimePlan && status().completed == 0 && count(0x91) == 0); ackCursor=Mock::sent.size(); finishStep(); CHECK(status().position6 == 123);
  } else if (name == "ram_stage3_commit_abort_during_park") {
    select(3); finishStep(); CHECK(planBegin(1,1,4)); CHECK(planAction(0,branch(stepper(123,320),1)));
    CHECK(planStep(0,0,1)); CHECK(planCommit()); pump(); planAbort(); CHECK(status().pendingStage == 3 && std::string(status().phase) == "return_wait"); finishPair();
    CHECK(status().stage == 3 && status().planVersion == 0 && !status().runtimePlan && std::string(status().phase) == "ready");
    CHECK(status().position4 == 0 && status().position5 == 0 && status().progress == 0 && count(0x91) == 0 && count(0xFE,6) == 0 && count(0xFE,4) == 2 && count(0xFE,5) == 2);
  } else if (name == "current_settings_not_reapplied_by_actions") {
    finishStep(); CHECK(home()); pump(); completePending(); CHECK(setDebug(true)); CHECK(debugStepper(0,320,0,true,6)); completePending(); CHECK(setDebug(false));
    upload(4,-123); finishStep(); CHECK(count(0x83) == 0 && status().currentConfiguredMask == 7 && status().current4Ma == 3000 && status().current5Ma == 3000 && status().current6Ma == 5200);
  } else if (name == "debug_removed_id7_rejected") {
    CHECK(setDebug(true)); CHECK(!debugStepper(-123,320,0,false,7) && !debugStepper(0,320,0,true,7));
    CHECK(count(0xFE) == 0 && count(0x91) == 0 && status().positionKnownMask == 7); CHECK(setDebug(false));
  } else if (name == "stage_home_all_destinations") {
    for(uint8_t stage : {uint8_t(1),uint8_t(2),uint8_t(3),uint8_t(4),uint8_t(5)}) {
      CHECK(selectStage(stage)); pump(); completePending(); CHECK(status().stage == stage && status().completed == 0 && status().pendingStage == 0 && !previous());
      if(stage == 2) cylinderReference(CylinderAction::Retract); CHECK(count(0x91) == 0);
    }
  } else if (name == "stage_home_latest_selection") {
    select(2); nextStep(); advance(50); CHECK(selectStage(4)); CHECK(selectStage(1)); CHECK(selectStage(3));
    CHECK(status().pendingStage == 3 && !status().cylinder.in1High && !status().cylinder.in2High); completePending();
    CHECK(status().stage == 3 && status().completed == 0 && count(0xFE) == 0 && count(0x91) == 0); cylinderReference(CylinderAction::Retract);
  } else if (name == "stage_home3_reentry_pair") {
    select(3); finishStep(); select(2); CHECK(status().position4 == 0 && status().position5 == 0 && Mock::servo[3] == 2250); finishStep();
    select(3); CHECK(status().position4 == 0 && status().position5 == 0 && count(0x91) == 0); finishStep(); previousStep(); CHECK(status().position4 == 0 && status().position5 == 0 && !previous());
  } else if (name == "stage_home3_pending_requires_park") {
    select(3); finishStep(); CHECK(home()); pump(); const int f4 = count(0xFE,4), f5 = count(0xFE,5);
    CHECK(std::string(status().phase) == "home_wait" && target(4) == 0 && target(5) == 0); CHECK(selectStage(2)); CHECK(selectStage(4));
    finish(4); CHECK(status().stage == 3 && std::string(status().phase) == "home_wait" && count(0xFE,4) == f4 && count(0xFE,5) == f5); finish(5);
    // HOME3 already established both FE-zero targets; Park0 sends no redundant move.
    CHECK(status().stage == 4 && std::string(status().phase) == "ready" && status().progress == 0 && status().pendingStage == 0);
    CHECK(count(0xFE,4) == f4 && count(0xFE,5) == f5 && count(0xFE,6) == 0 && count(0xFE,7) == 0 && count(0x91) == 0);
    CHECK(status().position4 == 0 && status().position5 == 0 && Mock::servo[3] == 2250 && count(0x91) == 0);
  } else if (name == "stage_home_failure_discards_pending") {
    finishStep(); CHECK(selectStage(1)); pump(); reply(6,0xFE,0); pump();
    CHECK(std::string(status().phase) == "fault" && status().pendingStage == 0 && count(0xF7) == 3 && !status().cylinder.active && count(0x91) == 0);
  } else if (name == "stage_home_queue_timeout") {
    finishStep(); Mock::rejectNode=6; Mock::rejectCommand=0xFE; CHECK(selectStage(1)); pump(); advance(500);
    CHECK(std::string(status().phase) == "fault" && count(0xFE) == 1 && count(0xF7) == 3 && count(0x91) == 0);
  } else if (name == "ram_commit_home_barrier") {
    finishStep(); CHECK(planBegin(1,1,2)); CHECK(planAction(0,branch(servo(4,1800),1))); CHECK(planStep(0,0,1)); CHECK(planCommit()); pump();
    CHECK(status().stage == 4 && status().planVersion == 0 && target() == 0 && !next()); finish(6);
    CHECK(status().stage == 2 && status().planVersion == 1 && std::string(status().phase) == "home_wait" && !next()); ackCursor=Mock::sent.size(); completePending();
    CHECK(status().completed == 0 && status().runtimePlan); finishStep(); CHECK(Mock::servo[4] == 1800);
  } else if (name == "stage_home_step0_immediate") {
    select(2); finishStep(); CHECK(selectStage(1)); pump(); CHECK(status().stage == 1 && status().completed == 0 && status().progress == 0 && !previous() && !status().cylinder.active);
    completePending(); CHECK(status().stage == 1 && status().completed == 0); finishStep(); previousStep(); CHECK(status().position6 == 0 && count(0x91) == 0);
  } else if (name == "stage_home_selection_cancels_pending") {
    select(2); nextStep(); advance(50); CHECK(selectStage(4)); CHECK(selectStage(2)); completePending();
    CHECK(status().stage == 2 && status().completed == 0 && status().progress == 0 && !previous()); cylinderReference(CylinderAction::Retract);
  } else if (name == "stage_return_busy_latest") {
    nextStep(); const int sent = count(0xFE,6); CHECK(selectStage(2)); CHECK(selectStage(1)); CHECK(selectStage(3));
    CHECK(status().stage == 4 && status().pendingStage == 3 && std::string(status().phase) == "busy" && count(0xFE,6) == sent);
    finish(6); CHECK(status().stage == 4 && status().pendingStage == 3 && std::string(status().phase) == "return_wait" && target(6) == 0 && count(0xFE,6) == sent + 1);
    finish(6); CHECK(status().stage == 3 && status().progress == 0 && count(0xFE,4) == 0 && count(0xFE,5) == 0 && count(0xFE,7) == 0 && count(0x91) == 0);
  } else if (name == "stage_return_manual_home_pending") {
    select(2); CHECK(home()); pump(); advance(50); CHECK(selectStage(4)); CHECK(selectStage(1)); CHECK(!status().cylinder.in2High);
    completePending(); CHECK(status().stage == 1 && status().progress == 0 && count(0xFE) == 0 && count(0x91) == 0); cylinderReference(CylinderAction::Retract);
  } else if (name == "ram_cross_stage_actuator_return") {
    CHECK(planBegin(1,3,4)); CHECK(planAction(0,branch(cylinder(CylinderAction::Extend,100,false),1))); CHECK(planAction(1,branch(servo(9,1600,0,false),1)));
    CHECK(planAction(2,branch(waitAll(),1))); CHECK(planStep(0,0,3)); CHECK(planCommit()); completePending(); finishStep(); cylinderReference(CylinderAction::Extend);
    CHECK(selectStage(1)); pump(); CHECK(status().stage == 4 && status().pendingStage == 1 && std::string(status().phase) == "return_wait" && status().cylinder.active && Mock::servo[9] == 1100);
    completePending(); CHECK(status().stage == 1 && status().completed == 0 && count(0xFE) == 0 && count(0x91) == 0); cylinderReference(CylinderAction::Retract);
  } else if (name == "ram_same_stage_return_all_used_actuators") {
    constexpr uint16_t total=SystemConfig::Mechanism::MOTOR_COUNT+SystemConfig::Mechanism::SERVO_COUNT+2;
    CHECK(planBegin(1,total,4)); uint16_t i=0; for(uint8_t node : SystemConfig::Mechanism::MOTOR_IDS) CHECK(planAction(i++,branch(stepperFor(node,-1234,320,0,false),1)));
    for(uint8_t pin : SystemConfig::Mechanism::SERVO_PINS) CHECK(planAction(i++,branch(servo(pin,1700,0,false),1)));
    CHECK(planAction(i++,branch(cylinder(CylinderAction::Extend,100,false),1))); CHECK(planAction(i,branch(waitAll(),1))); CHECK(planStep(0,0,total)); CHECK(planCommit()); completePending(); finishStep();
    CHECK(planBegin(1,1,4)); CHECK(planAction(0,branch(servo(9,1600),1))); CHECK(planStep(0,0,1)); CHECK(planCommit()); pump();
    CHECK(status().planVersion == 1 && status().cylinder.active); for(uint8_t node : SystemConfig::Mechanism::MOTOR_IDS) CHECK(target(node) == 0);
    finish(4); finish(5); finish(6); CHECK(status().planVersion == 1 && !next()); ackCursor=Mock::sent.size(); completePending();
    CHECK(status().planVersion == 2 && status().completed == 0); cylinderReference(CylinderAction::Retract); finishStep(); CHECK(Mock::servo[9] == 1600);
  } else if (name == "ram_abort_return_preserves_old_bank") {
    upload(4,321); finishStep(); CHECK(planBegin(1,1,1)); CHECK(planAction(0,branch(stepper(123,320),1))); CHECK(planStep(0,0,1)); CHECK(planCommit()); pump();
    CHECK(status().stage == 4 && status().pendingStage == 1 && status().planVersion == 1 && std::string(status().phase) == "return_wait"); planAbort();
    CHECK(status().pendingStage == 4 && status().planVersion == 1 && target(6) == 0); finish(6);
    CHECK(status().stage == 4 && status().runtimePlan && status().planVersion == 1 && status().progress == 0 && std::string(status().phase) == "ready");
    finishStep(); CHECK(status().position6 == 321 && count(0x91) == 0);
  } else if (name == "prop_basket_exact" || name == "prop_vase_exact" || name == "prop_tube_exact" ||
             name == "prop_chicken_exact" || name == "prop_bird_exact") {
    const uint8_t prop=name == "prop_basket_exact" ? 1 : name == "prop_vase_exact" ? 2 : name == "prop_tube_exact" ? 3 : name == "prop_chicken_exact" ? 4 : 5;
    const auto first=propMotor(prop,1);
    select(1); CHECK(count(0xFE) == 0 && count(0x91) == 0);
    CHECK(triggerProp(prop)); pump(); CHECK(status().prop == prop && status().steps == propStepCount(prop));
    CHECK(target() == first.position && speed() == first.rpm && lastMove().buf[3] == first.acc);
    CHECK(Mock::servo[5] == propPulse(prop,1,5) && Mock::servo[2] == propPulse(prop,1,2));
    CHECK(!next() && !triggerProp(1)); completePending();
    CHECK(status().completed == 1 && !status().propComplete);
    if (prop == 1) {
      const auto lift=propMotor(prop,2); const int moves=count(0xFE);
      nextStep(); CHECK(target() == first.position && count(0xFE) == moves && !next() && !previous());
      advance(999); CHECK(count(0xFE) == moves && Mock::servo[2] != propPulse(prop,2,2));
      advance(1); CHECK(target() == lift.position && speed() == lift.rpm && lastMove().buf[3] == lift.acc);
      CHECK(Mock::servo[2] == propPulse(prop,2,2) && Mock::servo[5] == propPulse(prop,2,5));
      completePending(); CHECK(status().position6 == lift.position && status().completed == 2 && !status().propComplete);
      const int lifted=count(0xFE); nextStep(); CHECK(!next() && !previous());
      advance(999); CHECK(!status().propComplete && count(0xFE) == lifted);
      advance(1); completePending(); CHECK(Mock::servo[2] == propPulse(prop,3,2) && count(0xFE) == lifted);
    } else if (prop == 2) { finishStep(); CHECK(Mock::servo[2] == 2111); nextStep(); CHECK(target() == propMotor(2,3).position && speed() == 1000 && lastMove().buf[3] == 245); completePending(); CHECK(Mock::servo[2] == 1389); }
    else if (prop == 3) { nextStep(); CHECK(target() == 0 && speed() == 1000 && lastMove().buf[3] == 255); completePending(); }
    else {
      const int grip=propPulse(prop,2,2); const auto lift=propMotor(prop,3);
      finishStep(); CHECK(status().completed == 2 && !status().propComplete && Mock::servo[2] == grip);
      nextStep(); CHECK(target() == lift.position && speed() == lift.rpm && lastMove().buf[3] == lift.acc);
      CHECK(Mock::servo[2] == grip && !next() && !previous()); completePending();
      CHECK(status().completed == 3 && !status().propComplete && status().position6 == lift.position && Mock::servo[2] == grip);
      const int moves=count(0xFE); finishStep();
      CHECK(status().position6 == lift.position && Mock::servo[2] == propPulse(prop,4,2) && count(0xFE) == moves);
    }
    CHECK(status().completed == propStepCount(prop) && status().propComplete && status().progress == propStepCount(prop));
    const size_t frames=Mock::sent.size(); CHECK(!next()); pump(); CHECK(Mock::sent.size() == frames);
    CHECK(count(0x91) == 0 && count(0xFE,4) == 0 && count(0xFE,5) == 0 && count(0xFE,7) == 0);
  } else if (name == "prop_chicken_previous_release_and_lift" || name == "prop_bird_previous_release_and_lift") {
    const uint8_t prop=name == "prop_chicken_previous_release_and_lift" ? 4 : 5;
    const int grip=propPulse(prop,2,2); const int32_t initial=propMotor(prop,1).position,lift=propMotor(prop,3).position;
    select(1); CHECK(triggerProp(prop)); pump(); completePending();
    finishStep(); finishStep(); CHECK(status().completed == 3 && status().position6 == lift && Mock::servo[2] == grip);
    finishStep(); CHECK(status().propComplete && Mock::servo[2] == propPulse(prop,4,2));
    const int liftedMoves=count(0xFE);
    previousStep(); CHECK(status().completed == 3 && status().position6 == lift && Mock::servo[2] == grip && !status().propComplete);
    CHECK(count(0xFE) == liftedMoves); // Undo release closes the claw without lowering it.
    previousStep(); CHECK(status().completed == 2 && status().position6 == initial && Mock::servo[2] == grip);
    CHECK(count(0xFE) == liftedMoves+1);
    previousStep(); CHECK(status().completed == 1 && status().position6 == initial && Mock::servo[2] == propPulse(prop,1,2));
    previousStep(); CHECK(status().completed == 0 && status().position6 == 0 && Mock::servo[2] == 1150 && Mock::servo[5] == 2100);
    CHECK(!previous() && count(0x91) == 0);
  } else if (name == "prop_trigger_requires_stage1_ready") {
    CHECK(!triggerProp(1) && !triggerProp(5)); select(1);
    CHECK(!triggerProp(0) && !triggerProp(6)); CHECK(setDebug(true)); CHECK(!triggerProp(1)); CHECK(setDebug(false));
    CHECK(triggerProp(5)); pump(); CHECK(!triggerProp(1) && !next() && status().prop == 5); completePending();
    CHECK(status().completed == 1 && status().prop == 5 && !status().propComplete);
    finishStep(); CHECK(status().completed == 2 && !status().propComplete);
    finishStep(); CHECK(status().completed == 3 && !status().propComplete && Mock::servo[2] == propPulse(5,2,2));
    finishStep(); CHECK(status().completed == 4 && status().propComplete && Mock::servo[2] == propPulse(5,4,2));
    CHECK(triggerProp(1)); pump(); completePending(); CHECK(status().prop == 1 && status().completed == 1 && !status().propComplete);
    while (!status().propComplete) finishStep();
    CHECK(status().completed == propStepCount(1) && status().position6 == propMotor(1,propStepCount(1)).position);
    CHECK(count(0x91) == 0);
  } else if (name == "prop_previous_uses_actual_entry") {
    select(1); CHECK(triggerProp(1)); pump(); completePending(); while (!status().propComplete) finishStep();
    const auto entry=status();
    CHECK(entry.position6 == propMotor(1,propStepCount(1)).position && Mock::servo[5] == propPulse(1,propStepCount(1),5) && Mock::servo[2] == propPulse(1,propStepCount(1),2));
    CHECK(triggerProp(4)); pump(); completePending(); CHECK(status().position6 == propMotor(4,1).position && Mock::servo[5] == propPulse(4,1,5));
    previousStep(); CHECK(status().prop == 4 && status().progress == 0 && status().position6 == entry.position6 && Mock::servo[5] == propPulse(1,propStepCount(1),5) && Mock::servo[2] == propPulse(1,propStepCount(1),2));
    CHECK(!previous() && !status().propComplete && count(0x91) == 0);
  } else if (name == "prop_same_retrigger_one_shot") {
    select(1); CHECK(triggerProp(1)); pump(); completePending(); while (!status().propComplete) finishStep();
    const auto entry=status(); CHECK(!next() && entry.position6 == propMotor(1,propStepCount(1)).position);
    const int sent=count(0xFE); CHECK(triggerProp(1)); pump(); completePending();
    CHECK(status().completed == 1 && status().progress == 1 && !status().propComplete && count(0xFE) == sent+1 && status().position6 == propMotor(1,1).position);
    finishStep(); CHECK(status().completed == 2 && !status().propComplete && count(0xFE) == sent+2 && status().position6 == propMotor(1,2).position);
    finishStep(); CHECK(status().completed == 3 && status().propComplete && count(0xFE) == sent+2 && Mock::servo[2] == propPulse(1,3,2));
    previousStep(); CHECK(status().completed == 2 && status().position6 == propMotor(1,2).position && Mock::servo[2] == propPulse(1,2,2) && !status().propComplete);
    previousStep(); CHECK(status().completed == 1 && status().position6 == propMotor(1,1).position && Mock::servo[2] == propPulse(1,1,2) && Mock::servo[5] == propPulse(1,1,5));
    previousStep(); CHECK(status().completed == 0 && status().position6 == entry.position6 && Mock::servo[2] == propPulse(1,propStepCount(1),2) && !status().previousAvailable);
  } else if (name == "prop_selection_no_auto_trigger") {
    selectProp(3); CHECK(status().steps == propStepCount(3) && count(0xFE) == 0 && count(0x91) == 0);
    selectProp(4); CHECK(status().steps == propStepCount(4) && status().completed == 0 && count(0xFE) == 0 && count(0x91) == 0);
    CHECK(selectStage(1)); pump(); completePending(); CHECK(status().prop == 2 && status().steps == 3 && status().completed == 0 && count(0xFE) == 0);
  } else if (name == "prop_selection_busy_latest") {
    select(1); CHECK(triggerProp(3)); pump(); CHECK(selectProfile(1,5)); CHECK(selectProfile(1,4));
    CHECK(status().stage == 1 && status().prop == 3 && status().pendingStage == 1 && status().pendingProp == 4 && std::string(status().phase) == "busy");
    finish(6); CHECK(status().prop == 3 && status().pendingProp == 4 && std::string(status().phase) == "return_wait" && target() == 0);
    finish(6); CHECK(status().prop == 4 && status().pendingProp == 0 && status().completed == 0 && !status().propComplete && count(0x91) == 0);
  } else if (name == "prop_selection_cancel_before_return") {
    select(1); CHECK(triggerProp(4)); pump(); CHECK(selectProfile(1,5)); CHECK(selectProfile(1,4));
    CHECK(status().pendingStage == 0 && status().pendingProp == 0); completePending();
    CHECK(status().prop == 4 && status().completed == 1 && status().position6 == propMotor(4,1).position && count(0xFE) == 1 && count(0x91) == 0);
  } else if (name == "profile_invalid_stage_and_prop") {
    CHECK(!selectProfile(0) && !selectProfile(6) && !selectProfile(1,6));
    for (uint8_t stage : {uint8_t(2),uint8_t(3),uint8_t(4),uint8_t(5)}) { CHECK(!selectProfile(stage,1)); CHECK(!planBegin(1,1,stage,1,1)); }
    CHECK(!planBegin(1,1,1,1,6) && !planBegin(1,1,6)); CHECK(!exitStage1(1) && !exitStage1(6));
    CHECK(status().stage == 4 && status().prop == 0 && count(0xFE) == 0 && count(0x91) == 0);
  } else if (name == "exit_stage1_ready_home_barrier") {
    select(1); CHECK(exitStage1()); pump(); CHECK(status().stage == 1 && status().pendingStage == 4 && status().prop == 2 && std::string(status().phase) == "home_wait");
    CHECK(count(0x91,6) == 1 && count(0xFE) == 0 && Mock::servo[2] == 1150 && Mock::servo[5] == 2100 && !next() && !triggerProp(1));
    reply(6,0x91,2); pump(); CHECK(status().stage == 1); reply(6,0x91,1); advance(500);
    CHECK(status().stage == 1 && count(0x91) == 1); reply(6,0x91,2); pump();
    CHECK(status().stage == 4 && status().prop == 0 && status().pendingStage == 0 && status().pendingProp == 0 && std::string(status().phase) == "ready" && status().completed == 0 && !previous());
  } else if (name == "exit_stage1_busy_waits_complete_step") {
    select(1); CHECK(triggerProp(2)); pump(); CHECK(exitStage1()); CHECK(status().pendingStage == 4 && std::string(status().phase) == "busy" && count(0x91) == 0);
    finish(6); CHECK(std::string(status().phase) == "home_wait" && count(0x91,6) == 1 && status().stage == 1 && Mock::servo[2] == 1150 && Mock::servo[5] == 2100);
    finish(6,0x91); CHECK(status().stage == 4 && status().position6 == 0 && count(0xFE) == 1);
  } else if (name == "exit_stage1_run_stops_after_current_step") {
    select(1); CHECK(runAll()); pump(); CHECK(exitStage1()); finish(6);
    CHECK(std::string(status().phase) == "home_wait" && status().stage == 1 && count(0xFE) == 1 && count(0x91) == 1 && Mock::servo[2] == 1150);
    finish(6,0x91); CHECK(status().stage == 4 && count(0xFE) == 1 && Mock::servo[2] == 1150);
  } else if (name == "exit_stage1_latest_normal_target") {
    select(1); CHECK(exitStage1()); pump(); CHECK(selectProfile(2)); CHECK(selectProfile(5)); CHECK(exitStage1(5));
    CHECK(status().pendingStage == 5 && std::string(status().phase) == "home_wait" && count(0x91) == 1 && count(0xFE) == 0);
    finish(6,0x91); CHECK(status().stage == 5 && status().prop == 0 && status().completed == 0 && count(0xFE) == 0);
  } else if (name == "exit_stage1_low_after_fe_accepted") {
    select(1); CHECK(triggerProp(3)); pump(); completePending(); CHECK(selectStage(2)); pump();
    CHECK(std::string(status().phase) == "return_wait" && target() == 0 && count(0x91) == 0);
    CHECK(exitStage1()); CHECK(std::string(status().phase) == "home_wait" && status().pendingStage == 4 && count(0x91) == 0);
    finish(6); CHECK(count(0x91,6) == 1 && status().stage == 1 && std::string(status().phase) == "home_wait");
    finish(6,0x91); CHECK(status().stage == 4 && count(0x91) == 1 && count(0xFE) == 2);
  } else if (name == "exit_stage1_low_before_fe_enqueued") {
    select(1); CHECK(triggerProp(1)); pump(); completePending(); CHECK(selectStage(2));
    CHECK(exitStage1()); pump(); CHECK(count(0xFE) == 1 && count(0x91,6) == 1 && status().pendingStage == 4);
    finish(6,0x91); CHECK(status().stage == 4 && count(0x91) == 1 && count(0xFE) == 1);
  } else if (name == "exit_stage1_inflight_manual_home_not_resent") {
    select(1); CHECK(triggerProp(1)); pump(); completePending(); CHECK(home()); pump(); CHECK(count(0x91) == 1);
    CHECK(exitStage1()); CHECK(selectProfile(2)); finish(6,0x91); completePending();
    CHECK(status().stage == 2 && std::string(status().phase) == "ready" && count(0x91) == 1 && count(0xFE) == 1 && status().completed == 0);
  } else if (name == "exit_stage1_queued_manual_home_not_resent") {
    select(1); CHECK(home()); CHECK(count(0x91) == 0); CHECK(exitStage1()); pump();
    CHECK(count(0x91) == 1 && status().stage == 1 && std::string(status().phase) == "home_wait");
    finish(6,0x91); CHECK(status().stage == 4 && std::string(status().phase) == "ready" && count(0x91) == 1 && count(0xFE) == 0);
  } else if (name == "exit_stage1_cancels_not_activated_high") {
    nextStep(); CHECK(selectStage(1)); CHECK(status().pendingStage == 1 && status().pendingProp == 2);
    CHECK(exitStage1()); CHECK(status().pendingStage == 0 && status().pendingProp == 0); completePending();
    CHECK(status().stage == 4 && status().completed == 1 && count(0x91) == 0 && count(0xFE) == 1);
  } else if (name == "exit_stage1_rejection_discards_pending") {
    select(1); CHECK(exitStage1()); pump(); reply(6,0x91,0);
    CHECK(std::string(status().phase) == "fault" && status().stage == 1 && status().pendingStage == 0 && status().pendingProp == 0 && count(0xF7) == 3 && !selectProfile(4));
    advance(30000); CHECK(count(0x91) == 1 && count(0xFE) == 0);
  } else if (name == "exit_stage1_home_start_timeout" || name == "exit_stage1_home_completion_timeout") {
    select(1); CHECK(exitStage1()); pump();
    if (name == "exit_stage1_home_completion_timeout") { reply(6,0x91,1); advance(SystemConfig::Mechanism::FINISH_TIMEOUT_MS); }
    else advance(SystemConfig::Mechanism::START_TIMEOUT_MS);
    CHECK(std::string(status().phase) == "fault" && count(0x91) == 1 && count(0xF7) == 3 && status().pendingStage == 0 && status().pendingProp == 0);
    CHECK(std::string(status().notice).find("ID 6 0x91") != std::string::npos);
  } else if (name == "exit_stage1_home_enqueue_timeout") {
    select(1); Mock::rejectNode=6; Mock::rejectCommand=0x91; CHECK(exitStage1()); pump();
    CHECK(std::string(status().phase) == "home_wait" && count(0x91) == 0); advance(SystemConfig::Mechanism::TX_TIMEOUT_MS);
    CHECK(std::string(status().phase) == "fault" && count(0x91) == 0 && count(0xF7) == 3 && status().pendingStage == 0 && status().pendingProp == 0);
  } else if (name == "exit_stage1_stop_cancels_homing") {
    select(1); CHECK(exitStage1()); pump(); SystemSafety::stop("exit STOP"); advance(30000); finish(6,0x91);
    CHECK(std::string(status().phase) == "fault" && count(0x91) == 1 && count(0xF7) == 3 && status().pendingStage == 0 && !triggerProp(1) && !exitStage1());
    for (uint8_t pin : SystemConfig::Mechanism::SERVO_PINS) CHECK(!Mock::servoAttached[pin]);
  } else if (name == "exit_stage1_ram_other_axes_fe_only") {
    uploadProfile(1,1,-234,4); CHECK(triggerProp(1)); pump(); completePending(); CHECK(status().position4 == -234);
    CHECK(exitStage1()); pump(); CHECK(count(0x91,6) == 1 && count(0x91,4) == 0 && target(4) == 0);
    finish(6,0x91); CHECK(status().stage == 1 && !next()); finish(4); CHECK(status().stage == 4 && status().position4 == 0 && count(0x91,5) == 0);
  } else if (name == "ram_nine_profiles_retained_and_replace") {
    for (uint8_t key=0;key<9;++key) {
      const uint8_t stage=static_cast<uint8_t>(key<5 ? 1 : key-3);
      const uint8_t prop=static_cast<uint8_t>(key<5 ? key+1 : 0);
      const uint8_t node=stage == 3 ? 4 : 6;
      uploadProfile(stage,prop,200+key,node);
    }
    CHECK(status().planVersion == 9);
    for (uint8_t key=0;key<9;++key) {
      const uint8_t stage=static_cast<uint8_t>(key<5 ? 1 : key-3);
      const uint8_t prop=static_cast<uint8_t>(key<5 ? key+1 : 0);
      const uint8_t node=stage == 3 ? 4 : 6;
      CHECK(selectProfile(stage,prop)); pump(); completePending(); CHECK(status().runtimePlan && status().steps == 1);
      finishStep(); CHECK(target(node) == 200+key && status().prop == prop);
      if (stage == 1) CHECK(status().propComplete && !next());
    }
    uploadProfile(1,3,903); CHECK(status().planVersion == 10); finishStep(); CHECK(status().position6 == 903);
    for (uint8_t prop=1;prop<=5;++prop) { selectProp(prop); finishStep(); CHECK(status().position6 == (prop == 3 ? 903 : 199+prop)); }
    CHECK(count(0x91) == 0);
  } else if (name == "ram_prop_commit_isolated_and_entry_only") {
    uploadProfile(1,1,101); CHECK(status().completed == 0 && status().position6 == 0 && !status().propComplete && count(0xFE) == 0);
    CHECK(triggerProp(1)); pump(); completePending(); CHECK(status().position6 == 101 && status().propComplete);
    CHECK(planBegin(1,1,1,1,2)); CHECK(planAction(0,branch(stepper(202,320),1))); CHECK(planStep(0,0,1)); CHECK(planCommit()); pump();
    CHECK(status().prop == 1 && status().pendingProp == 2 && status().planVersion == 1 && std::string(status().phase) == "return_wait" && target() == 0);
    CHECK(!selectProfile(1,3) && !triggerProp(3) && !exitStage1()); finish(6); CHECK(status().prop == 2 && status().planVersion == 2 && status().completed == 0 && status().position6 == 0);
    ackCursor=Mock::sent.size(); CHECK(triggerProp(2)); pump(); completePending(); CHECK(status().position6 == 202);
    selectProp(1); finishStep(); CHECK(status().position6 == 101 && status().planVersion == 2 && count(0x91) == 0);
  } else if (name == "ram_prop_abort_keeps_old_bank") {
    uploadProfile(1,1,101); finishStep(); CHECK(planBegin(1,1,1,1,2)); CHECK(planAction(0,branch(stepper(202,320),1))); CHECK(planStep(0,0,1)); CHECK(planCommit()); pump();
    planAbort(); CHECK(status().pendingStage == 1 && status().pendingProp == 1 && status().planVersion == 1); finish(6);
    CHECK(status().prop == 1 && status().runtimePlan && status().completed == 0 && status().planVersion == 1);
    ackCursor=Mock::sent.size(); finishStep(); CHECK(status().position6 == 101);
    selectProp(2); CHECK(!status().runtimePlan && status().steps == propStepCount(2) && count(0x91) == 0);
  } else if (name == "ram_prop_terminal_ignores_repeat_setting") {
    CHECK(planBegin(2,2,1,2,5)); CHECK(planAction(0,branch(stepper(100,320),1))); CHECK(planAction(1,branch(stepper(200,320),1)));
    CHECK(planStep(0,0,1)); CHECK(planStep(1,1,1)); CHECK(planCommit()); completePending();
    CHECK(triggerProp(5)); pump(); completePending(); CHECK(status().position6 == 100); finishStep();
    CHECK(status().position6 == 200 && status().propComplete && !next()); previousStep(); CHECK(status().position6 == 100 && !status().propComplete);
    finishStep(); CHECK(status().position6 == 200 && status().progress == 2 && !next());
  } else if (name == "ram_prop0_defaults_vase") {
    upload(1,222); CHECK(status().prop == 2 && status().runtimePlan && status().completed == 0);
    finishStep(); CHECK(status().position6 == 222 && status().propComplete && !next()); selectProp(1); CHECK(!status().runtimePlan && status().steps == propStepCount(1));
    selectProp(2); CHECK(status().runtimePlan && status().steps == 1); finishStep(); CHECK(status().position6 == 222);
  } else if (name == "stage5_placeholder_no_hardware") {
    select(5); const size_t frames=Mock::sent.size(),writes=Mock::servoWrites.size();
    finishStep(); CHECK(status().completed == 1 && status().prop == 0 && !status().propComplete); finishStep(); CHECK(status().progress == 2);
    CHECK(runAll()); pump(); completePending(); CHECK(status().progress == 1); CHECK(home()); pump();
    CHECK(std::string(status().phase) == "ready" && status().completed == 0 && !previous());
    CHECK(Mock::sent.size() == frames && Mock::servoWrites.size() == writes); select(4);
    CHECK(Mock::sent.size() == frames && Mock::servoWrites.size() == writes && count(0x91) == 0);
  } else if (name == "stage5_custom_home_no_motion_return_used_axes") {
    CHECK(planBegin(1,4,5)); CHECK(planAction(0,branch(stepperFor(4,123,320,0,false),1))); CHECK(planAction(1,branch(stepperFor(6,-234,320,0,false),1)));
    CHECK(planAction(2,branch(servo(9,1700,0,false),1))); CHECK(planAction(3,branch(waitAll(),1))); CHECK(planStep(0,0,4)); CHECK(planCommit()); completePending(); finishStep();
    const size_t frames=Mock::sent.size(),writes=Mock::servoWrites.size(); CHECK(home()); pump(); CHECK(status().completed == 0 && Mock::sent.size() == frames && Mock::servoWrites.size() == writes);
    CHECK(selectStage(4)); pump(); CHECK(status().stage == 5 && target(4) == 0 && target(6) == 0 && Mock::servo[9] == 1100 && count(0xFE,5) == 0 && count(0x91) == 0);
    finish(4); CHECK(status().stage == 5); finish(6); CHECK(status().stage == 4 && status().position4 == 0 && status().position6 == 0 && status().completed == 0);
  } else if (name == "stage5_return_same_ram_commit") {
    uploadProfile(5,0,-345,6); finishStep(); CHECK(planBegin(1,1,5)); CHECK(planAction(0,branch(delay(0),1))); CHECK(planStep(0,0,1)); CHECK(planCommit()); pump();
    CHECK(status().stage == 5 && status().planVersion == 1 && target(6) == 0); finish(6); CHECK(status().stage == 5 && status().planVersion == 2 && status().position6 == 0 && count(0x91) == 0);
  } else if (name == "cylinder_stage2_entry_home_barrier") {
    CHECK(selectStage(2)); pump(); CHECK(status().stage == 2 && std::string(status().phase) == "home_wait" && !next() && !previous());
    CHECK(status().cylinder.active && status().cylinder.action == CylinderAction::Retract && !status().cylinder.in1High && !status().cylinder.in2High);
    advance(50); CHECK(status().cylinder.in2High && !status().cylinder.in1High && Mock::gpioState[16] == HIGH && Mock::gpioState[17] == LOW); advance(4999); CHECK(!next()); advance(1);
    CHECK(std::string(status().phase) == "ready" && status().completed == 0 && status().progress == 0 && Mock::sent.empty()); cylinderReference(CylinderAction::Retract);
  } else if (name == "cylinder_switch_immediate_stop_then_return") {
    select(2); for(uint8_t step=1;step<=2;++step) finishStep(); nextStep(); advance(50);
    CHECK(status().cylinder.in1High && Mock::gpioState[17] == HIGH && Mock::gpioState[16] == LOW); CHECK(selectStage(4));
    CHECK(!status().cylinder.active && !status().cylinder.in1High && !status().cylinder.in2High && !status().cylinder.referenceKnown && status().completed == 0 && status().progress == 0);
    CHECK(Mock::gpioState[17] == LOW && Mock::gpioState[16] == LOW);
    pump(); CHECK(status().stage == 2 && status().pendingStage == 4 && status().cylinder.action == CylinderAction::Retract && !status().cylinder.in2High);
    advance(49); CHECK(!status().cylinder.in2High); advance(1); CHECK(status().cylinder.in2High && Mock::gpioState[16] == HIGH && Mock::gpioState[17] == LOW); completePending();
    CHECK(status().stage == 4 && status().completed == 0 && !previous() && Mock::sent.empty()); cylinderReference(CylinderAction::Retract);
  } else if (name == "cylinder_switch_discards_unissued_actions") {
    CHECK(planBegin(1,3,4)); CHECK(planAction(0,branch(delay(5000),1))); CHECK(planAction(1,branch(cylinder(CylinderAction::Extend,100),2)));
    CHECK(planAction(2,branch(servo(2,1900),3))); CHECK(planStep(0,0,3)); CHECK(planCommit()); completePending(); nextStep();
    CHECK(!status().cylinder.active && status().completed == 0); CHECK(selectStage(5)); pump();
    CHECK(status().cylinder.active && status().cylinder.action == CylinderAction::Retract && Mock::servo[2] != 1900); completePending();
    CHECK(status().stage == 5 && status().completed == 0 && Mock::servo[2] == 1150 && Mock::sent.empty());
    for(const auto &write : Mock::gpioWrites) CHECK(!(write.pin == SystemConfig::Cylinder::IN1_PIN && write.value == HIGH)); cylinderReference(CylinderAction::Retract);
  } else if (name == "cylinder_switch_waits_issued_mks_and_servo") {
    CHECK(planBegin(1,5,4)); CHECK(planAction(0,branch(stepper(-123,320,0,false),1))); CHECK(planAction(1,branch(servo(5,1400,1000,false),1)));
    CHECK(planAction(2,branch(waitAll(),1))); CHECK(planAction(3,branch(cylinder(CylinderAction::Extend,100),2))); CHECK(planAction(4,branch(servo(2,1900),3)));
    CHECK(planStep(0,0,5)); CHECK(planCommit()); completePending(); nextStep(); CHECK(selectStage(5)); pump();
    CHECK(status().stage == 4 && status().completed == 0 && !status().cylinder.active && std::string(status().phase) == "return_wait"); finish(6); advance(999);
    CHECK(status().stage == 4 && !status().cylinder.active && count(0xFE) == 1); advance(1); CHECK(target() == 0 && count(0xFE) == 2 && status().cylinder.action == CylinderAction::Retract);
    finish(6); ackCursor=Mock::sent.size(); completePending(); CHECK(status().stage == 5 && Mock::servo[2] == 1150 && count(0x91) == 0);
    for(const auto &write : Mock::gpioWrites) CHECK(!(write.pin == SystemConfig::Cylinder::IN1_PIN && write.value == HIGH));
  } else if (name == "cylinder_entry_home_interrupted_selection") {
    CHECK(selectStage(2)); pump(); advance(50); CHECK(status().cylinder.in2High);
    const auto before = status().cylinder; const size_t writes = Mock::gpioWrites.size(); CHECK(selectStage(1));
    CHECK(status().cylinder.active && status().cylinder.in2High && status().cylinder.remainingMs == before.remainingMs && Mock::gpioWrites.size() == writes);
    advance(4999); CHECK(status().stage == 2 && !next()); advance(1);
    CHECK(status().stage == 1 && status().prop == 2 && status().completed == 0 && Mock::sent.empty()); cylinderReference(CylinderAction::Retract);
  } else if (name == "cylinder_debug_stop_preserves_other_axes") {
    CHECK(setDebug(true)); CHECK(debugStepper(-123,320,0,false,6)); CHECK(debugServo(5,810,1000)); CHECK(debugCylinder(CylinderAction::Extend,5000)); pump(); advance(50);
    CHECK(status().cylinder.in1High && !debugCylinder(CylinderAction::Retract,100)); CHECK(debugCylinder(CylinderAction::Stop,0));
    CHECK(!status().cylinder.active && !status().cylinder.in1High && !status().cylinder.in2High && !SystemSafety::stopped() && count(0xF7) == 0 && !next());
    finish(6); advance(950); CHECK(std::string(status().phase) == "ready" && status().position6 == -123 && Mock::servo[5] == 1400 && !status().cylinder.referenceKnown);
    CHECK(setDebug(false)); CHECK(!debugCylinder(CylinderAction::Extend,100));
  } else if (name == "cylinder_ram_duplicate_actuator_rejected") {
    CHECK(planBegin(1,3,4)); CHECK(planAction(0,branch(cylinder(CylinderAction::Extend,100,false),1))); CHECK(planAction(1,branch(cylinder(CylinderAction::Stop,0,false),1)));
    CHECK(planAction(2,branch(waitAll(),1))); CHECK(planStep(0,0,3)); CHECK(!planCommit()); planAbort(); CHECK(status().planVersion == 0 && !status().cylinder.active);
  } else if (name == "cylinder_ram_missing_barrier_rejected") {
    CHECK(planBegin(1,1,4)); CHECK(planAction(0,branch(cylinder(CylinderAction::Extend,100,false),1))); CHECK(planStep(0,0,1)); CHECK(!planCommit()); planAbort(); CHECK(!status().cylinder.active);
  } else if (name == "cylinder_ram_invalid_command_rejected") {
    CHECK(!valid(cylinder(static_cast<CylinderAction>(3),100)) && !valid(cylinder(CylinderAction::Extend,0)) && !valid(cylinder(CylinderAction::Retract,60001)));
    CHECK(valid(cylinder(CylinderAction::Stop,0)) && valid(cylinder(CylinderAction::Stop,100)) && valid(cylinder(CylinderAction::Extend,60000)));
    CHECK(planBegin(1,1,4)); CHECK(!planAction(0,branch(stepperFor(7,0,320),1))); CHECK(!planAction(0,branch(cylinder(CylinderAction::Extend,0),1))); planAbort();
  } else if (name == "cylinder_previous_unknown_entry_stops") {
    CHECK(planBegin(1,1,4)); CHECK(planAction(0,branch(cylinder(CylinderAction::Extend,100),1))); CHECK(planStep(0,0,1)); CHECK(planCommit()); completePending();
    finishStep(); cylinderReference(CylinderAction::Extend); const size_t writes=Mock::gpioWrites.size(); previousStep();
    CHECK(status().progress == 0 && !status().cylinder.referenceKnown && !status().cylinder.active && !previous());
    for(size_t i=writes;i<Mock::gpioWrites.size();++i) CHECK(Mock::gpioWrites[i].value == LOW);
  } else if (name == "cylinder_previous_known_full_default_duration") {
    select(2); CHECK(planBegin(1,1,2)); CHECK(planAction(0,branch(cylinder(CylinderAction::Extend,100),1))); CHECK(planStep(0,0,1)); CHECK(planCommit()); completePending();
    finishStep(); cylinderReference(CylinderAction::Extend); CHECK(previous()); pump(); CHECK(status().cylinder.action == CylinderAction::Retract && status().cylinder.durationMs == 6000);
    advance(50); advance(5999); CHECK(!next()); advance(1); CHECK(status().progress == 0 && !previous()); cylinderReference(CylinderAction::Retract);
  } else if (name == "cylinder_stage5_custom_return_and_home") {
    CHECK(planBegin(1,1,5)); CHECK(planAction(0,branch(cylinder(CylinderAction::Extend,100),1))); CHECK(planStep(0,0,1)); CHECK(planCommit()); completePending(); finishStep(); cylinderReference(CylinderAction::Extend);
    const size_t writes=Mock::gpioWrites.size(); CHECK(home()); pump(); CHECK(Mock::gpioWrites.size() == writes && !status().cylinder.active); CHECK(selectStage(4)); pump();
    CHECK(status().stage == 5 && status().cylinder.action == CylinderAction::Retract && !next()); completePending(); CHECK(status().stage == 4 && status().completed == 0 && Mock::sent.empty()); cylinderReference(CylinderAction::Retract);
  } else if (name == "cylinder_global_stop_immediate") {
    select(2); for(uint8_t step=1;step<=2;++step) finishStep(); nextStep(); advance(50); CHECK(status().cylinder.in1High && Mock::gpioState[17] == HIGH && Mock::gpioState[16] == LOW); SystemSafety::stop("full STOP");
    CHECK(!status().cylinder.active && !status().cylinder.in1High && !status().cylinder.in2High && !status().cylinder.referenceKnown && count(0xF7) == 3);
    advance(30000); CHECK(std::string(status().phase) == "fault" && Mock::gpioState[SystemConfig::Cylinder::IN1_PIN] == LOW && Mock::gpioState[SystemConfig::Cylinder::IN2_PIN] == LOW && !next());
  } else throw std::runtime_error("Unknown mechanism scenario");
  checkFrames();
}
int main(int argc, char **argv) {
  try { CHECK(argc == 2); test(argv[1]); std::cout << "PASS " << argv[1] << "\n"; return 0; }
  catch (const std::exception &error) { std::cerr << error.what() << "\n"; return 1; }
}
