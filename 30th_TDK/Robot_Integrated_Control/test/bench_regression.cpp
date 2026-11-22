#include "system_test_support.h"

static_assert(SystemConfig::BENCH_TEST_MODE, "Bench regression must exercise the production bench branch");
namespace Bench {
using MechanismConfig::CylinderAction;
constexpr int32_t stage2TrimLimit=SystemConfig::Rc::STAGE2_TRIM_MAX_OFFSET_PULSES;
uint8_t presentMask = 0, blockedCommand[3]{}, rejectCommand[3]{};
bool heartbeats = false, completeMoves = true, startMoves = true, completeHomes = true;
uint8_t axisState = 1;
uint32_t axisError = 0;
size_t responseCursor = 0;
unsigned requestId = 100;

AlfredoCRSF &radio() { return SystemSupport::radio(); }
bool ready() { return std::string(MechanismController::status().phase) == "ready"; }
void respond() {
    while (responseCursor < Mock::sent.size()) {
        const auto frame = Mock::sent[responseCursor++];
        if (!SystemSupport::mksFrame(frame)) continue;
        const uint8_t node = static_cast<uint8_t>(frame.msg.id), command = frame.msg.buf[0];
        CHECK(node >= 4 && node <= 6);
        const uint8_t index = static_cast<uint8_t>(node - 4);
        if (!(presentMask & (1U << index)) || blockedCommand[index] == command) continue;
        if (rejectCommand[index] == command) {
            SystemSupport::incomingMks(node, command, 0); continue;
        }
        if (command == 0x8C || command == 0x83 || command == 0x92 ||
            (command == 0xF3 && frame.msg.buf[1] == 1))
            SystemSupport::incomingMks(node, command, 1);
        else if (command == 0x91 || command == 0xFE) {
            if (startMoves || command == 0x91) SystemSupport::incomingMks(node, command, 1);
            if (command == 0x91 ? completeHomes : completeMoves)
                SystemSupport::incomingMks(node, command, 2);
        }
    }
}
void tick(uint32_t ms = 5) {
    mockMillis += ms;
    respond();
    if (heartbeats) for (uint8_t i = 0; i < 4; ++i)
        Mock::incoming[SystemConfig::Chassis::CAN_BUS[i]].push_back(
            SystemSupport::heartbeat(i, axisState, axisError));
    loop();
}
void advance(uint32_t ms) {
    while (ms) { const uint32_t slice = std::min(ms, uint32_t(5)); tick(slice); ms -= slice; }
}
void initialize(uint8_t mask = 0) {
    presentMask = mask;
    setup();
    radio().linked = true;
    for (int &value : radio().channels) value = 1000;
    radio().channels[SystemConfig::Rc::STAGE1_TRIM_CHANNEL] = SystemConfig::Rc::STAGE1_TRIM_CENTER_US;
    radio().channels[SystemConfig::Chassis::FORWARD.number] = SystemConfig::Chassis::FORWARD.maximum;
    radio().channels[SystemConfig::Chassis::STRAFE.number] = SystemConfig::Chassis::STRAFE.middle;
    radio().channels[SystemConfig::Chassis::ROTATE.number] = SystemConfig::Chassis::ROTATE.middle;
    radio().channels[SystemConfig::Rc::STAGE1_SWITCH_CHANNEL] = 1000;
    radio().channels[12] = 2000;
}
void boot(uint8_t mask = 0) {
    initialize(mask);
    for (unsigned i = 0; i < 10000 && !ready() && !SystemSafety::stopped(); ++i) tick();
    CHECK(ready() && !SystemSafety::stopped());
    advance(100);
    CHECK(RcController::status().link && RcController::status().valid);
    CHECK(ChassisController::status().mode == ChassisController::Mode::Locked);
}
void complete() {
    for (unsigned i = 0; i < 14000 && !ready() && !SystemSafety::stopped(); ++i) tick();
    CHECK(ready() && !SystemSafety::stopped());
}
std::string request(const std::string &command, bool accepted = true) {
    const unsigned id = requestId++;
    Serial.input += "@" + std::to_string(id) + " " + command + "\n";
    for (unsigned i = 0; i < 2000; ++i) {
        tick();
        const auto line = SystemSupport::lineFor(id);
        if (line.empty()) continue;
        CHECK(line.find(accepted ? " OK " : " ERR ") != std::string::npos);
        return line;
    }
    throw std::runtime_error("Bench USB reply timed out: " + command);
}
void button(uint8_t channel, bool high) {
    radio().channels[channel] = high ? 2000 : 1000;
    tick(); advance(SystemConfig::Rc::DEBOUNCE_MS);
}
void select(uint8_t stage) {
    for (uint8_t channel : SystemConfig::Rc::PROP_CHANNELS) radio().channels[channel] = 1000;
    radio().channels[SystemConfig::Rc::NEXT_CHANNEL] = 1000;
    radio().channels[SystemConfig::Rc::PREV_CHANNEL] = 1000;
    tick(); advance(SystemConfig::Rc::DEBOUNCE_MS);
    button(SystemConfig::Rc::STAGE1_SWITCH_CHANNEL, stage == 1);
    if (stage != 1) {
        advance(SystemConfig::Rc::DEBOUNCE_MS);
        button(SystemSupport::normalStageChannel(stage), true);
    }
}
void next() {
    button(SystemConfig::Rc::NEXT_CHANNEL, false);
    button(SystemConfig::Rc::NEXT_CHANNEL, true);
}
void previous() {
    button(SystemConfig::Rc::PREV_CHANNEL, false);
    button(SystemConfig::Rc::PREV_CHANNEL, true);
}
void assertMasks(uint8_t online, uint8_t known) {
    const auto mechanism = MechanismController::status();
    CHECK(mechanism.motorOnlineMask == online);
    CHECK(mechanism.motorOfflineMask == static_cast<uint8_t>(7U ^ online));
    CHECK(mechanism.positionKnownMask == known);
    CHECK((mechanism.motorOnlineMask & mechanism.motorOfflineMask) == 0);
}
void assertNoAxisTrafficSince(uint8_t node, size_t since) {
    for (size_t i = since; i < Mock::sent.size(); ++i)
        CHECK(!(Mock::sent[i].bus == 1 && Mock::sent[i].msg.id == node));
}
void assertWheelsLocked() {
    const auto chassis = ChassisController::status();
    CHECK(chassis.mode == ChassisController::Mode::Locked && !chassis.emergencyLatched);
    for (float wheel : chassis.wheel) CHECK(wheel == 0);
}
void assertZeroCadence(uint32_t duration) {
    size_t cursor = Mock::sent.size();
    bool seen[4]{};
    uint32_t last[4]{};
    unsigned count[4]{};
    const auto updates = radio().updateCount;
    for (uint32_t elapsed = 0; elapsed < duration; elapsed += 5) {
        tick(); CHECK(!SystemSafety::stopped()); assertWheelsLocked();
        while (cursor < Mock::sent.size()) {
            const auto &frame = Mock::sent[cursor++];
            if (SystemSupport::mksFrame(frame) || (frame.msg.id & 31) != 0x0D) continue;
            int axis = -1;
            for (uint8_t i = 0; i < 4; ++i)
                if ((frame.msg.id >> 5) == SystemConfig::Chassis::NODE_ID[i] &&
                    frame.bus == SystemConfig::Chassis::CAN_BUS[i]) axis = i;
            CHECK(axis >= 0 && frame.msg.len == 8);
            float velocity = 1; memcpy(&velocity, frame.msg.buf, sizeof(velocity)); CHECK(velocity == 0);
            if (seen[axis]) // Periodic locked/IDLE retries may add extra zero packets; the 15 ms loop must never be delayed.
            CHECK(uint32_t(mockMillis - last[axis]) <= SystemConfig::Chassis::SEND_INTERVAL_MS);
            seen[axis] = true; last[axis] = mockMillis; ++count[axis];
        }
    }
    for (unsigned packets : count) CHECK(packets >= duration / SystemConfig::Chassis::SEND_INTERVAL_MS - 1);
    CHECK(radio().updateCount >= updates + duration / 5);
}
void assertStopped() {
    CHECK(SystemSafety::stopped());
    CHECK(std::string(MechanismController::status().phase) == "fault");
    CHECK(ChassisController::status().emergencyLatched);
    SystemSupport::assertCylinderStopped();
}
void writeFixture(const char *path) { SystemSupport::writeStatusFixture(path, request("status")); }
void nextCompleted(uint8_t expectedStep) { next(); complete(); CHECK(MechanismController::status().completed == expectedStep); }

void startSourceExtension() {
    CHECK(MechanismController::status().stage == 2 && MechanismController::status().completed == 0);
    uint8_t extensionStep=0;
    for(uint8_t step=1;step<=Stage2::STEP_COUNT && !extensionStep;++step) {
        const auto &plan=Stage2::STEPS[step-1];
        for(size_t i=0;i<plan.count;++i)
            if(plan.actions[i].kind==MechanismConfig::ActionKind::Cylinder &&
               plan.actions[i].cylinderAct==CylinderAction::Extend) { extensionStep=step; break; }
    }
    CHECK(extensionStep>0);
    for(uint8_t step=1;step<extensionStep;++step) nextCompleted(step);
    next(); CHECK(!ready() && CylinderController::status().action==CylinderAction::Extend);
}

void setPairTrimKnob(int value) {
    radio().channels[SystemConfig::Rc::STAGE1_TRIM_CHANNEL]=value; tick();
}
void test(const std::string &name) {
    if(name=="bench_ch13_strobe_during_missing_mks_startup") {
        initialize(); radio().channels[13]=2000; advance(150);
        CHECK(!ready() && RcController::status().lightStrobeEnabled);
        CHECK(PwmController::status().strobeEnabled && PwmController::status().rcControlled);
        CHECK(MechanismController::status().stage==4 && MechanismController::status().pendingStage==0);
        const auto writes=Mock::pwmWrites.size(); assertZeroCadence(500);
        CHECK(Mock::pwmWrites.size()==writes+10 && !SystemSafety::stopped());
        radio().linked=false; tick();
        CHECK(PwmController::currentValue()==0 && !RcController::status().lightStrobeEnabled);
    } else if(name=="bench_ch13_strobe_disconnected_chassis_and_cylinder") {
        boot(); button(13,true);
        CHECK(RcController::status().lightStrobeEnabled && PwmController::status().strobeEnabled);
        CHECK(MechanismController::status().stage==4); assertWheelsLocked();
        button(10,true); complete();
        CHECK(MechanismController::status().stage==2 && RcController::status().selectedStage==2);
        CHECK(RcController::status().lightStrobeEnabled);
        startSourceExtension(); advance(100);
        CHECK(CylinderController::status().active && Mock::gpioState[17]==HIGH && Mock::gpioState[16]==LOW);
        const auto writes=Mock::pwmWrites.size(); assertZeroCadence(500);
        CHECK(Mock::pwmWrites.size()==writes+10 && CylinderController::status().active);
        CHECK(MechanismController::status().stage==2 && !SystemSafety::stopped());
        button(13,false); CHECK(PwmController::currentValue()==0 && !PwmController::status().strobeEnabled);
        CHECK(CylinderController::status().active); assertWheelsLocked();
    } else if(name=="bench_ch13_strobe_idle_odrive_does_not_gate_light") {
        heartbeats=true; axisState=1; boot(); button(13,true);
        CHECK(RcController::status().lightStrobeEnabled && PwmController::status().strobeEnabled);
        assertWheelsLocked(); const auto writes=Mock::pwmWrites.size(); assertZeroCadence(500);
        CHECK(Mock::pwmWrites.size()==writes+10 && !SystemSafety::stopped());
        button(13,false); CHECK(PwmController::currentValue()==0);
        CHECK(!RcController::status().lightStrobeEnabled); assertWheelsLocked();
    } else if(name=="bench_disabled_stage3_offline_ch11_no_wait_or_stop") {
        boot(); const uint32_t enteredAt=mockMillis; const auto frames=Mock::sent.size();
        button(12,false); button(11,true); advance(150);
        CHECK(ready() && MechanismController::status().stage==4 && MechanismController::status().pendingStage==0);
        request("stage 3",false); request("plan.begin 1 1 3 1",false);
        button(10,true); complete();
        CHECK(ready() && MechanismController::status().stage==2 && uint32_t(mockMillis-enteredAt)<1000);
        CHECK(Mock::servo[3]==1580 && Mock::servo[9]==1800 && Mock::servo[4]==1300);
        CHECK(!MechanismController::status().stage2TrimAllowed);
        assertNoAxisTrafficSince(4,frames); assertNoAxisTrafficSince(5,frames); assertNoAxisTrafficSince(6,frames);
        startSourceExtension(); CHECK(CylinderController::status().active); complete(); assertWheelsLocked();
    } else if(name=="bench_disabled_stage3_online_pair_stage2_trim_retained") {
        boot(3); assertMasks(3,3);
        CHECK(Mock::servoAttached[3] && Mock::servo[3]==2250);
        CHECK(SystemSupport::mksCount(0x92,4)==1 && SystemSupport::mksCount(0x92,5)==1);
        CHECK(SystemSupport::mksCount(0x91,4)==0 && SystemSupport::mksCount(0x91,5)==0);
        select(2); complete(); setPairTrimKnob(2000); advance(200); complete();
        CHECK(MechanismController::status().stage2TrimAllowed && MechanismController::status().completed==0);
        CHECK(MechanismController::status().position4==stage2TrimLimit && MechanismController::status().position5==-stage2TrimLimit);
        CHECK(Mock::servo[3]==1580 && Mock::servo[4]==1300 && Mock::servo[9]==1800);
        request("stage 3",false); CHECK(ready() && MechanismController::status().stage==2);
        setPairTrimKnob(1500); advance(200); complete();
        CHECK(MechanismController::status().position4==0 && MechanismController::status().position5==0);
        assertZeroCadence(150); assertWheelsLocked();
    } else if(name=="bench_stage2_pair_trim_requires_both_online") {
        boot(1); select(2); complete();
        CHECK(Mock::servo[9]==1800 && Mock::servo[4]==1300 && Mock::servo[3]==1580);
        const auto frames=Mock::sent.size(); setPairTrimKnob(2000); advance(400);
        CHECK(ready() && !MechanismController::status().stage2TrimAllowed && !MechanismController::status().stage2TrimMoving);
        assertNoAxisTrafficSince(4,frames); assertNoAxisTrafficSince(5,frames);
        nextCompleted(1); CHECK(!SystemSafety::stopped()); assertWheelsLocked();
    } else if(name=="bench_stage2_pair_trim_all_offline_keeps_cylinder_usable") {
        boot(); select(2); complete(); setPairTrimKnob(2000); advance(400);
        CHECK(!MechanismController::status().stage2TrimAllowed && !MechanismController::status().stage2TrimMoving);
        CHECK(Mock::servo[9]==1800 && Mock::servo[4]==1300 && Mock::servo[3]==1580);
        const auto frames=Mock::sent.size(); startSourceExtension();
        const auto extensionStep=MechanismController::status().step;
        CHECK(!ready() && CylinderController::status().active && CylinderController::status().action==CylinderAction::Extend);
        assertNoAxisTrafficSince(4,frames); assertNoAxisTrafficSince(5,frames); complete();
        CHECK(MechanismController::status().completed==extensionStep && !SystemSafety::stopped()); assertWheelsLocked();
    } else if(name=="bench_stage2_pair_trim_two_axes_without_chassis_or_id6") {
        boot(3); select(2); complete(); CHECK(MechanismController::status().stage2TrimAllowed);
        completeMoves=false; setPairTrimKnob(2000); advance(150);
        CHECK(MechanismController::status().stage2TrimMoving && !ready());
        CHECK(SystemSupport::target(4)==stage2TrimLimit && SystemSupport::target(5)==-stage2TrimLimit);
        SystemSupport::incomingMks(4,0xFE,2); tick(); CHECK(!ready());
        SystemSupport::incomingMks(5,0xFE,2); tick(); complete();
        CHECK(MechanismController::status().position4==stage2TrimLimit && MechanismController::status().position5==-stage2TrimLimit);
        CHECK(MechanismController::status().completed==0 && !SystemSafety::stopped()); assertZeroCadence(150);
    } else if (name == "bench_stage2_boot_offline_full_retract_no_repeat") {
        initialize(); advance(5100);
        CHECK(!ready() && !SystemSafety::stopped());
        CHECK(CylinderController::status().fullyRetracted && !CylinderController::status().active);
        unsigned retracts=0;
        for (const auto &event:Mock::gpioWrites) if(event.value==HIGH) {
            CHECK(event.pin==SystemConfig::Cylinder::IN2_PIN); ++retracts;
        }
        CHECK(retracts==1);
        complete(); assertMasks(0,0); assertWheelsLocked();
        const auto writes=Mock::gpioWrites.size(); const uint32_t selectedAt=mockMillis;
        select(2);
        CHECK(ready() && MechanismController::status().stage==2 && uint32_t(mockMillis-selectedAt)<500);
        for(size_t i=writes;i<Mock::gpioWrites.size();++i) CHECK(Mock::gpioWrites[i].value==LOW);
        nextCompleted(1); CHECK(CylinderController::status().fullyRetracted);
        CHECK(SystemSupport::mksCount(0xFE)==0 && SystemSupport::mksCount(0x91)==0);
        assertZeroCadence(150);
    } else if (name == "bench_stage2_boot_stop_no_reenergize") {
        initialize(); advance(100);
        CHECK(CylinderController::status().active && Mock::gpioState[SystemConfig::Cylinder::IN2_PIN]==HIGH);
        request("stop"); assertStopped(); const auto writes=Mock::gpioWrites.size();
        advance(12000); assertStopped(); CHECK(!CylinderController::status().fullyRetracted);
        for(size_t i=writes;i<Mock::gpioWrites.size();++i) CHECK(Mock::gpioWrites[i].value==LOW);
    } else if (name == "bench_all_disconnected_ready_fixture") {
        boot(); assertMasks(0, 0);
        CHECK(mockMillis >= 31000 && mockMillis < 32000);
        CHECK(SystemSupport::mksCount(0x8C, 4) > 1 && SystemSupport::mksCount(0x8C, 5) > 1 && SystemSupport::mksCount(0x8C, 6) > 1);
        CHECK(SystemSupport::mksCount(0x83) == 0 && SystemSupport::mksCount(0x92) == 0 && SystemSupport::mksCount(0x91) == 0);
        CHECK(Serial.output.find("offline") != std::string::npos && !SystemSafety::stopped());
        SystemSupport::assertCylinderStopped(); assertZeroCadence(150);
        writeFixture(".pio/host-tests/bench_offline_status_fixture.json");
    } else if (name == "bench_only_id6_ready_fixture") {
        boot(4); assertMasks(4, 4);
        CHECK(SystemSupport::mksCount(0x91, 6) == 1 && SystemSupport::mksCount(0x92) == 0);
        CHECK(MechanismController::status().currentConfiguredMask == 4);
        request("next"); complete();
        CHECK(SystemSupport::target(6) == SystemSupport::stage4Position(1));
        CHECK(SystemSupport::mksCount(0xFE, 4) == 0 && SystemSupport::mksCount(0xFE, 5) == 0);
        writeFixture(".pio/host-tests/bench_id6_status_fixture.json");
    } else if (name == "bench_only_id45_ready_fixture") {
        boot(3); assertMasks(3, 3);
        CHECK(SystemSupport::mksCount(0x92, 4) == 1 && SystemSupport::mksCount(0x92, 5) == 1);
        CHECK(SystemSupport::mksCount(0x91) == 0);
        select(3); complete(); nextCompleted(1);
        CHECK(SystemSupport::mksCount(0xFE, 4) == 1 && SystemSupport::mksCount(0xFE, 5) == 1);
        CHECK(SystemSupport::target(4) == SystemSupport::stage3Motor(1,4).position);
        CHECK(SystemSupport::target(5) == SystemSupport::stage3Motor(1,5).position);
        CHECK(Mock::servo[3] == SystemSupport::stage3Pulse(1));
        writeFixture(".pio/host-tests/bench_pair_status_fixture.json");
    } else if (name == "bench_single_pair_axis_skips_missing") {
        boot(1); assertMasks(1,1); const size_t since=Mock::sent.size();
        select(3); complete(); nextCompleted(1);
        CHECK(SystemSupport::mksCount(0xFE,4)==1 && SystemSupport::mksCount(0xFE,5)==0);
        previous(); complete(); CHECK(MechanismController::status().completed==0);
        CHECK(SystemSupport::target(4)==0); select(2); complete();
        assertNoAxisTrafficSince(5,since); assertNoAxisTrafficSince(6,since);
        CHECK(MechanismController::status().stage==2);
    } else if (name == "bench_startup_enqueue_timeout_skips_axis") {
        Mock::rejectNode=4; Mock::rejectCommand=0x8C; boot(7); assertMasks(6,6);
        CHECK(SystemSupport::mksCount(0x8C,4)==0 && SystemSupport::mksCount(0x83,4)==0);
        CHECK(mockMillis >= 5000 && mockMillis < 7000 && !SystemSafety::stopped());
    } else if (name == "bench_missing_current_ack_skips_axis") {
        blockedCommand[0]=0x83; boot(7); assertMasks(6,6);
        CHECK(MechanismController::status().currentConfiguredMask==6);
        CHECK(SystemSupport::mksCount(0xF3,4,1)==0 && SystemSupport::mksCount(0x92,4)==0);
        CHECK(SystemSupport::mksCount(0x91,6)==1);
    } else if (name == "bench_missing_enable_ack_skips_axis") {
        blockedCommand[1]=0xF3; boot(7); assertMasks(5,5);
        CHECK(SystemSupport::mksCount(0x92,5)==0 && SystemSupport::mksCount(0x92,4)==1);
        CHECK(MechanismController::status().currentConfiguredMask==7);
    } else if (name == "bench_missing_stationary_zero_ack_unreferences_axis") {
        blockedCommand[0]=0x92; boot(7); assertMasks(6,6);
        const size_t since=Mock::sent.size(); select(2); complete();
        CHECK(Mock::servo[3]==1580 && Mock::servo[4]==1300 && Mock::servo[9]==1800);
        setPairTrimKnob(2000); advance(200);
        CHECK(ready() && !MechanismController::status().stage2TrimAllowed);
        CHECK(!MechanismController::status().stage2TrimMoving && SystemSupport::mksCount(0xFE,5)==0);
        assertNoAxisTrafficSince(4,since);
        startSourceExtension(); complete(); CHECK(!SystemSafety::stopped());
        CHECK(Mock::servoAttached[3] && Mock::servo[3]==SystemSupport::stage2Pulse(MechanismController::status().completed,3));
        CHECK(CylinderController::status().referenceAction==CylinderAction::Extend);
        request("mode debug"); request("debug.move 4 700 100 5",false);
        request("debug.move 5 -700 100 5"); complete();
        assertNoAxisTrafficSince(4,since);
        CHECK(SystemSupport::mksCount(0xFE,5)==1 && SystemSupport::target(5)==-700);
        CHECK(SystemSupport::mksCount(0x91,4)==0 && MechanismController::status().positionKnownMask==6);
    } else if (name == "bench_startup_home_enqueue_timeout_skips_axis") {
        Mock::rejectNode=6; Mock::rejectCommand=0x91; boot(7); assertMasks(3,3);
        CHECK(mockMillis >= 5000 && mockMillis < 7000 && !SystemSafety::stopped());
        CHECK(SystemSupport::mksCount(0x8C,6)==1 && SystemSupport::mksCount(0x83,6)==1 && SystemSupport::mksCount(0xF3,6,1)==1);
        CHECK(SystemSupport::mksCount(0x91,6)==0);
        CHECK(SystemSupport::mksCount(0xF7)==0 && SystemSupport::mksCount(0xF3,-1,0)==0);
        CHECK(!ChassisController::status().emergencyLatched);
        CHECK(Serial.output.find("offline") != std::string::npos);
        const size_t since=Mock::sent.size(); request("next"); complete(); request("home"); complete();
        assertNoAxisTrafficSince(6,since);
    } else if (name == "bench_missing_startup_home_first_ack_skips_axis") {
        blockedCommand[2]=0x91; boot(7); assertMasks(3,3);
        CHECK(SystemSupport::mksCount(0x91,6)==1);
        CHECK(SystemSupport::mksCount(0xF7,6)==1 && SystemSupport::mksCount(0xF3,6,0)==1);
        CHECK(SystemSupport::mksCount(0xF7,4)==0 && SystemSupport::mksCount(0xF7,5)==0);
        CHECK(SystemSupport::mksCount(0xF3,4,0)==0 && SystemSupport::mksCount(0xF3,5,0)==0);
        CHECK(!ChassisController::status().emergencyLatched);
        const size_t since=Mock::sent.size(); request("next"); complete(); request("home"); complete();
        assertNoAxisTrafficSince(6,since);
        CHECK(MechanismController::status().completed==0);
    } else if (name == "bench_startup_home_completion_timeout_remains_fault") {
        completeHomes=false; initialize(7); advance(34000); assertStopped();
        CHECK(SystemSupport::mksCount(0x91,6)==1);
        CHECK(std::string(SystemSafety::reason()).find("completion")!=std::string::npos);
    } else if (name == "bench_rejected_feedback_remains_fault") {
        rejectCommand[0]=0x8C; initialize(7); advance(1300); assertStopped();
        CHECK(SystemSupport::mksCount(0x83)==0);
    } else if (name == "bench_rejected_current_remains_fault") {
        rejectCommand[2]=0x83; initialize(7); advance(1400); assertStopped();
        CHECK(SystemSupport::mksCount(0x91)==0);
    } else if (name == "bench_late_offline_ack_does_not_rejoin") {
        boot(4); const size_t since=Mock::sent.size();
        for (uint8_t command : {uint8_t(0x8C),uint8_t(0x83),uint8_t(0xF3),uint8_t(0x92),uint8_t(0xFE)})
            SystemSupport::incomingMks(4,command,1);
        SystemSupport::incomingMks(4,0xFE,2); advance(50); assertMasks(4,4);
        request("stage 3"); complete(); request("next"); complete();
        assertNoAxisTrafficSince(4,since); assertNoAxisTrafficSince(5,since);
    } else if (name == "bench_offline_debug_axis_is_rejected") {
        boot(); request("mode debug"); const size_t since=Mock::sent.size();
        for (uint8_t node : SystemConfig::Mechanism::MOTOR_IDS) {
            request("debug.move "+std::to_string(node)+" 100 320 2",false);
            request("debug.gohome "+std::to_string(node),false);
        }
        CHECK(Mock::sent.size()>=since);
        for (uint8_t node : SystemConfig::Mechanism::MOTOR_IDS) assertNoAxisTrafficSince(node,since);
        request("debug.cylinder 1 100"); complete(); SystemSupport::assertCylinderStopped();
    } else if (name == "bench_rc_stage2_with_no_odrives") {
        boot(); CHECK(CylinderController::status().fullyRetracted);
        const uint32_t selectedAt=mockMillis; const auto writes=Mock::gpioWrites.size(); select(2);
        CHECK(MechanismController::status().stage==2 && ready() && uint32_t(mockMillis-selectedAt)<500);
        CHECK(!CylinderController::status().active && CylinderController::status().fullyRetracted);
        advance(100); SystemSupport::assertCylinderStopped();
        for(size_t i=writes;i<Mock::gpioWrites.size();++i) CHECK(Mock::gpioWrites[i].value==LOW);
        CHECK(MechanismController::status().completed==0);
        for(uint8_t step=1;step<=Stage2::STEP_COUNT;++step) {
            nextCompleted(step);
            for(uint8_t pin : {uint8_t(4),uint8_t(6),uint8_t(9)}) CHECK(Mock::servo[pin]==SystemSupport::stage2Pulse(step,pin));
            CHECK(CylinderController::status().referenceAction==SystemSupport::stage2Reference(step));
        }
        // A repeat may begin with a cylinder-only segment. Servos not mentioned
        // there retain the actual final pose of the preceding completed round.
        const uint8_t pins[]={4,6,9};
        int expectedPulse[]={Mock::servo[4],Mock::servo[6],Mock::servo[9]};
        auto expectedCylinder=CylinderController::status().referenceAction;
        const auto &repeatPlan=Stage2::STEPS[Stage2::REPEAT_FROM-1];
        for(size_t i=0;i<repeatPlan.count;++i) {
            const auto &action=repeatPlan.actions[i];
            if(action.kind==MechanismConfig::ActionKind::Servo)
                for(size_t n=0;n<sizeof(pins)/sizeof(pins[0]);++n)
                    if(action.pin==pins[n]) expectedPulse[n]=action.pulse;
            if(action.kind==MechanismConfig::ActionKind::Cylinder && action.cylinderAct!=CylinderAction::Stop)
                expectedCylinder=action.cylinderAct;
        }
        nextCompleted(Stage2::REPEAT_FROM);
        for(size_t n=0;n<sizeof(pins)/sizeof(pins[0]);++n) CHECK(Mock::servo[pins[n]]==expectedPulse[n]);
        CHECK(CylinderController::status().referenceAction==expectedCylinder);
        CHECK(SystemSupport::mksCount(0xFE)==0 && SystemSupport::mksCount(0x91)==0);
        writeFixture(".pio/host-tests/bench_stage2_status_fixture.json"); assertWheelsLocked();
    } else if (name == "bench_rc_stage2_with_odrives_idle") {
        heartbeats=true; axisState=1; boot(); CHECK(ChassisController::status().healthy);
        select(2); complete(); nextCompleted(1); assertWheelsLocked(); assertZeroCadence(150);
        for (const auto &axis : ChassisController::status().axes) CHECK(axis.state==1);
    } else if (name == "bench_rc_stage2_with_odrive_errors") {
        heartbeats=true; axisState=1; axisError=2; boot();
        CHECK(!ChassisController::status().healthy); select(2); complete(); nextCompleted(1); assertWheelsLocked();
    } else if (name == "bench_rc_stage3_all_motors_absent") {
        boot(); select(3); complete(); const size_t since=Mock::sent.size();
        for (uint8_t step : {uint8_t(1),uint8_t(2),uint8_t(3),uint8_t(4),uint8_t(2)}) nextCompleted(step);
        previous(); complete(); CHECK(MechanismController::status().previousAvailable);
        for (uint8_t node : SystemConfig::Mechanism::MOTOR_IDS) assertNoAxisTrafficSince(node,since);
        CHECK(Mock::servo[3]>=500 && Mock::servo[3]<=2500);
    } else if (name == "bench_stage1_trim_without_odrives_or_mks") {
        boot(); select(1); complete();
        const int base=MechanismController::status().pulse2Base;
        radio().channels[SystemConfig::Rc::STAGE1_TRIM_CHANNEL]=2000; advance(50);
        CHECK(RcController::status().stage1TrimUs==150 && MechanismController::status().stage1TrimAllowed);
        CHECK(MechanismController::status().pulse2Base==base && Mock::servo[2]==base+150);
        assertWheelsLocked();
        radio().channels[SystemConfig::Rc::STAGE1_TRIM_CHANNEL]=1000; advance(50);
        CHECK(RcController::status().stage1TrimUs==-150 && Mock::servo[2]==base-150);
        CHECK(SystemSupport::mksCount(0xFE)==0 && !SystemSafety::stopped());
        button(SystemConfig::Rc::STAGE1_SWITCH_CHANNEL,false); complete();
        CHECK(RcController::status().stage1TrimUs==0 && MechanismController::status().stage1TrimUs==0);
        CHECK(Mock::servo[2]==1150); assertWheelsLocked();
    } else if (name == "bench_rc_stage1_only_id6") {
        boot(4); select(1); complete();
        button(SystemConfig::Rc::PROP_BASKET_CH,false); button(SystemConfig::Rc::PROP_BASKET_CH,true); complete();
        CHECK(MechanismController::status().prop==1 && MechanismController::status().completed==1);
        CHECK(SystemSupport::target(6)==SystemSupport::propMotor(1,1).position && Mock::servo[2]==SystemSupport::propPulse(1,1,2) && Mock::servo[5]==2056 && !MechanismController::status().propComplete);
        nextCompleted(2); CHECK(!MechanismController::status().propComplete && SystemSupport::target(6)==SystemSupport::propMotor(1,2).position);
        CHECK(SystemSupport::lastMove(6).buf[1]==1 && SystemSupport::lastMove(6).buf[2]==244 && SystemSupport::lastMove(6).buf[3]==150);
        CHECK(Mock::servo[2]==SystemSupport::propPulse(1,2,2));
        nextCompleted(3); CHECK(MechanismController::status().propComplete && Mock::servo[2]==SystemSupport::propPulse(1,3,2));
        previous(); complete(); CHECK(MechanismController::status().completed==2 && !MechanismController::status().propComplete);
        previous(); complete(); CHECK(MechanismController::status().completed==1 && SystemSupport::target(6)==SystemSupport::propMotor(1,1).position);
        nextCompleted(2); nextCompleted(3); CHECK(Mock::servo[2]==SystemSupport::propPulse(1,3,2) && Mock::servo[5]==2056); assertWheelsLocked();
        button(SystemConfig::Rc::STAGE1_SWITCH_CHANNEL,false); complete();
        CHECK(MechanismController::status().stage==4 && SystemSupport::mksCount(0x91,6)==2);
    } else if (name == "bench_rc_stage1_no_id6_keeps_servos") {
        boot(); select(1); complete();
        button(SystemConfig::Rc::PROP_VASE_CH,false); button(SystemConfig::Rc::PROP_VASE_CH,true); complete();
        CHECK(MechanismController::status().prop==2 && MechanismController::status().completed==1);
        CHECK(Mock::servo[2]==1833 && Mock::servo[5]==2056);
        nextCompleted(2); nextCompleted(3); previous(); complete();
        CHECK(MechanismController::status().completed==2 && Mock::servo[2]==2111 && SystemSupport::mksCount(0xFE)==0);
    } else if (name == "bench_rc_stage1_no_id6_animal_release") {
        boot(); select(1); complete(); const size_t since=Mock::sent.size();
        for (uint8_t prop : {uint8_t(4),uint8_t(5)}) {
            const int grip=SystemSupport::propPulse(prop,2,2);
            button(SystemConfig::Rc::PROP_CHANNELS[prop-1],false); button(SystemConfig::Rc::PROP_CHANNELS[prop-1],true); complete();
            CHECK(MechanismController::status().prop==prop && MechanismController::status().completed==1);
            nextCompleted(2); CHECK(Mock::servo[2]==grip && !MechanismController::status().propComplete);
            nextCompleted(3); CHECK(Mock::servo[2]==grip && !MechanismController::status().propComplete);
            nextCompleted(4); CHECK(Mock::servo[2]==1667 && MechanismController::status().propComplete);
            previous(); complete(); CHECK(MechanismController::status().completed==3 && Mock::servo[2]==grip && !MechanismController::status().propComplete);
            assertMasks(0,0); assertWheelsLocked();
        }
        for (uint8_t node : SystemConfig::Mechanism::MOTOR_IDS) assertNoAxisTrafficSince(node,since);
    } else if (name == "bench_stage1_low_exit_skips_offline_home") {
        boot(); select(1); complete();
        button(SystemConfig::Rc::PROP_BASKET_CH,false); button(SystemConfig::Rc::PROP_BASKET_CH,true); complete();
        CHECK(MechanismController::status().stage==1 && MechanismController::status().completed==1);
        button(SystemConfig::Rc::STAGE1_SWITCH_CHANNEL,false); complete(); advance(200);
        CHECK(MechanismController::status().stage==4 && ready() && MechanismController::status().completed==0);
        CHECK(SystemSupport::mksCount(0x91)==0 && !SystemSafety::stopped());
    } else if (name == "bench_previous_cylinder_estimate") {
        boot(); select(2); complete();
        for(uint8_t step=1;step<=5;++step) nextCompleted(step);
        previous(); CHECK(CylinderController::status().active); advance(100);
        CHECK(Mock::gpioState[17]==HIGH && Mock::gpioState[16]==LOW); complete();
        CHECK(MechanismController::status().completed==4 && Mock::servo[6]==SystemSupport::stage2Pulse(4,6));
        CHECK(CylinderController::status().referenceAction==CylinderAction::Extend);
        CHECK(Mock::servo[9]==1000 && Mock::servo[4]==2100);
    } else if (name == "bench_busy_next_requires_fresh_release") {
        boot(); select(2); complete(); nextCompleted(1); nextCompleted(2); next(); CHECK(!ready());
        advance(7000); CHECK(ready() && MechanismController::status().completed==3);
        advance(7000); CHECK(MechanismController::status().completed==3);
        button(SystemConfig::Rc::NEXT_CHANNEL,false);
        radio().channels[SystemConfig::Rc::NEXT_CHANNEL]=2000;
        tick(); advance(SystemConfig::Rc::DEBOUNCE_MS-5); CHECK(MechanismController::status().completed==3);
        advance(5); complete(); CHECK(MechanismController::status().completed==4);
    } else if (name == "bench_rc_link_still_required") {
        boot(); radio().linked=false; advance(100); CHECK(!RcController::status().link);
        select(2); next(); advance(200);
        CHECK(MechanismController::status().stage==4 && MechanismController::status().completed==0);
        CHECK(!CylinderController::status().active && !SystemSafety::stopped());
    } else if (name == "bench_invalid_rc_still_required") {
        boot(); radio().channels[SystemConfig::Chassis::ROTATE.number]=3000; advance(50);
        CHECK(!RcController::status().valid); select(2); next(); advance(200);
        CHECK(MechanismController::status().stage==4 && MechanismController::status().completed==0);
    } else if (name == "bench_usb_stop_is_still_global") {
        boot(); select(2); complete(); startSourceExtension(); advance(100);
        CHECK(Mock::gpioState[SystemConfig::Cylinder::IN1_PIN]==HIGH && Mock::gpioState[SystemConfig::Cylinder::IN2_PIN]==LOW);
        request("stop"); assertStopped(); request("next",false); request("home",false);
        select(3); next(); advance(200); assertStopped();
    } else if (name == "bench_switch_cancels_cylinder_before_return") {
        boot(); select(2); complete(); startSourceExtension(); advance(100);
        CHECK(Mock::gpioState[SystemConfig::Cylinder::IN1_PIN]==HIGH); const uint32_t switchedAt=mockMillis;
        request("stage 3"); CHECK(Mock::gpioState[SystemConfig::Cylinder::IN1_PIN]==LOW && Mock::gpioState[SystemConfig::Cylinder::IN2_PIN]==LOW);
        CHECK(mockMillis-switchedAt < SystemConfig::Cylinder::REVERSAL_DEADTIME_MS);
        complete(); CHECK(MechanismController::status().stage==3 && MechanismController::status().completed==0);
        CHECK(CylinderController::status().referenceAction==CylinderAction::Retract);
        nextCompleted(1); CHECK(SystemSupport::mksCount(0xFE)==0);
    } else if (name == "bench_rc_switch_cylinder_tolerates_offline_return") {
        boot(); select(2); complete(); startSourceExtension(); advance(100); select(3);
        CHECK(Mock::gpioState[SystemConfig::Cylinder::IN1_PIN]==LOW && Mock::gpioState[SystemConfig::Cylinder::IN2_PIN]==LOW);
        complete(); CHECK(MechanismController::status().stage==3);
        select(4); complete(); CHECK(MechanismController::status().stage==4);
        CHECK(SystemSupport::mksCount(0xFE)==0 && SystemSupport::mksCount(0x91)==0);
    } else if (name == "bench_cylinder_keeps_locked_chassis_15ms") {
        boot(); select(2); complete(); startSourceExtension(); CHECK(!ready());
        assertZeroCadence(600); CHECK(!ready() && CylinderController::status().active);
        complete(); assertWheelsLocked();
    } else if (name == "bench_runtime_missing_ack_still_stops_cylinder") {
        boot(4); CHECK(MechanismController::planBegin(1,3,4,1));
        CHECK(MechanismController::planAction(0,MechanismConfig::branch(MechanismConfig::stepperFor(6,100,320,2,false),1)));
        CHECK(MechanismController::planAction(1,MechanismConfig::branch(MechanismConfig::cylinder(CylinderAction::Extend,6000,false),1)));
        CHECK(MechanismController::planAction(2,MechanismConfig::branch(MechanismConfig::waitAll(),1)));
        CHECK(MechanismController::planStep(0,0,3)); CHECK(MechanismController::planCommit()); complete();
        startMoves=false; completeMoves=false; request("next"); advance(100);
        CHECK(CylinderController::status().active && Mock::gpioState[SystemConfig::Cylinder::IN1_PIN]==HIGH);
        advance(1600); assertStopped();
        CHECK(std::string(SystemSafety::reason()).find("start ACK")!=std::string::npos);
    } else if (name == "bench_runtime_negative_reply_still_stops") {
        boot(4); completeMoves=false; request("next");
        SystemSupport::incomingMks(6,0xFE,3); tick(); assertStopped();
    } else if (name == "bench_ram_plan_skips_offline_axis_only") {
        boot(); CHECK(MechanismController::planBegin(1,3,4,1));
        CHECK(MechanismController::planAction(0,MechanismConfig::branch(MechanismConfig::stepperFor(6,100,320,2,false),1)));
        CHECK(MechanismController::planAction(1,MechanismConfig::branch(MechanismConfig::cylinder(CylinderAction::Extend,100,false),1)));
        CHECK(MechanismController::planAction(2,MechanismConfig::branch(MechanismConfig::waitAll(),1)));
        CHECK(MechanismController::planStep(0,0,3)); CHECK(MechanismController::planCommit()); complete();
        const size_t since=Mock::sent.size(); request("next"); advance(60);
        CHECK(CylinderController::status().active && Mock::gpioState[SystemConfig::Cylinder::IN1_PIN]==HIGH);
        complete(); CHECK(MechanismController::status().completed==1); assertNoAxisTrafficSince(6,since);
        CHECK(CylinderController::status().referenceKnown);
    } else if (name == "bench_pwm_with_disconnected_chassis") {
        boot(); CHECK(PwmController::configure(55,5)); select(2); complete(); startSourceExtension(); advance(100);
        CHECK(CylinderController::status().active && Mock::gpioState[17] == HIGH && Mock::gpioState[16] == LOW);
        const size_t writes = Mock::pwmWrites.size(); assertZeroCadence(2550);
        CHECK(Mock::pwmWrites.size() == writes + 510 && Mock::pwmResolution == 8);
        int minimum = 55, maximum = 0;
        for (size_t i = writes; i < Mock::pwmWrites.size(); ++i) {
            CHECK(Mock::pwmWrites[i].pin == 14);
            CHECK(Mock::pwmWrites[i].at - Mock::pwmWrites[i - 1].at == 5);
            minimum = std::min(minimum, Mock::pwmWrites[i].value);
            maximum = std::max(maximum, Mock::pwmWrites[i].value);
        }
        CHECK(minimum == 0 && maximum == 55 && CylinderController::status().active);
        for (const auto &event : Mock::gpioWrites) CHECK(event.pin != 14);
        CHECK(Mock::gpioState[17] == HIGH && Mock::gpioState[16] == LOW);
        assertWheelsLocked();
    } else if (name == "bench_boot_held_stage2_offline_retry") {
        initialize(); radio().channels[12]=1000; radio().channels[10]=2000;
        advance(38000);
        CHECK(ready() && MechanismController::status().stage==2 && RcController::status().selectedStage==2);
        assertMasks(0,0); assertWheelsLocked();
        CHECK(CylinderController::status().referenceKnown && CylinderController::status().referenceAction==CylinderAction::Retract);
        CHECK(ChassisController::status().directionStage==2 && SystemSupport::mksCount(0xFE)==0 && SystemSupport::mksCount(0x91)==0);
        nextCompleted(1); CHECK(MechanismController::status().completed==1);
    } else if (name == "bench_partial_pair_pending_direction_and_cylinder") {
        boot(1); select(3); complete(); nextCompleted(1); completeMoves=false;
        const size_t sentAt=Mock::sent.size(); request("stage 2"); advance(100);
        CHECK(MechanismController::status().stage==3 && MechanismController::status().pendingStage==2);
        CHECK(ChassisController::status().directionStage==2 && SystemSupport::target(4)==0);
        assertNoAxisTrafficSince(5,sentAt); assertNoAxisTrafficSince(6,sentAt);
        const size_t pwmWrites=Mock::pwmWrites.size(); assertZeroCadence(1000);
        CHECK(MechanismController::status().stage==3 && Mock::pwmWrites.size()==pwmWrites+200);
        SystemSupport::incomingMks(4,0xFE,2); tick(); complete();
        CHECK(MechanismController::status().stage==2 && MechanismController::status().pendingStage==0);
        CHECK(CylinderController::status().referenceKnown && CylinderController::status().referenceAction==CylinderAction::Retract);
        assertMasks(1,1); assertWheelsLocked(); assertNoAxisTrafficSince(5,sentAt); assertNoAxisTrafficSince(6,sentAt);
    } else if (name == "bench_default_cylinder_only") {
        CHECK(SystemConfig::BENCH_TEST_MODE); boot(); select(2); complete();
        for(uint8_t step=1;step<=5;++step) nextCompleted(step);
        CHECK(CylinderController::status().referenceAction==SystemSupport::stage2Reference(5));
        CHECK(SystemSupport::mksCount(0xFE)==0 && SystemSupport::mksCount(0x91)==0);
        assertWheelsLocked();
    } else throw std::runtime_error("Unknown bench case: "+name);
    CHECK(!SystemSafety::stopped() || std::string(MechanismController::status().phase)=="fault");
    SystemSupport::assertNoId7(); SystemSupport::assertCylinderGpioHistory();
}
} // namespace Bench
int main(int argc,char **argv) {
    try { CHECK(argc==2); Bench::test(argv[1]); printf("PASS %s\n",argv[1]); return 0; }
    catch (const std::exception &error) { fprintf(stderr,"FAIL %s: %s\n",argc>1?argv[1]:"?",error.what()); return 1; }
}
