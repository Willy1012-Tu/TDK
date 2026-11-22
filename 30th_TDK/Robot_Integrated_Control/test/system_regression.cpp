#include "system_test_support.h"

using namespace SystemSupport;
constexpr int32_t stage2TrimLimit=SystemConfig::Rc::STAGE2_TRIM_MAX_OFFSET_PULSES;

// Derive the first powered cylinder segment from the current field-tuned profile.
uint8_t firstStage2ExtensionStep() {
    for(uint8_t step=1;step<=Stage2::STEP_COUNT;++step) {
        const auto &plan=Stage2::STEPS[step-1];
        for(size_t i=0;i<plan.count;++i)
            if(plan.actions[i].kind==MechanismConfig::ActionKind::Cylinder &&
               plan.actions[i].cylinderAct==MechanismConfig::CylinderAction::Extend) return step;
    }
    throw std::runtime_error("Stage2 has no cylinder extension segment");
}




bool testRemappedChannelsSystem(const std::string &name) {
    if (name == "system_rc_ch10_stage2_without_clear") {
      readyRobot(); const unsigned clearAt=odriveCount(0x18), movesAt=mksCount(0xFE);
      button(12,false); button(10,true);
      CHECK(MechanismController::status().stage==2 && mechanismReady());
      CHECK(!CylinderController::status().active && CylinderController::status().fullyRetracted);
      completeMechanism();
      CHECK(MechanismController::status().completed==0 && MechanismController::status().progress==0);
      CHECK(!MechanismController::status().previousAvailable && mksCount(0xFE)==movesAt);
      CHECK(RcController::status().selectedStage==2 && odriveCount(0x18)==clearAt);
      CHECK(Mock::servo[4]==1300 && Mock::servo[9]==1800 && Mock::servo[6]==1400 && Mock::servo[3]==1580);
      CHECK(CylinderController::status().referenceKnown && CylinderController::status().referenceAction==MechanismConfig::CylinderAction::Retract);
      assertCylinderStopped(); advance(100); CHECK(odriveCount(0x18)==clearAt);
    } else if (name == "system_rc_ch9_clear_without_mechanism_motion") {
        readyRobot(); const unsigned clearAt = odriveCount(0x18), movesAt = mksCount(0xFE);
        const auto profileAt = MechanismController::status().planVersion;
        const size_t sentAt = Mock::sent.size(); button(9, true);
        CHECK(odriveCount(0x18, sentAt) == 4 && odriveCount(0x18) == clearAt + 4);
        CHECK(mksCount(0xFE) == movesAt && MechanismController::status().planVersion == profileAt);
        CHECK(MechanismController::status().stage == 4 && RcController::status().selectedStage == 4);
        CHECK(mechanismReady() && MechanismController::status().completed == 0);
        bool cleared[4]{};
        for (size_t i = sentAt; i < Mock::sent.size(); ++i) {
            const auto &frame = Mock::sent[i];
            if (mksFrame(frame) || (frame.msg.id & 31) != 0x18) continue;
            CHECK(frame.msg.len == 0);
            for (uint8_t axis = 0; axis < 4; ++axis)
                if (frame.bus == SystemConfig::Chassis::CAN_BUS[axis] &&
                    (frame.msg.id >> 5) == SystemConfig::Chassis::NODE_ID[axis]) cleared[axis] = true;
        }
        for (bool sent : cleared) CHECK(sent);
        advance(100); CHECK(odriveCount(0x18) == clearAt + 4 && mksCount(0xFE) == movesAt);
    } else return false;
    return true;
}


// Exercise the actual RC edge/debounce logic and cooperative main loop across repeated rounds.
// Manual completion keeps both the original step and outgoing reset independently observable.
void exerciseRcStage3Return(uint8_t destination, bool retarget) {
    readyRobot();
    CHECK(stage3Motor(1,4).position != 0 && stage3Motor(1,5).position != 0);
    unsigned expectedHomes = mksCount(0x91);
    for (unsigned round = 0; round < 6; ++round) {
        const uint8_t finalStage = retarget ? static_cast<uint8_t>(round % 2 ? 4 : 2) : destination;
        const uint8_t otherStage = finalStage == 2 ? 4 : 2;
        const bool leavingStage1 = MechanismController::status().stage==1;
        autoCompleteMoves = true; selectRcStage(3); completeMechanism();
        if (leavingStage1) ++expectedHomes;
        CHECK(mksCount(0x91)==expectedHomes);
        CHECK(MechanismController::status().stage == 3 && RcController::status().selectedStage == 3);
        CHECK(MechanismController::status().completed == 0 && MechanismController::status().progress == 0);
        CHECK(MechanismController::status().position4 == 0 && MechanismController::status().position5 == 0);
        CHECK(Mock::servo[3] == 2250 && Mock::servo[9] == 1100);
        const unsigned moves4 = mksCount(0xFE,4), moves5 = mksCount(0xFE,5);
        const unsigned moves6 = mksCount(0xFE,6), moves7 = mksCount(0xFE,7);
        autoCompleteMoves = false; nextRcPulse();
        CHECK(mechanismBusy() && mksCount(0xFE,4) == moves4 + 1 && mksCount(0xFE,5) == moves5 + 1);
        CHECK(target(4) == stage3Motor(1,4).position && target(5) == stage3Motor(1,5).position);
        selectRcStage(retarget ? otherStage : finalStage);
        if (retarget) selectRcStage(finalStage);
        CHECK(MechanismController::status().stage == 3 && MechanismController::status().pendingStage == finalStage);
        CHECK(RcController::status().selectedStage == finalStage && std::string(MechanismController::status().phase) == "busy");
        sticks(SystemConfig::Chassis::FORWARD.maximum, SystemConfig::Chassis::STRAFE.middle, SystemConfig::Chassis::ROTATE.middle);
        advance(15); assertSteeringCadence(finalStage,1,0,0);
        const uint8_t first = round % 2 ? 5 : 4, second = first == 4 ? 5 : 4;
        finish(first);
        CHECK(MechanismController::status().stage == 3 && std::string(MechanismController::status().phase) == "busy");
        CHECK(mksCount(0xFE,4) == moves4 + 1 && mksCount(0xFE,5) == moves5 + 1);
        if (retarget) selectRcStage(otherStage);
        finish(second);
        // Stage3 may still be waiting for its configured parallel servo ramp.
        for (const auto &a : Stage3::YAML_STEP_1)
            if (a.kind == MechanismConfig::ActionKind::Servo) advance(a.ms);
        CHECK(MechanismController::status().stage == 3 && std::string(MechanismController::status().phase) == "return_wait");
        CHECK(MechanismController::status().completed == 0 && MechanismController::status().progress == 0);
        CHECK(target(4) == 0 && target(5) == 0 && Mock::servo[3] == 2250);
        CHECK(mksCount(0xFE,4) == moves4 + 2 && mksCount(0xFE,5) == moves5 + 2);
        for (uint8_t node : {uint8_t(4),uint8_t(5)}) {
            const auto frame = lastMove(node);
            CHECK(frame.len == 8 && frame.buf[1] == 1 && frame.buf[2] == 64 && frame.buf[3] == 0);
        }
        if (retarget) selectRcStage(finalStage);
        // A navigation press while returning is consumed; keeping it high cannot launch STEP1.
        button(SystemConfig::Rc::NEXT_CHANNEL,true); assertSteeringCadence(finalStage,1,0,0);
        finish(second);
        CHECK(MechanismController::status().stage == 3 && MechanismController::status().pendingStage == finalStage);
        CHECK(std::string(MechanismController::status().phase) == "return_wait" && !mechanismReady());
        if (retarget) { selectRcStage(otherStage); selectRcStage(finalStage); }
        CHECK(mksCount(0xFE,6) == moves6 && mksCount(0xFE,7) == moves7);
        assertSteeringCadence(finalStage,1,0,0); finish(first); completeMechanism();
        CHECK(mechanismReady() && MechanismController::status().stage == finalStage);
        CHECK(RcController::status().selectedStage == finalStage && MechanismController::status().pendingStage == 0);
        CHECK(MechanismController::status().completed == 0 && MechanismController::status().progress == 0 && !MechanismController::status().previousAvailable);
        CHECK(MechanismController::status().position4 == 0 && MechanismController::status().position5 == 0);
        CHECK(mksCount(0xFE,6) == moves6 && mksCount(0xFE,7) == moves7 && mksCount(0x91) == expectedHomes);
        advance(15); assertSteeringCadence(finalStage,1,0,0);
        CHECK(MechanismController::status().completed == 0);
        autoCompleteMoves = true; nextRcPulse(); completeMechanism();
        CHECK(MechanismController::status().completed == 1 && MechanismController::status().progress == 1);
        if (finalStage==2) CHECK(mksCount(0xFE,7)==0 && CylinderController::status().referenceAction==MechanismConfig::CylinderAction::Retract);
        else CHECK(mksCount(0xFE,6)==moves6+1);
        assertSteeringCadence(finalStage,1,0,0);
        CHECK(mksCount(0x91,4) == 0 && mksCount(0x91,5) == 0 && mksCount(0x91) == expectedHomes);
    }
}

bool testRcStage3Endurance(const std::string &name) {
    if (name == "rc_stage3_to_stage1_endurance") exerciseRcStage3Return(1,false);
    else if (name == "rc_stage3_to_stage2_endurance") exerciseRcStage3Return(2,false);
    else if (name == "rc_stage3_busy_retarget_endurance") exerciseRcStage3Return(1,true);
    else return false;
    return true;
}

bool testStage1PropsSystem(const std::string &name) {
    if (name == "rc_stage1_all_props_exact") {
        readyRobot();
        for (uint8_t prop=1;prop<=5;++prop) {
            selectRcStage(1); completeMechanism();
            CHECK(MechanismController::status().prop==2 && MechanismController::status().completed==0);
            const unsigned moves=mksCount(0xFE,6), homes=mksCount(0x91,6);
            autoCompleteMoves=false; propRcPulse(prop);
            const auto started=MechanismController::status();
            CHECK(started.stage==1 && started.prop==prop && started.steps==propStepCount(prop));
            CHECK(RcController::status().stage1Mode && RcController::status().selectedProp==prop);
            CHECK(mksCount(0xFE,6)==moves+1 && target(6)==propMotor(prop,1).position);
            const auto move=lastMove(6);
            CHECK((uint16_t(move.buf[1]) << 8 | move.buf[2]) == propMotor(prop,1).rpm && move.buf[3] == propMotor(prop,1).acc);
            CHECK(Mock::servo[2]==propPulse(prop,1,2) && Mock::servo[5]==propPulse(prop,1,5));
            sticks(SystemConfig::Chassis::FORWARD.maximum,SystemConfig::Chassis::STRAFE.middle,SystemConfig::Chassis::ROTATE.middle);
            advance(15); assertSteeringCadence(1,1,0,0); finish(6);
            CHECK(mechanismReady() && MechanismController::status().completed==1);
            CHECK(!MechanismController::status().propComplete);
            autoCompleteMoves=true;
            nextRcPulse(); completeMechanism();
            CHECK(MechanismController::status().completed==2);
            if (prop==1) {
                const auto lift=propMotor(prop,2);
                CHECK(MechanismController::status().position6==lift.position && target(6)==lift.position);
                CHECK(lastMove(6).buf[1]==1 && lastMove(6).buf[2]==244 && lastMove(6).buf[3]==lift.acc);
                CHECK(Mock::servo[2]==propPulse(prop,2,2) && Mock::servo[5]==propPulse(prop,2,5));
                CHECK(!MechanismController::status().propComplete);
                const unsigned liftMoves=mksCount(0xFE,6); nextRcPulse(); completeMechanism();
                CHECK(MechanismController::status().completed==3 && Mock::servo[2]==propPulse(prop,3,2));
                CHECK(mksCount(0xFE,6)==liftMoves && MechanismController::status().position6==lift.position);
            } else {
                if (prop==2) {
                    CHECK(Mock::servo[2]==2111 && MechanismController::status().position6==8000);
                    nextRcPulse(); completeMechanism();
                    CHECK(MechanismController::status().completed==3 && MechanismController::status().position6==propMotor(2,3).position);
                    CHECK(Mock::servo[2]==1389 && lastMove(6).buf[1]==3 && lastMove(6).buf[2]==232 && lastMove(6).buf[3]==245);
                } else if (prop==3) {
                    CHECK(MechanismController::status().position6==0 && target(6)==0);
                    CHECK(lastMove(6).buf[1]==3 && lastMove(6).buf[2]==232 && lastMove(6).buf[3]==255);
                    CHECK(Mock::servo[2]==1667);
                } else {
                    const int grip=propPulse(prop,2,2);
                    CHECK(Mock::servo[2]==grip && MechanismController::status().position6==propMotor(prop,1).position && !MechanismController::status().propComplete);
                    nextRcPulse(); completeMechanism();
                    CHECK(MechanismController::status().completed==3 && !MechanismController::status().propComplete);
                    CHECK(MechanismController::status().position6==propMotor(prop,3).position && Mock::servo[2]==grip);
                    CHECK((uint16_t(lastMove(6).buf[1]) << 8 | lastMove(6).buf[2]) == propMotor(prop,3).rpm && lastMove(6).buf[3] == propMotor(prop,3).acc);
                    const unsigned liftMoves=mksCount(0xFE,6);
                    nextRcPulse(); completeMechanism();
                    CHECK(MechanismController::status().completed==4 && MechanismController::status().position6==propMotor(prop,3).position && Mock::servo[2]==1667);
                    CHECK(mksCount(0xFE,6)==liftMoves);
                }
            }
            CHECK(MechanismController::status().propComplete && MechanismController::status().progress==propStepCount(prop));
            const unsigned completeMoves=mksCount(0xFE,6);
            nextRcPulse(); advance(90);
            CHECK(mechanismReady() && MechanismController::status().completed==propStepCount(prop));
            CHECK(mksCount(0xFE,6)==completeMoves && Mock::servo[5]==propPulse(prop,1,5));
            if (prop==5) writeStatusFixture(".pio/host-tests/stage1_props_status_fixture.json",request("status"));
            stage1Switch(1000); completeMechanism();
            CHECK(mechanismReady() && MechanismController::status().stage==4 && MechanismController::status().completed==0);
            CHECK(mksCount(0x91,6)==homes+1 && MechanismController::status().position6==0);
            CHECK(Mock::servo[2]==1150 && Mock::servo[5]==2100);
        }
        CHECK(mksCount(0x91,4)==0 && mksCount(0x91,5)==0 && mksCount(0x91,7)==0);
    } else if (name == "rc_stage1_chicken_lift_next_not_queued" || name == "rc_stage1_bird_lift_next_not_queued") {
        const uint8_t prop=name == "rc_stage1_chicken_lift_next_not_queued" ? 4 : 5;
        const int grip=propPulse(prop,2,2);
        readyRobot(); selectRcStage(1); completeMechanism(); propRcPulse(prop); completeMechanism();
        nextRcPulse(); completeMechanism(); CHECK(MechanismController::status().completed==2 && Mock::servo[2]==grip);
        autoCompleteMoves=false; nextRcPulse();
        CHECK(mechanismBusy() && target(6)==propMotor(prop,3).position && Mock::servo[2]==grip && !MechanismController::status().propComplete);
        sticks(SystemConfig::Chassis::FORWARD.maximum,SystemConfig::Chassis::STRAFE.middle,SystemConfig::Chassis::ROTATE.middle);
        nextRcPulse(); assertSteeringCadence(1,1,0,0); finish(6); advance(120);
        CHECK(mechanismReady() && MechanismController::status().completed==3 && !MechanismController::status().propComplete && Mock::servo[2]==grip);
        const unsigned moves=mksCount(0xFE,6); nextRcPulse(); completeMechanism();
        CHECK(MechanismController::status().completed==4 && MechanismController::status().propComplete && Mock::servo[2]==1667 && mksCount(0xFE,6)==moves);
        button(SystemConfig::Rc::PREV_CHANNEL,false); button(SystemConfig::Rc::PREV_CHANNEL,true); completeMechanism();
        CHECK(MechanismController::status().completed==3 && !MechanismController::status().propComplete && MechanismController::status().position6==propMotor(prop,3).position && Mock::servo[2]==grip);
        nextRcPulse(); completeMechanism(); CHECK(MechanismController::status().propComplete && Mock::servo[2]==1667 && mksCount(0xFE,6)==moves);
    } else if (name == "rc_stage1_low_waits_step_and_home" || name == "rc_stage1_low_home_latest_normal") {
        const bool retarget=name=="rc_stage1_low_home_latest_normal";
        readyRobot(); selectRcStage(1); completeMechanism();
        autoCompleteMoves=false; autoCompleteHomes=false; propRcPulse(2);
        const unsigned homes=mksCount(0x91,6), moves=mksCount(0xFE);
        stage1Switch(1000);
        CHECK(MechanismController::status().stage==1 && MechanismController::status().pendingStage==4);
        CHECK(std::string(MechanismController::status().phase)=="busy" && mksCount(0x91,6)==homes);
        sticks(SystemConfig::Chassis::FORWARD.maximum,SystemConfig::Chassis::STRAFE.middle,SystemConfig::Chassis::ROTATE.middle);
        advance(15); assertSteeringCadence(4,1,0,0); finish(6);
        CHECK(MechanismController::status().stage==1 && std::string(MechanismController::status().phase)=="home_wait");
        CHECK(mksCount(0x91,6)==homes+1 && mksCount(0xFE)==moves);
        CHECK(Mock::servo[2]==1150 && Mock::servo[5]==2100);
        if (retarget) {
            button(10,true); button(13,true); button(10,false); button(10,true);
            CHECK(MechanismController::status().pendingStage==2 && RcController::status().selectedStage==2);
        }
        button(SystemConfig::Rc::NEXT_CHANNEL,true);
        assertSteeringCadence(retarget ? 2 : 4,1,0,0);
        incomingMks(6,0x91,2,2); incomingMks(6,0x91,2,1,true); tick();
        CHECK(MechanismController::status().stage==1 && !mechanismReady());
        finish(6,0x91); completeMechanism(); advance(15);
        const uint8_t destination=retarget ? 2 : 4;
        CHECK(mechanismReady() && MechanismController::status().stage==destination && MechanismController::status().position6==0);
        CHECK(MechanismController::status().completed==0 && MechanismController::status().progress==0 && !MechanismController::status().previousAvailable);
        CHECK(mksCount(0xFE)==moves && mksCount(0x91,6)==homes+1 && mksCount(0x91,7)==0);
        assertSteeringCadence(destination,1,0,0); CHECK(MechanismController::status().completed==0);
        autoCompleteMoves=true; nextRcPulse(); completeMechanism();
        CHECK(MechanismController::status().completed==1);
        if (destination==2) CHECK(CylinderController::status().referenceAction==MechanismConfig::CylinderAction::Retract);
        else CHECK(target(6)==stage4Position(1));
    } else if (name == "rc_stage1_mid_and_transition_held") {
        readyRobot(); const unsigned moves=mksCount(0xFE);
        stage1Switch(2000); completeMechanism();
        CHECK(MechanismController::status().stage==1 && MechanismController::status().prop==2 && MechanismController::status().completed==0);
        CHECK(mksCount(0xFE)==moves); // CH12 is still held from normal Stage4; it cannot trigger tube.
        stage1Switch(1500); advance(90); stage1Switch(2000);
        CHECK(RcController::status().stage1Mode && MechanismController::status().completed==0 && mksCount(0xFE)==moves);
        propRcPulse(3); completeMechanism(); CHECK(MechanismController::status().prop==3 && MechanismController::status().completed==1);
        stage1Switch(1500); advance(90);
        CHECK(RcController::status().stage1Mode && MechanismController::status().prop==3);
        const unsigned homes=mksCount(0x91,6); stage1Switch(1000); completeMechanism();
        CHECK(MechanismController::status().stage==4 && mksCount(0x91,6)==homes+1);
        advance(90); CHECK(MechanismController::status().stage==4 && MechanismController::status().completed==0);
        button(10,true); completeMechanism(); CHECK(MechanismController::status().stage==2 && MechanismController::status().completed==0);
    } else if (name == "rc_stage1_mid_boot_no_prop") {
        initialize(); radio().channels[7]=1500; completeMechanism();
        CHECK(mechanismReady() && ChassisController::status().mode==ChassisController::Mode::Running);
        CHECK(RcController::status().switchPosition==1 && RcController::status().selectedStage==0 && mksCount(0xFE)==0);
        button(10,true); stage1Switch(2000); completeMechanism();
        CHECK(MechanismController::status().stage==1 && MechanismController::status().prop==2 && MechanismController::status().completed==0 && mksCount(0xFE)==0);
        propRcPulse(1); completeMechanism();
        CHECK(MechanismController::status().prop==1 && MechanismController::status().completed==1 && !MechanismController::status().propComplete && Mock::servo[2]==propPulse(1,1,2));
        nextRcPulse(); completeMechanism(); CHECK(!MechanismController::status().propComplete && MechanismController::status().position6==propMotor(1,2).position);
        nextRcPulse(); completeMechanism(); CHECK(MechanismController::status().propComplete && Mock::servo[2]==propPulse(1,3,2));
    } else if (name == "rc_stage1_busy_prop_consumed") {
        readyRobot(); selectRcStage(1); completeMechanism(); autoCompleteMoves=false; propRcPulse(2);
        button(10,true); CHECK(MechanismController::status().prop==2 && MechanismController::status().pendingProp==0);
        const unsigned moves=mksCount(0xFE,6); finish(6); advance(120);
        CHECK(mechanismReady() && MechanismController::status().prop==2 && MechanismController::status().completed==1 && mksCount(0xFE,6)==moves);
        propRcPulse(1); completeMechanism();
        CHECK(MechanismController::status().prop==1 && MechanismController::status().completed==1 && !MechanismController::status().propComplete);
        CHECK(Mock::servo[2]==propPulse(1,1,2) && mksCount(0x91,6)==1);
        nextRcPulse(); completeMechanism(); CHECK(!MechanismController::status().propComplete && MechanismController::status().position6==propMotor(1,2).position);
        nextRcPulse(); completeMechanism(); CHECK(MechanismController::status().propComplete && Mock::servo[2]==propPulse(1,3,2));
    } else if (name == "rc_stage1_simultaneous_props_reject") {
        readyRobot(); selectRcStage(1); completeMechanism();
        button(10,false); button(11,false); const unsigned moves=mksCount(0xFE);
        const auto rejected=RcController::status().rejectedActions;
        radio().channels[10]=radio().channels[11]=2000; tick(); advance(SystemConfig::Rc::DEBOUNCE_MS);
        CHECK(RcController::status().rejectedActions==rejected+1 && mksCount(0xFE)==moves);
        CHECK(MechanismController::status().prop==2 && MechanismController::status().completed==0);
    } else if (name == "rc_stage1_low_home_stop") {
        readyRobot(); selectRcStage(1); completeMechanism(); propRcPulse(5); completeMechanism();
        autoCompleteHomes=false; stage1Switch(1000);
        CHECK(std::string(MechanismController::status().phase)=="home_wait" && MechanismController::status().stage==1);
        const size_t stoppedAt=Mock::sent.size(); request("stop"); assertEmergencyStopped(stoppedAt);
        const unsigned moves=mksCount(0xFE), homes=mksCount(0x91);
        incomingMks(6,0x91,1); incomingMks(6,0x91,2); advance(500);
        CHECK(SystemSafety::stopped() && mksCount(0xFE)==moves && mksCount(0x91)==homes && MechanismController::status().pendingStage==0);
    } else if (name == "stage5_standby_wire") {
        readyRobot(); const unsigned moves=mksCount(0xFE), homes=mksCount(0x91);
        selectRcStage(5); completeMechanism();
        CHECK(MechanismController::status().stage==5 && MechanismController::status().prop==0 && MechanismController::status().steps==1);
        CHECK(MechanismController::status().completed==0 && RcController::status().selectedStage==5);
        nextRcPulse(); completeMechanism();
        CHECK(MechanismController::status().completed==1 && !MechanismController::status().propComplete);
        CHECK(mksCount(0xFE)==moves && mksCount(0x91)==homes);
        sticks(SystemConfig::Chassis::FORWARD.maximum,SystemConfig::Chassis::STRAFE.middle,SystemConfig::Chassis::ROTATE.middle);
        advance(15); assertSteeringCadence(5,1,0,0);
        writeStatusFixture(".pio/host-tests/stage5_status_fixture.json",request("status"));
    } else if (name == "wire_stage1_ram_prop_identity") {
        readyRobot();
        for (uint8_t prop=1;prop<=5;++prop) {
            request("plan.begin 1 1 1 1 "+std::to_string(prop));
            request("plan.action 0 0 "+std::to_string(123*prop)+" 320 0 0 0 0 1 1 6");
            request("plan.step 0 0 1"); request("plan.commit"); completeMechanism();
            CHECK(MechanismController::status().stage==1 && MechanismController::status().prop==prop && MechanismController::status().runtimePlan);
            CHECK(MechanismController::status().completed==0 && !MechanismController::status().propComplete);
            request("next"); completeMechanism();
            CHECK(MechanismController::status().position6==123*prop && MechanismController::status().propComplete);
            request("next",false);
        }
        request("stage 1 1"); completeMechanism(); request("next"); completeMechanism();
        CHECK(MechanismController::status().prop==1 && MechanismController::status().position6==123 && MechanismController::status().planVersion==5);
        writeStatusFixture(".pio/host-tests/stage1_ram_status_fixture.json",request("status"));
        const unsigned version=MechanismController::status().planVersion;
        request("stage 2 1",false); request("stage 1 6",false); request("prop 0",false); request("prop 6",false);
        CHECK(MechanismController::status().prop==1 && MechanismController::status().planVersion==version);
    } else return false;
    return true;
}

void assertPwmWindow(size_t start, uint32_t samples) {
    CHECK(Mock::pwmWrites.size() == start + samples);
    int minimum = 55, maximum = 0;
    for (size_t i = start; i < Mock::pwmWrites.size(); ++i) {
        const auto &event = Mock::pwmWrites[i];
        CHECK(event.pin == 14 && event.value >= 0 && event.value <= 55);
        if (i > start) {
            // Only compare breathing samples in this window; prior RC-Off writes
            // need not share its five-millisecond cadence.
            CHECK(uint32_t(event.at - Mock::pwmWrites[i - 1].at) == 5);
            CHECK(std::abs(event.value - Mock::pwmWrites[i - 1].value) == 1);
        }
        minimum = std::min(minimum, event.value); maximum = std::max(maximum, event.value);
    }
    if (samples >= 110) CHECK(minimum == 0 && maximum == 55);
    CHECK(Mock::pwmResolution == 8 && Mock::pwmDuty[14] == PwmController::currentValue());
    for (const auto &event : Mock::gpioWrites) CHECK(event.pin != 14);
    assertCylinderGpioHistory();
}
// CH13 owns only the independent Pin14 light in General context. The real loop
// must still service mechanisms and 15ms chassis traffic while it flashes.
bool testCh13StrobeSystem(const std::string &name) {
    if(name=="ch13_strobe_boot_off_then_held_high_during_startup") {
        initialize();
        CHECK(PwmController::currentValue()==0 && !PwmController::status().strobeEnabled);
        CHECK(PwmController::status().rcControlled);
        radio().channels[13]=2000; advance(150);
        CHECK(!mechanismReady() && RcController::status().lightStrobeEnabled);
        CHECK(PwmController::status().strobeEnabled && PwmController::status().rcControlled);
        CHECK(MechanismController::status().stage==4 && MechanismController::status().pendingStage==0);
        CHECK(CylinderController::status().active && !SystemSafety::stopped());
        const auto writes=Mock::pwmWrites.size(); advance(500);
        CHECK(Mock::pwmWrites.size()==writes+10);
        for(size_t i=writes;i<Mock::pwmWrites.size();++i) {
            CHECK(Mock::pwmWrites[i].pin==14);
            CHECK(Mock::pwmWrites[i].value==0 || Mock::pwmWrites[i].value==255);
            CHECK(Mock::pwmWrites[i].at-Mock::pwmWrites[i-1].at==50);
        }
    } else if(name=="ch13_strobe_debounce_hysteresis_no_stage5") {
        readyRobot(); const unsigned moves=mksCount(0xFE);
        const auto plan=MechanismController::status().planVersion;
        radio().channels[13]=2000; tick(); advance(SystemConfig::Rc::DEBOUNCE_MS-5);
        CHECK(!RcController::status().lightStrobeEnabled && PwmController::currentValue()==0);
        advance(5); CHECK(RcController::status().lightStrobeEnabled && PwmController::currentValue()==255);
        radio().channels[13]=1650; advance(150);
        CHECK(RcController::status().lightStrobeEnabled && MechanismController::status().stage==4);
        radio().channels[13]=1000; tick(); advance(SystemConfig::Rc::DEBOUNCE_MS-5);
        CHECK(RcController::status().lightStrobeEnabled);
        advance(5); CHECK(!RcController::status().lightStrobeEnabled && PwmController::currentValue()==0);
        const auto writes=Mock::pwmWrites.size(); advance(300);
        CHECK(Mock::pwmWrites.size()==writes && MechanismController::status().stage==4);
        CHECK(MechanismController::status().pendingStage==0 && MechanismController::status().planVersion==plan);
        CHECK(mksCount(0xFE)==moves && ChassisController::status().directionStage==4);
    } else if(name=="ch13_strobe_excluded_from_selectors_and_next") {
        readyRobot(); button(12,false); button(13,true);
        CHECK(MechanismController::status().stage==4 && MechanismController::status().pendingStage==0);
        button(10,true); completeMechanism();
        CHECK(MechanismController::status().stage==2 && RcController::status().selectedStage==2);
        CHECK(RcController::status().lightStrobeEnabled && ChassisController::status().directionStage==2);
        nextRcPulse(); completeMechanism(); CHECK(MechanismController::status().completed==1);
        button(10,false); button(14,true); completeMechanism();
        CHECK(MechanismController::status().stage==6 && RcController::status().selectedStage==6);
        CHECK(RcController::status().lightStrobeEnabled && ChassisController::status().directionStage==6);
    } else if(name=="ch13_strobe_invalid_light_does_not_block_other_selector") {
        readyRobot(); button(13,true); radio().channels[13]=3000; tick();
        CHECK(!RcController::status().lightStrobeEnabled && PwmController::currentValue()==0);
        CHECK(RcController::status().valid && !SystemSafety::stopped());
        button(10,true); completeMechanism();
        CHECK(MechanismController::status().stage==2 && RcController::status().selectedStage==2);
        CHECK(!PwmController::status().strobeEnabled);
        radio().channels[13]=2000; advance(100);
        CHECK(RcController::status().lightStrobeEnabled);
        radio().channels[SystemConfig::Chassis::ROTATE.number]=3000; tick();
        CHECK(!RcController::status().valid && !RcController::status().lightStrobeEnabled);
        CHECK(!PwmController::status().strobeEnabled && PwmController::currentValue()==0);
    } else if(name=="ch13_strobe_disconnect_forces_off") {
        readyRobot(); button(13,true); CHECK(RcController::status().lightStrobeEnabled);
        radio().linked=false; tick();
        CHECK(!RcController::status().lightStrobeEnabled && !PwmController::status().strobeEnabled);
        CHECK(PwmController::currentValue()==0 && PwmController::status().rcControlled);
        const auto writes=Mock::pwmWrites.size(); advance(500);
        CHECK(Mock::pwmWrites.size()==writes && !SystemSafety::stopped());
    } else if(name=="ch13_strobe_stage1_preserves_chicken_prop") {
        readyRobot(); button(13,true); CHECK(PwmController::status().strobeEnabled);
        stage1Switch(2000); completeMechanism();
        CHECK(MechanismController::status().stage==1 && RcController::status().stage1Mode);
        CHECK(!RcController::status().lightStrobeEnabled && PwmController::currentValue()==0);
        autoCompleteMoves=false; const unsigned moves=mksCount(0xFE,6); propRcPulse(4);
        CHECK(MechanismController::status().prop==4 && mksCount(0xFE,6)==moves+1);
        CHECK(target(6)==propMotor(4,1).position && mechanismBusy());
        CHECK(!PwmController::status().strobeEnabled && PwmController::currentValue()==0);
        finish(6); completeMechanism();
        CHECK(MechanismController::status().completed==1);
        CHECK(Mock::servo[2]==propPulse(4,1,2) && Mock::servo[5]==propPulse(4,1,5));
        advance(150); CHECK(mksCount(0xFE,6)==moves+1 && PwmController::currentValue()==0);
    } else if(name=="ch13_strobe_busy_keeps_chassis_15ms_and_stage") {
        readyRobot(); autoCompleteMoves=false; request("next");
        CHECK(mechanismBusy()); const unsigned moves=mksCount(0xFE);
        const auto step=MechanismController::status().step; button(13,true);
        CHECK(RcController::status().lightStrobeEnabled && mechanismBusy());
        CHECK(MechanismController::status().stage==4 && MechanismController::status().pendingStage==0);
        sticks(SystemConfig::Chassis::FORWARD.maximum,SystemConfig::Chassis::STRAFE.middle,SystemConfig::Chassis::ROTATE.middle);
        const auto writes=Mock::pwmWrites.size(); assertSteeringCadence(4,1,0,0,0,300);
        CHECK(Mock::pwmWrites.size()==writes+6 && mksCount(0xFE)==moves);
        CHECK(MechanismController::status().step==step && !SystemSafety::stopped());
        finish(6); completeMechanism(); CHECK(RcController::status().lightStrobeEnabled);
    } else if(name=="ch13_strobe_global_stop_keeps_independent_light") {
        readyRobot(); button(13,true); const size_t stoppedAt=Mock::sent.size();
        request("stop"); assertEmergencyStopped(stoppedAt);
        CHECK(RcController::status().lightStrobeEnabled && PwmController::status().strobeEnabled);
        const auto writes=Mock::pwmWrites.size(); advance(500);
        CHECK(Mock::pwmWrites.size()==writes+10 && SystemSafety::stopped());
        assertCylinderStopped(); button(13,false);
        CHECK(!PwmController::status().strobeEnabled && PwmController::currentValue()==0);
        CHECK(SystemSafety::stopped());
    } else if(name=="ch13_strobe_serial_stage5_and_json") {
        readyRobot(); button(13,true); request("stage 5"); completeMechanism();
        CHECK(MechanismController::status().stage==5 && PwmController::status().strobeEnabled);
        const auto wire=request("status");
        CHECK(wire.find("\"light_strobe_enabled\":true")!=std::string::npos);
        CHECK(wire.find("\"strobe_enabled\":true")!=std::string::npos);
        CHECK(wire.find("\"rc_controlled\":true")!=std::string::npos);
        CHECK(wire.find("\"strobe_interval_ms\":50")!=std::string::npos);
        CHECK(wire.find("\"strobe_value\":255")!=std::string::npos);
        CHECK(wire.find("\"rc_channel\":13")!=std::string::npos);
        CHECK(wire.size()<SystemConfig::USB_REPLY_BYTES);
        writeStatusFixture(".pio/host-tests/ch13_strobe_status_fixture.json",wire);
        button(13,false); const auto off=request("status");
        CHECK(off.find("\"light_strobe_enabled\":false")!=std::string::npos);
        CHECK(off.find("\"strobe_enabled\":false")!=std::string::npos);
    } else return false;
    return true;
}

bool testPwmSystem(const std::string &name) {
    if (name == "system_pwm_runs_during_mks_startup") {
        initialize();
        CHECK(PwmController::currentValue() == 0 && PwmController::status().rcControlled);
        advance(100); // Let General context claim RC-Off before explicitly testing GUI breathing.
        CHECK(PwmController::configure(55,5));
        CHECK(Mock::gpioMode[14] == OUTPUT && Mock::gpioMode[16] == OUTPUT && Mock::gpioMode[17] == OUTPUT);
        const size_t writes = Mock::pwmWrites.size(); advance(50);
        assertPwmWindow(writes, 10);
        CHECK(PwmController::currentValue() == 10 && !mechanismReady());
        CHECK(mksCount(0x8C) == 0 && mksCount(0x91) == 0);
        CHECK(CylinderController::status().active && CylinderController::status().action==MechanismConfig::CylinderAction::Retract);
        CHECK(Mock::gpioState[SystemConfig::Cylinder::IN1_PIN]==LOW && Mock::gpioState[SystemConfig::Cylinder::IN2_PIN]==HIGH);
    } else if (name == "system_pwm_cylinder_and_chassis_15ms") {
        readyRobot(); CHECK(PwmController::configure(55,5)); selectRcStage(2); completeMechanism();
        request("mode debug"); request("debug.cylinder 1 6000"); advance(100);
        CHECK(CylinderController::status().active && Mock::gpioState[17] == HIGH && Mock::gpioState[16] == LOW);
        sticks(SystemConfig::Chassis::FORWARD.maximum, SystemConfig::Chassis::STRAFE.middle,
               SystemConfig::Chassis::ROTATE.middle);
        advance(15); const size_t writes = Mock::pwmWrites.size();
        assertSteeringCadence(2, 1, 0, 0, 0, 5100);
        assertPwmWindow(writes, 1020);
        CHECK(CylinderController::status().active && Mock::gpioState[17] == HIGH && Mock::gpioState[16] == LOW);
        for (float wheel : ChassisController::status().wheel) CHECK(floatClose(fabsf(wheel), 90.0f));
        CHECK(!SystemSafety::stopped());
    } else if (name == "system_pwm_continues_after_global_stop") {
        readyRobot(); CHECK(PwmController::configure(55,5)); request("mode debug"); request("debug.cylinder 1 6000"); advance(100);
        CHECK(Mock::gpioState[17] == HIGH && Mock::gpioState[16] == LOW);
        const size_t stoppedAt = Mock::sent.size(); request("stop"); assertEmergencyStopped(stoppedAt);
        assertCylinderStopped(); const auto value = PwmController::currentValue();
        const size_t writes = Mock::pwmWrites.size(); advance(550);
        assertPwmWindow(writes, 110);
        CHECK(PwmController::currentValue() == value && SystemSafety::stopped());
        assertCylinderStopped();
        for (float wheel : ChassisController::status().wheel) CHECK(wheel == 0);
    } else if (name == "system_pwm_gateway_status_defaults") {
        readyRobot(); const auto wire=request("hello");
        CHECK(wire.find("\"pwm\":{\"available\":true,\"pin\":14,\"value\":")!=std::string::npos);
        CHECK(wire.find("\"max_value\":55,\"step_interval_ms\":5,\"default_max_value\":55,\"default_step_interval_ms\":5,\"min_step_interval_ms\":1,\"max_step_interval_ms\":1000")!=std::string::npos);
        CHECK(wire.size()<SystemConfig::USB_REPLY_BYTES);
        writeStatusFixture(".pio/host-tests/pwm_default_status_fixture.json",wire);
    } else if (name == "system_pwm_gateway_strict_validation") {
        readyRobot(); request("pwm.set 200 20");
        CHECK(PwmController::status().maxValue==200 && PwmController::status().stepIntervalMs==20);
        for(const char *command : {"pwm.set", "pwm.set 55", "pwm.set 55 5 extra", "pwm.set -1 5", "pwm.set 256 5", "pwm.set 55 0", "pwm.set 55 1001", "pwm.set 55.0 5", "pwm.set 55 5.0", "pwm.set true 5", "pwm.set NaN 5", "pwm.set +55 5", "pwm.set 0x37 5", "pwm.set 9999999999999999999999999999999 5", "pwm.set 55 4294967295"}) {
            request(command,false); const auto state=PwmController::status();
            CHECK(state.maxValue==200 && state.stepIntervalMs==20 && state.value<=200);
        }
        request("pwm.set 0 1"); CHECK(PwmController::currentValue()==0 && PwmController::status().maxValue==0);
        request("pwm.set 255 1000"); CHECK(PwmController::status().maxValue==255 && PwmController::status().stepIntervalMs==1000);
        CHECK(!SystemSafety::stopped() && mechanismReady());
    } else if (name == "system_pwm_gateway_hot_tuning_chassis_cylinder") {
        readyRobot(); selectRcStage(2); completeMechanism(); request("mode debug"); request("debug.cylinder 1 6000"); advance(100);
        sticks(SystemConfig::Chassis::FORWARD.maximum,SystemConfig::Chassis::STRAFE.middle,SystemConfig::Chassis::ROTATE.middle); advance(15);
        CHECK(mechanismBusy() && CylinderController::status().active && Mock::gpioState[17]==HIGH && Mock::gpioState[16]==LOW);
        request("pwm.set 120 10"); const size_t first=Mock::pwmWrites.size();
        assertSteeringCadence(2,1,0,0,0,1800);
        CHECK(Mock::pwmWrites.size()==first+180 && PwmController::status().maxValue==120 && PwmController::status().stepIntervalMs==10);
        for(size_t i=first;i<Mock::pwmWrites.size();++i) {
            CHECK(Mock::pwmWrites[i].pin==14 && Mock::pwmWrites[i].value>=0 && Mock::pwmWrites[i].value<=120);
            if(i>first) CHECK(Mock::pwmWrites[i].at-Mock::pwmWrites[i-1].at==10);
        }
        CHECK(CylinderController::status().active && mechanismBusy() && !SystemSafety::stopped());
        writeStatusFixture(".pio/host-tests/pwm_live_status_fixture.json",request("status"));
        request("pwm.set 10 5"); CHECK(PwmController::currentValue()<=10);
        request("pwm.set 0 5"); CHECK(PwmController::currentValue()==0);
        const size_t disabled=Mock::pwmWrites.size(); assertSteeringCadence(2,1,0,0,0,500);
        CHECK(Mock::pwmWrites.size()==disabled && CylinderController::status().active && Mock::gpioState[17]==HIGH && Mock::gpioState[16]==LOW);
        request("pwm.set 55 5"); assertSteeringCadence(2,1,0,0,0,500);
        CHECK(PwmController::status().maxValue==55 && PwmController::currentValue()<=55 && CylinderController::status().active);
        assertCylinderGpioHistory();
    } else if (name == "system_pwm_gateway_config_after_latched_stop") {
        readyRobot(); request("mode debug"); request("debug.cylinder 1 6000"); advance(100);
        const size_t stoppedAt=Mock::sent.size(); request("stop"); assertEmergencyStopped(stoppedAt);
        request("pwm.set 255 1"); CHECK(PwmController::status().maxValue==255 && PwmController::status().stepIntervalMs==1);
        const size_t writes=Mock::pwmWrites.size(); advance(50); CHECK(Mock::pwmWrites.size()==writes+10);
        CHECK(SystemSafety::stopped() && std::string(MechanismController::status().phase)=="fault"); assertCylinderStopped();
        request("pwm.set 0 1000"); CHECK(PwmController::currentValue()==0);
        request("next",false); request("home",false); request("mode normal",false);
        CHECK(SystemSafety::stopped()); assertCylinderStopped();
        writeStatusFixture(".pio/host-tests/pwm_stopped_status_fixture.json",request("status"));
    } else if (name == "system_pwm_gateway_backpressure_stop_priority") {
        readyRobot(); request("mode debug"); request("debug.cylinder 1 6000"); advance(100); request("pwm.set 77 9");
        Serial.capacity=0; for(unsigned i=0;i<8;++i) queue("status"); advance(100);
        const unsigned tuningId=queue("pwm.set 11 5"),stopId=queue("stop"); const size_t stoppedAt=Mock::sent.size(); tick();
        assertEmergencyStopped(stoppedAt);
        CHECK(PwmController::status().maxValue==77 && PwmController::status().stepIntervalMs==9);
        Serial.capacity=4096; advance(600);
        CHECK(lineFor(tuningId).find(" ERR usb_backpressure")!=std::string::npos);
        CHECK(lineFor(stopId).find(" OK accepted")!=std::string::npos);
        request("pwm.set 88 10"); CHECK(PwmController::status().maxValue==88 && SystemSafety::stopped()); assertCylinderStopped();
    } else if (name == "system_pwm_gateway_status_extreme_fields_fit") {
        readyRobot(); request("mode debug");
        for(uint8_t node : SystemConfig::Mechanism::MOTOR_IDS) request("debug.move "+std::to_string(node)+" -8388608 3000 255");
        completeMechanism(); CHECK(PwmController::configure(255,1000));
        for(uint8_t i=0;i<4;++i) Mock::incoming[SystemConfig::Chassis::CAN_BUS[i]].push_back(heartbeat(i,255,UINT32_MAX));
        tick(5,false); radio().linked=false; for(int &channel : radio().channels) channel=INT32_MIN;
        mockMillis=UINT32_MAX-500U; CHECK(MechanismController::debugCylinder(MechanismConfig::CylinderAction::Extend,60000));
        advance(50,false); CHECK(CylinderController::status().active && Mock::gpioState[17]==HIGH && Mock::gpioState[16]==LOW);
        Serial.input+="@1000000000 status\n"; std::string wire;
        for(unsigned i=0;i<1000 && wire.empty();++i) { tick(5,false); wire=lineFor(1000000000); }
        CHECK(wire.find("@1000000000 OK {")==0 && wire.size()<SystemConfig::USB_REPLY_BYTES);
        CHECK(wire.find("\"error\":4294967295")!=std::string::npos && wire.find("\"position6\":-8388608")!=std::string::npos);
        CHECK(wire.find("\"max_value\":255,\"step_interval_ms\":1000")!=std::string::npos && !SystemSafety::stopped());
        writeStatusFixture(".pio/host-tests/pwm_extreme_status_fixture.json",wire);
    } else return false;
    return true;
}

// Full retraction is a timed estimate; shorter completed retracts used by STEP/PREV
// must never suppress a later full HOME2. Exercise the real setup/loop sequence.
unsigned cylinderEnergizations(uint8_t pin) {
    unsigned count=0;
    for (const auto &event : Mock::gpioWrites)
        if (event.pin==pin && event.value==HIGH) ++count;
    return count;
}
bool testStage2BootHome(const std::string &name) {
    using MechanismConfig::CylinderAction;
    using namespace SystemConfig::Cylinder;
    if (name == "cylinder_boot_retract_deadtime_ready_barrier") {
        initialize();
        CHECK(CylinderController::status().active && CylinderController::status().action==CylinderAction::Retract);
        CHECK(CylinderController::status().durationMs==HOME_RETRACT_MS && !CylinderController::status().fullyRetracted);
        CHECK(Mock::gpioState[IN1_PIN]==LOW && Mock::gpioState[IN2_PIN]==LOW);
        advance(REVERSAL_DEADTIME_MS-1);
        CHECK(Mock::gpioState[IN1_PIN]==LOW && Mock::gpioState[IN2_PIN]==LOW && !mechanismReady());
        tick(1);
        CHECK(Mock::gpioState[IN1_PIN]==LOW && Mock::gpioState[IN2_PIN]==HIGH);
        CHECK(CylinderController::status().remainingMs==HOME_RETRACT_MS);
        advance(HOME_RETRACT_MS-1);
        CHECK(!mechanismReady() && CylinderController::status().active && !CylinderController::status().fullyRetracted);
        tick(1); completeMechanism();
        CHECK(mockMillis<HOME_RETRACT_MS+REVERSAL_DEADTIME_MS+100);
        CHECK(CylinderController::status().fullyRetracted && cylinderEnergizations(IN2_PIN)==1);
        CHECK(cylinderEnergizations(IN1_PIN)==0);
        assertCylinderStopped();
    } else if (name == "cylinder_boot_mks_completes_first") {
        initialize(); advance(2200);
        CHECK(mksCount(0x91,6)==1 && MechanismController::status().positionKnownMask==7);
        CHECK(!mechanismReady() && CylinderController::status().active);
        CHECK(ChassisController::status().mode==ChassisController::Mode::Running);
        completeMechanism();
        CHECK(CylinderController::status().fullyRetracted && cylinderEnergizations(IN2_PIN)==1);
        CHECK(mksCount(0x91,6)==1 && mksCount(0x91,4)==0 && mksCount(0x91,5)==0);
    } else if (name == "cylinder_boot_mks_completes_later") {
        autoCompleteHomes=false; initialize(); advance(HOME_RETRACT_MS+REVERSAL_DEADTIME_MS+500);
        CHECK(!mechanismReady() && mksCount(0x91,6)==1);
        CHECK(CylinderController::status().fullyRetracted && !CylinderController::status().active);
        CHECK(cylinderEnergizations(IN2_PIN)==1);
        finish(6,0x91); completeMechanism();
        CHECK(mechanismReady() && CylinderController::status().fullyRetracted && cylinderEnergizations(IN2_PIN)==1);
    } else if (name == "cylinder_boot_stop_deadtime" || name == "cylinder_boot_stop_motion") {
        const bool moving=name=="cylinder_boot_stop_motion";
        initialize(); advance(moving ? 200U : 10U);
        CHECK(CylinderController::status().active);
        CHECK(Mock::gpioState[IN2_PIN]==(moving ? HIGH : LOW));
        request("stop");
        CHECK(SystemSafety::stopped() && !CylinderController::status().fullyRetracted);
        assertCylinderStopped(); const auto writes=Mock::gpioWrites.size();
        advance(HOME_RETRACT_MS+REVERSAL_DEADTIME_MS+100);
        assertCylinderStopped(); CHECK(!mechanismReady());
        for (size_t i=writes;i<Mock::gpioWrites.size();++i) CHECK(Mock::gpioWrites[i].value==LOW);
    } else if (name == "stage2_boot_entry_fast_next") {
        readyRobot(); CHECK(CylinderController::status().fullyRetracted);
        const auto before=cylinderEnergizations(IN2_PIN); const uint32_t selectedAt=mockMillis;
        selectRcStage(2);
        CHECK(mechanismReady() && MechanismController::status().stage==2 && uint32_t(mockMillis-selectedAt)<500);
        CHECK(cylinderEnergizations(IN2_PIN)==before && !CylinderController::status().active);
        CHECK(MechanismController::status().completed==0 && Mock::servo[4]==1300 && Mock::servo[6]==1400 && Mock::servo[9]==1800 && Mock::servo[3]==1580);
        nextRcPulse(); completeMechanism();
        CHECK(MechanismController::status().completed==1 && Mock::servo[4]==stage2Pulse(1,4) && Mock::servo[9]==stage2Pulse(1,9));
        CHECK(cylinderEnergizations(IN2_PIN)==before);
    } else if (name == "stage2_boot_partial_retract_requires_home") {
        readyRobot(); request("mode debug"); request("debug.cylinder 2 600"); completeMechanism();
        CHECK(CylinderController::status().referenceKnown && CylinderController::status().referenceAction==CylinderAction::Retract);
        CHECK(!CylinderController::status().fullyRetracted);
        request("mode normal"); request("stage 2");
        CHECK(!mechanismReady() && CylinderController::status().active && CylinderController::status().durationMs==HOME_RETRACT_MS);
        advance(100); CHECK(Mock::gpioState[IN2_PIN]==HIGH && Mock::gpioState[IN1_PIN]==LOW);
        completeMechanism(); CHECK(CylinderController::status().fullyRetracted && MechanismController::status().completed==0);
        nextRcPulse(); completeMechanism(); CHECK(MechanismController::status().completed==1);
    } else if (name == "stage2_boot_exit_safe_reentry_fast") {
        readyRobot(); request("stage 2"); completeMechanism();
        request("mode debug"); request("debug.cylinder 1 100"); completeMechanism();
        CHECK(!CylinderController::status().fullyRetracted);
        request("mode normal"); const auto before=cylinderEnergizations(IN2_PIN); request("stage 4");
        CHECK(!mechanismReady() && MechanismController::status().pendingStage==4);
        completeMechanism();
        CHECK(MechanismController::status().stage==4 && CylinderController::status().fullyRetracted);
        CHECK(cylinderEnergizations(IN2_PIN)==before+1);
        const uint32_t selectedAt=mockMillis; request("stage 2");
        CHECK(mechanismReady() && MechanismController::status().stage==2 && uint32_t(mockMillis-selectedAt)<100);
        CHECK(cylinderEnergizations(IN2_PIN)==before+1);
    } else if (name == "cylinder_boot_chassis_15ms_nonblocking") {
        initialize(); advance(1400);
        CHECK(ChassisController::status().mode==ChassisController::Mode::Running && !mechanismReady());
        sticks(SystemConfig::Chassis::FORWARD.maximum,SystemConfig::Chassis::STRAFE.middle,SystemConfig::Chassis::ROTATE.middle);
        advance(15); assertSteeringCadence(4,1,0,0,0,3500);
        CHECK(CylinderController::status().active && !mechanismReady());
        completeMechanism(); CHECK(CylinderController::status().fullyRetracted);
        CHECK(cylinderEnergizations(IN2_PIN)==1 && cylinderEnergizations(IN1_PIN)==0);
    } else return false;
    CHECK(!SystemSafety::stopped() || name=="cylinder_boot_stop_deadtime" || name=="cylinder_boot_stop_motion");
    return true;
}

bool testCylinderSystem(const std::string &name) {
    using MechanismConfig::CylinderAction;
    if (name == "cylinder_prepare_pins_before_runtime_and_repeat") {
        using namespace SystemConfig::Cylinder;
        mockMillis=37; Mock::gpioState[IN1_PIN]=HIGH; Mock::gpioState[IN2_PIN]=HIGH;
        Mock::gpioMode[IN1_PIN]=INPUT; Mock::gpioMode[IN2_PIN]=INPUT;
        CylinderController::preparePins();
        CHECK(mockMillis==37 && Mock::sent.empty() && Mock::servoWrites.empty() && Serial.beginBaud==0);
        CHECK(Mock::gpioState[IN1_PIN]==LOW && Mock::gpioState[IN2_PIN]==LOW);
        CHECK(Mock::gpioMode[IN1_PIN]==OUTPUT && Mock::gpioMode[IN2_PIN]==OUTPUT);
        CHECK(Mock::gpioModes.size()==2 && Mock::gpioModes[0].value==LOW && Mock::gpioModes[1].value==LOW);
        for(const auto &w:Mock::gpioWrites) CHECK(w.value==LOW);
        CylinderController::preparePins();
        CHECK(Mock::gpioModes.size()==4 && mockMillis==37);
        CHECK(!CylinderController::status().active && !CylinderController::status().referenceKnown);
    } else if (name == "cylinder_setup_claims_low_before_other_gpio") {
        using namespace SystemConfig::Cylinder;
        Mock::gpioState[IN1_PIN]=HIGH; Mock::gpioState[IN2_PIN]=HIGH;
        initialize();
        CHECK(Mock::gpioModes.size()>=3);
        CHECK(Mock::gpioModes[0].pin==IN1_PIN && Mock::gpioModes[1].pin==IN2_PIN);
        CHECK(Mock::gpioModes[0].value==LOW && Mock::gpioModes[1].value==LOW);
        CHECK(Mock::gpioWrites[0].pin==IN1_PIN && Mock::gpioWrites[1].pin==IN2_PIN);
        for(const auto &w:Mock::gpioWrites) CHECK(w.value==LOW);
        CHECK(Serial.beginBaud==SystemConfig::SERIAL_BAUD);
        CHECK(CylinderController::status().active && CylinderController::status().action==CylinderAction::Retract);
        CHECK(Mock::gpioState[IN1_PIN]==LOW && Mock::gpioState[IN2_PIN]==LOW);
        advance(500);
        CHECK(std::string(MechanismController::status().phase)=="booting");
        CHECK(Mock::gpioState[IN1_PIN]==LOW && Mock::gpioState[IN2_PIN]==HIGH);
        for(const auto &w:Mock::gpioWrites) if(w.value==HIGH) CHECK(w.pin==IN2_PIN);
        CHECK(!CylinderController::status().fullyRetracted);
    } else if (name == "cylinder_reinitialize_never_resumes_previous_motion") {
        using namespace SystemConfig::Cylinder;
        CylinderController::begin(); CHECK(CylinderController::start(CylinderAction::Extend,1000));
        mockMillis+=REVERSAL_DEADTIME_MS; CylinderController::poll();
        CHECK(Mock::gpioState[IN1_PIN]==HIGH);
        const auto from=Mock::gpioWrites.size();
        CylinderController::begin(); mockMillis+=5000; CylinderController::poll();
        assertCylinderStopped(); CHECK(!CylinderController::status().referenceKnown);
        for(size_t i=from;i<Mock::gpioWrites.size();++i) CHECK(Mock::gpioWrites[i].value==LOW);
        CHECK(Mock::gpioModes.back().value==LOW);
    } else if (name == "cylinder_safe_boot_gpio") {
        readyRobot();
        CHECK(Mock::gpioMode[SystemConfig::Cylinder::IN1_PIN]==OUTPUT && Mock::gpioMode[SystemConfig::Cylinder::IN2_PIN]==OUTPUT);
        for (const auto &event : Mock::gpioWrites) if(event.value==HIGH) CHECK(event.pin==SystemConfig::Cylinder::IN2_PIN);
        CHECK(CylinderController::status().referenceKnown && CylinderController::status().fullyRetracted);
        CHECK(cylinderEnergizations(SystemConfig::Cylinder::IN2_PIN)==1);
        CHECK(mksCount(0x91)==1 && MechanismController::status().positionKnownMask==7);
        assertCylinderStopped(); const auto wire=request("status");
        CHECK(wire.find("position7")==std::string::npos && wire.find("current7_ma")==std::string::npos);
    } else if (name == "cylinder_deadtime_full_duration_rollover") {
        CylinderController::begin(); mockMillis=UINT32_MAX-24;
        CHECK(CylinderController::start(CylinderAction::Extend,100));
        mockMillis+=49; CylinderController::poll();
        CHECK(CylinderController::status().active && Mock::gpioState[SystemConfig::Cylinder::IN1_PIN]==LOW && Mock::gpioState[SystemConfig::Cylinder::IN2_PIN]==LOW);
        mockMillis+=1; CylinderController::poll();
        CHECK(Mock::gpioState[SystemConfig::Cylinder::IN1_PIN]==HIGH && Mock::gpioState[SystemConfig::Cylinder::IN2_PIN]==LOW && CylinderController::status().remainingMs==100);
        mockMillis+=99; CylinderController::poll(); CHECK(CylinderController::status().active);
        mockMillis+=1; CylinderController::poll(); assertCylinderStopped();
        CHECK(CylinderController::status().referenceKnown && CylinderController::status().referenceAction==CylinderAction::Extend);
        CHECK(CylinderController::start(CylinderAction::Retract,80));
        mockMillis+=49; CylinderController::poll(); CHECK(Mock::gpioState[SystemConfig::Cylinder::IN1_PIN]==LOW && Mock::gpioState[SystemConfig::Cylinder::IN2_PIN]==LOW);
        mockMillis+=1; CylinderController::poll(); CHECK(Mock::gpioState[SystemConfig::Cylinder::IN1_PIN]==LOW && Mock::gpioState[SystemConfig::Cylinder::IN2_PIN]==HIGH);
        mockMillis+=79; CylinderController::poll(); CHECK(CylinderController::status().active);
        mockMillis+=1; CylinderController::poll(); assertCylinderStopped();
        CHECK(CylinderController::status().referenceAction==CylinderAction::Retract && CylinderController::status().referenceKnown);
        CHECK(CylinderController::start(CylinderAction::Stop,25));
        mockMillis+=24; CylinderController::poll(); CHECK(CylinderController::status().active && Mock::gpioState[SystemConfig::Cylinder::IN1_PIN]==LOW && Mock::gpioState[SystemConfig::Cylinder::IN2_PIN]==LOW);
        mockMillis+=1; CylinderController::poll(); assertCylinderStopped();
        CHECK(!CylinderController::start(CylinderAction::Extend,0));
        CHECK(!CylinderController::start(static_cast<CylinderAction>(3),100));
        CHECK(!CylinderController::start(CylinderAction::Retract,60001));
    } else if (name == "cylinder_rc_current_six_steps_chassis") {
        readyRobot(); selectRcStage(2);
        CHECK(MechanismController::status().stage==2 && mechanismReady() && CylinderController::status().fullyRetracted);
        advance(100); assertCylinderStopped();
        CHECK(MechanismController::status().completed==0);
        CHECK(CylinderController::status().referenceKnown && CylinderController::status().referenceAction==CylinderAction::Retract);
        nextRcPulse(); completeMechanism(); CHECK(MechanismController::status().completed==1 && !CylinderController::status().active);
        CHECK(Mock::servo[9]==stage2Pulse(1,9) && Mock::servo[4]==stage2Pulse(1,4));
        nextRcPulse(); completeMechanism(); CHECK(MechanismController::status().completed==2 && Mock::servo[9]==stage2Pulse(2,9) && Mock::servo[4]==stage2Pulse(2,4));
        nextRcPulse(); advance(100); CHECK(CylinderController::status().active && CylinderController::status().durationMs==3000);
        CHECK(Mock::gpioState[17]==HIGH && Mock::gpioState[16]==LOW); request("next",false); request("prev",false);
        button(6,false); button(6,true); // Busy press is consumed, even after the timer completes.
        sticks(SystemConfig::Chassis::FORWARD.maximum,SystemConfig::Chassis::STRAFE.middle,SystemConfig::Chassis::ROTATE.middle);
        assertSteeringCadence(2,1,0,0,0,2800); completeMechanism();
        CHECK(MechanismController::status().completed==3 && CylinderController::status().referenceAction==CylinderAction::Extend);
        advance(90); CHECK(MechanismController::status().completed==3);
        const int feedPulse=stage2Pulse(4,6);
        nextRcPulse(); advance(500); CHECK(!mechanismReady() && Mock::servo[6]>feedPulse && Mock::servo[6]<1400);
        request("next",false); advance(600); CHECK(!mechanismReady() && Mock::servo[6]==feedPulse);
        // Current YAML holds the feed pose through its 1000ms delay; no return action is present.
        completeMechanism(); CHECK(MechanismController::status().completed==4 && Mock::servo[6]==feedPulse);
        nextRcPulse(); advance(100); CHECK(CylinderController::status().active && CylinderController::status().durationMs==3000);
        CHECK(Mock::gpioState[16]==HIGH && Mock::gpioState[17]==LOW);
        assertSteeringCadence(2,1,0,0,0,2800); completeMechanism();
        CHECK(MechanismController::status().completed==5 && CylinderController::status().referenceAction==CylinderAction::Retract);
        nextRcPulse(); advance(500); CHECK(!mechanismReady() && Mock::servo[4]>1300 && Mock::servo[4]<2100 && Mock::servo[9]>1000 && Mock::servo[9]<1800);
        completeMechanism(); CHECK(MechanismController::status().completed==6 && Mock::servo[4]==1300 && Mock::servo[9]==1800);
        nextRcPulse(); completeMechanism(); CHECK(MechanismController::status().completed==2 && Mock::servo[4]==2100 && Mock::servo[9]==1000);
        CHECK(mksCount(0xFE)==0 && mksCount(0x91)==1); assertCylinderStopped();
    } else if (name == "cylinder_stop_before_can_retry") {
        readyRobot(); request("mode debug"); request("debug.cylinder 1 6000"); advance(100);
        CHECK(Mock::gpioState[SystemConfig::Cylinder::IN1_PIN]==HIGH && Mock::gpioState[SystemConfig::Cylinder::IN2_PIN]==LOW);
        Serial.capacity=0; Mock::rejectNode=4; Mock::rejectCommand=0xF7;
        const size_t writes=Mock::gpioWrites.size(); queue("stop"); tick(1);
        CHECK(SystemSafety::stopped() && ChassisController::status().emergencyLatched);
        assertCylinderStopped(); CHECK(mksCount(0xF7,4)==0);
        CHECK(Mock::gpioWrites.size()>=writes+2);
        for (size_t i=writes;i<Mock::gpioWrites.size();++i) CHECK(Mock::gpioWrites[i].value==LOW);
        advance(200); assertCylinderStopped();
        Mock::rejectNode=-1; advance(50); CHECK(mksCount(0xF7,4)==1);
        for (size_t i=writes;i<Mock::gpioWrites.size();++i) CHECK(Mock::gpioWrites[i].value==LOW);
        for (uint8_t pin : SystemConfig::Mechanism::SERVO_PINS) CHECK(!Mock::servoAttached[pin]);
    } else if (name == "cylinder_switch_cancels_unissued_tail") {
        readyRobot(); request("plan.begin 1 5 2 1");
        request("plan.action 0 4 0 0 0 0 0 6000 0 1 0 1");
        request("plan.action 1 1 0 0 0 6 600 1000 0 1 0");
        request("plan.action 2 3 0 0 0 0 0 0 1 1 0");
        request("plan.action 3 2 0 0 0 0 0 1000 1 2 0");
        request("plan.action 4 1 0 0 0 4 1777 0 1 3 0");
        request("plan.step 0 0 5"); request("plan.commit"); completeMechanism();
        request("next"); advance(200); CHECK(Mock::gpioState[SystemConfig::Cylinder::IN1_PIN]==HIGH && Mock::servo[6]>600);
        const size_t writes=Mock::gpioWrites.size(); const uint32_t selectedAt=mockMillis;
        request("stage 3"); CHECK(Mock::gpioState[SystemConfig::Cylinder::IN1_PIN]==LOW && Mock::gpioState[SystemConfig::Cylinder::IN2_PIN]==LOW);
        CHECK(MechanismController::status().stage==2 && MechanismController::status().pendingStage==3);
        sticks(SystemConfig::Chassis::FORWARD.maximum,SystemConfig::Chassis::STRAFE.middle,SystemConfig::Chassis::ROTATE.middle);
        assertSteeringCadence(3,1,0,0,0,450); CHECK(Mock::gpioState[SystemConfig::Cylinder::IN1_PIN]==LOW && Mock::gpioState[SystemConfig::Cylinder::IN2_PIN]==LOW);
        advance(500); CHECK(CylinderController::status().active && CylinderController::status().durationMs==5000);
        bool returned=false;
        for (size_t i=writes;i<Mock::gpioWrites.size();++i) if (Mock::gpioWrites[i].pin==SystemConfig::Cylinder::IN2_PIN && Mock::gpioWrites[i].value==HIGH) {
            CHECK(uint32_t(Mock::gpioWrites[i].at-selectedAt)<1100); returned=true;
        }
        CHECK(returned); completeMechanism(); CHECK(MechanismController::status().stage==3 && MechanismController::status().completed==0);
        for (const auto &event : Mock::servoWrites) CHECK(!(event.pin==4 && event.pulse==1777));
        CHECK(Mock::servo[6]==1400 && Mock::servo[4]==2000 && Mock::servo[9]==1100);
    } else if (name == "cylinder_wire_nine_profile_banks") {
        readyRobot();
        const uint8_t stages[9]={1,1,1,1,1,2,3,4,5}, props[9]={1,2,3,4,5,0,0,0,0};
        for (uint8_t i=0;i<9;++i) {
            request("plan.begin 1 1 "+std::to_string(stages[i])+" 1 "+std::to_string(props[i]));
            request("plan.action 0 4 0 0 0 0 0 "+std::to_string(200+i)+" 1 1 0 0");
            request("plan.step 0 0 1"); request("plan.commit"); completeMechanism();
            request("next"); completeMechanism(); CHECK(MechanismController::status().planVersion==uint32_t(i+1));
        }
        for (uint8_t i=0;i<9;++i) {
            request("stage "+std::to_string(stages[i])+" "+std::to_string(props[i])); completeMechanism();
            CHECK(MechanismController::status().runtimePlan && MechanismController::status().steps==1);
            CHECK(MechanismController::status().prop==props[i] && MechanismController::status().planVersion==9);
            request("next"); CHECK(CylinderController::status().active && CylinderController::status().durationMs==uint32_t(200+i));
            CHECK(Mock::gpioState[SystemConfig::Cylinder::IN1_PIN]==LOW && Mock::gpioState[SystemConfig::Cylinder::IN2_PIN]==LOW); completeMechanism();
        }
        request("plan.begin 1 3 4 1");
        request("plan.action 0 4 0 0 0 0 0 100 0 1 0 1");
        request("plan.action 1 4 0 0 0 0 0 100 0 1 0 2");
        request("plan.action 2 3 0 0 0 0 0 0 1 1 0");
        request("plan.step 0 0 3"); request("plan.commit",false); request("plan.abort");
        CHECK(MechanismController::status().planVersion==9); assertCylinderStopped();
    } else if (name == "cylinder_live_status_fixture") {
        readyRobot(); request("mode debug");
        request("debug.cylinder 1 0",false); request("debug.cylinder 3 100",false); request("debug.cylinder 2 60001",false);
        request("debug.cylinder 1 6000"); advance(100);
        CHECK(CylinderController::status().active && Mock::gpioState[SystemConfig::Cylinder::IN1_PIN]==HIGH && Mock::gpioState[SystemConfig::Cylinder::IN2_PIN]==LOW);
        writeStatusFixture(".pio/host-tests/cylinder_live_status_fixture.json",request("status"));
        request("debug.cylinder 0 0"); completeMechanism(); assertCylinderStopped();
    } else return false;
    return true;
}

bool testLowerAxisCurrentSystem(const std::string &name) {
    if (name == "startup_current_lower_axes_wait_chassis_status") {
        holdCurrentAckNode = 4; initialize(); advance(1200);
        auto pending = MechanismController::status();
        CHECK(pending.startupNode == 4 && pending.startupCommand == 0x83 && pending.startupCurrentMa == 3000);
        CHECK(pending.current4Ma == 0 && pending.current5Ma == 0 && pending.current6Ma == 0 && pending.currentConfiguredMask == 0);
        CHECK(mksCount(0xF3, -1, 1) == 0 && mksCount(0x92) == 0 && mksCount(0x91) == 0);
        const auto wire4 = request("status");
        CHECK(wire4.find("\"current4_ma\":0,\"current5_ma\":0,\"current6_ma\":0,\"current_configured_mask\":0") != std::string::npos);
        writeStatusFixture(".pio/host-tests/current_wait4_status_fixture.json", wire4);
        request("next", false); request("home", false);
        radio().channels[SystemConfig::Chassis::FORWARD.number] = SystemConfig::Chassis::FORWARD.maximum;
        size_t sentAt = Mock::sent.size(); auto updatesAt = radio().updateCount;
        advance(1000);
        CHECK(mksCount(0x83,4) >= 3 && mksCount(0xF3,4,1) == 0 && mksCount(0x92) == 0 && mksCount(0x91) == 0 && mksCount(0xFE) == 0);
        CHECK(odriveCount(0x0D,sentAt) >= 264 && radio().updateCount >= updatesAt + 200);
        CHECK(ChassisController::status().mode == ChassisController::Mode::Running && !SystemSafety::stopped());
        holdCurrentAckNode = 5; incomingMks(4,0x83,1); advance(50);
        pending = MechanismController::status();
        CHECK(pending.startupNode == 5 && pending.startupCommand == 0x83 && pending.startupCurrentMa == 3000);
        CHECK(pending.current4Ma == 3000 && pending.current5Ma == 0 && pending.current6Ma == 0 && pending.currentConfiguredMask == 1);
        CHECK(mksCount(0xF3,4,1) == 1 && mksCount(0xF3,5,1) == 0 && mksCount(0x92) == 0 && mksCount(0x91) == 0);
        const auto wire5 = request("status");
        CHECK(wire5.find("\"current4_ma\":3000,\"current5_ma\":0,\"current6_ma\":0,\"current_configured_mask\":1") != std::string::npos);
        writeStatusFixture(".pio/host-tests/current_wait5_status_fixture.json", wire5);
        sentAt = Mock::sent.size(); updatesAt = radio().updateCount; advance(500);
        CHECK(mksCount(0x83,5) == 2 && mksCount(0xF3,5,1) == 0 && mksCount(0x92) == 0 && mksCount(0x91) == 0);
        CHECK(odriveCount(0x0D,sentAt) >= 132 && radio().updateCount >= updatesAt + 100);
        holdCurrentAckNode = 0; incomingMks(5,0x83,1); advance(250); completeMechanism();
        CHECK(mechanismReady() && MechanismController::status().currentConfiguredMask == 7);
        CHECK(MechanismController::status().current4Ma == 3000 && MechanismController::status().current5Ma == 3000);
        CHECK(mksCount(0x92,4) == 1 && mksCount(0x92,5) == 1 && mksCount(0x91,6) == 1 && mksCount(0x91,7) == 0);
    } else if (name == "startup_current_id5_reject_stops_whole_robot") {
        holdCurrentAckNode = 5; initialize(); advance(1250);
        CHECK(MechanismController::status().startupNode == 5 && mksCount(0x83,5) == 1);
        CHECK(MechanismController::status().current4Ma == 3000 && MechanismController::status().current5Ma == 0 && MechanismController::status().currentConfiguredMask == 1);
        CHECK(mksCount(0x92) == 0 && mksCount(0x91) == 0);
        const size_t stoppedAt = Mock::sent.size(); incomingMks(5,0x83,0); tick(); assertEmergencyStopped(stoppedAt);
        CHECK(std::string(SystemSafety::reason()).find("ID 5 0x83 3000 mA") != std::string::npos);
        const auto wire = request("status");
        CHECK(wire.find("\"current4_ma\":3000,\"current5_ma\":0,\"current6_ma\":0,\"current_configured_mask\":1") != std::string::npos);
        const unsigned currentsAt = mksCount(0x83); incomingMks(5,0x83,1); advance(11000);
        CHECK(mksCount(0x83) == currentsAt && mksCount(0xF3,5,1) == 0 && mksCount(0x92) == 0 && mksCount(0x91) == 0);
    } else return false;
    return true;
}

bool testStageSwitchSystem(const std::string &name) {
    using MechanismConfig::CylinderAction;
    if (name == "stage_switch_boot_held_selector_retries") {
        initialize();
        radio().channels[normalStageChannel(4)]=1000;
        radio().channels[normalStageChannel(2)]=2000;
        advance(7000);
        CHECK(mechanismReady() && MechanismController::status().stage==2);
        CHECK(RcController::status().selectedStage==2 && ChassisController::status().directionStage==2);
        CHECK(MechanismController::status().completed==0 && mksCount(0xFE)==0 && mksCount(0x91)==1);
        CHECK(CylinderController::status().referenceKnown && CylinderController::status().referenceAction==CylinderAction::Retract);
        CHECK(!SystemSafety::stopped());
        const unsigned moves=mksCount(0xFE), homes=mksCount(0x91);
        advance(300); CHECK(mechanismReady() && mksCount(0xFE)==moves && mksCount(0x91)==homes);
    } else if (name == "stage_switch_held_recovers_from_chassis_lock") {
        initialize();
        radio().channels[normalStageChannel(4)]=1000;
        radio().channels[normalStageChannel(2)]=2000;
        sticks(SystemConfig::Chassis::FORWARD.maximum,SystemConfig::Chassis::STRAFE.middle,SystemConfig::Chassis::ROTATE.middle);
        advance(2000);
        CHECK(mechanismReady() && MechanismController::status().stage==4);
        CHECK(ChassisController::status().mode==ChassisController::Mode::Locked && RcController::status().selectedStage==0);
        sticks(SystemConfig::Chassis::FORWARD.middle,SystemConfig::Chassis::STRAFE.middle,SystemConfig::Chassis::ROTATE.middle);
        advance(1000);
        CHECK(ChassisController::status().mode==ChassisController::Mode::Running);
        CHECK(MechanismController::status().stage==2 && std::string(MechanismController::status().phase)=="home_wait");
        completeMechanism(); CHECK(RcController::status().selectedStage==2);
        nextRcPulse(); completeMechanism(); CHECK(MechanismController::status().completed==1);
    } else if (name == "stage_switch_latest_release_fallback_and_usb_ownership") {
        readyRobot(); button(normalStageChannel(2),true); completeMechanism();
        CHECK(MechanismController::status().stage==2);
        button(normalStageChannel(3),true); completeMechanism();
        CHECK(MechanismController::status().stage==3 && RcController::status().selectedStage==3);
        button(normalStageChannel(3),false); completeMechanism();
        CHECK(MechanismController::status().stage==2 && RcController::status().selectedStage==2);
        CHECK(radio().channels[normalStageChannel(2)]==2000 && radio().channels[normalStageChannel(4)]==2000);
        request("stage 4"); completeMechanism(); const unsigned moves=mksCount(0xFE), homes=mksCount(0x91);
        advance(500);
        CHECK(MechanismController::status().stage==4 && MechanismController::status().pendingStage==0);
        CHECK(RcController::status().selectedStage==2 && mksCount(0xFE)==moves && mksCount(0x91)==homes);
        button(normalStageChannel(2),false); completeMechanism();
        CHECK(MechanismController::status().stage==4 && RcController::status().selectedStage==4);
    } else if (name == "stage_switch_cylinder_retarget_keeps_deadline_and_frame") {
        readyRobot(); request("stage 2"); completeMechanism();
        CHECK(CylinderController::start(CylinderAction::Extend,100)); advance(160); assertCylinderStopped();
        CHECK(CylinderController::status().referenceAction==CylinderAction::Extend);
        request("stage 3"); advance(100);
        CHECK(MechanismController::status().stage==2 && MechanismController::status().pendingStage==3);
        CHECK(CylinderController::status().active && Mock::gpioState[16]==HIGH && Mock::gpioState[17]==LOW);
        const uint32_t startedAt=mockMillis, remaining=CylinderController::status().remainingMs;
        const size_t cylinderWrites=Mock::gpioWrites.size(), pwmWrites=Mock::pwmWrites.size();
        sticks(SystemConfig::Chassis::FORWARD.maximum,SystemConfig::Chassis::STRAFE.middle,SystemConfig::Chassis::ROTATE.middle);
        assertSteeringCadence(3,1,0,0,0,1000);
        request("stage 4"); CHECK(Mock::gpioWrites.size()==cylinderWrites);
        assertSteeringCadence(4,1,0,0,0,1000);
        request("stage 1"); CHECK(Mock::gpioWrites.size()==cylinderWrites);
        CHECK(CylinderController::status().remainingMs<remaining-1900);
        CHECK(MechanismController::status().stage==2 && MechanismController::status().pendingStage==1);
        assertSteeringCadence(1,1,0,0);
        const auto wire=request("status");
        CHECK(wire.find("\"direction_stage\":1")!=std::string::npos);
        writeStatusFixture(".pio/host-tests/stage_switch_pending_status_fixture.json",wire);
        completeMechanism();
        CHECK(uint32_t(mockMillis-startedAt)<=remaining+15);
        CHECK(MechanismController::status().stage==1 && mechanismReady() && MechanismController::status().completed==0);
        CHECK(Mock::pwmWrites.size()>pwmWrites+800); assertCylinderStopped();
    } else if (name == "stage_switch_known_retract_skips_only_automatic_home") {
        readyRobot(); request("stage 2"); completeMechanism();
        CHECK(CylinderController::status().referenceKnown && CylinderController::status().referenceAction==CylinderAction::Retract);
        const uint32_t before=mockMillis; const size_t writes=Mock::gpioWrites.size();
        request("stage 4"); completeMechanism(); request("stage 2"); completeMechanism();
        CHECK(uint32_t(mockMillis-before)<200);
        for (size_t i=writes;i<Mock::gpioWrites.size();++i) CHECK(Mock::gpioWrites[i].value==LOW);
        const uint32_t homeAt=mockMillis; request("home"); advance(100);
        CHECK(!mechanismReady() && CylinderController::status().active && CylinderController::status().durationMs==5000);
        request("stage 4");
        CHECK(MechanismController::status().stage==2 && MechanismController::status().pendingStage==4);
        completeMechanism();
        CHECK(uint32_t(mockMillis-homeAt)>=5000 && uint32_t(mockMillis-homeAt)<5200);
        CHECK(MechanismController::status().stage==4 && mechanismReady()); assertCylinderStopped();
    } else if (name == "stage_switch_missing_return_ack_still_stops_safely") {
        readyRobot(); request("stage 3"); completeMechanism(); request("next"); completeMechanism();
        autoCompleteMoves=false; request("stage 2"); finish(4);
        CHECK(MechanismController::status().stage==3 && MechanismController::status().pendingStage==2 && !mechanismReady());
        sticks(SystemConfig::Chassis::FORWARD.maximum,SystemConfig::Chassis::STRAFE.middle,SystemConfig::Chassis::ROTATE.middle);
        assertSteeringCadence(2,1,0,0,0,500);
        const uint32_t limit=SystemConfig::Mechanism::FINISH_TIMEOUT_MS;
        advance(limit);
        CHECK(SystemSafety::stopped() && std::string(MechanismController::status().phase)=="fault");
        CHECK(MechanismController::status().stage==3 && ChassisController::status().emergencyLatched);
        CHECK(std::string(SystemSafety::reason()).find("completion")!=std::string::npos);
        assertCylinderStopped(); request("next",false); request("stage 2",false);
    } else if (name == "stage_switch_stop_preserves_old_stage_and_pwm") {
        readyRobot(); request("stage 2"); completeMechanism();
        CHECK(CylinderController::start(CylinderAction::Extend,100)); advance(160);
        request("stage 1"); advance(100);
        CHECK(MechanismController::status().stage==2 && MechanismController::status().pendingStage==1);
        CHECK(ChassisController::status().directionStage==1);
        request("stop"); assertCylinderStopped();
        CHECK(MechanismController::status().stage==2 && MechanismController::status().pendingStage==0);
        const size_t pwmWrites=Mock::pwmWrites.size(); advance(500);
        CHECK(SystemSafety::stopped() && ChassisController::status().emergencyLatched && Mock::pwmWrites.size()==pwmWrites+100);
        request("stage 3",false); CHECK(MechanismController::status().stage==2);
    } else return false;
    return true;
}

bool testExtraServoSystem(const std::string &name) {
    if (name == "servo9_debug_endpoints_wire_fixture") {
        readyRobot(); request("mode debug");
        request("debug.servo 10 0 0"); request("debug.servo 11 1800 0"); request("debug.servo 12 450 0");
        CHECK(Mock::servo[10] == 500 && Mock::servo[11] == 2500 && Mock::servo[12] == 1000);
        const auto state=MechanismController::status();
        CHECK(state.pulse10 == 500 && state.pulse11 == 2500 && state.pulse12 == 1000);
        for (uint8_t i=0;i<6;++i) CHECK(Mock::servo[SystemConfig::Mechanism::SERVO_PINS[i]] == SystemConfig::Mechanism::SERVO_START_US[i]);
        request("debug.servo 10 1801 0",false); request("debug.servo 11 900 60001",false); request("debug.servo 13 900 0",false);
        const auto wire=request("status");
        CHECK(wire.find("\"pulse10\":500") != std::string::npos && wire.find("\"pulse11\":2500") != std::string::npos && wire.find("\"pulse12\":1000") != std::string::npos);
        writeStatusFixture(".pio/host-tests/servo9_debug_status_fixture.json",wire);
    } else if (name == "servo9_smooth_chassis_pwm_and_stop") {
        readyRobot(); request("mode debug");
        request("debug.servo 10 0 1500"); request("debug.servo 11 1800 2000"); request("debug.servo 12 0 2500");
        radio().channels[SystemConfig::Chassis::FORWARD.number]=SystemConfig::Chassis::FORWARD.maximum;
        const size_t sentAt=Mock::sent.size(), pwmAt=Mock::pwmWrites.size(); const auto updatesAt=radio().updateCount;
        advance(1000);
        CHECK(Mock::servo[10] > 500 && Mock::servo[10] < 1500 && Mock::servo[11] > 1500 && Mock::servo[11] < 2500 && Mock::servo[12] > 500 && Mock::servo[12] < 1500);
        CHECK(mechanismBusy() && !SystemSafety::stopped());
        CHECK(odriveCount(0x0D,sentAt) >= 264 && radio().updateCount >= updatesAt+200 && Mock::pwmWrites.size() >= pwmAt+200);
        const size_t stopAt=Mock::sent.size(); request("stop"); assertEmergencyStopped(stopAt);
        const size_t writes=Mock::servoWrites.size(); advance(3000);
        CHECK(Mock::servoWrites.size() == writes);
        for (uint8_t pin : SystemConfig::Mechanism::SERVO_PINS) CHECK(!Mock::servoAttached[pin]);
        request("debug.servo 10 1800 0",false); request("debug.servo 11 0 0",false); request("debug.servo 12 900 0",false);
    } else return false;
    return true;
}


// CH4 is decoded by the real CRSF module and applied by the normal main loop.
void trimStage1Ready() {
    readyRobot(); selectRcStage(1); completeMechanism();
    CHECK(RcController::status().stage1Mode && MechanismController::status().stage == 1);
}
void setTrimKnob(int value) {
    radio().channels[SystemConfig::Rc::STAGE1_TRIM_CHANNEL]=value; tick();
}
void uploadTrimServo(int pulse, uint32_t duration=0) {
    CHECK(MechanismController::planBegin(1,1,1,1,2));
    CHECK(MechanismController::planAction(0,MechanismConfig::branch(MechanismConfig::servo(2,pulse,duration),1)));
    CHECK(MechanismController::planStep(0,0,1)); CHECK(MechanismController::planCommit()); completeMechanism();
}
bool testStage1TrimSystem(const std::string &name) {
    if (name == "trim_system_deadband_endpoints_and_outside_standard") {
        trimStage1Ready();
        const int inputs[]={1000,1470,1500,1530,2000,900,2100};
        const int offsets[]={-150,0,0,0,150,-150,150};
        for (unsigned i=0;i<sizeof(inputs)/sizeof(inputs[0]);++i) {
            setTrimKnob(inputs[i]); const auto state=MechanismController::status();
            CHECK(RcController::status().stage1TrimUs == offsets[i]);
            CHECK(state.pulse2Base == 1150 && state.pulse2 == 1150+offsets[i]);
            CHECK(Mock::servo[2] == state.pulse2 && state.stage1TrimUs == offsets[i]);
        }
    } else if (name == "trim_system_prop_hold_previous_and_base_snapshots") {
        trimStage1Ready(); setTrimKnob(2000); propRcPulse(4); completeMechanism();
        CHECK(Mock::servo[2] == propPulse(4,1,2)+150);
        nextRcPulse(); completeMechanism(); const int base=propPulse(4,2,2);
        CHECK(Mock::servo[2] == base+150 && MechanismController::status().pulse2Base == base);
        setTrimKnob(1000); const size_t writes=Mock::servoWrites.size(); advance(250);
        CHECK(Mock::servo[2] == base-150 && Mock::servoWrites.size() == writes);
        button(SystemConfig::Rc::PREV_CHANNEL,false); button(SystemConfig::Rc::PREV_CHANNEL,true); completeMechanism();
        CHECK(MechanismController::status().pulse2Base == propPulse(4,1,2));
        CHECK(Mock::servo[2] == propPulse(4,1,2)-150);
    } else if (name == "trim_system_ram_ramp_live_knob_keeps_base") {
        trimStage1Ready(); uploadTrimServo(1950,1000); setTrimKnob(2000);
        CHECK(MechanismController::next()); tick(); const uint32_t start=mockMillis;
        advance(400); const auto before=MechanismController::status();
        CHECK(before.pulse2Base > 1400 && before.pulse2Base < 1550 && before.pulse2 == before.pulse2Base+150);
        setTrimKnob(1000); const auto after=MechanismController::status();
        CHECK(after.pulse2Base >= before.pulse2Base && after.pulse2Base-before.pulse2Base <= 20);
        CHECK(after.pulse2 == after.pulse2Base-150);
        completeMechanism(); CHECK(mockMillis-start >= 1000);
        CHECK(MechanismController::status().pulse2Base == 1950 && Mock::servo[2] == 1800);
    } else if (name == "trim_system_clamp_500_and_2500_effective_delta") {
        trimStage1Ready(); uploadTrimServo(2500); setTrimKnob(2000); request("next"); completeMechanism();
        CHECK(MechanismController::status().pulse2Base == 2500 && Mock::servo[2] == 2500);
        CHECK(MechanismController::status().stage1TrimUs == 0 && RcController::status().stage1TrimUs == 150);
        uploadTrimServo(500); setTrimKnob(1000); request("next"); completeMechanism();
        CHECK(MechanismController::status().pulse2Base == 500 && Mock::servo[2] == 500 && MechanismController::status().stage1TrimUs == 0);
        setTrimKnob(2000); CHECK(Mock::servo[2] == 650 && MechanismController::status().stage1TrimUs == 150);
    } else if (name == "trim_system_ch7_low_exit_and_stage4_no_leak") {
        trimStage1Ready(); setTrimKnob(2000); propRcPulse(2); completeMechanism();
        CHECK(Mock::servo[2] == propPulse(2,1,2)+150);
        autoCompleteHomes=false; stage1Switch(1000); tick();
        CHECK(MechanismController::status().stage1TrimUs == 0 && RcController::status().stage1TrimUs == 0);
        CHECK(MechanismController::status().pendingStage == 4 && Mock::servo[2] == 1150);
        autoCompleteHomes=true; finish(6,0x91); completeMechanism();
        CHECK(MechanismController::status().stage == 4 && !MechanismController::status().stage1TrimAllowed);
        int stage4Pulse2=MechanismConfig::SERVO2_START_US;
        for (size_t i=0;i<Stage4::STEPS[0].count;++i) {
            const auto &action=Stage4::STEPS[0].actions[i];
            if (action.kind==MechanismConfig::ActionKind::Servo && action.pin==2) stage4Pulse2=action.pulse;
        }
        nextRcPulse(); completeMechanism(); CHECK(Mock::servo[2] == stage4Pulse2);
        setTrimKnob(1000); CHECK(Mock::servo[2] == stage4Pulse2 && RcController::status().stage1TrimUs == 0);
    } else if (name == "trim_system_serial_stage_other_contexts_zero") {
        readyRobot(); setTrimKnob(2000); request("stage 1"); completeMechanism();
        CHECK(!RcController::status().stage1Mode && !MechanismController::status().stage1TrimAllowed && Mock::servo[2] == 1150);
        for (uint8_t stage : {uint8_t(2),uint8_t(3),uint8_t(4),uint8_t(5)}) {
            const auto previousStage=MechanismController::status().stage;
            const bool enabled=MechanismConfig::stageEnabled(stage);
            request("stage "+std::to_string(stage),enabled); completeMechanism();
            CHECK(MechanismController::status().stage == (enabled ? stage : previousStage));
            CHECK(MechanismController::status().stage1TrimUs == 0);
            CHECK(!MechanismController::status().stage1TrimAllowed && Mock::servo[2] == 1150);
        }
    } else if (name == "trim_system_home_suspended_then_next_resumes") {
        trimStage1Ready(); setTrimKnob(2000); request("home"); completeMechanism(); advance(100);
        CHECK(!MechanismController::status().stage1TrimAllowed && Mock::servo[2] == 1150);
        nextRcPulse(); completeMechanism(); CHECK(MechanismController::status().stage1TrimAllowed);
        CHECK(MechanismController::status().pulse2Base == propPulse(2,1,2) && Mock::servo[2] == propPulse(2,1,2)+150);
    } else if (name == "trim_system_debug_home_suspended_then_normal_next") {
        trimStage1Ready(); setTrimKnob(2000); request("mode debug"); request("debug.gohome 6"); completeMechanism();
        CHECK(!MechanismController::status().stage1TrimAllowed && Mock::servo[2] == MechanismController::status().pulse2Base);
        request("mode normal"); request("next"); completeMechanism();
        CHECK(MechanismController::status().stage1TrimAllowed && Mock::servo[2] == propPulse(2,1,2)+150);
    } else if (name == "trim_system_link_loss_invalid_controls_and_recovery") {
        trimStage1Ready(); setTrimKnob(2000); CHECK(Mock::servo[2] == 1300);
        radio().linked=false; tick(); CHECK(RcController::status().stage1TrimUs == 0 && Mock::servo[2] == 1150);
        radio().linked=true; advance(600); setTrimKnob(2000);
        CHECK(Mock::servo[2] == 1300);
        radio().channels[SystemConfig::Chassis::FORWARD.number]=2200; tick();
        CHECK(!RcController::status().valid && Mock::servo[2] == 1150 && RcController::status().stage1TrimUs == 0);
    } else if (name == "trim_system_invalid_ch4_does_not_lock_chassis") {
        trimStage1Ready(); setTrimKnob(2000);
        for (int invalid : {0,899,2101,2500}) {
            setTrimKnob(invalid);
            CHECK(RcController::status().valid && RcController::status().stage1TrimUs == 0 && Mock::servo[2] == 1150);
            CHECK(ChassisController::status().mode == ChassisController::Mode::Running && !SystemSafety::stopped());
        }
        setTrimKnob(1000); CHECK(Mock::servo[2] == 1000);
    } else if (name == "trim_system_stop_detaches_and_no_pwm_restart") {
        trimStage1Ready(); setTrimKnob(2000); request("stop"); const size_t writes=Mock::servoWrites.size();
        setTrimKnob(1000); advance(500);
        CHECK(Mock::servoWrites.size() == writes && !Mock::servoAttached[2]);
        CHECK(RcController::status().stage1TrimUs == 0 && MechanismController::status().stage1TrimUs == 0);
        CHECK(!MechanismController::status().stage1TrimAllowed);
    } else if (name == "trim_system_chassis_15ms_and_actual_wire_fixture") {
        trimStage1Ready(); setTrimKnob(2000);
        sticks(SystemConfig::Chassis::FORWARD.maximum,SystemConfig::Chassis::STRAFE.middle,SystemConfig::Chassis::ROTATE.middle);
        advance(15); assertSteeringCadence(1,1,0,0,0,300);
        CHECK(RcController::status().forward == SystemConfig::Chassis::FORWARD.maximum && Mock::servo[2] == 1300);
        const auto wire=request("status");
        CHECK(wire.find("\"pulse2_base\":1150") != std::string::npos);
        CHECK(wire.find("\"stage1_trim_us\":150") != std::string::npos);
        writeStatusFixture(".pio/host-tests/stage1_trim_status_fixture.json",wire);
        request("home"); completeMechanism();
        writeStatusFixture(".pio/host-tests/stage1_trim_home_status_fixture.json",request("status"));
    } else return false;
    return true;
}

bool testPropHomeCycle(const std::string &name) {
    if (!(name == "prop_cycle_vase" || name == "prop_cycle_chicken" || name == "prop_cycle_bird" ||
          name == "prop_cycle_rc_next")) return false;
    const uint8_t prop = name == "prop_cycle_vase" ? 2 : name == "prop_cycle_chicken" ? 4 : 5;
    readyRobot();
    if (name == "prop_cycle_rc_next") {
        stage1Switch(2000); completeMechanism();
        propRcPulse(prop); completeMechanism();
    } else { request("stage 1 " + std::to_string(prop)); completeMechanism(); }
    while (!MechanismController::status().propComplete) { request("next"); completeMechanism(); }
    CHECK(MechanismController::status().completed == 4);
    if (prop != 2) CHECK(Mock::servo[2] == 1611 && Mock::servo[5] == 1422 && MechanismController::status().position6 == -1000);
    const auto homes = mksCount(0x91,6);
    autoCompleteHomes = false;
    if (name == "prop_cycle_rc_next") nextRcPulse(); else request("next");
    CHECK(mksCount(0x91,6) == homes + 1 && mechanismBusy());
    CHECK(std::string(MechanismController::status().phase) == "home_wait");
    CHECK(!MechanismController::next());
    finish(6,0x91); completeMechanism();
    CHECK(mechanismReady() && MechanismController::status().completed == 0 && !MechanismController::status().propComplete);
    CHECK(MechanismController::status().position6 == 0 && Mock::servo[2] == 1150 && Mock::servo[5] == 2100);
    advance(100); CHECK(MechanismController::status().completed == 0);
    if (name == "prop_cycle_rc_next") nextRcPulse(); else request("next");
    completeMechanism();
    CHECK(MechanismController::status().completed == 1 && MechanismController::status().prop == prop);
    return true;
}

// Read the latest fruit profile tuning rather than freezing old field coordinates.
int32_t fruitPosition(uint8_t step) {
    CHECK(step>=1 && step<=Stage6::STEP_COUNT);
    int32_t position=0;
    for (uint8_t n=0;n<step;++n)
        for (size_t i=0;i<Stage6::STEPS[n].count;++i) {
            const auto &action=Stage6::STEPS[n].actions[i];
            if (action.kind==MechanismConfig::ActionKind::Stepper && action.node==6)
                position=action.position;
        }
    return position;
}
uint32_t fruitStepDuration(uint8_t step) {
    CHECK(step>=1 && step<=Stage6::STEP_COUNT);
    uint32_t total=0;
    for (size_t i=0;i<Stage6::STEPS[step-1].count;++i)
        total+=Stage6::STEPS[step-1].actions[i].ms;
    return total;
}

bool testFruit(const std::string &name) {
    if (!(name == "fruit_rc_selection" || name == "fruit_auto_return" ||
          name == "fruit_run_and_stop" || name == "fruit_switch_during_return" ||
          name == "fruit_ram_profile")) return false;
    readyRobot();
    if (name == "fruit_rc_selection") {
        button(14,true); completeMechanism();
        CHECK(MechanismController::status().stage==6 && !RcController::status().processionMode);
        CHECK(ChassisController::status().directionStage==6);
        nextRcPulse(); completeMechanism();
        CHECK(MechanismController::status().completed==1 && target(6)==fruitPosition(1));
        // CH12 must keep the six-step toss profile independently of CH14 fruit.
        button(12,false); button(12,true); completeMechanism();
        CHECK(MechanismController::status().stage==4 && MechanismController::status().steps==6);
        CHECK(ChassisController::status().directionStage==4);
        nextRcPulse(); completeMechanism();
        CHECK(MechanismController::status().completed==1 && target(6)==stage4Position(1));
        CHECK(target(6)!=fruitPosition(1));
        for (size_t i=0;i<Stage4::STEPS[0].count;++i) {
            const auto &action=Stage4::STEPS[0].actions[i];
            if (action.kind==MechanismConfig::ActionKind::Servo)
                CHECK(Mock::servo[action.pin]==action.pulse);
        }
        // Releasing CH12 restores the still-held CH14 selection.
        button(12,false); completeMechanism();
        CHECK(MechanismController::status().stage==6 && MechanismController::status().steps==5);
        nextRcPulse(); completeMechanism();
        CHECK(MechanismController::status().completed==1 && target(6)==fruitPosition(1));
        stage1Switch(2000); completeMechanism();
        button(14,false); button(14,true); completeMechanism();
        CHECK(MechanismController::status().stage==1 && MechanismController::status().prop==5);
        return true;
    }
    request("stage 6"); completeMechanism();
    CHECK(MechanismController::status().steps==5);
    const auto homes=mksCount(0x91,6);
    if(name=="fruit_ram_profile") {
        request("plan.begin 1 1 6 1 0");
        request("plan.action 0 1 0 0 0 2 1611 100 1 1 0");
        request("plan.step 0 0 1"); request("plan.commit"); completeMechanism();
        CHECK(MechanismController::status().runtimePlan && MechanismController::status().stage==6);
        request("next"); completeMechanism();
        CHECK(MechanismController::status().completed==0 && MechanismController::status().position6==0);
        return true;
    }
    if(name=="fruit_run_and_stop") {
        request("run"); advance(100); const auto stoppedAt=Mock::sent.size();
        request("stop"); assertEmergencyStopped(stoppedAt); return true;
    }
    for (unsigned step=1;step<=4;++step) {
        request("next"); completeMechanism();
        CHECK(MechanismController::status().completed==step);
    }
    CHECK(MechanismController::status().position6==fruitPosition(4));
    autoCompleteMoves=false;
    request("next"); advance(fruitStepDuration(Stage6::STEP_COUNT)+200);
    CHECK(std::string(MechanismController::status().phase)=="home_wait" && target(6)==0);
    CHECK(mksCount(0x91,6)==homes && !MechanismController::next());
    if(name=="fruit_switch_during_return") {
        request("stage 4"); completeMechanism();
        CHECK(MechanismController::status().stage==4 && mechanismReady()); return true;
    }
    finish(6); completeMechanism();
    CHECK(MechanismController::status().completed==0 && MechanismController::status().progress==0);
    CHECK(Mock::servo[2]==1150 && Mock::servo[5]==2100);
    request("next"); completeMechanism(); CHECK(target(6)==fruitPosition(1) && MechanismController::status().completed==1);
    request("run"); completeMechanism();
    CHECK(MechanismController::status().completed==0 && target(6)==0 && mksCount(0x91,6)==homes);
    return true;
}


// Exercise CH4 height trim through the real RC, FE ACK barrier, and cooperative loop.
// Keep the untrimmed profile coordinate distinct from the physical trim command.
void settleStage4Height() {
    advance(SystemConfig::Rc::STAGE4_TRIM_UPDATE_MS+20);
    completeMechanism();
    advance(SystemConfig::Rc::STAGE4_TRIM_UPDATE_MS+20);
    completeMechanism();
}
void stage4TrimFirstStep() {
    readyRobot();
    CHECK(MechanismController::status().stage==4 && !RcController::status().stage1Mode);
    request("next"); settleStage4Height();
    CHECK(MechanismController::status().completed==1 && MechanismController::status().stage4TrimAllowed);
}
int32_t limitedFruitHeight(int32_t base, int offset=0) {
    return std::max(SystemConfig::Fruit::POSITION_MIN,
                    std::min(SystemConfig::Fruit::POSITION_MAX,base+offset));
}
void assertStage4Height(int32_t base, int offset) {
    const auto state=MechanismController::status();
    const int32_t expected=state.stage==6?limitedFruitHeight(base,offset):std::min(SystemConfig::ID6_POSITION_MAX,base+offset);
    CHECK(mechanismReady() && state.position6Base==base);
    CHECK(state.position6==expected && state.stage4TrimPulses==expected-base);
    CHECK(state.stage4TrimTarget==expected && !state.stage4TrimMoving);
}
void uploadStage4HeightPlan(int32_t base) {
    CHECK(MechanismController::planBegin(2,2,4,1));
    CHECK(MechanismController::planAction(0,MechanismConfig::branch(MechanismConfig::stepperFor(6,base,320,5),1)));
    CHECK(MechanismController::planAction(1,MechanismConfig::branch(MechanismConfig::servo(2,1600,200),1)));
    CHECK(MechanismController::planStep(0,0,1));
    CHECK(MechanismController::planStep(1,1,1));
    CHECK(MechanismController::planCommit()); completeMechanism();
}
bool testStage4HeightTrim(const std::string &name) {
    if (name=="stage4_trim_endpoints_deadband_and_no_accumulation") {
        readyRobot(); const auto startupMoves=mksCount(0xFE,6);
        setTrimKnob(2000); advance(250);
        CHECK(mksCount(0xFE,6)==startupMoves && !MechanismController::status().stage4TrimAllowed);
        setTrimKnob(1500); request("next"); settleStage4Height();
        const int32_t base=stage4Position(1);
        const int values[]={1000,1470,1500,1530,2000,900,2100,2000,1000};
        const int offsets[]={-200,0,0,0,200,-200,200,200,-200};
        for (unsigned i=0;i<sizeof(values)/sizeof(values[0]);++i) {
            setTrimKnob(values[i]); settleStage4Height();
            CHECK(RcController::status().stage4TrimPulses==offsets[i]);
            assertStage4Height(base,offsets[i]);
            CHECK(MechanismController::status().completed==1 && MechanismController::status().progress==1);
            const auto count=mksCount(0xFE,6); advance(300);
            CHECK(mksCount(0xFE,6)==count); // Holding a knob never integrates the offset.
        }
    } else if (name=="stage4_trim_ack_barrier_latest_knob_and_next_no_queue") {
        stage4TrimFirstStep(); const auto base=stage4Position(1);
        const auto count=mksCount(0xFE,6); autoCompleteMoves=false;
        setTrimKnob(2000); advance(120);
        CHECK(mksCount(0xFE,6)==count+1 && target(6)==base+200);
        CHECK(MechanismController::status().stage4TrimMoving && !mechanismReady());
        CHECK(MechanismController::status().position6==base && MechanismController::status().stage4TrimPulses==0);
        CHECK(lastMove(6).buf[1]==0 && lastMove(6).buf[2]==100 && lastMove(6).buf[3]==5);
        request("next",false); nextRcPulse();
        setTrimKnob(1000); advance(200);
        CHECK(mksCount(0xFE,6)==count+1 && MechanismController::status().completed==1);
        finish(6); advance(120);
        CHECK(target(6)==base-200 && mksCount(0xFE,6)==count+2);
        CHECK(MechanismController::status().position6==base+200 && MechanismController::status().stage4TrimPulses==200);
        finish(6); settleStage4Height(); assertStage4Height(base,-200);
        CHECK(MechanismController::status().completed==1 && MechanismController::status().step==1);
        advance(200); CHECK(MechanismController::status().completed==1);
        // A fresh release/press, rather than the held busy-time key, advances to STEP2.
        nextRcPulse(); settleStage4Height();
        CHECK(MechanismController::status().completed==2 && MechanismController::status().position6Base==base);
    } else if (name=="stage4_trim_full_toss_step_barrier") {
        stage4TrimFirstStep(); request("next"); settleStage4Height();
        request("next"); settleStage4Height();
        setTrimKnob(2000); settleStage4Height(); assertStage4Height(stage4Position(3),200);
        autoCompleteMoves=false; const auto count=mksCount(0xFE,6);
        request("next"); CHECK(target(6)==stage4Position(4));
        CHECK(lastMove(6).buf[1]==5 && lastMove(6).buf[2]==220 && lastMove(6).buf[3]==0);
        setTrimKnob(1000); advance(500);
        CHECK(mksCount(0xFE,6)==count+1 && !MechanismController::status().stage4TrimMoving);
        const auto &step=Stage4::STEPS[3]; int32_t buffer=0;
        for (size_t i=0;i<step.count;++i)
            if (step.actions[i].kind==MechanismConfig::ActionKind::Stepper) buffer=step.actions[i].position;
        finish(6); advance(20);
        CHECK(target(6)==buffer && mksCount(0xFE,6)==count+2);
        CHECK(!MechanismController::status().stage4TrimMoving && MechanismController::status().completed==3);
        advance(500); CHECK(mksCount(0xFE,6)==count+2);
        finish(6); advance(120);
        CHECK(target(6)==buffer-200 && mksCount(0xFE,6)==count+3);
        CHECK(MechanismController::status().completed==4 && MechanismController::status().stage4TrimMoving);
        finish(6); settleStage4Height(); assertStage4Height(buffer,-200);
    } else if (name=="stage4_trim_servo_step_and_previous_keep_untrimmed_base") {
        stage4TrimFirstStep(); const auto base=stage4Position(1);
        setTrimKnob(2000); settleStage4Height(); assertStage4Height(base,200);
        const auto count=mksCount(0xFE,6); request("next"); settleStage4Height();
        CHECK(MechanismController::status().completed==2 && mksCount(0xFE,6)==count);
        assertStage4Height(base,200);
        setTrimKnob(1000); settleStage4Height(); assertStage4Height(base,-200);
        request("prev"); settleStage4Height();
        CHECK(MechanismController::status().completed==1 && MechanismController::status().progress==1);
        assertStage4Height(base,-200);
        setTrimKnob(2000); settleStage4Height(); assertStage4Height(base,200);
    } else if (name=="stage4_trim_home_debug_suspend_until_explicit_step") {
        stage4TrimFirstStep(); setTrimKnob(2000); settleStage4Height();
        const auto homes=mksCount(0x91,6); request("home"); completeMechanism();
        CHECK(mksCount(0x91,6)==homes+1 && MechanismController::status().position6==0);
        CHECK(!MechanismController::status().stage4TrimAllowed && !MechanismController::status().stage4TrimMoving);
        const auto count=mksCount(0xFE,6); setTrimKnob(1000); advance(300);
        CHECK(mksCount(0xFE,6)==count && MechanismController::status().completed==0);
        request("next"); settleStage4Height(); assertStage4Height(stage4Position(1),-200);
        request("mode debug"); request("debug.move 6 123 100 5"); completeMechanism();
        CHECK(!MechanismController::status().stage4TrimAllowed && MechanismController::status().position6==123);
        request("mode normal"); const auto debugCount=mksCount(0xFE,6); setTrimKnob(2000); advance(300);
        CHECK(mksCount(0xFE,6)==debugCount && !MechanismController::status().stage4TrimAllowed);
        request("next"); settleStage4Height();
        CHECK(MechanismController::status().stage4TrimAllowed && MechanismController::status().stage4TrimPulses==200);
    } else if (name=="stage4_trim_switch_does_not_leak_to_fruit_or_stage1") {
        stage4TrimFirstStep(); setTrimKnob(2000); settleStage4Height();
        button(14,true); completeMechanism();
        CHECK(MechanismController::status().stage==6 && !MechanismController::status().stage4TrimAllowed);
        const auto beforeFruit=mksCount(0xFE,6); advance(300);
        CHECK(mksCount(0xFE,6)==beforeFruit && !MechanismController::status().stage4TrimAllowed);
        request("next"); settleStage4Height();
        assertStage4Height(fruitPosition(1),2000);
        CHECK(MechanismController::status().stage4TrimAllowed);
        setTrimKnob(1000); settleStage4Height(); assertStage4Height(fruitPosition(1),-2000);
        stage1Switch(2000); completeMechanism(); propRcPulse(4); completeMechanism();
        CHECK(MechanismController::status().stage==1 && target(6)==propMotor(4,1).position);
        CHECK(!MechanismController::status().stage4TrimAllowed && RcController::status().stage4TrimPulses==0);
        CHECK(MechanismController::status().stage1TrimUs==-150);
        const auto propCount=mksCount(0xFE,6); setTrimKnob(2000); advance(300);
        CHECK(mksCount(0xFE,6)==propCount && MechanismController::status().stage1TrimUs==150);
    } else if (name=="stage4_trim_bad_rc_and_link_loss_require_new_step") {
        stage4TrimFirstStep(); setTrimKnob(2000); settleStage4Height();
        const auto count=mksCount(0xFE,6);
        setTrimKnob(899); advance(150);
        CHECK(RcController::status().valid && RcController::status().stage4TrimPulses==0);
        CHECK(!MechanismController::status().stage4TrimAllowed && !SystemSafety::stopped());
        setTrimKnob(1000); advance(150); CHECK(mksCount(0xFE,6)==count);
        request("next"); settleStage4Height(); CHECK(MechanismController::status().stage4TrimAllowed);
        autoCompleteMoves=false; setTrimKnob(2000); advance(120);
        CHECK(MechanismController::status().stage4TrimMoving);
        radio().linked=false; tick();
        CHECK(!MechanismController::status().stage4TrimAllowed && RcController::status().stage4TrimPulses==0);
        finish(6); radio().linked=true; advance(600); setTrimKnob(1000);
        const auto disconnectedCount=mksCount(0xFE,6); advance(250);
        CHECK(mksCount(0xFE,6)==disconnectedCount && !MechanismController::status().stage4TrimAllowed);
        request("prev"); settleStage4Height(); assertStage4Height(stage4Position(1),-200);
        radio().channels[SystemConfig::Chassis::FORWARD.number]=2200; tick();
        CHECK(!RcController::status().valid && !MechanismController::status().stage4TrimAllowed);
        CHECK(RcController::status().stage4TrimPulses==0 && !SystemSafety::stopped());
    } else if (name=="stage4_trim_ram_plan_and_status_json") {
        readyRobot(); uploadStage4HeightPlan(765); setTrimKnob(2000);
        request("next"); settleStage4Height(); assertStage4Height(765,200);
        CHECK(MechanismController::status().runtimePlan && MechanismController::status().completed==1);
        request("next"); settleStage4Height(); assertStage4Height(765,200);
        CHECK(MechanismController::status().completed==2 && MechanismController::status().progress==2);
        const auto wire=request("status");
        CHECK(wire.find("\"position6_base\":765")!=std::string::npos);
        CHECK(wire.find("\"stage4_trim_pulses\":200")!=std::string::npos);
        CHECK(wire.find("\"stage4_trim_target\":965")!=std::string::npos);
        CHECK(wire.find("\"stage4_trim_allowed\":true")!=std::string::npos);
        CHECK(wire.find("\"stage4_trim_moving\":false")!=std::string::npos);
        writeStatusFixture(".pio/host-tests/stage4_trim_status_fixture.json",wire);
        request("home"); completeMechanism();
        CHECK(!MechanismController::status().stage4TrimAllowed && MechanismController::status().completed==0);
        request("run"); settleStage4Height();
        CHECK(MechanismController::status().stage4TrimAllowed && MechanismController::status().completed==2);
        assertStage4Height(765,200);
    } else if (name=="stage4_trim_chassis_cadence_and_stop_no_restart") {
        stage4TrimFirstStep(); autoCompleteMoves=false;
        setTrimKnob(2000); advance(120); CHECK(MechanismController::status().stage4TrimMoving);
        sticks(SystemConfig::Chassis::FORWARD.maximum,SystemConfig::Chassis::STRAFE.middle,SystemConfig::Chassis::ROTATE.middle);
        const auto updates=radio().updateCount; const size_t sentAt=Mock::sent.size();
        advance(1000);
        CHECK(odriveCount(0x0D,sentAt)>=264 && radio().updateCount>=updates+200);
        CHECK(ChassisController::status().mode==ChassisController::Mode::Running && !SystemSafety::stopped());
        const auto stopAt=Mock::sent.size(); request("stop"); assertEmergencyStopped(stopAt);
        const auto moves=mksCount(0xFE,6); setTrimKnob(1000); finish(6); advance(250);
        CHECK(mksCount(0xFE,6)==moves && !MechanismController::status().stage4TrimAllowed);
        CHECK(!MechanismController::status().stage4TrimMoving && RcController::status().stage4TrimPulses==0);
    } else return false;
    return true;
}


void fruitTrimFirstStep() {
    readyRobot(); button(14,true); completeMechanism();
    CHECK(MechanismController::status().stage==6 && !RcController::status().stage1Mode);
    CHECK(!MechanismController::status().stage4TrimAllowed);
    request("next"); settleStage4Height();
    CHECK(MechanismController::status().completed==1 && MechanismController::status().stage4TrimAllowed);
}
bool testFruitHeightTrim(const std::string &name) {
    if (name=="fruit_trim_endpoints_deadband_and_no_accumulation") {
        readyRobot(); button(14,true); completeMechanism();
        const auto initial=mksCount(0xFE,6); setTrimKnob(2000); advance(300);
        CHECK(mksCount(0xFE,6)==initial && !MechanismController::status().stage4TrimAllowed);
        request("next"); settleStage4Height();
        const int values[]={1000,1470,1500,1530,2000,900,2100,2000,1000};
        const int offsets[]={-2000,0,0,0,2000,-2000,2000,2000,-2000};
        for (unsigned i=0;i<sizeof(values)/sizeof(values[0]);++i) {
            setTrimKnob(values[i]); settleStage4Height();
            CHECK(RcController::status().stage4TrimPulses==offsets[i]);
            assertStage4Height(fruitPosition(1),offsets[i]);
            CHECK(MechanismController::status().completed==1 && MechanismController::status().progress==1);
            const auto count=mksCount(0xFE,6); advance(300); CHECK(mksCount(0xFE,6)==count);
        }
    } else if (name=="fruit_trim_servo_previous_and_status_json") {
        fruitTrimFirstStep(); setTrimKnob(2000); settleStage4Height();
        assertStage4Height(fruitPosition(1),2000);
        const auto count=mksCount(0xFE,6); request("next"); settleStage4Height();
        CHECK(MechanismController::status().completed==2 && mksCount(0xFE,6)==count);
        assertStage4Height(fruitPosition(1),2000);
        const auto wire=request("status");
        CHECK(wire.find("\"stage\":6")!=std::string::npos);
        CHECK(wire.find("\"position6_base\":"+std::to_string(fruitPosition(1)))!=std::string::npos);
        CHECK(wire.find("\"stage4_trim_pulses\":"+std::to_string(limitedFruitHeight(fruitPosition(1),2000)-fruitPosition(1)))!=std::string::npos);
        CHECK(wire.find("\"stage4_trim_allowed\":true")!=std::string::npos);
        setTrimKnob(1000); settleStage4Height(); request("prev"); settleStage4Height();
        CHECK(MechanismController::status().completed==1 && MechanismController::status().progress==1);
        assertStage4Height(fruitPosition(1),-2000);
    } else if (name=="fruit_trim_motion_barrier_latest_knob_and_no_queue") {
        readyRobot(); button(14,true); completeMechanism();
        autoCompleteMoves=false; const auto count=mksCount(0xFE,6);
        request("next"); setTrimKnob(2000); advance(fruitStepDuration(1)+200);
        CHECK(target(6)==fruitPosition(1) && mksCount(0xFE,6)==count+1);
        CHECK(!MechanismController::status().stage4TrimMoving && MechanismController::status().completed==0);
        request("next",false); request("prev",false);
        setTrimKnob(1000); finish(6); advance(120);
        CHECK(target(6)==fruitPosition(1)-2000 && mksCount(0xFE,6)==count+2);
        CHECK(MechanismController::status().completed==1 && MechanismController::status().stage4TrimMoving);
        CHECK(lastMove(6).buf[1]==0 && lastMove(6).buf[2]==100 && lastMove(6).buf[3]==5);
        request("next",false); request("prev",false); nextRcPulse();
        finish(6); settleStage4Height(); assertStage4Height(fruitPosition(1),-2000);
        CHECK(MechanismController::status().completed==1);
        advance(300); CHECK(MechanismController::status().completed==1);
        nextRcPulse(); settleStage4Height(); CHECK(MechanismController::status().completed==2);
    } else if (name=="fruit_trim_auto_home_clears_offset_and_new_round") {
        fruitTrimFirstStep();
        for (uint8_t step=2;step<Stage6::STEP_COUNT;++step) {
            request("next"); settleStage4Height(); CHECK(MechanismController::status().completed==step);
        }
        setTrimKnob(2000); settleStage4Height(); assertStage4Height(fruitPosition(Stage6::STEP_COUNT-1),2000);
        const auto homes=mksCount(0x91,6); autoCompleteMoves=false;
        request("next"); advance(fruitStepDuration(Stage6::STEP_COUNT)+200);
        CHECK(std::string(MechanismController::status().phase)=="home_wait" && target(6)==0);
        CHECK(!MechanismController::status().stage4TrimAllowed && !MechanismController::status().stage4TrimMoving);
        CHECK(mksCount(0x91,6)==homes && !MechanismController::next());
        finish(6); completeMechanism();
        CHECK(MechanismController::status().completed==0 && MechanismController::status().position6==0);
        CHECK(Mock::servo[2]==1150 && Mock::servo[5]==2100);
        const auto count=mksCount(0xFE,6); advance(500);
        CHECK(mksCount(0xFE,6)==count && !MechanismController::status().stage4TrimAllowed);
        CHECK(RcController::status().stage4TrimPulses==0);
        request("next"); settleStage4Height(); assertStage4Height(fruitPosition(1),2000);
        CHECK(MechanismController::status().completed==1 && MechanismController::status().stage4TrimAllowed);
    } else if (name=="fruit_trim_home_switch_and_stage1_isolation") {
        fruitTrimFirstStep(); setTrimKnob(2000); settleStage4Height();
        const auto homes=mksCount(0x91,6); request("home"); completeMechanism();
        CHECK(mksCount(0x91,6)==homes+1 && MechanismController::status().position6==0);
        CHECK(!MechanismController::status().stage4TrimAllowed);
        const auto count=mksCount(0xFE,6); setTrimKnob(1000); advance(300);
        CHECK(mksCount(0xFE,6)==count);
        request("next"); settleStage4Height(); assertStage4Height(fruitPosition(1),-2000);
        button(10,true); completeMechanism();
        CHECK(MechanismController::status().stage==2 && !MechanismController::status().stage4TrimAllowed);
        CHECK(RcController::status().stage4TrimPulses==0);
        const auto stage2Count=mksCount(0xFE,6); setTrimKnob(2000); advance(300);
        CHECK(mksCount(0xFE,6)==stage2Count);
        stage1Switch(2000); completeMechanism(); propRcPulse(4); completeMechanism();
        CHECK(MechanismController::status().stage==1 && MechanismController::status().stage1TrimUs==150);
        CHECK(!MechanismController::status().stage4TrimAllowed && RcController::status().stage4TrimPulses==0);
        const auto stage1Count=mksCount(0xFE,6); setTrimKnob(1000); advance(300);
        CHECK(mksCount(0xFE,6)==stage1Count && MechanismController::status().stage1TrimUs==-150);
    } else if (name=="fruit_trim_disconnect_and_stop_require_new_step") {
        fruitTrimFirstStep();
        // Preserve a unique held selector across reconnect: the receiver correctly
        // forgets press ordering on link loss and cannot rank CH12+CH14 both HIGH.
        button(12,false); setTrimKnob(2000); settleStage4Height();
        radio().linked=false; tick();
        CHECK(!MechanismController::status().stage4TrimAllowed && RcController::status().stage4TrimPulses==0);
        radio().linked=true; advance(600); setTrimKnob(1000);
        const auto count=mksCount(0xFE,6); advance(300);
        CHECK(mksCount(0xFE,6)==count && !MechanismController::status().stage4TrimAllowed);
        request("next"); settleStage4Height();
        assertStage4Height(fruitPosition(1),-2000);
        CHECK(MechanismController::status().completed==2);
        autoCompleteMoves=false; setTrimKnob(2000); advance(120);
        CHECK(MechanismController::status().stage4TrimMoving);
        const auto stopAt=Mock::sent.size(); request("stop"); assertEmergencyStopped(stopAt);
        const auto stoppedCount=mksCount(0xFE,6); finish(6); setTrimKnob(1000); advance(300);
        CHECK(mksCount(0xFE,6)==stoppedCount && !MechanismController::status().stage4TrimAllowed);
        CHECK(!MechanismController::status().stage4TrimMoving && RcController::status().stage4TrimPulses==0);
    } else return false;
    return true;
}


// Exercise soft limits and local axis isolation through the real cooperative loop.
// A fruit fault must stop ID6, keep its hold current, and leave independent stages usable.
void fruitSafetySelected() {
    readyRobot(); button(14,true); completeMechanism();
    CHECK(MechanismController::status().stage==6);
    setTrimKnob(1500);
}
void uploadFruitSafetyPlan(int32_t first, int32_t second) {
    CHECK(MechanismController::planBegin(3,3,6,1));
    CHECK(MechanismController::planAction(0,MechanismConfig::branch(MechanismConfig::stepperFor(6,first,320,5),1)));
    CHECK(MechanismController::planAction(1,MechanismConfig::branch(MechanismConfig::stepperFor(6,second,320,5),1)));
    CHECK(MechanismController::planAction(2,MechanismConfig::branch(MechanismConfig::servo(2,1611,100),1)));
    for (uint16_t i=0;i<3;++i) CHECK(MechanismController::planStep(i,i,1));
    CHECK(MechanismController::planCommit()); completeMechanism();
}
void assertFruitIsolated() {
    const auto state=MechanismController::status();
    CHECK(mechanismReady() && !SystemSafety::stopped());
    CHECK((state.motorRecoveryMask & 4)!=0 && (state.positionKnownMask & 4)==0);
    CHECK((state.motorOnlineMask & 4)!=0 && (state.motorOfflineMask & 4)==0);
    CHECK(!state.stage4TrimAllowed && !state.stage4TrimMoving);
    CHECK(mksCount(0xF3,6,0)==0);
    CHECK(state.localWarning!=nullptr && state.localWarning[0]!='\0');
    for (uint8_t pin : SystemConfig::Mechanism::SERVO_PINS) CHECK(Mock::servoAttached[pin]);
}
void rejectFruitMotion() {
    fruitSafetySelected(); autoCompleteMoves=false; request("next");
    incomingMks(6,0xFE,3); tick(); advance(30);
    assertFruitIsolated();
    CHECK(MechanismController::status().stageRecoveryRequired);
}
void assertFruitFeBounds(size_t since=0) {
    for (size_t i=since;i<Mock::sent.size();++i) {
        const auto &frame=Mock::sent[i];
        if (!mksFrame(frame) || frame.msg.id!=6 || frame.msg.buf[0]!=0xFE) continue;
        const auto &b=frame.msg.buf;
        const int32_t raw=(int32_t(b[4])<<16)|(int32_t(b[5])<<8)|b[6];
        const int32_t position=(raw&0x800000)?raw-0x1000000:raw;
        CHECK(position>=SystemConfig::Fruit::POSITION_MIN && position<=SystemConfig::Fruit::POSITION_MAX);
    }
}
bool testFruitSafety(const std::string &name) {
    if (name=="fruit_safety_top_limit_no_empty_job") {
        fruitSafetySelected(); uploadFruitSafetyPlan(10500,10500);
        request("next"); settleStage4Height(); CHECK(target(6)==10500);
        const auto before=mksCount(0xFE,6);
        setTrimKnob(2000); advance(600);
        CHECK(RcController::status().stage4TrimPulses==2000);
        assertStage4Height(10500,2000);
        CHECK(mksCount(0xFE,6)==before);
        for (unsigned i=0;i<5;++i) {
            advance(120); CHECK(mechanismReady() && !MechanismController::status().stage4TrimMoving);
        }
        CHECK(mksCount(0xFE,6)==before);
        request("next"); settleStage4Height(); CHECK(MechanismController::status().completed==2);
        setTrimKnob(1000); settleStage4Height(); assertStage4Height(10500,-2000);
        setTrimKnob(2000); settleStage4Height(); assertStage4Height(10500,2000);
        assertFruitFeBounds();
    } else if (name=="fruit_safety_bottom_limit_no_empty_job") {
        fruitSafetySelected(); uploadFruitSafetyPlan(-1400,500);
        request("next"); settleStage4Height(); const auto before=mksCount(0xFE,6);
        setTrimKnob(1000); advance(600); assertStage4Height(-1400,-2000);
        CHECK(RcController::status().stage4TrimPulses==-2000 && mksCount(0xFE,6)==before);
        request("next"); settleStage4Height(); CHECK(MechanismController::status().completed==2);
        assertStage4Height(500,-2000); CHECK(target(6)==-1400);
        assertFruitFeBounds();
    } else if (name=="fruit_safety_ram_debug_previous_limits") {
        fruitSafetySelected(); uploadFruitSafetyPlan(20000,-20000);
        const size_t from=Mock::sent.size();
        request("next"); settleStage4Height(); assertStage4Height(10500,0);
        setTrimKnob(1000); settleStage4Height(); assertStage4Height(10500,-2000);
        request("next"); settleStage4Height(); assertStage4Height(-1400,-2000);
        setTrimKnob(2000); settleStage4Height(); assertStage4Height(-1400,2000);
        request("prev"); settleStage4Height(); assertStage4Height(10500,2000);
        CHECK(MechanismController::status().completed==1);
        request("mode debug");
        request("debug.move 6 20000 100 5"); completeMechanism(); CHECK(target(6)==10500);
        request("debug.move 6 -20000 100 5"); completeMechanism(); CHECK(target(6)==-1400);
        assertFruitFeBounds(from);
        request("mode normal"); request("home"); completeMechanism(); CHECK(MechanismController::status().position6==0);
    } else if (name=="fruit_safety_completion_timeout_keeps_chassis") {
        fruitSafetySelected(); autoCompleteMoves=false; request("next");
        const auto updates=radio().updateCount; const size_t from=Mock::sent.size();
        advance(1000);
        CHECK(mechanismBusy() && odriveCount(0x0D,from)>=264 && radio().updateCount>=updates+200);
        advance(SystemConfig::Fruit::FINISH_TIMEOUT_MS+100);
        assertFruitIsolated(); CHECK(mksCount(0xF7,6)>=1);
        CHECK(ChassisController::status().mode==ChassisController::Mode::Running);
        const auto moves=mksCount(0xFE,6); request("stage 2"); completeMechanism();
        CHECK(MechanismController::status().stage==2 && !MechanismController::status().stageRecoveryRequired);
        request("next"); completeMechanism(); CHECK(MechanismController::status().completed==1);
        CHECK(mksCount(0xFE,6)==moves && !SystemSafety::stopped());
    } else if (name=="fruit_safety_start_timeout_late_ack_ignored") {
        fruitSafetySelected(); autoStartMoves=false; autoCompleteMoves=false; request("next");
        advance(SystemConfig::Mechanism::START_TIMEOUT_MS+50); assertFruitIsolated();
        const auto completed=MechanismController::status().completed;
        incomingMks(6,0xFE,1); incomingMks(6,0xFE,2); tick(); advance(100);
        assertFruitIsolated(); CHECK(MechanismController::status().completed==completed);
        CHECK((MechanismController::status().positionKnownMask&4)==0);
        // Only ID6 lost its ACK; independent healthy axes must resume normal replies.
        autoStartMoves=true;
        request("stage 3"); completeMechanism(); CHECK(MechanismController::status().stage==3);
        request("next"); completeMechanism(); CHECK(MechanismController::status().completed==1);
        CHECK(!SystemSafety::stopped());
    } else if (name=="fruit_safety_rejected_move_local_warning") {
        rejectFruitMotion(); const auto moves=mksCount(0xFE,6);
        request("next",false); request("prev",false); request("run",false);
        CHECK(mksCount(0xFE,6)==moves && mksCount(0xF7,6)>=1);
        const auto wire=request("status");
        CHECK(wire.find("\"motor_recovery_mask\":4")!=std::string::npos);
        CHECK(wire.find("\"stage_recovery_required\":true")!=std::string::npos);
        CHECK(wire.find("\"mechanism_warning\":")!=std::string::npos);
        CHECK(wire.find("\"fruit_position_min\":-1400")!=std::string::npos);
        CHECK(wire.find("\"fruit_position_max\":10500")!=std::string::npos);
        request("stage 2"); completeMechanism(); CHECK(MechanismController::status().stage==2);
        request("stage 3"); completeMechanism(); request("next"); completeMechanism();
        CHECK(MechanismController::status().stage==3 && MechanismController::status().completed==1);
        CHECK(mksCount(0xFE,6)==moves && (MechanismController::status().motorRecoveryMask&4)!=0);
    } else if (name=="fruit_safety_usb_switch_cancels_unfinished_move") {
        fruitSafetySelected(); autoCompleteMoves=false; request("next");
        const auto moves=mksCount(0xFE,6), homes=mksCount(0x91,6); const uint32_t from=mockMillis;
        request("stage 2"); completeMechanism();
        CHECK(uint32_t(mockMillis-from)<6000 && MechanismController::status().stage==2);
        CHECK(mksCount(0xFE,6)==moves && mksCount(0x91,6)==homes && mksCount(0xF7,6)>=1);
        CHECK((MechanismController::status().motorRecoveryMask&4)!=0 && !SystemSafety::stopped());
        request("next"); completeMechanism(); CHECK(MechanismController::status().completed==1);
    } else if (name=="fruit_safety_rc_switch_cancels_trim") {
        fruitTrimFirstStep(); button(12,false); autoCompleteMoves=false;
        setTrimKnob(1000); advance(120); CHECK(MechanismController::status().stage4TrimMoving);
        const auto moves=mksCount(0xFE,6); button(11,true); completeMechanism();
        CHECK(MechanismController::status().stage==3 && RcController::status().selectedStage==3);
        CHECK(mksCount(0xFE,6)==moves && mksCount(0xF7,6)>=1);
        CHECK(!MechanismController::status().stage4TrimAllowed && !SystemSafety::stopped());
        CHECK(ChassisController::status().directionStage==3);
        nextRcPulse(); completeMechanism(); CHECK(MechanismController::status().completed==1);
    } else if (name=="fruit_safety_shared_id6_requires_manual_home") {
        rejectFruitMotion(); const auto moves=mksCount(0xFE,6);
        request("stage 4"); completeMechanism(); CHECK(MechanismController::status().stageRecoveryRequired);
        request("next",false); request("prev",false); request("run",false);
        request("stage 1"); completeMechanism(); CHECK(MechanismController::status().stageRecoveryRequired);
        CHECK(!MechanismController::triggerProp(4)); request("next",false); request("run",false);
        CHECK(mksCount(0xFE,6)==moves);
        const auto homes=mksCount(0x91,6); autoCompleteHomes=false; request("home"); advance(100);
        CHECK(mksCount(0x91,6)==homes+1 && (MechanismController::status().motorRecoveryMask&4)!=0);
        incomingMks(6,0xFE,2); tick(); CHECK(!mechanismReady());
        CHECK((MechanismController::status().positionKnownMask&4)==0);
        finish(6,0x91); completeMechanism();
        CHECK(MechanismController::status().motorRecoveryMask==0 && !MechanismController::status().stageRecoveryRequired);
        CHECK((MechanismController::status().positionKnownMask&4)!=0 && MechanismController::status().position6==0);
        CHECK(MechanismController::triggerProp(4)); completeMechanism();
        CHECK(MechanismController::status().completed==1 && target(6)==9200);
        CHECK(!MechanismController::status().stageRecoveryRequired);
    } else if (name=="fruit_safety_healthy_stage1_chicken_bird_rc") {
        fruitTrimFirstStep(); CHECK(MechanismController::status().motorRecoveryMask==0);
        stage1Switch(2000); completeMechanism();
        CHECK(RcController::status().stage1Mode && MechanismController::status().stage==1);
        for (uint8_t prop : {uint8_t(4),uint8_t(5)}) {
            propRcPulse(prop); completeMechanism();
            CHECK(MechanismController::status().stage==1 && MechanismController::status().prop==prop);
            CHECK(MechanismController::status().completed==1 && target(6)==9200);
            CHECK(MechanismController::status().position6==9200 && MechanismController::status().position6Base==9200);
            CHECK(!MechanismController::status().stageRecoveryRequired && MechanismController::status().motorRecoveryMask==0);
            CHECK(!SystemSafety::stopped() && (MechanismController::status().positionKnownMask&4)!=0);
            request("home"); completeMechanism();
            CHECK(MechanismController::status().position6==0 && !MechanismController::status().stageRecoveryRequired);
        }
    } else if (name=="fruit_safety_failed_recovery_home_can_retry") {
        rejectFruitMotion(); autoCompleteHomes=false; request("home"); advance(100);
        incomingMks(6,0x91,0); tick(); advance(30); assertFruitIsolated();
        CHECK(MechanismController::status().stageRecoveryRequired);
        request("stage 2"); completeMechanism(); request("next"); completeMechanism();
        CHECK(MechanismController::status().stage==2 && MechanismController::status().completed==1);
        request("stage 6"); completeMechanism(); const auto homes=mksCount(0x91,6);
        autoCompleteHomes=true; request("home"); completeMechanism();
        CHECK(mksCount(0x91,6)==homes+1 && MechanismController::status().motorRecoveryMask==0);
        request("next"); completeMechanism(); CHECK(MechanismController::status().completed==1);
    } else if (name=="fruit_safety_failed_stop_enqueue_is_barrier") {
        fruitSafetySelected(); autoCompleteMoves=false; request("next");
        Mock::rejectNode=6; Mock::rejectCommand=0xF7;
        const auto moves=mksCount(0xFE), homes=mksCount(0x91); request("stage 3");
        advance(800);
        CHECK(!SystemSafety::stopped() && MechanismController::status().pendingStage==3);
        CHECK(mksCount(0xF7,6)==0 && mksCount(0xFE)==moves && mksCount(0x91)==homes);
        CHECK(!MechanismController::next() && !MechanismController::home());
        Mock::rejectNode=-1; advance(60); completeMechanism();
        CHECK(mksCount(0xF7,6)>=1 && MechanismController::status().stage==3);
        CHECK((MechanismController::status().motorRecoveryMask&4)!=0);
        request("next"); completeMechanism(); CHECK(MechanismController::status().completed==1);
    } else if (name=="fruit_safety_fe_enqueue_timeout_is_local") {
        fruitSafetySelected(); Mock::rejectNode=6; Mock::rejectCommand=0xFE;
        const auto moves=mksCount(0xFE,6); request("next"); advance(SystemConfig::Mechanism::TX_TIMEOUT_MS+100);
        assertFruitIsolated(); CHECK(mksCount(0xFE,6)==moves && mksCount(0xF7,6)>=1);
        Mock::rejectNode=-1; request("stage 2"); completeMechanism();
        CHECK(MechanismController::status().stage==2 && !SystemSafety::stopped());
    } else if (name=="fruit_safety_real_global_stop_cannot_bypass") {
        rejectFruitMotion(); const auto stoppedAt=Mock::sent.size(); request("stop");
        assertEmergencyStopped(stoppedAt); const auto moves=mksCount(0xFE), homes=mksCount(0x91);
        request("stage 2",false); request("stage 3",false); request("home",false); request("next",false);
        incomingMks(6,0x91,1); incomingMks(6,0x91,2); tick(); advance(500);
        CHECK(SystemSafety::stopped() && mksCount(0xFE)==moves && mksCount(0x91)==homes);
    } else return false;
    return true;
}


// ID6 is shared by Stage1/4/6: the hard upper bound protects every source,
// while other axes and non-fruit lower travel remain independent.
bool testGlobalId6Upper(const std::string &name) {
    if (name=="global_id6_upper_stage1_ram_previous_debug") {
        readyRobot();
        CHECK(MechanismController::planBegin(3,3,1,1,4));
        CHECK(MechanismController::planAction(0,MechanismConfig::branch(MechanismConfig::stepperFor(6,12000,320,5),1)));
        CHECK(MechanismController::planAction(1,MechanismConfig::branch(MechanismConfig::stepperFor(6,3000,320,5),1)));
        CHECK(MechanismController::planAction(2,MechanismConfig::branch(MechanismConfig::servo(2,1611,100),1)));
        for (uint16_t i=0;i<3;++i) CHECK(MechanismController::planStep(i,i,1));
        CHECK(MechanismController::planCommit()); completeMechanism();
        CHECK(MechanismController::status().stage==1 && MechanismController::status().prop==4);
        const size_t from=Mock::sent.size();
        request("next"); completeMechanism(); CHECK(target(6)==10500 && MechanismController::status().completed==1);
        request("next"); completeMechanism(); CHECK(target(6)==3000 && MechanismController::status().completed==2);
        request("prev"); completeMechanism(); CHECK(target(6)==10500 && MechanismController::status().completed==1);
        request("mode debug");
        request("debug.move 6 -2000 100 5"); completeMechanism(); CHECK(target(6)==-2000);
        request("debug.move 6 14000 100 5"); completeMechanism(); CHECK(target(6)==10500);
        CHECK(!MechanismController::status().stageRecoveryRequired && !SystemSafety::stopped());
        bool observedUpper=false;
        for(size_t i=from;i<Mock::sent.size();++i) {
            const auto &frame=Mock::sent[i];
            if (!mksFrame(frame) || frame.msg.id!=6 || frame.msg.buf[0]!=0xFE) continue;
            const auto &b=frame.msg.buf;
            const int32_t raw=(int32_t(b[4])<<16)|(int32_t(b[5])<<8)|b[6];
            const int32_t position=(raw&0x800000)?raw-0x1000000:raw;
            CHECK(position<=10500); observedUpper=observedUpper||position==10500;
        }
        CHECK(observedUpper);
    } else if(name=="global_id6_upper_stage4_ram_trim_previous_run") {
        readyRobot(); uploadStage4HeightPlan(12000);
        request("next"); settleStage4Height(); assertStage4Height(10500,0);
        const auto moves=mksCount(0xFE,6);
        setTrimKnob(2000); advance(500); assertStage4Height(10500,200);
        CHECK(mksCount(0xFE,6)==moves && RcController::status().stage4TrimPulses==200);
        setTrimKnob(1000); settleStage4Height();
        CHECK(target(6)==SystemConfig::ID6_POSITION_MAX-200); assertStage4Height(10500,-200);
        request("next"); settleStage4Height(); CHECK(MechanismController::status().completed==2);
        request("prev"); settleStage4Height(); CHECK(MechanismController::status().completed==1);
        assertStage4Height(10500,-200);
        setTrimKnob(2000); settleStage4Height(); CHECK(target(6)==10500);
        request("home"); completeMechanism(); request("run"); settleStage4Height();
        CHECK(MechanismController::status().completed==2 && target(6)==10500);
        CHECK(!MechanismController::status().stageRecoveryRequired && !SystemSafety::stopped());
    } else if(name=="global_id6_upper_debug_preserves_other_axes") {
        readyRobot(); request("mode debug");
        request("debug.move 4 12000 100 5"); completeMechanism(); CHECK(target(4)==12000);
        request("debug.move 5 13000 100 5"); completeMechanism(); CHECK(target(5)==13000);
        request("debug.move 6 12000 100 5"); completeMechanism(); CHECK(target(6)==10500);
        request("debug.move 6 -2000 100 5"); completeMechanism(); CHECK(target(6)==-2000);
        CHECK(MechanismController::status().position4==12000 && MechanismController::status().position5==13000);
        const auto wire=request("status");
        CHECK(wire.find("\"id6_position_max\":10500")!=std::string::npos);
        CHECK(wire.find("\"fruit_position_max\":10500")!=std::string::npos);
        CHECK(!MechanismController::status().stageRecoveryRequired && !SystemSafety::stopped());
    } else return false;
    return true;
}


// Stage 2 owns a paired CH4 height trim. Offsets are absolute relative to the
// last formal state, and completion of one CAN axis must never release the pair.
void settleStage2Pair() {
    advance(150); completeMechanism(); advance(150); completeMechanism();
}
void stage2PairReady() {
    readyRobot(); selectRcStage(2); completeMechanism(); advance(150);
    CHECK(MechanismController::status().stage==2 && mechanismReady());
    CHECK(!RcController::status().stage1Mode && MechanismController::status().stage2TrimAllowed);
}
void assertStage2Pair(int32_t base4, int32_t base5, int32_t offset) {
    const auto state=MechanismController::status();
    const auto position4=std::max(int32_t(0),base4+offset);
    const auto position5=std::min(int32_t(0),base5-offset);
    CHECK(mechanismReady() && !state.stage2TrimMoving && state.stage2TrimAllowed);
    CHECK(state.position4Base==base4 && state.position5Base==base5);
    CHECK(state.position4==position4 && state.position5==position5);
    CHECK(state.stage2TrimTarget4==position4 && state.stage2TrimTarget5==position5);
    CHECK(state.position4>=0 && state.position5<=0);
}
void uploadStage2PairPlan() {
    CHECK(MechanismController::planBegin(2,4,2,1));
    CHECK(MechanismController::planAction(0,MechanismConfig::branch(MechanismConfig::stepperFor(4,1600,320,2,false),1)));
    CHECK(MechanismController::planAction(1,MechanismConfig::branch(MechanismConfig::stepperFor(5,-1600,320,2,false),1)));
    CHECK(MechanismController::planAction(2,MechanismConfig::branch(MechanismConfig::waitAll(),1)));
    CHECK(MechanismController::planAction(3,MechanismConfig::branch(MechanismConfig::servo(4,1500),1)));
    CHECK(MechanismController::planStep(0,0,3));
    CHECK(MechanismController::planStep(1,3,1));
    CHECK(MechanismController::planCommit()); completeMechanism();
}
bool testStage2PairTrim(const std::string &name) {
    if (name=="stage2_pair_trim_entry_pose_step0_and_yaml_preserved") {
        stage2PairReady();
        CHECK(Mock::servo[9]==1800 && Mock::servo[4]==1300 && Mock::servo[3]==1580);
        CHECK(MechanismController::status().step==0 && MechanismController::status().completed==0);
        CHECK(MechanismController::status().progress==0 && !MechanismController::status().previousAvailable);
        CHECK(CylinderController::status().fullyRetracted);
        const auto retracts=cylinderEnergizations(SystemConfig::Cylinder::IN2_PIN);
        const auto t=mockMillis; setTrimKnob(2000); settleStage2Pair();
        CHECK(uint32_t(mockMillis-t)<500 && cylinderEnergizations(SystemConfig::Cylinder::IN2_PIN)==retracts);
        assertStage2Pair(0,0,stage2TrimLimit); CHECK(MechanismController::status().completed==0);
        setTrimKnob(1500); settleStage2Pair();
        uint8_t firstPin6Step=0;
        for(uint8_t step=1;step<=Stage2::STEP_COUNT && !firstPin6Step;++step) {
            const auto &plan=Stage2::STEPS[step-1];
            for(size_t i=0;i<plan.count;++i)
                if(plan.actions[i].kind==MechanismConfig::ActionKind::Servo && plan.actions[i].pin==6)
                    firstPin6Step=step;
        }
        CHECK(firstPin6Step>0);
        for(uint8_t step=1;step<=firstPin6Step;++step) {
            nextRcPulse(); completeMechanism(); CHECK(MechanismController::status().completed==step);
            const auto &plan=Stage2::STEPS[step-1];
            for(size_t i=0;i<plan.count;++i) {
                const auto &action=plan.actions[i];
                if(action.kind==MechanismConfig::ActionKind::Servo) CHECK(Mock::servo[action.pin]==action.pulse);
            }
        }
        CHECK(Mock::servo[6]==stage2Pulse(firstPin6Step,6));
        CHECK(Mock::servo[9]==stage2Pulse(firstPin6Step,9) && Mock::servo[4]==stage2Pulse(firstPin6Step,4));
    } else if(name=="stage2_pair_trim_step0_endpoints_floor_and_no_accumulation") {
        stage2PairReady();
        const int knobs[]={1000,1470,1500,1530,2000,900,2100,2000,1000};
        const int offsets[]={-stage2TrimLimit,0,0,0,stage2TrimLimit,-stage2TrimLimit,stage2TrimLimit,stage2TrimLimit,-stage2TrimLimit};
        for(unsigned i=0;i<sizeof(knobs)/sizeof(knobs[0]);++i) {
            setTrimKnob(knobs[i]); settleStage2Pair();
            CHECK(RcController::status().stage2TrimPulses==offsets[i]);
            assertStage2Pair(0,0,offsets[i]);
            CHECK(MechanismController::status().completed==0 && MechanismController::status().progress==0);
            const auto count4=mksCount(0xFE,4), count5=mksCount(0xFE,5); advance(350);
            CHECK(mksCount(0xFE,4)==count4 && mksCount(0xFE,5)==count5);
        }
        CHECK(mksCount(0x91,4)==0 && mksCount(0x91,5)==0);
    } else if(name=="stage2_pair_trim_both_ack_barrier_and_next_no_queue") {
        stage2PairReady(); autoCompleteMoves=false;
        const auto count4=mksCount(0xFE,4), count5=mksCount(0xFE,5);
        setTrimKnob(2000); advance(150);
        CHECK(mksCount(0xFE,4)==count4+1 && mksCount(0xFE,5)==count5+1);
        CHECK(target(4)==stage2TrimLimit && target(5)==-stage2TrimLimit && !mechanismReady());
        CHECK(MechanismController::status().stage2TrimMoving && MechanismController::status().completed==0);
        for(uint8_t node:{uint8_t(4),uint8_t(5)})
            CHECK(lastMove(node).buf[1]==0 && lastMove(node).buf[2]==100 && lastMove(node).buf[3]==5);
        request("next",false); nextRcPulse(); finish(4); advance(150);
        CHECK(!mechanismReady() && MechanismController::status().stage2TrimMoving);
        CHECK(MechanismController::status().position5==0 && MechanismController::status().completed==0);
        CHECK(mksCount(0xFE,4)==count4+1 && mksCount(0xFE,5)==count5+1);
        finish(5); settleStage2Pair(); assertStage2Pair(0,0,stage2TrimLimit);
        advance(300); CHECK(MechanismController::status().completed==0);
        nextRcPulse(); completeMechanism(); CHECK(MechanismController::status().completed==1);
    } else if(name=="stage2_pair_trim_formal_and_previous_untrimmed_bases") {
        stage2PairReady(); uploadStage2PairPlan(); request("next"); settleStage2Pair();
        assertStage2Pair(1600,-1600,0);
        setTrimKnob(1000); settleStage2Pair(); assertStage2Pair(1600,-1600,-stage2TrimLimit);
        CHECK(MechanismController::status().stage2TrimPulses==-stage2TrimLimit && MechanismController::status().completed==1);
        setTrimKnob(2000); settleStage2Pair(); assertStage2Pair(1600,-1600,stage2TrimLimit);
        request("next"); settleStage2Pair(); assertStage2Pair(1600,-1600,stage2TrimLimit);
        CHECK(MechanismController::status().completed==2 && Mock::servo[4]==1500);
        request("prev"); settleStage2Pair(); assertStage2Pair(1600,-1600,stage2TrimLimit);
        CHECK(MechanismController::status().completed==1 && Mock::servo[4]==1300);
    } else if(name=="stage2_pair_trim_home_debug_suspend_until_explicit_step") {
        stage2PairReady(); setTrimKnob(2000); settleStage2Pair(); request("home"); completeMechanism();
        CHECK(!MechanismController::status().stage2TrimAllowed && !MechanismController::status().stage2TrimMoving);
        const auto homes4=mksCount(0x91,4),homes5=mksCount(0x91,5);
        CHECK(MechanismController::status().position4==0 && MechanismController::status().position5==0);
        const auto count4=mksCount(0xFE,4),count5=mksCount(0xFE,5); setTrimKnob(1000); advance(300);
        CHECK(mksCount(0xFE,4)==count4 && mksCount(0xFE,5)==count5);
        request("mode debug"); request("debug.move 4 1000 100 5"); completeMechanism(); request("mode normal");
        setTrimKnob(2000); advance(300);
        CHECK(!MechanismController::status().stage2TrimAllowed && mksCount(0xFE,5)==count5);
        request("next"); settleStage2Pair(); CHECK(MechanismController::status().stage2TrimAllowed);
        CHECK(mksCount(0x91,4)==homes4 && mksCount(0x91,5)==homes5);
    } else if(name=="stage2_pair_trim_update_rate_and_latest_knob") {
        stage2PairReady();
        uint32_t lastSent=0; unsigned observed=0;
        auto before=mksCount(0xFE,4);
        for(unsigned i=0;i<120;++i) {
            radio().channels[SystemConfig::Rc::STAGE1_TRIM_CHANNEL]=1700+int(i%4)*100;
            tick(); const auto count=mksCount(0xFE,4);
            if(count!=before) {
                CHECK(count==before+1);
                if(observed) CHECK(uint32_t(mockMillis-lastSent)>=100);
                lastSent=mockMillis; ++observed; before=count;
            }
        }
        CHECK(observed>=2);
        setTrimKnob(2000); settleStage2Pair(); assertStage2Pair(0,0,stage2TrimLimit);
        CHECK(MechanismController::status().completed==0 && MechanismController::status().progress==0);
    } else if(name=="stage2_pair_trim_bad_knob_and_link_loss_suspend") {
        stage2PairReady(); setTrimKnob(2000); settleStage2Pair();
        const auto before=mksCount(0xFE); setTrimKnob(899); advance(150);
        CHECK(!MechanismController::status().stage2TrimAllowed && RcController::status().stage2TrimPulses==0);
        setTrimKnob(1000); advance(300); CHECK(mksCount(0xFE)==before && !SystemSafety::stopped());
        request("next"); settleStage2Pair(); CHECK(MechanismController::status().stage2TrimAllowed);
        radio().linked=false; tick(); CHECK(RcController::status().stage2TrimPulses==0);
        const auto offline=mksCount(0xFE); radio().linked=true; advance(600); setTrimKnob(2000); advance(300);
        CHECK(!MechanismController::status().stage2TrimAllowed && mksCount(0xFE)==offline && !SystemSafety::stopped());
    } else if(name=="stage2_pair_trim_switch_returns_pair_and_servo3") {
        stage2PairReady(); setTrimKnob(2000); settleStage2Pair();
        const auto homes4=mksCount(0x91,4),homes5=mksCount(0x91,5);
        setTrimKnob(1500); selectRcStage(4); completeMechanism();
        CHECK(MechanismController::status().stage==4 && !MechanismController::status().stage2TrimAllowed);
        CHECK(MechanismController::status().position4==0 && MechanismController::status().position5==0 && Mock::servo[3]==2250);
        CHECK(mksCount(0x91,4)==homes4 && mksCount(0x91,5)==homes5);
        const auto count4=mksCount(0xFE,4),count5=mksCount(0xFE,5);
        setTrimKnob(2000); advance(300);
        CHECK(mksCount(0xFE,4)==count4 && mksCount(0xFE,5)==count5 && RcController::status().stage2TrimPulses==0);
        selectRcStage(2); settleStage2Pair(); CHECK(MechanismController::status().stage2TrimAllowed);
        CHECK(Mock::servo[9]==1800 && Mock::servo[4]==1300 && Mock::servo[3]==1580);
    } else if(name=="stage2_pair_trim_second_enqueue_failure_stops_both") {
        stage2PairReady(); autoCompleteMoves=false;
        const auto before4=mksCount(0xFE,4),before5=mksCount(0xFE,5);
        Mock::rejectNode=5; Mock::rejectCommand=0xFE;
        setTrimKnob(2000); advance(800);
        CHECK(SystemSafety::stopped() && std::string(MechanismController::status().phase)=="fault");
        CHECK(!mechanismReady() && MechanismController::status().completed==0 && MechanismController::status().progress==0);
        CHECK(mksCount(0xFE,4)==before4+1 && mksCount(0xFE,5)==before5);
        for(uint8_t node:{uint8_t(4),uint8_t(5)}) {
            CHECK(mksCount(0xF7,node)>=1 && mksCount(0xF3,node,0)>=1);
        }
        CHECK(!MechanismController::status().stage2TrimAllowed && !MechanismController::status().stage2TrimMoving);
        assertCylinderStopped(); const auto moves=mksCount(0xFE);
        Mock::rejectNode=-1; finish(4); finish(5); setTrimKnob(1000); advance(500);
        request("next",false);
        CHECK(SystemSafety::stopped() && mksCount(0xFE)==moves && MechanismController::status().completed==0);
        assertCylinderStopped();
    } else if(name=="stage2_pair_trim_ram_debug_sign_limits_stage3_unaffected") {
        stage2PairReady(); const size_t stage2Frames=Mock::sent.size();
        CHECK(MechanismController::planBegin(2,2,2,1));
        CHECK(MechanismController::planAction(0,MechanismConfig::branch(MechanismConfig::stepperFor(4,-123,320,2),1)));
        CHECK(MechanismController::planAction(1,MechanismConfig::branch(MechanismConfig::stepperFor(5,456,320,2),1)));
        CHECK(MechanismController::planStep(0,0,1) && MechanismController::planStep(1,1,1));
        CHECK(MechanismController::planCommit()); completeMechanism();
        const auto zero4=mksCount(0xFE,4),zero5=mksCount(0xFE,5);
        request("next"); completeMechanism();
        CHECK(MechanismController::status().position4==0 && MechanismController::status().position4Base==0);
        CHECK(mksCount(0xFE,4)==zero4 && mksCount(0xFE,5)==zero5);
        request("next"); completeMechanism();
        CHECK(MechanismController::status().position5==0 && MechanismController::status().position5Base==0);
        CHECK(mksCount(0xFE,4)==zero4 && mksCount(0xFE,5)==zero5);
        // A clipped target equal to the known position correctly produces no FE.
        // Start away from zero before checking the actual clipped Debug frames.
        request("mode debug");
        request("debug.move 4 1000 100 5"); completeMechanism(); CHECK(target(4)==1000);
        const auto moved4=mksCount(0xFE,4);
        request("debug.move 4 -700 100 5"); completeMechanism();
        CHECK(target(4)==0 && MechanismController::status().position4==0 && mksCount(0xFE,4)==moved4+1);
        request("debug.move 5 -1000 100 5"); completeMechanism(); CHECK(target(5)==-1000);
        const auto moved5=mksCount(0xFE,5);
        request("debug.move 5 700 100 5"); completeMechanism();
        CHECK(target(5)==0 && MechanismController::status().position5==0 && mksCount(0xFE,5)==moved5+1);
        for(size_t i=stage2Frames;i<Mock::sent.size();++i) {
            const auto &frame=Mock::sent[i];
            if(!mksFrame(frame) || frame.msg.buf[0]!=0xFE || (frame.msg.id!=4 && frame.msg.id!=5)) continue;
            const auto &data=frame.msg.buf;
            int32_t position=(int32_t(data[4])<<16)|(int32_t(data[5])<<8)|data[6];
            if(position&0x800000) position-=0x1000000;
            CHECK(frame.msg.id==4 ? position>=0 : position<=0);
        }
        request("mode normal"); request("stage 3",false);
        CHECK(MechanismController::status().stage==2 && mechanismReady());
        request("stage 4"); completeMechanism();
        // Disabling the Stage3 profile leaves shared-axis Debug available.
        request("mode debug");
        request("debug.move 4 -321 100 5"); completeMechanism(); CHECK(target(4)==-321);
        request("debug.move 5 654 100 5"); completeMechanism(); CHECK(target(5)==654);
        CHECK(!SystemSafety::stopped() && mksCount(0x91,4)==0 && mksCount(0x91,5)==0);
    } else if(name=="stage2_pair_trim_stop_never_restarts_pair") {
        stage2PairReady(); autoCompleteMoves=false; setTrimKnob(2000); advance(150);
        CHECK(MechanismController::status().stage2TrimMoving); request("stop");
        const auto moves=mksCount(0xFE); finish(4); finish(5); setTrimKnob(1000); advance(350);
        CHECK(SystemSafety::stopped() && !MechanismController::status().stage2TrimAllowed);
        CHECK(!MechanismController::status().stage2TrimMoving && RcController::status().stage2TrimPulses==0);
        CHECK(mksCount(0xFE)==moves); assertCylinderStopped();
    } else if(name=="stage2_pair_trim_chassis_cadence_and_status") {
        stage2PairReady(); autoCompleteMoves=false; setTrimKnob(2000); advance(150);
        sticks(SystemConfig::Chassis::FORWARD.maximum,SystemConfig::Chassis::STRAFE.middle,SystemConfig::Chassis::ROTATE.middle);
        advance(15); assertSteeringCadence(2,1,0,0,0,250);
        CHECK(MechanismController::status().stage2TrimMoving && !SystemSafety::stopped());
        finish(4); finish(5); settleStage2Pair(); assertStage2Pair(0,0,stage2TrimLimit);
        CHECK(MechanismController::status().completed==0 && RcController::status().stage2TrimPulses==stage2TrimLimit);
        const auto wire=request("status");
        CHECK(wire.find("\"stage2_trim_pulses\":"+std::to_string(stage2TrimLimit))!=std::string::npos);
    } else return false;
    return true;
}

// Stage 3 is selectable only when enabled; shared physical actuators must stay live.
bool testDisabledStage3(const std::string &name) {
    if(name=="disabled_stage3_startup_hardware_retained") {
        readyRobot();
        CHECK(!SystemConfig::Mechanism::STAGE3_ENABLED);
        static_assert(SystemConfig::Mechanism::MOTOR_COUNT==3, "Shared MKS hardware remains configured");
        const auto state=MechanismController::status();
        CHECK(state.motorOnlineMask==7 && state.positionKnownMask==7 && state.currentConfiguredMask==7);
        CHECK(state.current4Ma==3000 && state.current5Ma==3000 && state.current6Ma==5200);
        for(uint8_t node:{uint8_t(4),uint8_t(5),uint8_t(6)}) {
            CHECK(mksCount(0x8C,node)==1 && mksCount(0x83,node)==1 && mksCount(0xF3,node,1)==1);
            CHECK(mksCount(0x91,node)==(node==6 ? 1U : 0U));
            CHECK(mksCount(0x92,node)==(node==6 ? 0U : 1U));
        }
        CHECK(mksCount(0xFE)==0 && Mock::servoAttached[3] && Mock::servo[3]==2250);
        const auto wire=request("status");
        CHECK(wire.find("\"stage_ids\":[1,2,4,5,6]")!=std::string::npos);
        CHECK(wire.find("\"stage3_enabled\":false")!=std::string::npos);
        CHECK(wire.find("\"motor_ids\":[4,5,6]")!=std::string::npos);
        CHECK(wire.find("\"servo_pins\":[2,3,4,5,6,9,10,11,12]")!=std::string::npos);
    } else if(name=="disabled_stage3_api_and_ram_rejected") {
        readyRobot(); const auto before=MechanismController::status();
        const auto moves=mksCount(0xFE),homes=mksCount(0x91);
        CHECK(!MechanismController::selectStage(3) && !MechanismController::selectProfile(3));
        CHECK(!MechanismController::planBegin(1,1,3,1));
        request("stage 3",false); request("plan.begin 1 1 3 1",false);
        const auto after=MechanismController::status();
        CHECK(mechanismReady() && after.stage==before.stage && after.pendingStage==0);
        CHECK(after.planVersion==before.planVersion && !after.uploading);
        CHECK(mksCount(0xFE)==moves && mksCount(0x91)==homes && !SystemSafety::stopped());
        request("next"); completeMechanism(); CHECK(MechanismController::status().completed==1);
    } else if(name=="disabled_stage3_rc_held_priority_ignored") {
        readyRobot(); button(12,false); button(10,true); completeMechanism();
        CHECK(MechanismController::status().stage==2 && RcController::status().selectedStage==2);
        const auto moves=mksCount(0xFE),homes=mksCount(0x91);
        button(11,true); advance(300);
        CHECK(mechanismReady() && MechanismController::status().stage==2 && MechanismController::status().pendingStage==0);
        CHECK(RcController::status().selectedStage==2 && mksCount(0xFE)==moves && mksCount(0x91)==homes);
        button(12,true); completeMechanism(); CHECK(MechanismController::status().stage==4);
        button(12,false); completeMechanism();
        CHECK(MechanismController::status().stage==2 && RcController::status().selectedStage==2);
        button(11,false); button(11,true); advance(300);
        CHECK(mechanismReady() && MechanismController::status().stage==2 && !SystemSafety::stopped());
    } else if(name=="disabled_stage3_rc_invalid_boot_selector_does_not_block") {
        initialize();
        radio().channels[11]=899; // Disabled CH11 must not invalidate a live selector.
        radio().channels[12]=1000; radio().channels[10]=2000;
        for(unsigned i=0;i<2000 && !(mechanismReady() && MechanismController::status().stage==2);++i) {
            tick(); CHECK(!SystemSafety::stopped());
        }
        CHECK(mechanismReady() && MechanismController::status().stage==2 && RcController::status().selectedStage==2);
        radio().channels[12]=2000;
        for(unsigned i=0;i<40;++i) { radio().channels[11]=(i%2 ? 2000 : 1000); tick(); }
        completeMechanism();
        CHECK(MechanismController::status().stage==4 && RcController::status().selectedStage==4);
        CHECK(!SystemSafety::stopped());
    } else if(name=="disabled_stage3_stage1_vase_channel_retained") {
        readyRobot(); selectRcStage(1); completeMechanism();
        const auto homes=mksCount(0x91);
        CHECK(!MechanismController::exitStage1(3)); request("stage1.exit 3",false); advance(50);
        CHECK(mechanismReady() && MechanismController::status().stage==1 && MechanismController::status().pendingStage==0);
        CHECK(mksCount(0x91)==homes);
        const auto moves=mksCount(0xFE,6); propRcPulse(2); completeMechanism();
        const auto state=MechanismController::status();
        CHECK(RcController::status().stage1Mode && RcController::status().selectedProp==2);
        CHECK(state.stage==1 && state.prop==2 && state.completed==1 && state.steps==propStepCount(2));
        CHECK(mksCount(0xFE,6)==moves+1 && target(6)==propMotor(2,1).position);
        CHECK(Mock::servo[2]==propPulse(2,1,2) && Mock::servo[5]==propPulse(2,1,5));
        CHECK(mksCount(0xFE,4)==0 && mksCount(0xFE,5)==0 && !SystemSafety::stopped());
    } else if(name=="disabled_stage3_stage2_pair_return_and_debug_retained") {
        stage2PairReady();
        CHECK(Mock::servo[9]==1800 && Mock::servo[4]==1300 && Mock::servo[3]==1580);
        setTrimKnob(2000); settleStage2Pair(); assertStage2Pair(0,0,stage2TrimLimit);
        selectRcStage(4); completeMechanism();
        CHECK(MechanismController::status().stage==4 && MechanismController::status().position4==0 && MechanismController::status().position5==0);
        CHECK(Mock::servo[3]==2250 && mksCount(0x91,4)==0 && mksCount(0x91,5)==0);
        request("mode debug");
        request("debug.move 4 456 100 5"); completeMechanism();
        request("debug.move 5 -654 100 5"); completeMechanism();
        request("debug.servo 3 972 0"); completeMechanism();
        CHECK(MechanismController::status().position4==456 && MechanismController::status().position5==-654);
        CHECK(Mock::servoAttached[3] && Mock::servo[3]==1580 && !SystemSafety::stopped());
    } else return false;
    return true;
}

void test(const std::string &name) {
    if (testCh13StrobeSystem(name)) return;
    if (testDisabledStage3(name)) return;
    if (testStage2PairTrim(name)) return;
    if (testStage2BootHome(name)) return;
    if (testGlobalId6Upper(name)) return;
    if (testFruitSafety(name)) return;
    if (testFruitHeightTrim(name)) return;
    if (testStage4HeightTrim(name)) return;
    if (testFruit(name)) return;
    if (testPropHomeCycle(name)) return;
    if (testStage1TrimSystem(name) || testExtraServoSystem(name) || testStageSwitchSystem(name) || testPwmSystem(name) || testCylinderSystem(name) || testStage1PropsSystem(name) || testRcStage3Endurance(name) || testLowerAxisCurrentSystem(name) || testRemappedChannelsSystem(name)) return;
    if (name == "hardware_boot_demux") {
        readyRobot();
        CHECK(Serial.beginBaud == 115200 && Serial2.beginBaud == 420000);
        CHECK(Mock::baud[1] == 1000000 && Mock::baud[2] == 1000000);
        CHECK(ChassisController::status().healthy);
        for (uint8_t node : SystemConfig::Mechanism::MOTOR_IDS) {
            CHECK(mksCount(0x8C, node) == 1);
            CHECK(mksCount(0x83, node) == 1);
            std::vector<uint8_t> nodeCommands;
            for (const auto &frame : Mock::sent)
                if (mksFrame(frame) && frame.msg.id == node) nodeCommands.push_back(frame.msg.buf[0]);
            const std::vector<uint8_t> expectedCommands = node >= 6 ? std::vector<uint8_t>{0x8C, 0x83, 0xF3, 0x91}
                                                                   : std::vector<uint8_t>{0x8C, 0x83, 0xF3, 0x92};
            CHECK(nodeCommands == expectedCommands);
            CHECK(mksCount(0x91, node) == (node >= 6 ? 1U : 0U));
            CHECK(mksCount(0x92, node) == (node <= 5 ? 1U : 0U));
            CHECK(mksCount(0xF3, node, 1) == 1);
        }
        CHECK(mksCount(0xFE) == 0 && mksCount(0xFD) == 0);
        const auto state = MechanismController::status();
        CHECK(state.position4 == 0 && state.position5 == 0 && state.position6 == 0);
        CHECK(Mock::servo[2] == 1150 && Mock::servo[5] == 2100 && Mock::servoAttached[3]);
        CHECK(Mock::servo[4] == 2000 && Mock::servo[6] == 1400 && Mock::servo[9] == 1100);
        CHECK(state.positionKnownMask == 7 && SystemConfig::Mechanism::MOTOR_COUNT == 3);
        CHECK(state.current4Ma == 3000 && state.current5Ma == 3000 && state.current6Ma == 5200 && state.currentConfiguredMask == 7);
        CHECK(state.startupCurrentMa == 0);
        CHECK(Mock::servo[3] == 2250);
        const uint8_t expectedPins[9] = {2, 3, 4, 5, 6, 9, 10, 11, 12};
        const int expectedStandby[9] = {1150, 2250, 2000, 2100, 1400, 1100, 1500, 1500, 1500};
        static_assert(SystemConfig::Mechanism::SERVO_COUNT == 9, "Nine standby servo channels are required");
        for (uint8_t i = 0; i < SystemConfig::Mechanism::SERVO_COUNT; ++i) {
            CHECK(SystemConfig::Mechanism::SERVO_PINS[i] == expectedPins[i]);
            CHECK(SystemConfig::Mechanism::SERVO_START_US[i] == expectedStandby[i]);
            CHECK(Mock::servoAttached[expectedPins[i]] && Mock::servo[expectedPins[i]] == expectedStandby[i]);
        }
        for (const auto &frame : Mock::sent) {
            if (mksFrame(frame)) {
                CHECK(frame.msg.id >= 4 && frame.msg.id <= 6);
                if (frame.msg.buf[0] == 0x83) {
                    CHECK(frame.msg.len == 4 && frame.msg.id >= 4 && frame.msg.id <= 6);
                    CHECK(frame.msg.buf[1] == (frame.msg.id == 6 ? 0x14 : 0x0B));
                    CHECK(frame.msg.buf[2] == (frame.msg.id == 6 ? 0x50 : 0xB8));
                    CHECK(frame.msg.buf[3] == (frame.msg.id == 6 ? 0xED : uint8_t(frame.msg.id + 0x46)));
                }
                uint8_t checksum = uint8_t(frame.msg.id);
                for (uint8_t i = 0; i + 1 < frame.msg.len; ++i) checksum += frame.msg.buf[i];
                CHECK(checksum == frame.msg.buf[frame.msg.len - 1]);
            } else {
                const uint32_t node = frame.msg.id >> 5;
                CHECK((frame.bus == 1 && (node == 10 || node == 11)) ||
                      (frame.bus == 2 && (node == 12 || node == 13)));
            }
        }
        autoCompleteMoves = false;
        request("next"); CHECK(mechanismBusy());
        incomingMks(6, 0xFE, 2, 2); incomingMks(6, 0xFE, 2, 1, true); tick();
        CHECK(mechanismBusy() && MechanismController::status().position6 == 0);
        finish(); CHECK(mechanismReady() && MechanismController::status().position6 == stage4Position(1));
    } else if (name == "startup_retry_chassis_status") {
        initialize(); autoAckStartup = false; advance(1000);
        CHECK(mksCount(0x8C, 4) == 1 && mksCount(0x91) == 0);
        const std::string boot = request("status");
        CHECK(boot.find("\"firmware_id\":\"Robot_Integrated_Control\"") != std::string::npos);
        CHECK(boot.find("\"firmware_revision\":\"bench-test-v7\"") != std::string::npos);
        CHECK(boot.find("\"startup_return_zero\":true") != std::string::npos);
        CHECK(boot.find("\"startup_phase\":\"feedback_wait\"") != std::string::npos);
        CHECK(boot.find("\"startup_node\":4") != std::string::npos);
        CHECK(boot.find("\"startup_command\":140") != std::string::npos);
        CHECK(boot.find("\"position_known_mask\":0") != std::string::npos);
        const size_t sentAt = Mock::sent.size();
        const uint32_t updatesAt = radio().updateCount;
        advance(2000);
        CHECK(!SystemSafety::stopped() && !mechanismReady());
        CHECK(mksCount(0x8C, 4) >= 4 && mksCount(0x91) == 0 && mksCount(0xFE) == 0);
        CHECK(odriveCount(0x0D, sentAt) >= 400 && radio().updateCount >= updatesAt + 350);
        CHECK(ChassisController::status().mode == ChassisController::Mode::Running);
        autoAckStartup = true; advance(700);
        CHECK(!mechanismReady() && !SystemSafety::stopped());
        CHECK(CylinderController::status().active && !CylinderController::status().fullyRetracted);
        CHECK(MechanismController::status().positionKnownMask==7);
        completeMechanism();
        CHECK(mechanismReady() && !SystemSafety::stopped() && CylinderController::status().fullyRetracted);
        for (uint8_t node : SystemConfig::Mechanism::MOTOR_IDS) CHECK(mksCount(0x91, node) == (node >= 6 ? 1U : 0U));
        const std::string ready = request("status");
        CHECK(ready.find("\"startup_phase\":\"ready\"") != std::string::npos);
        CHECK(ready.find("\"position_known_mask\":7") != std::string::npos);
    } else if (name == "startup_current_wait_chassis_status") {
        holdCurrentAckNode = 6; initialize(); advance(1200);
        CHECK(mksCount(0x83, 6) == 1 && mksCount(0xF3, 6, 1) == 0 && mksCount(0x91) == 0);
        auto pending = MechanismController::status();
        CHECK(pending.startupNode == 6 && pending.startupCommand == 0x83 && pending.startupCurrentMa == 5200);
        CHECK(pending.current4Ma == 3000 && pending.current5Ma == 3000 && pending.current6Ma == 0 && pending.currentConfiguredMask == 3);
        const auto wire = request("status");
        CHECK(wire.find("\"startup_phase\":\"current_wait\"") != std::string::npos);
        CHECK(wire.find("\"startup_command\":131") != std::string::npos);
        CHECK(wire.find("\"startup_current_ma\":5200") != std::string::npos);
        CHECK(wire.find("\"current4_ma\":3000,\"current5_ma\":3000") != std::string::npos);
        writeStatusFixture(".pio/host-tests/current_wait6_status_fixture.json", wire);
        CHECK(wire.find("\"current6_ma\":0,\"current_configured_mask\":3") != std::string::npos);
        CHECK(wire.find("\"startup_current_settings\":[{\"node\":4,\"milliamps\":3000},{\"node\":5,\"milliamps\":3000},{\"node\":6,\"milliamps\":5200}]") != std::string::npos);
        request("next", false); request("home", false); request("stage 2", false); request("mode debug", false);
        radio().channels[SystemConfig::Chassis::FORWARD.number] = SystemConfig::Chassis::FORWARD.maximum;
        const size_t sentAt = Mock::sent.size(); const auto updatesAt = radio().updateCount;
        advance(2000);
        CHECK(mksCount(0x83, 6) >= 4 && mksCount(0xF3, 6, 1) == 0 && mksCount(0x91) == 0);
        CHECK(odriveCount(0x0D, sentAt) >= 528 && radio().updateCount >= updatesAt + 400);
        CHECK(ChassisController::status().mode == ChassisController::Mode::Running && !SystemSafety::stopped());
        holdCurrentAckNode=0; incomingMks(6,0x83,1); advance(200); completeMechanism();
        CHECK(mechanismReady() && MechanismController::status().currentConfiguredMask==7);
        CHECK(mksCount(0x91,6)==1 && mksCount(0x91,7)==0);
        const auto ready=request("status"); CHECK(ready.find("current7_ma")==std::string::npos);
    } else if (name == "startup_current_reject_stops_whole_robot") {
        holdCurrentAckNode=6; initialize(); advance(1250);
        CHECK(MechanismController::status().startupNode==6 && mksCount(0x83,6)==1);
        CHECK(MechanismController::status().current6Ma==0 && mksCount(0x91)==0);
        const size_t stoppedAt=Mock::sent.size(); incomingMks(6,0x83,0); tick(); assertEmergencyStopped(stoppedAt);
        CHECK(std::string(SystemSafety::reason()).find("ID 6 0x83 5200 mA")!=std::string::npos);
        const auto rejected=request("status");
        CHECK(rejected.find("\"current6_ma\":0,\"current_configured_mask\":3")!=std::string::npos);
        const unsigned currentsAt=mksCount(0x83); advance(11000);
        CHECK(mksCount(0x83)==currentsAt && mksCount(0xF3,6,1)==0 && mksCount(0x91)==0);
    } else if (name == "startup_current_usb_stop") {
        holdCurrentAckNode = 6; initialize(); advance(1200);
        CHECK(MechanismController::status().startupCommand == 0x83);
        const size_t stoppedAt = Mock::sent.size(); request("stop"); assertEmergencyStopped(stoppedAt);
        const unsigned currentsAt = mksCount(0x83); incomingMks(6, 0x83, 1); advance(11000);
        CHECK(mksCount(0x83) == currentsAt && mksCount(0xF3, 6, 1) == 0 && mksCount(0x91) == 0);
    } else if (name == "ramp_chassis_concurrency") {
        readyRobot(); request("next"); completeMechanism(); request("next"); completeMechanism();
        autoCompleteMoves = false; request("next");
        CHECK(target() == stage4Position(3) && mechanismBusy());
        const size_t sentAt = Mock::sent.size();
        const uint32_t updatesAt = radio().updateCount;
        const int pulseAt = Mock::servo[5];
        radio().channels[SystemConfig::Chassis::FORWARD.number] = SystemConfig::Chassis::FORWARD.maximum;
        advance(1000);
        CHECK(Mock::servo[5] > pulseAt && Mock::servo[5] < 1550);
        CHECK(odriveCount(0x0D, sentAt) >= 260 && radio().updateCount >= updatesAt + 200);
        CHECK(ChassisController::status().mode == ChassisController::Mode::Running);
        CHECK(ChassisController::status().wheel[0] < 0 && ChassisController::status().wheel[1] > 0);
        finish(); CHECK(mechanismBusy()); // Motor completion alone does not release the ramp barrier.
        advance(1100);
        CHECK(mechanismReady() && Mock::servo[5] == 1550);
        CHECK(odriveCount(0x0D, sentAt) >= 552 && !SystemSafety::stopped());
    } else if (name == "usb_backpressure_global_stop") {
        readyRobot(); radio().channels[SystemConfig::Chassis::FORWARD.number] = SystemConfig::Chassis::FORWARD.maximum;
        Serial.capacity = 0;
        for (unsigned i = 0; i < 100; ++i) queue("status");
        const size_t startedAt = Mock::sent.size();
        const uint32_t updatesAt = radio().updateCount;
        advance(500);
        CHECK(Serial.input.empty() && ChassisController::status().mode == ChassisController::Mode::Running);
        CHECK(odriveCount(0x0D, startedAt) >= 128 && radio().updateCount >= updatesAt + 100);
        const unsigned stopId = queue("stop"); const size_t stoppedAt = Mock::sent.size(); tick();
        assertEmergencyStopped(stoppedAt);
        CHECK(lineFor(stopId).empty());
        Serial.capacity = 4096; advance(600);
        CHECK(lineFor(stopId).find(" OK accepted") != std::string::npos);
        CHECK(ChassisController::status().mode == ChassisController::Mode::Locked);
    } else if (name == "usb_stop_preempts_ready_rc") {
        readyRobot(); autoCompleteMoves = false;
        radio().channels[6] = 2000; tick(1); advance(29);
        CHECK(mechanismReady() && MechanismController::status().step == 0);
        const size_t stoppedAt = Mock::sent.size(); queue("stop"); tick(1);
        assertEmergencyStopped(stoppedAt);
        CHECK(mksCount(0xFE) == 0 && MechanismController::status().step == 0);
    } else if (name == "rc_web_busy_interlock") {
        readyRobot(); autoCompleteMoves = false;
        request("next"); CHECK(target() == stage4Position(1) && mechanismBusy());
        const unsigned firstMoveCount = mksCount(0xFE);
        request("next", false); request("prev", false);
        button(6, true); CHECK(mksCount(0xFE) == firstMoveCount);
        finish(); CHECK(mechanismReady());
        advance(80); CHECK(MechanismController::status().completed == 1);
        request("next"); CHECK(MechanismController::status().completed == 2);
        button(6, false); button(6, true);
        CHECK(target() == stage4Position(3) && mechanismBusy() && mksCount(0xFE) == firstMoveCount + 1);
        request("next", false); finish(); advance(2100);
        CHECK(mechanismReady() && MechanismController::status().completed == 3);
        advance(50); CHECK(MechanismController::status().completed == 3);
    } else if (name == "status_capabilities_fixture") {
        readyRobot(); const std::string hello = request("hello");
        CHECK(hello.find("\"protocol\":1") != std::string::npos);
        CHECK(hello.find("\"firmware_id\":\"Robot_Integrated_Control\"") != std::string::npos);
        CHECK(hello.find("\"firmware_revision\":\"bench-test-v7\"") != std::string::npos);
        CHECK(hello.find("\"startup_return_zero\":true") != std::string::npos);
        CHECK(hello.find("\"startup_phase\":\"ready\"") != std::string::npos);
        CHECK(hello.find("\"startup_current_ma\":0") != std::string::npos);
        CHECK(hello.find("\"current4_ma\":3000,\"current5_ma\":3000") != std::string::npos);
        CHECK(hello.find("\"current6_ma\":5200,\"current_configured_mask\":7") != std::string::npos);
        CHECK(hello.find("\"position_known_mask\":7") != std::string::npos);
        CHECK(hello.find("\"stage_ids\":[1,2,4,5,6]") != std::string::npos);
        CHECK(hello.find("\"stage1_prop_ids\":[1,2,3,4,5]") != std::string::npos);
        CHECK(hello.find("\"motor_ids\":[4,5,6]") != std::string::npos);
        CHECK(hello.find("\"previous_available\":false") != std::string::npos);
        CHECK(hello.find("\"servo_pins\":[2,3,4,5,6,9,10,11,12]") != std::string::npos);
        CHECK(hello.find("\"stage3_enabled\":false") != std::string::npos);
        CHECK(hello.find("\"chassis\":") != std::string::npos && hello.find("\"rc\":") != std::string::npos);
        const std::string status = request("status");
        const size_t begin = status.find('{'); CHECK(begin != std::string::npos);
        FILE *file = fopen(".pio/host-tests/status_fixture.json", "wb"); CHECK(file != nullptr);
        const std::string json = status.substr(begin);
        CHECK(fwrite(json.data(), 1, json.size(), file) == json.size()); CHECK(fclose(file) == 0);
    } else if (name == "wire_profiles_retained_park") {
        readyRobot();
        for (uint8_t stage : ALL_STAGES) {
            request("plan.begin 1 1 " + std::to_string(stage) + " 1");
            if (stage == 5) request("plan.action 0 2 0 0 0 0 0 0 1 1 0");
            else if (stage == 2) request("plan.action 0 1 0 0 0 4 1777 0 1 1 0");
            else if (stage == 3) request("plan.action 0 0 -333 320 0 0 0 0 1 1 4");
            else request("plan.action 0 0 " + std::to_string(stage == 1 ? 101 : -444) + " 320 0 0 0 0 1 1 6");
            request("plan.step 0 0 1"); request("plan.commit"); completeMechanism();
            CHECK(MechanismController::status().stage == stage && MechanismController::status().pendingStage == 0);
            CHECK(MechanismController::status().planVersion == stage);
            request("next"); completeMechanism();
        }
        for (uint8_t stage : ALL_STAGES) {
            const unsigned homesAt = mksCount(0x91);
            request("stage " + std::to_string(stage)); completeMechanism();
            const auto selected = MechanismController::status();
            CHECK(selected.stage == stage && selected.runtimePlan && selected.steps == 1 && selected.completed == 0 && selected.progress == 0 && !selected.previousAvailable);
            const unsigned movesAt = mksCount(0xFE);
            request("next"); completeMechanism();
            const auto done = MechanismController::status();
            if (stage == 1) CHECK(done.position6 == 101);
            else if (stage == 2) CHECK(mksCount(0xFE) == movesAt && Mock::servo[4] == 1777);
            else if (stage == 3) CHECK(done.position4 == -333);
            else if (stage == 5) CHECK(done.completed==1 && mksCount(0xFE)==movesAt);
            else CHECK(done.position6 == -444);
            CHECK(mksCount(0x91) == homesAt && Mock::servoAttached[3]);
        }
        for (const auto &frame : Mock::sent) if (mksFrame(frame)) CHECK(frame.msg.id >= 4 && frame.msg.id <= 6);
    } else if (name == "serial_validation_signed24") {
        readyRobot(); request("mode debug");
        const unsigned movesAt = mksCount(0xFE);
        for (const std::string &command : {"debug.move 6 -8388609 320 0", "debug.move 6 8388608 320 0",
                "debug.move 8 100 320 0", "debug.move 3 100 320 0",
                "debug.gohome 4", "debug.gohome 5", "debug.move 6 100 0 0", "debug.move 6 100 320 256",
                "debug.move 6 -600", "debug.servo 1 900 0", "debug.servo 2 1801 0",
                "stage 0", "stage 6", "next extra", "mode normal extra", "debug.move 6 1x 320 0"})
            request(command, false);
        CHECK(mksCount(0xFE) == movesAt);
        request("debug.move 6 -8388608 320 0"); completeMechanism();
        CHECK(target(6) == -8388608 && lastMove(6).buf[4] == 0x80);
        request("debug.move 6 8388607 320 0"); completeMechanism();
        CHECK(target(6) == 10500 && lastMove(6).buf[4] == 0x00);
        request("debug.move 4 8388607 320 0"); completeMechanism();
        CHECK(target(4) == 8388607 && lastMove(4).buf[4] == 0x7F);
        request("mode normal");
        const unsigned truncatedId = requestId++;
        Serial.input += "@" + std::to_string(truncatedId) + " next";
        const unsigned beforeTruncated = mksCount(0xFE); advance(30);
        CHECK(lineFor(truncatedId).empty() && mksCount(0xFE) == beforeTruncated);
        Serial.input += "\n"; tick(); advance(30);
        CHECK(lineFor(truncatedId).find(" OK accepted") != std::string::npos);
    } else if (name == "rc_stage2_steps_previous") {
      readyRobot(); selectRcStage(2); completeMechanism();
      CHECK(MechanismController::status().completed==0 && Mock::servo[9]==1100 && Mock::servo[4]==2000);
      for (uint8_t step=1;step<=Stage2::STEP_COUNT;++step) {
        nextRcPulse(); completeMechanism(); CHECK(MechanismController::status().completed==step);
        for (uint8_t pin : {uint8_t(4),uint8_t(6),uint8_t(9)}) CHECK(Mock::servo[pin]==stage2Pulse(step,pin));
      }
      nextRcPulse(); completeMechanism(); CHECK(MechanismController::status().completed==Stage2::REPEAT_FROM);
      CHECK(Mock::servo[6]==stage2Pulse(6,6) && CylinderController::status().referenceAction==MechanismConfig::CylinderAction::Retract);
      button(5,false); button(5,true); completeMechanism(); CHECK(MechanismController::status().completed==6);
      button(5,false); button(5,true); completeMechanism(); CHECK(MechanismController::status().completed==5);
      CHECK(CylinderController::status().referenceKnown && CylinderController::status().referenceAction==MechanismConfig::CylinderAction::Retract);
      button(5,false); button(5,true); advance(100); CHECK(Mock::gpioState[17]==HIGH && Mock::gpioState[16]==LOW);
      completeMechanism(); CHECK(MechanismController::status().completed==4 && CylinderController::status().referenceAction==MechanismConfig::CylinderAction::Extend);
      CHECK(mksCount(0xFE)==0 && mksCount(0x91)==1); assertCylinderStopped();
    } else if (name == "rc_stage3_cycle_previous_home") {
        readyRobot(); button(12, false); button(11, true); button(6, false);
        CHECK(MechanismController::status().stage == 3 && MechanismController::status().completed == 0);
        const uint8_t steps[5] = {1,2,3,4,2};
        for (unsigned i=0;i<5;++i) {
            button(6,true);completeMechanism();
            CHECK(target(4)==stage3Motor(steps[i],4).position && target(5)==stage3Motor(steps[i],5).position);
            CHECK(Mock::servo[3]==stage3Pulse(steps[i]) && MechanismController::status().completed==steps[i]);
            button(6,false);
        }
        button(5,true);completeMechanism();
        CHECK(target(4)==stage3Motor(4,4).position && target(5)==stage3Motor(4,5).position && MechanismController::status().completed==4);
        request("home");completeMechanism();
        CHECK(target(4)==0 && target(5)==0 && Mock::servo[3]==2250);
        CHECK(mksCount(0x91,4)==0 && mksCount(0x91,5)==0 && mksCount(0x91)==1);
    } else if (name == "stage3_pending_park_chassis") {
      readyRobot(); request("stage 3"); completeMechanism(); autoCompleteMoves=false; request("next");
      CHECK(target(4)==stage3Motor(1,4).position && target(5)==stage3Motor(1,5).position && mechanismBusy());
      request("stage 2"); CHECK(MechanismController::status().pendingStage==2); CHECK(mksCount(0xFE,7)==0); request("next",false);
      const size_t sentAt=Mock::sent.size(); const auto updatesAt=radio().updateCount; advance(500);
      CHECK(odriveCount(0x0D,sentAt)>=128 && radio().updateCount>=updatesAt+100);
      finish(4); CHECK(target(4)==stage3Motor(1,4).position && MechanismController::status().stage==3);
      finish(5); CHECK(target(4)==0 && target(5)==0 && Mock::servo[3]==2250);
      CHECK(MechanismController::status().stage==3 && std::string(MechanismController::status().phase)=="return_wait" && mksCount(0xFE,7)==0);
      finish(4); CHECK(MechanismController::status().stage==3); finish(5); completeMechanism();
      CHECK(MechanismController::status().stage==2 && mechanismReady() && MechanismController::status().progress==0 && mksCount(0xFE,7)==0);
      CHECK(mksCount(0x91)==1 && mksCount(0x91,4)==0 && mksCount(0x91,5)==0 && Mock::servo[9]==1100);
    } else if (name == "boot_single_home_completion_chassis") {
        initialize(); autoCompleteHomes=false; advance(1250);
        CHECK(mksCount(0x91,6)==1 && mksCount(0x91,7)==0 && !mechanismReady());
        CHECK(MechanismController::status().positionKnownMask==3);
        CHECK(mksCount(0x92,4)==1 && mksCount(0x92,5)==1 && mksCount(0x91,4)==0 && mksCount(0x91,5)==0);
        request("next",false); const size_t sentAt=Mock::sent.size(); const auto updatesAt=radio().updateCount;
        advance(1000); CHECK(odriveCount(0x0D,sentAt)>=260 && radio().updateCount>=updatesAt+200);
        CHECK(!SystemSafety::stopped() && ChassisController::status().mode==ChassisController::Mode::Running);
        finish(6,0x91); CHECK(!mechanismReady() && CylinderController::status().active);
        completeMechanism(); CHECK(mechanismReady() && MechanismController::status().positionKnownMask==7);
        CHECK(MechanismController::status().position6==0); assertCylinderStopped();
    } else if (name == "stage2_entry_busy_chassis") {
      readyRobot(); request("next"); completeMechanism(); const unsigned firstMove=mksCount(0xFE,6);
      autoCompleteMoves=false; selectRcStage(2);
      CHECK(MechanismController::status().stage==4 && MechanismController::status().pendingStage==2);
      CHECK(target(6)==0 && mksCount(0xFE,6)==firstMove+1 && mksCount(0xFE,7)==0);
      request("next",false); button(6,true); finish(6);
      CHECK(MechanismController::status().stage==2 && mechanismReady() && !CylinderController::status().active);
      CHECK(CylinderController::status().fullyRetracted);
      sticks(SystemConfig::Chassis::FORWARD.maximum,SystemConfig::Chassis::STRAFE.middle,SystemConfig::Chassis::ROTATE.middle);
      advance(15); assertSteeringCadence(2,1,0,0,0,4500); CHECK(mechanismReady());
      completeMechanism(); CHECK(MechanismController::status().completed==0); advance(100); CHECK(MechanismController::status().completed==0);
      nextRcPulse(); completeMechanism(); CHECK(MechanismController::status().completed==1);
      CHECK(CylinderController::status().referenceAction==stage2Reference(1) && Mock::servo[9]==stage2Pulse(1,9) && Mock::servo[4]==stage2Pulse(1,4));
    } else if (name == "stage_home_pending_latest_chassis") {
      readyRobot(); request("next"); completeMechanism(); autoCompleteMoves=false;
      request("stage 2"); request("stage 1"); request("stage 2");
      CHECK(MechanismController::status().stage==4 && MechanismController::status().pendingStage==2 && target(6)==0);
      const size_t firstAt=Mock::sent.size(); advance(500); CHECK(odriveCount(0x0D,firstAt)>=128);
      finish(6); completeMechanism(); CHECK(MechanismController::status().stage==2);
      request("next"); completeMechanism(); CHECK(CylinderController::start(MechanismConfig::CylinderAction::Extend,100)); advance(160);
      const unsigned f6=mksCount(0xFE,6); request("stage 4"); request("stage 1");
      CHECK(MechanismController::status().stage==2 && MechanismController::status().pendingStage==1);
      const size_t secondAt=Mock::sent.size(); advance(500); CHECK(odriveCount(0x0D,secondAt)>=128);
      request("next",false); completeMechanism();
      CHECK(MechanismController::status().stage==1 && MechanismController::status().progress==0 && mksCount(0xFE,6)==f6);
      CHECK(mksCount(0x91)==1 && mksCount(0x91,4)==0 && mksCount(0x91,5)==0);
    } else if (name == "stage_home_stop_chassis") {
      readyRobot(); request("next"); completeMechanism(); autoCompleteMoves=false; request("stage 2"); request("stage 1");
      CHECK(std::string(MechanismController::status().phase)=="return_wait" && MechanismController::status().stage==4); const unsigned moves=mksCount(0xFE);
      const size_t stoppedAt=Mock::sent.size(); request("stop"); assertEmergencyStopped(stoppedAt);
      advance(500); CHECK(mksCount(0x91)==1 && mksCount(0xFE)==moves && mksCount(0xFE,7)==0 && MechanismController::status().pendingStage==0);
    } else if (name == "stage_home_entry_previous_chassis") {
        readyRobot(); request("next"); completeMechanism(); CHECK(MechanismController::status().position6==stage4Position(1));
        request("stage 1"); completeMechanism(); CHECK(MechanismController::status().position6==0 && MechanismController::status().completed==0 && !MechanismController::status().previousAvailable);
        request("next"); completeMechanism(); request("prev"); completeMechanism();
        CHECK(MechanismController::status().position6==0 && Mock::servo[2]==1150 && Mock::servo[5]==2100 && MechanismController::status().progress==0);
        request("prev",false);
    } else if (name == "stage_return_busy_step_chassis") {
        readyRobot(); autoCompleteMoves=false; request("next"); const unsigned first=mksCount(0xFE,6);
        request("stage 2"); request("stage 1"); request("stage 2");
        CHECK(MechanismController::status().stage==4 && MechanismController::status().pendingStage==2 && std::string(MechanismController::status().phase)=="busy" && mksCount(0xFE,6)==first);
        const size_t sentAt=Mock::sent.size(); const auto updatesAt=radio().updateCount; advance(500);
        CHECK(odriveCount(0x0D,sentAt)>=128 && radio().updateCount>=updatesAt+100); finish(6);
        CHECK(MechanismController::status().stage==4 && std::string(MechanismController::status().phase)=="return_wait" && target(6)==0 && mksCount(0xFE,6)==first+1);
        finish(6); completeMechanism(); CHECK(mechanismReady() && MechanismController::status().stage==2 && MechanismController::status().completed==0 && mksCount(0xFE,7)==0 && mksCount(0x91)==1);
    } else if (name == "orientation_all_stage_bases") {
        readyRobot();
        for (uint8_t stage : ALL_STAGES) {
            request("stage " + std::to_string(stage)); completeMechanism();
            for (unsigned direction = 0; direction < 6; ++direction) {
                const bool positive = direction % 2 == 0;
                int forward=SystemConfig::Chassis::FORWARD.middle, strafe=SystemConfig::Chassis::STRAFE.middle, rotate=SystemConfig::Chassis::ROTATE.middle;
                float rawForward=0, rawStrafe=0, rawRotate=0;
                if (direction < 2) { forward=positive?SystemConfig::Chassis::FORWARD.maximum:SystemConfig::Chassis::FORWARD.minimum; rawForward=positive?1.0f:-1.0f; }
                else if (direction < 4) { strafe=positive?SystemConfig::Chassis::STRAFE.maximum:SystemConfig::Chassis::STRAFE.minimum; rawStrafe=positive?1.0f:-1.0f; }
                else { rotate=positive?SystemConfig::Chassis::ROTATE.maximum:SystemConfig::Chassis::ROTATE.minimum; rawRotate=positive?1.0f:-1.0f; }
                sticks(forward,strafe,rotate); assertSteeringCadence(stage,rawForward,rawStrafe,rawRotate);
            }
        }
        CHECK(mksCount(0x91)==1 && mksCount(0x91,4)==0 && mksCount(0x91,5)==0);
    } else if (name == "orientation_calibration_before_mapping") {
        readyRobot();
        for (uint8_t stage : {uint8_t(1),uint8_t(2)}) {
            request("stage " + std::to_string(stage)); completeMechanism();
            const auto forward=SystemConfig::Chassis::FORWARD, strafe=SystemConfig::Chassis::STRAFE, rotate=SystemConfig::Chassis::ROTATE;
            const int dead=SystemConfig::Chassis::DEAD_ZONE;
            sticks(forward.middle+dead,strafe.middle-dead,rotate.middle+dead); assertSteeringCadence(stage,0,0,0);
            const int forwardSpan=forward.maximum-forward.middle-dead, strafeSpan=strafe.middle-dead-strafe.minimum;
            sticks(forward.middle+dead+forwardSpan/2,strafe.middle-dead-strafeSpan/3,rotate.middle);
            assertSteeringCadence(stage,float(forwardSpan/2)/float(forwardSpan),-float(strafeSpan/3)/float(strafeSpan),0);
            sticks(forward.maximum,strafe.maximum,rotate.maximum); assertSteeringCadence(stage,1,1,1);
        }
    } else if (name == "orientation_serial_return_pending_immediate") {
        readyRobot(); request("next"); completeMechanism(); autoCompleteMoves=false;
        sticks(SystemConfig::Chassis::FORWARD.maximum,SystemConfig::Chassis::STRAFE.middle,SystemConfig::Chassis::ROTATE.middle); advance(15);
        request("stage 2"); request("stage 1");
        CHECK(MechanismController::status().stage==4 && MechanismController::status().pendingStage==1 && std::string(MechanismController::status().phase)=="return_wait");
        CHECK(RcController::status().selectedStage==4 && target(6)==0); assertSteeringCadence(1,1,0,0);
        finish(6); CHECK(mechanismReady() && MechanismController::status().stage==1 && RcController::status().selectedStage==4);
        advance(15); assertSteeringCadence(1,1,0,0); CHECK(mksCount(0x91)==1);
    } else if (name == "orientation_rc_pending_immediate") {
        readyRobot(); request("next"); completeMechanism(); autoCompleteMoves=false;
        sticks(SystemConfig::Chassis::FORWARD.maximum,SystemConfig::Chassis::STRAFE.middle,SystemConfig::Chassis::ROTATE.middle);
        button(SystemConfig::Rc::STAGE_CHANNELS[2],false); button(SystemConfig::Rc::STAGE_CHANNELS[0],true);
        CHECK(RcController::status().selectedStage==2 && MechanismController::status().stage==4 && MechanismController::status().pendingStage==2);
        CHECK(std::string(MechanismController::status().phase)=="return_wait"); assertSteeringCadence(2,1,0,0);
        finish(6); completeMechanism(); advance(15); CHECK(mechanismReady() && MechanismController::status().stage==2); assertSteeringCadence(2,1,0,0);
    } else if (name == "orientation_busy_step_and_return") {
        readyRobot(); request("stage 2"); completeMechanism();
        const uint8_t extensionStep=firstStage2ExtensionStep();
        for(uint8_t step=1;step<extensionStep;++step) { request("next"); completeMechanism(); }
        request("next"); advance(100);
        CHECK(CylinderController::status().active && CylinderController::status().action==MechanismConfig::CylinderAction::Extend);
        CHECK(Mock::gpioState[SystemConfig::Cylinder::IN1_PIN]==HIGH && Mock::gpioState[SystemConfig::Cylinder::IN2_PIN]==LOW);
        sticks(SystemConfig::Chassis::FORWARD.maximum,SystemConfig::Chassis::STRAFE.middle,SystemConfig::Chassis::ROTATE.middle);
        request("stage 4"); CHECK(Mock::gpioState[SystemConfig::Cylinder::IN1_PIN]==LOW && Mock::gpioState[SystemConfig::Cylinder::IN2_PIN]==LOW);
        CHECK(MechanismController::status().stage==2 && MechanismController::status().pendingStage==4);
        assertSteeringCadence(4,1,0,0); completeMechanism(); advance(15);
        CHECK(MechanismController::status().stage==4); assertSteeringCadence(4,1,0,0);
    } else if (name == "orientation_stage3_pair_park_barrier") {
        readyRobot(); request("stage 3"); completeMechanism(); request("next"); completeMechanism(); autoCompleteMoves=false;
        sticks(SystemConfig::Chassis::FORWARD.maximum,SystemConfig::Chassis::STRAFE.middle,SystemConfig::Chassis::ROTATE.middle); advance(15); request("stage 2");
        CHECK(MechanismController::status().stage==3 && MechanismController::status().pendingStage==2 && target(4)==0 && target(5)==0);
        assertSteeringCadence(2,1,0,0); finish(5); CHECK(MechanismController::status().stage==3 && !mechanismReady()); assertSteeringCadence(2,1,0,0);
        finish(4); completeMechanism(); advance(15); CHECK(MechanismController::status().stage==2 && mechanismReady()); assertSteeringCadence(2,1,0,0);
        CHECK(mksCount(0x91)==1 && mksCount(0x91,4)==0 && mksCount(0x91,5)==0);
    }   else if (name == "serial_removed_node7_and_cylinder_stop") {
        readyRobot(); request("mode debug");
        request("debug.move 7 -321 320 2",false); request("debug.gohome 7",false);
        CHECK(mksCount(0xFE,7)==0 && mksCount(0x91,7)==0);
        request("debug.cylinder 1 6000"); advance(100);
        CHECK(Mock::gpioState[SystemConfig::Cylinder::IN1_PIN]==HIGH && Mock::gpioState[SystemConfig::Cylinder::IN2_PIN]==LOW);
        const size_t stoppedAt=Mock::sent.size(); request("stop"); assertEmergencyStopped(stoppedAt);
    } else if (name == "manual_chassis_no_imu_dependency") {
        readyRobot(); const size_t sentAt=Mock::sent.size(); const auto updatesAt=radio().updateCount;
        advance(2500);
        CHECK(odriveCount(0x0D,sentAt)>=664 && radio().updateCount>=updatesAt+500);
        CHECK(mechanismReady() && ChassisController::status().mode==ChassisController::Mode::Running && !SystemSafety::stopped());
        for (float wheel:ChassisController::status().wheel) CHECK(wheel==0);
        for (const auto &event:Mock::gpioModes) CHECK(event.pin!=18 && event.pin!=19);
        const auto wire=request("status");
        CHECK(wire.find("\"imu\":")==std::string::npos);
        CHECK(wire.find("\"yaw\":")==std::string::npos);
        CHECK(wire.find("yaw_compensation")==std::string::npos);
        CHECK(Serial.output.find("IMU warning")==std::string::npos);
        sticks(SystemConfig::Chassis::FORWARD.maximum,SystemConfig::Chassis::STRAFE.middle,SystemConfig::Chassis::ROTATE.middle);
        advance(30); assertSteeringCadence(4,1,0,0);
        const size_t stoppedAt=Mock::sent.size(); request("stop"); assertEmergencyStopped(stoppedAt);
    }   else if (name == "serial_overflow_stop") {
        readyRobot();
        Serial.input += "@999 next " + std::string(600, 'x') + "\n";
        const unsigned stopId = queue("stop"); const size_t stoppedAt = Mock::sent.size();
        for (unsigned i = 0; i < 20 && !SystemSafety::stopped(); ++i) tick();
        CHECK(lineFor(999).empty()); assertEmergencyStopped(stoppedAt);
        advance(30); CHECK(lineFor(stopId).find(" OK accepted") != std::string::npos);
        CHECK(mksCount(0xFE) == 0);
    } else if (name == "mks_fault_stops_whole_robot") {
        readyRobot(); radio().channels[SystemConfig::Chassis::FORWARD.number] = SystemConfig::Chassis::FORWARD.maximum;
        advance(30); autoCompleteMoves = false; request("next");
        CHECK(mechanismBusy() && ChassisController::status().mode == ChassisController::Mode::Running);
        const size_t stoppedAt = Mock::sent.size();
        incomingMks(6, 0xFE, 3); tick(); assertEmergencyStopped(stoppedAt);
        CHECK(std::string(SystemSafety::reason()).find("MKS") != std::string::npos);
        request("next", false); request("home", false); request("mode debug", false);
    } else if (name == "heartbeat_timeout_during_motion") {
        readyRobot(); autoCompleteMoves = false; request("next");
        CHECK(mechanismBusy()); advance(505, false);
        CHECK(ChassisController::status().mode == ChassisController::Mode::Locked);
        CHECK(!ChassisController::status().healthy && mechanismBusy() && !SystemSafety::stopped());
        for (float wheel : ChassisController::status().wheel) CHECK(wheel == 0);
        finish(); CHECK(mechanismReady());
    } else throw std::runtime_error("Unknown full-system case");
}
int main(int argc, char **argv) {
    try { CHECK(argc == 2); test(argv[1]); assertNoId7(); assertCylinderGpioHistory(); printf("PASS %s\n", argv[1]); return 0; }
    catch (const std::exception &error) { fprintf(stderr, "FAIL %s: %s\n", argc > 1 ? argv[1] : "?", error.what()); return 1; }
}
