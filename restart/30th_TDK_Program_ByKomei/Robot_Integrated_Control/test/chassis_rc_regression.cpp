#include <stdio.h>
#include <stdexcept>
#include <string>
#include <vector>
#include <cstring>
#include <cmath>
#include <limits>
#include "system_config.h"
#include "mechanism_controller.h"
#include "chassis_controller.h"
#include "rc_controller.h"
#include "can_transport.h"
#include "system_safety.h"
#include <Arduino.h>
#include <AlfredoCRSF.h>

#define CHECK(expr) do { if (!(expr)) throw std::runtime_error(#expr); } while (0)

namespace Harness {
struct Frame { uint8_t bus; uint32_t id; std::vector<uint8_t> bytes; };
std::vector<Frame> sent;
uint32_t attempts = 0, failAt = 0;
bool stopped = false;
MechanismController::Status mechanism{};
unsigned nextCount = 0, previousCount = 0, selectCount = 0, triggerCount = 0, exitCount = 0;
unsigned selectAttempts = 0;
bool rejectSelection = false;
uint8_t lastTriggeredProp = 0;
bool normalReady() {
    return !mechanism.debug && !mechanism.uploading && strcmp(mechanism.phase, "ready") == 0;
}
}
namespace CanTransport {
bool send(uint8_t bus, uint32_t id, const uint8_t *data, uint8_t length) {
    ++Harness::attempts;
    if (Harness::attempts == Harness::failAt) return false;
    Harness::Frame frame{bus, id, {}};
    if (length) frame.bytes.assign(data, data + length);
    Harness::sent.push_back(frame);
    return true;
}
}

namespace SystemSafety {
bool stopped() { return Harness::stopped; }
void stop(const char *) { Harness::stopped = true; ChassisController::emergencyStop(); }
}
namespace MechanismController {
Status status() { return Harness::mechanism; }
bool next() {
    if (!Harness::normalReady()) return false;
    ++Harness::nextCount; Harness::mechanism.phase = "busy"; return true;
}
bool previous() {
    if (!Harness::normalReady() || !Harness::mechanism.previousAvailable) return false;
    ++Harness::previousCount; Harness::mechanism.phase = "busy"; return true;
}
bool selectProfile(uint8_t stage, uint8_t prop) {
    ++Harness::selectAttempts;
    if (Harness::rejectSelection) return false;
    if (stage < 1 || stage > 6 || (stage != 1 && prop) || prop > 5 || Harness::mechanism.debug ||
        Harness::mechanism.uploading || strcmp(Harness::mechanism.phase,"fault") == 0) return false;
    prop = stage == 1 ? (prop ? prop : uint8_t(2)) : uint8_t(0);
    ++Harness::selectCount;
    if (!Harness::normalReady()) {
        const bool returning = strcmp(Harness::mechanism.phase,"return_wait") == 0 ||
                               strcmp(Harness::mechanism.phase,"home_wait") == 0;
        Harness::mechanism.pendingStage = !returning && stage == Harness::mechanism.stage && prop == Harness::mechanism.prop ? 0 : stage;
        Harness::mechanism.pendingProp = Harness::mechanism.pendingStage ? prop : 0;
    } else if (Harness::mechanism.stage == 3 && stage != 3) {
        Harness::mechanism.pendingStage=stage; Harness::mechanism.pendingProp=prop;
        Harness::mechanism.phase="return_wait";
    } else { Harness::mechanism.stage=stage; Harness::mechanism.prop=prop; }
    return true;
}
bool selectStage(uint8_t stage) { return selectProfile(stage,0); }
bool triggerProp(uint8_t prop) {
    if (!Harness::normalReady() || Harness::mechanism.stage != 1 || prop < 1 || prop > 5) return false;
    ++Harness::triggerCount; Harness::lastTriggeredProp=prop;
    Harness::mechanism.prop=prop; Harness::mechanism.propComplete=false;
    Harness::mechanism.phase="busy"; return true;
}
bool exitStage1(uint8_t stage) {
    if (stage < 2 || stage > 6 || Harness::mechanism.debug || Harness::mechanism.uploading ||
        strcmp(Harness::mechanism.phase,"fault") == 0) return false;
    ++Harness::exitCount;
    if (Harness::mechanism.stage != 1) return selectProfile(stage,0);
    Harness::mechanism.pendingStage=stage; Harness::mechanism.pendingProp=0;
    if (Harness::normalReady()) Harness::mechanism.phase="home_wait";
    return true;
}
}
#include "../src/chassis_controller.cpp"
#include "../src/rc_controller.cpp"

using namespace SystemConfig::Chassis;
using ChassisController::Mode;
bool close(float a, float b) { return fabsf(a - b) < 0.001f; }
AlfredoCRSF &radio() { return *AlfredoCRSF::instance; }
void neutral() {
    radio().linked = true;
    for (int &channel : radio().channels) channel = 1000;
    radio().channels[FORWARD.number] = FORWARD.middle;
    radio().channels[STRAFE.number] = STRAFE.middle;
    radio().channels[ROTATE.number] = ROTATE.middle;
    radio().channels[SystemConfig::Rc::STAGE1_TRIM_CHANNEL] = SystemConfig::Rc::STAGE1_TRIM_CENTER_US;
    radio().channels[SystemConfig::Rc::STAGE1_SWITCH_CHANNEL] = 1000;
    radio().channels[SystemConfig::Rc::STAGE_CHANNELS[2]] = 2000;
}
CAN_message_t heartbeat(uint8_t index, uint8_t state = 1, uint32_t error = 0) {
    CAN_message_t result{};
    result.id = (uint32_t(NODE_ID[index]) << 5) | 0x01;
    result.len = 8;
    memcpy(result.buf, &error, 4); result.buf[4] = state;
    return result;
}
void tick(uint32_t at, uint8_t state = 1, bool fresh = true) {
    mockMillis = at;
    if (fresh) for (uint8_t i = 0; i < 4; ++i)
        ChassisController::receive(CAN_BUS[i], heartbeat(i, state));
    RcController::poll(); ChassisController::poll();
}
void initialize() {
    Harness::mechanism.phase = "ready";
    Harness::mechanism.stage = 4;
    Harness::mechanism.previousAvailable = true;
    RcController::begin(); ChassisController::begin(); neutral();
    // These legacy cases isolate RC/mapping. Actual ramp/ICR behavior is
    // exercised with production defaults by chassis_motion_regression.cpp.
    auto motion = ChassisController::defaultMotionSettings(); motion.rampEnabled = false;
    motion.icrLeftYMm = motion.icrRightYMm = 0; // Isolate legacy center-based mixing.
    CHECK(ChassisController::configureMotion(motion));
}
void arming() {
    tick(100); tick(350); tick(599);
    CHECK(ChassisController::status().mode == Mode::Locked);
    tick(600); CHECK(ChassisController::status().mode == Mode::Arming);
}
void running() {
    arming(); tick(615, 8);
    CHECK(ChassisController::status().mode == Mode::Running);
    tick(650, 8); tick(680, 8);
    CHECK(RcController::status().selectedStage == 4);
}
unsigned commandCount(uint8_t command) {
    unsigned count = 0;
    for (const auto &frame : Harness::sent) if ((frame.id & 31) == command) ++count;
    return count;
}
void ready() { Harness::mechanism.phase = "ready"; }
void button(uint8_t channel, uint32_t at, bool high = true) {
    radio().channels[channel] = high ? 2000 : 1000;
    tick(at, 8); tick(at + SystemConfig::Rc::DEBOUNCE_MS, 8);
}
void zeroWheels() { for (float value : ChassisController::status().wheel) CHECK(value == 0); }
void onlyStops() {
    for (const auto &frame : Harness::sent) {
        const uint8_t command = uint8_t(frame.id & 31);
        CHECK(command == 0x0D || command == 0x07 || command == 0x18);
        if (command == 0x0D) {
            float value; memcpy(&value, frame.bytes.data(), 4); CHECK(value == 0);
        } else if (command == 0x07) {
            uint32_t value; memcpy(&value, frame.bytes.data(), 4); CHECK(value == 1);
        }
    }
}



void wheelBounds() {
    for (float value : ChassisController::status().wheel)
        CHECK(std::isfinite(value) && fabsf(value) <= SAFETY_MAX_SPEED + 0.001f);
}
void joystickOnly() {
    const auto rc = RcController::status();
    float expected[4];
    const uint8_t stage=Harness::mechanism.pendingStage ? Harness::mechanism.pendingStage : Harness::mechanism.stage;
    ChassisMath::mixWheels(rc.forward, rc.strafe, rc.rotate, expected, stage);
    const auto state = ChassisController::status();
    for (uint8_t i = 0; i < 4; ++i) CHECK(close(state.wheel[i], expected[i]));
}


// Keep field tuning configurable while checking polarity, clipping and timing.




bool testRemappedChannels(const std::string &name) {
    if (name == "rc_remap_contract") {
        static_assert(FORWARD.number == 2 && STRAFE.number == 3 && ROTATE.number == 1, "Physical joystick channels");
        static_assert(FORWARD.minimum == 1129 && FORWARD.middle == 1416 && FORWARD.maximum == 1717, "Forward calibration retained");
        static_assert(STRAFE.minimum == 1241 && STRAFE.middle == 1500 && STRAFE.maximum == 1747, "Strafe calibration retained");
        static_assert(ROTATE.minimum == 1182 && ROTATE.middle == 1454 && ROTATE.maximum == 1739, "Rotation calibration retained");
        static_assert(SystemConfig::Rc::PREV_CHANNEL == 5 && SystemConfig::Rc::NEXT_CHANNEL == 6, "Navigation buttons retained");
        static_assert(SystemConfig::Rc::STAGE_CHANNELS[0] == 10 && SystemConfig::Rc::STAGE_CHANNELS[1] == 11 &&
                      SystemConfig::Rc::STAGE_CHANNELS[2] == 12 && SystemConfig::Rc::STAGE_CHANNELS[3] == 13,
                      "Normal stages 2-5 channels");
        static_assert(SystemConfig::Rc::CLEAR_CHANNEL == 9, "Clear errors remains independent");
        static_assert(SystemConfig::Rc::STAGE1_SWITCH_CHANNEL == 7, "Stage1 three-position switch");
        static_assert(SystemConfig::Rc::STAGE_MAP[0] == 2 && SystemConfig::Rc::STAGE_MAP[1] == 3 &&
                      SystemConfig::Rc::STAGE_MAP[2] == 4 && SystemConfig::Rc::STAGE_MAP[3] == 5,"Normal stage identities");
        static_assert(SystemConfig::Rc::PROP_CHANNELS[0] == 10 && SystemConfig::Rc::PROP_CHANNELS[1] == 11 &&
                      SystemConfig::Rc::PROP_CHANNELS[2] == 12 && SystemConfig::Rc::PROP_CHANNELS[3] == 13 &&
                      SystemConfig::Rc::PROP_CHANNELS[4] == 14,"Stage1 prop channels");
        for (uint8_t stageChannel : SystemConfig::Rc::STAGE_CHANNELS)
            CHECK(stageChannel != SystemConfig::Rc::CLEAR_CHANNEL);
    } else if (name == "rc_ch2_forward_actual_mapping") {
        running(); radio().channels[2] = FORWARD.maximum; tick(695, 8);
        auto state = ChassisController::status(); const auto rc = RcController::status();
        CHECK(rc.forward == FORWARD.maximum && rc.strafe == STRAFE.middle && rc.rotate == ROTATE.middle);
        CHECK(close(state.wheel[0], -JOYSTICK_MAX_SPEED) && close(state.wheel[1], JOYSTICK_MAX_SPEED));
        CHECK(close(state.wheel[2], JOYSTICK_MAX_SPEED) && close(state.wheel[3], -JOYSTICK_MAX_SPEED));
        radio().channels[2] = FORWARD.minimum; tick(710, 8); state = ChassisController::status();
        CHECK(close(state.wheel[0], JOYSTICK_MAX_SPEED) && close(state.wheel[1], -JOYSTICK_MAX_SPEED));
        CHECK(close(state.wheel[2], -JOYSTICK_MAX_SPEED) && close(state.wheel[3], JOYSTICK_MAX_SPEED));
    } else if (name == "rc_ch3_strafe_actual_mapping") {
        running(); radio().channels[3] = STRAFE.maximum; tick(695, 8);
        const auto rc = RcController::status();
        CHECK(rc.forward == FORWARD.middle && rc.strafe == STRAFE.maximum && rc.rotate == ROTATE.middle);
        for (float wheel : ChassisController::status().wheel) CHECK(close(wheel, -JOYSTICK_MAX_SPEED));
        radio().channels[3] = STRAFE.minimum; tick(710, 8);
        for (float wheel : ChassisController::status().wheel) CHECK(close(wheel, JOYSTICK_MAX_SPEED));
    } else if (name == "rc_ch4_unused_arming_and_motion") {
        radio().channels[4] = 2101; // An invalid trim channel cannot invalidate the three joystick axes.
        running(); CHECK(RcController::status().valid && RcController::status().neutral); zeroWheels();
        radio().channels[4] = 900; tick(695, 8); CHECK(RcController::status().neutral); zeroWheels();
        radio().channels[4] = 2100; tick(710, 8); CHECK(RcController::status().neutral); zeroWheels();
        radio().channels[2] = FORWARD.maximum; tick(725, 8); joystickOnly();
        CHECK(close(ChassisController::status().wheel[0], -JOYSTICK_MAX_SPEED));
        CHECK(ChassisController::status().mode == Mode::Running);
    } else if (name == "rc_ch14_unused_no_stage_selection") {
        running(); const unsigned selectionsAt = Harness::selectCount;
        Harness::sent.clear(); button(14, 700);
        CHECK(Harness::selectCount == selectionsAt && Harness::mechanism.stage == 4);
        CHECK(RcController::status().selectedStage == 4 && commandCount(0x18) == 0);
        CHECK(Harness::nextCount == 0 && Harness::previousCount == 0);
    } else if (name == "rc_ch10_stage2_without_clear") {
        running(); const unsigned selectionsAt = Harness::selectCount;
        Harness::sent.clear(); button(10, 700);
        CHECK(Harness::mechanism.stage == 2 && RcController::status().selectedStage == 2);
        CHECK(Harness::selectCount == selectionsAt + 1 && commandCount(0x18) == 0);
        tick(800, 8); CHECK(commandCount(0x18) == 0 && Harness::selectCount == selectionsAt + 1);
    } else if (name == "rc_ch9_clear_without_stage_selection") {
        running(); const unsigned selectionsAt = Harness::selectCount;
        Harness::sent.clear(); button(9, 700);
        CHECK(commandCount(0x18) == 4 && Harness::selectCount == selectionsAt);
        CHECK(Harness::mechanism.stage == 4 && RcController::status().selectedStage == 4);
        tick(800, 8); CHECK(commandCount(0x18) == 4);
        for (const auto &frame : Harness::sent) if ((frame.id & 31) == 0x18) CHECK(frame.bytes.empty());
    } else return false;
    return true;
}

// Fixed wheel-order goldens: front-left, front-right, rear-left, rear-right.
// Each row is a normalized raw-stick command: F+, F-, S+, S-.
// Stage 1: vx=-raw_vy, vy=raw_vx. Stage 2 negates both translation axes.
constexpr float stageCardinal[4][4][4] = {
    {{-1,-1,-1,-1}, {1,1,1,1}, {1,-1,-1,1}, {-1,1,1,-1}},
    {{1,-1,-1,1}, {-1,1,1,-1}, {1,1,1,1}, {-1,-1,-1,-1}},
    {{-1,1,1,-1}, {1,-1,-1,1}, {-1,-1,-1,-1}, {1,1,1,1}},
    {{-1,1,1,-1}, {1,-1,-1,1}, {-1,-1,-1,-1}, {1,1,1,1}}
};
constexpr float bodyRotation[4] = {-1,-1,1,1};
void goldenWheels(const float (&actual)[4], const float (&factors)[4]) {
    for (uint8_t i=0;i<4;++i) CHECK(close(actual[i], factors[i]*JOYSTICK_MAX_SPEED));
}
void cardinalVectors(uint8_t stage) {
    const int forward[4] = {FORWARD.maximum, FORWARD.minimum, FORWARD.middle, FORWARD.middle};
    const int strafe[4] = {STRAFE.middle, STRAFE.middle, STRAFE.maximum, STRAFE.minimum};
    for (uint8_t direction=0;direction<4;++direction) {
        float wheel[4];
        ChassisMath::mixWheels(forward[direction],strafe[direction],ROTATE.middle,wheel,stage);
        goldenWheels(wheel,stageCardinal[stage-1][direction]);
        for (float value : wheel) CHECK(close(fabsf(value),JOYSTICK_MAX_SPEED));
    }
}
bool testStageDirection(const std::string &name) {
    if (name == "stage1_cardinal_vectors") cardinalVectors(1);
    else if (name == "stage2_cardinal_vectors") cardinalVectors(2);
    else if (name == "stage3_cardinal_vectors") cardinalVectors(3);
    else if (name == "stage4_cardinal_vectors") cardinalVectors(4);
    else if (name == "stage_all_diagonal_vectors") {
        // Raw F/S signs: ++, +-, -+, --. Nonzero wheels retain the full J budget.
        constexpr float expected[4][4][4] = {
            {{0,-1,-1,0}, {-1,0,0,-1}, {1,0,0,1}, {0,1,1,0}},
            {{1,0,0,1}, {0,-1,-1,0}, {0,1,1,0}, {-1,0,0,-1}},
            {{-1,0,0,-1}, {0,1,1,0}, {0,-1,-1,0}, {1,0,0,1}},
            {{-1,0,0,-1}, {0,1,1,0}, {0,-1,-1,0}, {1,0,0,1}}
        };
        const int forward[4] = {FORWARD.maximum,FORWARD.maximum,FORWARD.minimum,FORWARD.minimum};
        const int strafe[4] = {STRAFE.maximum,STRAFE.minimum,STRAFE.maximum,STRAFE.minimum};
        for (uint8_t stage=1;stage<=4;++stage)
            for (uint8_t direction=0;direction<4;++direction) {
                float wheel[4];
                ChassisMath::mixWheels(forward[direction],strafe[direction],ROTATE.middle,wheel,stage);
                goldenWheels(wheel,expected[stage-1][direction]);
            }
    } else if (name == "stage_all_rotation_unchanged") {
        for (uint8_t stage=1;stage<=4;++stage) {
            float wheel[4];
            ChassisMath::mixWheels(FORWARD.middle,STRAFE.middle,ROTATE.maximum,wheel,stage);
            goldenWheels(wheel,{-1,-1,1,1});
            ChassisMath::mixWheels(FORWARD.middle,STRAFE.middle,ROTATE.minimum,wheel,stage);
            goldenWheels(wheel,{1,1,-1,-1});
        }
    } else if (name == "stage_all_translation_rotation_mix") {
        // F+, S-, R+ then R-: preserve rotation while applying stage translation.
        constexpr float positive[4][4] = {
            {-1,-1.0f/3,1.0f/3,-1.0f/3},
            {-1.0f/3,-1,-1.0f/3,1.0f/3},
            {-1.0f/3,1.0f/3,1,1.0f/3},
            {-1.0f/3,1.0f/3,1,1.0f/3}
        };
        constexpr float negative[4][4] = {
            {-1.0f/3,1.0f/3,-1.0f/3,-1},
            {1.0f/3,-1.0f/3,-1,-1.0f/3},
            {1.0f/3,1,1.0f/3,-1.0f/3},
            {1.0f/3,1,1.0f/3,-1.0f/3}
        };
        constexpr float forwardRotate[4][4] = {
            {-1,-1,0,0}, {0,-1,0,1}, {-1,0,1,0}, {-1,0,1,0}
        };
        for (uint8_t stage=1;stage<=4;++stage) {
            float wheel[4];
            ChassisMath::mixWheels(FORWARD.maximum,STRAFE.minimum,ROTATE.maximum,wheel,stage);
            goldenWheels(wheel,positive[stage-1]);
            ChassisMath::mixWheels(FORWARD.maximum,STRAFE.minimum,ROTATE.minimum,wheel,stage);
            goldenWheels(wheel,negative[stage-1]);
            ChassisMath::mixWheels(FORWARD.maximum,STRAFE.middle,ROTATE.maximum,wheel,stage);
            goldenWheels(wheel,forwardRotate[stage-1]);
        }
    } else if (name == "stage_all_deadzone_and_calibration") {
        const Channel channels[3] = {FORWARD,STRAFE,ROTATE};
        CHECK(FORWARD.middle!=STRAFE.middle);
        CHECK(FORWARD.maximum-FORWARD.middle != FORWARD.middle-FORWARD.minimum);
        CHECK(STRAFE.maximum-STRAFE.middle != STRAFE.middle-STRAFE.minimum);
        for (uint8_t stage=1;stage<=4;++stage) {
            for (uint8_t axis=0;axis<3;++axis)
                for (int direction : {-1,1}) {
                    int raw[3] = {FORWARD.middle,STRAFE.middle,ROTATE.middle};
                    const auto channel=channels[axis];
                    raw[axis]=channel.middle+direction*DEAD_ZONE;
                    float wheel[4];
                    ChassisMath::mixWheels(raw[0],raw[1],raw[2],wheel,stage);
                    goldenWheels(wheel,{0,0,0,0});
                    raw[axis]+=direction; // First nonzero tick, using THIS raw axis calibration.
                    const float fraction=direction>0
                        ? 1.0f/float(channel.maximum-channel.middle-DEAD_ZONE)
                        : -1.0f/float(channel.middle-DEAD_ZONE-channel.minimum);
                    ChassisMath::mixWheels(raw[0],raw[1],raw[2],wheel,stage);
                    for (uint8_t i=0;i<4;++i) {
                        const float basis=axis==2 ? bodyRotation[i] : stageCardinal[stage-1][axis*2][i];
                        CHECK(close(wheel[i],basis*fraction*JOYSTICK_MAX_SPEED));
                    }
                }
            // Asymmetric partial axes remain below the normalization threshold.
            constexpr float f=1.0f/(FORWARD.maximum-FORWARD.middle-DEAD_ZONE);
            constexpr float s=-1.0f/(STRAFE.middle-DEAD_ZONE-STRAFE.minimum);
            constexpr float r=-1.0f/(ROTATE.middle-DEAD_ZONE-ROTATE.minimum);
            float wheel[4];
            ChassisMath::mixWheels(FORWARD.middle+DEAD_ZONE+1,STRAFE.middle-DEAD_ZONE-1,
                                  ROTATE.middle-DEAD_ZONE-1,wheel,stage);
            for (uint8_t i=0;i<4;++i)
                CHECK(close(wheel[i],JOYSTICK_MAX_SPEED*(stageCardinal[stage-1][0][i]*f+
                    stageCardinal[stage-1][2][i]*s+bodyRotation[i]*r)));
        }
    } else if (name == "stage_default_and_invalid_identity") {
        float wheel[4];
        const float expected[4] = {-1.0f/3,1.0f/3,1,1.0f/3};
        ChassisMath::mixWheels(FORWARD.maximum,STRAFE.minimum,ROTATE.maximum,wheel);
        goldenWheels(wheel,expected); // Existing four-argument API defaults to Stage 4.
        for (uint8_t stage : {uint8_t(0),uint8_t(3),uint8_t(4),uint8_t(5),uint8_t(255)}) {
            ChassisMath::mixWheels(FORWARD.maximum,STRAFE.minimum,ROTATE.maximum,wheel,stage);
            goldenWheels(wheel,expected);
        }
    }  else if (name == "stage_poll_uses_accepted_pending_before_active") {
        running(); radio().channels[FORWARD.number]=FORWARD.maximum;
        uint32_t at=695;
        for (uint8_t stage : {uint8_t(1),uint8_t(2),uint8_t(3),uint8_t(4)}) {
            Harness::mechanism.stage=stage;
            Harness::mechanism.pendingStage=static_cast<uint8_t>(stage==1 ? 2 : 1);
            Harness::mechanism.phase="return_wait";
            CHECK(RcController::status().selectedStage==4);
            tick(at,8); at+=15;
            const auto state=ChassisController::status();
            goldenWheels(state.wheel,stageCardinal[Harness::mechanism.pendingStage-1][0]); joystickOnly();
            CHECK(Harness::mechanism.stage==stage);
        }
    }  else if (name == "stage_rc_selection_updates_motion") {
        running(); radio().channels[FORWARD.number]=FORWARD.maximum; tick(695,8);
        goldenWheels(ChassisController::status().wheel,stageCardinal[3][0]);
        button(10,700); CHECK(Harness::mechanism.stage==2);
        goldenWheels(ChassisController::status().wheel,stageCardinal[1][0]); joystickOnly();
        button(11,750); CHECK(Harness::mechanism.stage==3);
        goldenWheels(ChassisController::status().wheel,stageCardinal[2][0]); joystickOnly();
        button(12,800,false); button(12,850);
        CHECK(Harness::mechanism.stage==3 && Harness::mechanism.pendingStage==4);
        goldenWheels(ChassisController::status().wheel,stageCardinal[3][0]); joystickOnly();
        Harness::mechanism.stage=4; Harness::mechanism.pendingStage=0; ready(); tick(895,8);
        goldenWheels(ChassisController::status().wheel,stageCardinal[3][0]);
        button(13,900); CHECK(Harness::mechanism.stage==5);
        goldenWheels(ChassisController::status().wheel,stageCardinal[3][0]); joystickOnly();
    } else return false;
    return true;
}

void modeSwitch(int value,uint32_t at) {
    radio().channels[SystemConfig::Rc::STAGE1_SWITCH_CHANNEL]=value;
    tick(at,8); tick(at+SystemConfig::Rc::DEBOUNCE_MS,8);
}
void enableStage1Trim() {
    running();
    Harness::mechanism.stage1TrimAllowed = true;
    modeSwitch(2000,700);
    CHECK(RcController::status().stage1Mode && Harness::mechanism.stage == 1);
}
int trimSample(int value, uint32_t at) {
    radio().channels[SystemConfig::Rc::STAGE1_TRIM_CHANNEL] = value;
    tick(at,8);
    return RcController::status().stage1TrimUs;
}
bool testStage1Trim(const std::string &name) {
    using namespace SystemConfig::Rc;
    if (name == "rc_stage1_trim_endpoints_and_valid_overrange") {
        enableStage1Trim();
        CHECK(trimSample(1000,750) == -150 && trimSample(2000,765) == 150);
        CHECK(trimSample(900,780) == -150 && trimSample(2100,795) == 150);
        CHECK(trimSample(1250,810) == -70 && trimSample(1750,825) == 70);
    } else if (name == "rc_stage1_trim_center_deadband") {
        enableStage1Trim(); uint32_t at=750;
        for (int value=1470;value<=1530;++value) {
            CHECK(trimSample(value,at) == 0); at+=15;
        }
        CHECK(trimSample(1466,at) == -1); at+=15;
        CHECK(trimSample(1534,at) == 1);
    } else if (name == "rc_stage1_trim_monotonic_and_bounded") {
        enableStage1Trim(); uint32_t at=750; int previous=-150;
        for (int value=1000;value<=2000;++value) {
            const int offset=trimSample(value,at); at+=15;
            CHECK(offset>=previous && offset>=-STAGE1_TRIM_MAX_OFFSET_US &&
                  offset<=STAGE1_TRIM_MAX_OFFSET_US);
            previous=offset;
        }
        CHECK(previous==150);
    } else if (name == "rc_stage1_trim_busy_and_terminal_hold") {
        enableStage1Trim(); Harness::mechanism.phase="busy";
        CHECK(trimSample(2000,750)==150);
        ready(); Harness::mechanism.propComplete=true;
        CHECK(trimSample(1000,765)==-150);
        CHECK(Harness::triggerCount==0 && Harness::nextCount==0 && Harness::previousCount==0);
    } else if (name == "rc_stage1_trim_stage_and_context_guards") {
        running(); Harness::mechanism.stage1TrimAllowed=true;
        CHECK(trimSample(2000,700)==0); // General context, even with a stale mock eligibility flag.
        Harness::mechanism.stage=1;
        CHECK(trimSample(2000,715)==0); // Web-selected Stage 1 does not create RC Stage 1 context.
        modeSwitch(2000,750); CHECK(trimSample(2000,800)==150);
        for (uint8_t stage=2;stage<=5;++stage) {
            Harness::mechanism.stage=stage;
            CHECK(trimSample(2000,uint32_t(815+15*stage))==0);
        }
    } else if (name == "rc_stage1_trim_home_pending_fault_guard") {
        enableStage1Trim(); CHECK(trimSample(2000,750)==150);
        Harness::mechanism.stage1TrimAllowed=false; Harness::mechanism.phase="home_wait";
        CHECK(trimSample(2000,765)==0);
        Harness::mechanism.pendingStage=2; Harness::mechanism.phase="return_wait";
        CHECK(trimSample(2000,780)==0);
        Harness::mechanism.pendingStage=0; Harness::mechanism.phase="fault";
        CHECK(trimSample(2000,795)==0);
    } else if (name == "rc_stage1_trim_low_exit_clears_during_return") {
        enableStage1Trim(); CHECK(trimSample(2000,750)==150);
        modeSwitch(1000,800);
        CHECK(Harness::mechanism.stage==1 && Harness::mechanism.pendingStage==4);
        CHECK(!RcController::status().stage1Mode && RcController::status().stage1TrimUs==0);
        CHECK(trimSample(1000,850)==0);
    } else if (name == "rc_stage1_trim_mid_retains_active_context") {
        enableStage1Trim(); CHECK(trimSample(2000,750)==150);
        modeSwitch(1500,800);
        CHECK(RcController::status().stage1Mode && RcController::status().switchPosition==1);
        CHECK(trimSample(1000,850)==-150);
    } else if (name == "rc_stage1_trim_link_loss_and_relink") {
        enableStage1Trim(); CHECK(trimSample(2000,750)==150);
        radio().linked=false; tick(765,8);
        CHECK(RcController::status().stage1TrimUs==0 && !RcController::status().stage1Mode);
        radio().linked=true; tick(780,8);
        CHECK(RcController::status().stage1TrimUs==0);
        tick(810,8); CHECK(RcController::status().stage1Mode);
        CHECK(RcController::status().stage1TrimUs==150);
    } else if (name == "rc_stage1_trim_invalid_knob_only_disables_trim") {
        enableStage1Trim(); uint32_t at=750;
        for (int value : {0,899,2101,65535}) {
            CHECK(trimSample(value,at)==0); at+=15;
            CHECK(RcController::status().valid && RcController::status().neutral);
            CHECK(ChassisController::status().mode==Mode::Running);
        }
        CHECK(trimSample(2000,at)==150);
    } else if (name == "rc_stage1_trim_invalid_sticks_disable_trim") {
        enableStage1Trim(); CHECK(trimSample(2000,750)==150);
        radio().channels[FORWARD.number]=2101; tick(765,8);
        CHECK(!RcController::status().valid && RcController::status().stage1TrimUs==0);
        CHECK(ChassisController::status().mode==Mode::Locked);
    } else if (name == "rc_stage1_trim_stop_immediately_clears_snapshot") {
        enableStage1Trim(); CHECK(trimSample(2000,750)==150);
        SystemSafety::stop("test trim STOP");
        CHECK(RcController::status().stage1TrimUs==0); // No receiver poll required.
        CHECK(trimSample(1000,765)==0);
    } else if (name == "rc_stage1_trim_does_not_change_chassis_mixing") {
        enableStage1Trim(); radio().channels[FORWARD.number]=FORWARD.maximum;
        uint32_t at=750;
        for (int knob : {1000,1500,2000,2101}) {
            trimSample(knob,at); at+=15;
            const auto rc=RcController::status();
            CHECK(rc.valid && rc.forward==FORWARD.maximum && rc.strafe==STRAFE.middle &&
                  rc.rotate==ROTATE.middle);
            goldenWheels(ChassisController::status().wheel,stageCardinal[0][0]);
        }
    } else return false;
    return true;
}
bool testStage1RcMode(const std::string &name) {
    if (name == "rc_stage1_high_selects_without_motion") {
        running(); modeSwitch(2000,700);
        CHECK(Harness::mechanism.stage==1 && Harness::mechanism.prop==2);
        CHECK(RcController::status().stage1Mode && RcController::status().selectedStage==1 && RcController::status().switchPosition==2);
        CHECK(Harness::triggerCount==0 && Harness::nextCount==0 && Harness::exitCount==0);
    } else if (name == "rc_stage1_mid_preserves_mode") {
        running(); const unsigned selectedAt=Harness::selectCount;
        modeSwitch(1500,700); CHECK(!RcController::status().stage1Mode && Harness::mechanism.stage==4);
        modeSwitch(2000,750); modeSwitch(1500,800); modeSwitch(2000,850);
        CHECK(RcController::status().stage1Mode && Harness::mechanism.stage==1);
        CHECK(Harness::selectCount==selectedAt+1 && Harness::triggerCount==0 && Harness::exitCount==0);
    } else if (name == "rc_stage1_mid_boot_waits_for_mode") {
        radio().channels[7]=1500; arming(); tick(615,8); tick(650,8); tick(680,8);
        CHECK(RcController::status().selectedStage==0 && Harness::selectCount==0);
        CHECK(RcController::status().switchPosition==1 && !RcController::status().stage1Mode);
        button(10,700); CHECK(Harness::selectCount==0 && Harness::triggerCount==0);
        modeSwitch(2000,750); CHECK(Harness::mechanism.stage==1 && Harness::triggerCount==0);
    } else if (name == "rc_stage1_props_all_channels") {
        running(); modeSwitch(2000,700); uint32_t at=750;
        for (uint8_t prop=1;prop<=5;++prop) {
            ready(); button(static_cast<uint8_t>(9+prop),at,false); at+=50;
            button(static_cast<uint8_t>(9+prop),at); at+=50;
            CHECK(Harness::lastTriggeredProp==prop && Harness::mechanism.prop==prop);
            CHECK(Harness::triggerCount==prop && RcController::status().selectedProp==prop);
        }
    } else if (name == "rc_stage1_prop_busy_not_queued") {
        running(); modeSwitch(2000,700); tick(765,8); button(10,780); CHECK(Harness::triggerCount==1);
        button(11,830);
        CHECK(Harness::triggerCount==1 && Harness::mechanism.prop==1);
        ready(); tick(900,8); tick(950,8);
        CHECK(Harness::triggerCount==1); button(11,1000,false); button(11,1050);
        CHECK(Harness::triggerCount==2 && Harness::lastTriggeredProp==2);
    } else if (name == "rc_stage1_transition_held_disarmed") {
        running(); modeSwitch(2000,700); tick(780,8);
        CHECK(Harness::triggerCount==0); // CH12 was already high as the normal Stage4 selector.
        button(12,800,false); button(12,850); CHECK(Harness::lastTriggeredProp==3);
        ready(); modeSwitch(1000,900);
        CHECK(Harness::exitCount==1 && Harness::mechanism.pendingStage==4 && strcmp(Harness::mechanism.phase,"home_wait")==0);
        const unsigned selections=Harness::selectCount;
        Harness::mechanism.stage=4; Harness::mechanism.prop=0; Harness::mechanism.pendingStage=0; ready(); tick(1000,8);
        CHECK(!RcController::status().stage1Mode && Harness::selectCount==selections);
        button(13,1050); CHECK(Harness::mechanism.stage==5);
    } else if (name == "rc_stage1_low_busy_latest_normal_selection") {
        running(); modeSwitch(2000,700); tick(765,8); button(11,780); modeSwitch(1000,850);
        CHECK(!RcController::status().stage1Mode && Harness::exitCount==1);
        CHECK(Harness::mechanism.stage==1 && Harness::mechanism.pendingStage==4 && strcmp(Harness::mechanism.phase,"busy")==0);
        Harness::mechanism.phase="home_wait";
        button(10,900); button(13,950);
        CHECK(Harness::mechanism.stage==1 && Harness::mechanism.pendingStage==5);
        CHECK(Harness::triggerCount==1 && RcController::status().selectedStage==5);
    } else return false;
    return true;
}

bool testHeldStageSelection(const std::string &name) {
    if (name == "rc_held_boot_unique_stage2") {
        radio().channels[12]=1000; radio().channels[10]=2000;
        arming(); tick(615,8); tick(650,8); tick(680,8);
        CHECK(Harness::mechanism.stage==2 && RcController::status().selectedStage==2);
        CHECK(Harness::selectCount==1);
        tick(730,8); tick(800,8); CHECK(Harness::selectCount==1);
    } else if (name == "rc_held_boot_unique_stage3") {
        radio().channels[12]=1000; radio().channels[11]=2000;
        arming(); tick(615,8); tick(650,8); tick(680,8);
        CHECK(Harness::mechanism.stage==3 && RcController::status().selectedStage==3 && Harness::selectCount==1);
    } else if (name == "rc_held_boot_retries_mechanism_startup") {
        radio().channels[12]=1000; radio().channels[10]=2000; Harness::mechanism.phase="booting";
        arming(); tick(615,8); tick(650,8); tick(680,8);
        CHECK(Harness::selectCount==0 && Harness::selectAttempts==0 && RcController::status().rejectedActions==0);
        ready(); tick(700,8); CHECK(Harness::mechanism.stage==2 && Harness::selectCount==1);
        tick(750,8); tick(800,8); CHECK(Harness::selectCount==1);
    } else if (name == "rc_held_boot_ambiguity_resolves_unique_level") {
        radio().channels[10]=2000; arming(); tick(615,8); tick(650,8); tick(680,8);
        CHECK(Harness::selectCount==0 && RcController::status().selectedStage==0);
        button(12,700,false);
        CHECK(Harness::mechanism.stage==2 && RcController::status().selectedStage==2 && Harness::selectCount==1);
    } else if (name == "rc_held_recent_priority_and_release_fallback") {
        running(); button(10,700); button(11,750); button(13,800);
        CHECK(RcController::status().selectedStage==5 && Harness::mechanism.pendingStage==5);
        button(13,850,false);
        CHECK(RcController::status().selectedStage==3 && Harness::mechanism.pendingStage==3);
        button(11,900,false);
        CHECK(RcController::status().selectedStage==2 && Harness::mechanism.pendingStage==2);
        button(10,950,false);
        CHECK(RcController::status().selectedStage==4 && Harness::mechanism.pendingStage==4);
        button(12,1000,false); CHECK(RcController::status().selectedStage==0);
        const unsigned accepted=Harness::selectCount; tick(1100,8);
        CHECK(Harness::selectCount==accepted && accepted==7);
    } else if (name == "rc_held_release_nonwinner_preserves_request") {
        running(); button(10,700); button(13,750);
        const unsigned accepted=Harness::selectCount; button(10,800,false);
        CHECK(Harness::selectCount==accepted && RcController::status().selectedStage==5);
        button(13,850,false);
        CHECK(RcController::status().selectedStage==4 && Harness::selectCount==accepted+1);
    } else if (name == "rc_held_selection_waits_chassis_unlock") {
        radio().channels[FORWARD.number]=FORWARD.maximum;
        tick(100); tick(150); button(10,200);
        CHECK(Harness::selectCount==0 && RcController::status().rejectedActions==0);
        neutral(); radio().channels[10]=2000; tick(300); tick(800); tick(815,8); tick(850,8);
        CHECK(Harness::mechanism.stage==2 && Harness::selectCount==1);
        CHECK(ChassisController::status().mode==Mode::Running);
    } else if (name == "rc_held_selection_waits_debug") {
        running(); const unsigned accepted=Harness::selectCount, attempts=Harness::selectAttempts;
        Harness::mechanism.debug=true; button(10,700); tick(800,8); tick(900,8);
        CHECK(Harness::selectCount==accepted && Harness::selectAttempts==attempts && RcController::status().rejectedActions==0);
        Harness::mechanism.debug=false; tick(950,8);
        CHECK(Harness::mechanism.stage==2 && Harness::selectCount==accepted+1);
        tick(1000,8); CHECK(Harness::selectCount==accepted+1);
    } else if (name == "rc_held_selection_waits_upload") {
        running(); const unsigned accepted=Harness::selectCount; Harness::mechanism.uploading=true;
        button(10,700); tick(800,8); CHECK(Harness::selectCount==accepted);
        Harness::mechanism.uploading=false; tick(850,8);
        CHECK(Harness::mechanism.stage==2 && Harness::selectCount==accepted+1);
    } else if (name == "rc_held_rejected_api_retries_quietly") {
        running(); const unsigned accepted=Harness::selectCount;
        Harness::rejectSelection=true; button(10,700); tick(750,8); tick(800,8);
        CHECK(Harness::selectCount==accepted && RcController::status().rejectedActions==1);
        Harness::rejectSelection=false; tick(850,8);
        CHECK(Harness::mechanism.stage==2 && Harness::selectCount==accepted+1);
        const unsigned attempts=Harness::selectAttempts; tick(900,8); tick(950,8);
        CHECK(Harness::selectAttempts==attempts);
    } else if (name == "rc_held_invalid_selector_recovers_without_web_override") {
        running(); button(10,700);
        const unsigned accepted=Harness::selectCount; Harness::mechanism.stage=5;
        radio().channels[10]=500; tick(750,8); CHECK(RcController::status().selectedStage==0);
        radio().channels[10]=2000; tick(760,8); tick(789,8); CHECK(RcController::status().selectedStage==0);
        tick(790,8); CHECK(RcController::status().selectedStage==2);
        CHECK(Harness::selectCount==accepted && Harness::mechanism.stage==5);
    } else if (name == "rc_held_invalid_joystick_retains_pending") {
        running(); const unsigned accepted=Harness::selectCount;
        radio().channels[ROTATE.number]=3000; tick(700,8); button(10,750);
        CHECK(Harness::selectCount==accepted && ChassisController::status().mode==Mode::Locked);
        neutral(); radio().channels[10]=2000; tick(900); tick(1400); tick(1415,8); tick(1450,8);
        CHECK(Harness::mechanism.stage==2 && Harness::selectCount==accepted+1);
    } else if (name == "rc_held_debounce_rollover") {
        running(); radio().channels[10]=2000;
        tick(UINT32_MAX-20,8); tick(8,8); CHECK(RcController::status().selectedStage==4);
        tick(9,8); CHECK(RcController::status().selectedStage==2);
        radio().channels[10]=1000; tick(20,8); tick(50,8);
        CHECK(RcController::status().selectedStage==4);
    } else if (name == "rc_held_level_does_not_override_web_pending") {
        running(); const unsigned accepted=Harness::selectCount;
        Harness::mechanism.phase="busy"; Harness::mechanism.pendingStage=3;
        tick(750,8); tick(850,8);
        CHECK(Harness::selectCount==accepted && Harness::mechanism.pendingStage==3);
        button(10,900); CHECK(Harness::mechanism.pendingStage==2 && Harness::selectCount==accepted+1);
    } else if (name == "rc_held_blocked_release_replaces_request") {
        running(); const unsigned accepted=Harness::selectCount;
        Harness::mechanism.debug=true; button(10,700); button(10,750,false);
        CHECK(Harness::selectCount==accepted);
        Harness::mechanism.debug=false; tick(800,8);
        CHECK(Harness::mechanism.stage==4 && Harness::selectCount==accepted+1);
    } else if (name == "rc_held_cold_no_selection_then_level") {
        radio().channels[12]=1000; arming(); tick(615,8); tick(650,8); tick(680,8);
        CHECK(RcController::status().selectedStage==0 && Harness::selectCount==0);
        button(10,700); CHECK(Harness::mechanism.stage==2 && Harness::selectCount==1);
    } else if (name == "rc_held_stage1_keys_need_stable_release") {
        running(); modeSwitch(2000,700); button(11,750); ready(); modeSwitch(1000,800);
        CHECK(Harness::exitCount==1 && Harness::mechanism.pendingStage==4);
        Harness::mechanism.stage=4; Harness::mechanism.prop=0; Harness::mechanism.pendingStage=0; ready();
        const unsigned accepted=Harness::selectCount; tick(900,8); tick(950,8);
        CHECK(Harness::selectCount==accepted && RcController::status().selectedStage==4);
        radio().channels[11]=1000; tick(1000,8); tick(1029,8);
        radio().channels[11]=2000; tick(1030,8); tick(1060,8);
        CHECK(Harness::selectCount==accepted); // 29 ms release cannot rearm a held prop.
        button(11,1100,false); button(11,1150);
        CHECK(Harness::mechanism.stage==3 && Harness::selectCount==accepted+1);
    } else if (name == "rc_held_mid_keeps_general_context") {
        running(); modeSwitch(1500,700); button(10,750);
        CHECK(!RcController::status().stage1Mode && Harness::mechanism.stage==2 && RcController::status().switchPosition==1);
    } else if (name == "rc_held_pending_global_stop_stays_blocked") {
        running(); const unsigned accepted=Harness::selectCount; Harness::mechanism.debug=true;
        button(10,700); SystemSafety::stop("STOP"); Harness::mechanism.debug=false;
        tick(800,8); tick(900,8);
        CHECK(Harness::selectCount==accepted && Harness::mechanism.stage==4);
    } else if (name == "rc_held_navigation_during_block_requires_release") {
        running(); Harness::mechanism.debug=true; button(10,700); button(6,750);
        Harness::mechanism.debug=false; tick(800,8); tick(850,8);
        CHECK(Harness::mechanism.stage==2 && Harness::nextCount==0);
        button(6,900,false); button(6,950); CHECK(Harness::nextCount==1);
    } else if (name == "rc_held_stage_hysteresis_and_debounce") {
        running(); button(10,700); const unsigned accepted=Harness::selectCount;
        radio().channels[10]=1650; tick(750,8); tick(800,8);
        CHECK(RcController::status().selectedStage==2 && Harness::selectCount==accepted);
        radio().channels[10]=1599; tick(850,8); tick(879,8); CHECK(RcController::status().selectedStage==2);
        tick(880,8); CHECK(RcController::status().selectedStage==4);
        radio().channels[10]=1700; tick(900,8); tick(950,8); CHECK(RcController::status().selectedStage==4);
        radio().channels[10]=1701; tick(1000,8); tick(1030,8); CHECK(RcController::status().selectedStage==2);
    } else return false;
    return true;
}

bool testProcessionSelection(const std::string &name) {
    if (name == "rc_procession_normal_rejection_keeps_bank_until_acceptance") {
        running(); button(14,700);
        CHECK(RcController::status().processionMode);
        const auto accepted=Harness::selectCount;
        Harness::rejectSelection=true; button(10,750);
        CHECK(RcController::status().processionMode && ChassisController::status().processionMode);
        CHECK(Harness::selectCount==accepted && Harness::mechanism.stage==4);
        Harness::rejectSelection=false; tick(800,8);
        CHECK(!RcController::status().processionMode && Harness::mechanism.stage==2);
        CHECK(Harness::selectCount==accepted+1);
    } else if (name == "rc_procession_works_while_mechanism_debug_and_retains_mid") {
        running(); Harness::mechanism.debug=true; const auto selections=Harness::selectAttempts;
        button(14,700); CHECK(RcController::status().processionMode);
        CHECK(Harness::selectAttempts==selections && Harness::mechanism.stage==4);
        modeSwitch(1500,750); CHECK(RcController::status().processionMode);
        button(10,800); CHECK(RcController::status().processionMode);
        Harness::mechanism.debug=false; tick(850,8);
        CHECK(!RcController::status().processionMode && Harness::mechanism.stage==2);
    } else if (name == "rc_procession_link_loss_clears_override_and_stops") {
        running(); button(14,700); CHECK(RcController::status().processionMode);
        radio().channels[FORWARD.number]=FORWARD.maximum; tick(750,8);
        CHECK(ChassisController::status().wheel[0]!=0);
        radio().linked=false; tick(755,8);
        CHECK(!RcController::status().processionMode && !ChassisController::status().processionMode);
        CHECK(ChassisController::status().mode==Mode::Locked); zeroWheels();
    } else return false;
    return true;
}

void test(const std::string &name) {
    if (testStage1Trim(name) || testProcessionSelection(name) || testHeldStageSelection(name) || testStage1RcMode(name) || testStageDirection(name) || testRemappedChannels(name)) return;
    if (name == "math") {
        for (const auto channel : {FORWARD, STRAFE, ROTATE}) {
            CHECK(ChassisMath::mapJoystick(channel.minimum, channel) == -1);
            CHECK(ChassisMath::mapJoystick(channel.maximum, channel) == 1);
            CHECK(ChassisMath::mapJoystick(900, channel) == -1);
            CHECK(ChassisMath::mapJoystick(2100, channel) == 1);
            CHECK(ChassisMath::mapJoystick(channel.middle - DEAD_ZONE, channel) == 0);
            CHECK(ChassisMath::mapJoystick(channel.middle + DEAD_ZONE, channel) == 0);
        }
        float wheel[4];
        ChassisMath::mixWheels(FORWARD.maximum, STRAFE.maximum, ROTATE.maximum, wheel);
        CHECK(close(wheel[0], -JOYSTICK_MAX_SPEED) && close(wheel[1], -JOYSTICK_MAX_SPEED/3) && close(wheel[2], JOYSTICK_MAX_SPEED/3) && close(wheel[3], -JOYSTICK_MAX_SPEED/3));
        for (int f = 900; f <= 2100; f += 60)
            for (int s = 900; s <= 2100; s += 60)
                for (int r = 900; r <= 2100; r += 60) {
                    ChassisMath::mixWheels(f, s, r, wheel);
                    for (float value : wheel) CHECK(fabsf(value) <= JOYSTICK_MAX_SPEED + 0.001f);
                }
    } else if (name == "boot_routing") {
        CHECK(Serial2.beginBaud == SystemConfig::Rc::CRSF_BAUD);
        CHECK(Harness::sent.size() == 8); onlyStops();
        for (uint8_t i = 0; i < 4; ++i) {
            CHECK(Harness::sent[i * 2].bus == CAN_BUS[i]);
            CHECK((Harness::sent[i * 2].id >> 5) == NODE_ID[i]);
            CHECK(Harness::sent[i * 2].bytes.size() == 8);
        }
        tick(100, 1, false); tick(1000, 1, false);
        CHECK(ChassisController::status().mode == Mode::Locked);
    } else if (name == "center_hold") {
        radio().channels[FORWARD.number] = FORWARD.maximum; tick(100); tick(700);
        CHECK(ChassisController::status().mode == Mode::Locked);
        neutral(); tick(800); tick(1299);
        CHECK(ChassisController::status().mode == Mode::Locked);
        tick(1300); CHECK(ChassisController::status().mode == Mode::Arming);
        tick(1315, 8); CHECK(ChassisController::status().mode == Mode::Running);
    } else if (name == "motion_15ms") {
        running(); radio().channels[FORWARD.number] = FORWARD.maximum;
        Harness::sent.clear(); tick(694, 8); CHECK(Harness::sent.empty());
        tick(695, 8); CHECK(commandCount(0x0D) == 4);
        const auto state = ChassisController::status();
        CHECK(close(state.wheel[0], -JOYSTICK_MAX_SPEED) && close(state.wheel[1], JOYSTICK_MAX_SPEED));
        CHECK(close(state.wheel[2], JOYSTICK_MAX_SPEED) && close(state.wheel[3], -JOYSTICK_MAX_SPEED));
    } else if (name == "rc_loss" || name == "invalid_joystick") {
        running(); radio().channels[FORWARD.number] = FORWARD.maximum; tick(695, 8);
        if (name == "rc_loss") radio().linked = false; else radio().channels[ROTATE.number] = 2101;
        Harness::sent.clear(); tick(696, 8);
        CHECK(ChassisController::status().mode == Mode::Locked); zeroWheels(); onlyStops();
    } else if (name == "heartbeat_timeout") {
        running(); tick(1180, 8, false);
        CHECK(ChassisController::status().mode == Mode::Running);
        tick(1181, 8, false); CHECK(ChassisController::status().mode == Mode::Locked);
    } else if (name == "axis_error" || name == "closed_loop_exit") {
        running(); mockMillis = 681;
        ChassisController::receive(2, heartbeat(2, name == "axis_error" ? 8 : 1,
                                              name == "axis_error" ? 7 : 0));
        RcController::poll(); ChassisController::poll();
        CHECK(ChassisController::status().mode == Mode::Locked); zeroWheels();
    } else if (name == "wrong_heartbeat") {
        mockMillis = 100; auto message = heartbeat(0);
        ChassisController::receive(2, message); CHECK(!ChassisController::status().axes[0].seen);
        message.flags.extended = true; ChassisController::receive(1, message);
        CHECK(!ChassisController::status().axes[0].seen);
        message.flags.extended = false; message.flags.remote = true;
        ChassisController::receive(1, message); CHECK(!ChassisController::status().axes[0].seen);
        message.flags.remote = false; message.len = 4;
        ChassisController::receive(1, message); CHECK(!ChassisController::status().axes[0].seen);
        message = heartbeat(0); ChassisController::receive(1, message);
        CHECK(ChassisController::status().axes[0].seen);
    } else if (name == "arming_moved" || name == "arming_timeout" || name == "late_arming_reply") {
        arming();
        if (name == "arming_moved") { radio().channels[FORWARD.number] = FORWARD.maximum; tick(615); }
        else {
            for (uint32_t at = 850; at <= 3600; at += 250) tick(at);
            CHECK(ChassisController::status().mode == Mode::Arming);
            tick(3601, name == "late_arming_reply" ? 8 : 1);
        }
        CHECK(ChassisController::status().mode == Mode::Locked);
    } else if (name == "velocity_send_failure") {
        running(); radio().channels[FORWARD.number] = FORWARD.maximum;
        Harness::failAt = Harness::attempts + 2; Harness::sent.clear(); tick(695, 8);
        CHECK(ChassisController::status().mode == Mode::Locked); zeroWheels();
        bool failedWheelSentPositive = false;
        for (const auto &frame : Harness::sent) {
            if ((frame.id >> 5) >= 12 && (frame.id & 31) == 0x0D) {
                float value; memcpy(&value, frame.bytes.data(), 4);
                failedWheelSentPositive |= value != 0;
            }
        }
        CHECK(!failedWheelSentPositive);
    } else if (name == "normal_recovery") {
        running(); radio().linked = false; tick(695, 8);
        neutral(); tick(700); tick(1199); CHECK(ChassisController::status().mode == Mode::Locked);
        tick(1200); CHECK(ChassisController::status().mode == Mode::Arming);
        tick(1215, 8); CHECK(ChassisController::status().mode == Mode::Running);
    } else if (name == "global_stop_latch") {
        running(); SystemSafety::stop("Test STOP");
        CHECK(ChassisController::status().emergencyLatched);
        tick(700); tick(1200); tick(1700);
        CHECK(ChassisController::status().mode == Mode::Locked);
        button(SystemConfig::Rc::CLEAR_CHANNEL, 1800); tick(2400);
        CHECK(ChassisController::status().mode == Mode::Locked);
        CHECK(Harness::nextCount == 0);
    } else if (name == "rollover") {
        tick(UINT32_MAX - 300); tick(UINT32_MAX - 50); tick(198);
        CHECK(ChassisController::status().mode == Mode::Locked);
        tick(199); CHECK(ChassisController::status().mode == Mode::Arming);
        tick(214, 8); CHECK(ChassisController::status().mode == Mode::Running);
    } else if (name == "next_prev") {
        running(); button(6, 700); CHECK(Harness::nextCount == 1);
        ready(); button(6, 750); CHECK(Harness::nextCount == 1);
        button(6, 800, false); button(5, 850); CHECK(Harness::previousCount == 1);
    } else if (name == "busy_press_not_queued") {
        running(); Harness::mechanism.phase = "busy";
        button(6, 700); ready(); button(6, 750);
        CHECK(Harness::nextCount == 0);
        button(6, 800, false); button(6, 850); CHECK(Harness::nextCount == 1);
    } else if (name == "startup_held_button") {
        radio().channels[6] = 2000; running(); tick(730, 8);
        CHECK(Harness::nextCount == 0);
        button(6, 800, false); button(6, 850); CHECK(Harness::nextCount == 1);
    } else if (name == "relink_held_button") {
        running(); radio().linked = false; tick(695, 8);
        neutral(); radio().channels[6] = 2000;
        tick(700); tick(1200); tick(1215, 8); tick(1250, 8); tick(1300, 8);
        CHECK(Harness::nextCount == 0);
        button(6, 1400, false); button(6, 1450); CHECK(Harness::nextCount == 1);
    } else if (name == "button_debounce") {
        running(); radio().channels[6] = 2000; tick(700, 8); tick(729, 8);
        CHECK(Harness::nextCount == 0);
        radio().channels[6] = 1000; tick(730, 8); tick(760, 8);
        radio().channels[6] = 2000; tick(770, 8); tick(800, 8);
        CHECK(Harness::nextCount == 1);
    } else if (name == "both_navigation_buttons") {
        running(); radio().channels[5] = radio().channels[6] = 2000;
        tick(700, 8); tick(730, 8);
        CHECK(Harness::nextCount == 0 && Harness::previousCount == 0);
        CHECK(RcController::status().rejectedActions == 1);
    } else if (name == "web_stage_shared_navigation") {
        running(); CHECK(RcController::status().selectedStage == 4);
        // A Web RAM commit changes the shared active stage while the RC switch stays high.
        Harness::mechanism.stage = 2; Harness::mechanism.phase = "busy";
        button(6, 700); ready(); button(6, 750);
        CHECK(Harness::nextCount == 0 && Harness::mechanism.stage == 2);
        button(6, 800, false); button(6, 850);
        CHECK(Harness::nextCount == 1 && Harness::mechanism.stage == 2);
        ready(); button(5, 900, false); button(5, 950);
        CHECK(Harness::previousCount == 1 && Harness::mechanism.stage == 2);
        CHECK(RcController::status().selectedStage == 4);
        CHECK(Harness::selectCount == 1); // Held RC stage switch never retargets the Web stage.
    } else if (name == "stage_channel_mapping") {
        running(); button(10,700); CHECK(Harness::mechanism.stage==2);
        button(11,750); CHECK(Harness::mechanism.stage==3);
        button(12,800,false); button(12,850); CHECK(Harness::mechanism.pendingStage==4);
        button(13,900); CHECK(Harness::mechanism.pendingStage==5);
    } else if (name == "stage_initial_ambiguous") {
        radio().channels[SystemConfig::Rc::STAGE_CHANNELS[0]] = 2000; arming(); tick(615, 8); tick(650, 8); tick(680, 8);
        CHECK(RcController::status().selectedStage == 0 && Harness::selectCount == 0);
        button(6, 700); CHECK(Harness::nextCount == 0);
        button(11, 750); CHECK(Harness::mechanism.stage == 3);
    } else if (name == "stage_initial_glitch") {
        running(); const unsigned originalSelections = Harness::selectCount;
        radio().channels[SystemConfig::Rc::STAGE_CHANNELS[2]] = 500; tick(700, 8);
        radio().channels[SystemConfig::Rc::STAGE_CHANNELS[2]] = 2000; tick(710, 8);
        radio().channels[SystemConfig::Rc::STAGE_CHANNELS[2]] = 1000; tick(720, 8); tick(740, 8);
        CHECK(Harness::selectCount == originalSelections);
        tick(750, 8); CHECK(RcController::status().selectedStage == 0);
    } else if (name == "stage_release_fallback_held_selector") {
        running(); button(10, 700); CHECK(RcController::status().selectedStage == 2);
        button(10, 750, false); CHECK(RcController::status().selectedStage == 4);
        tick(850, 8); CHECK(RcController::status().selectedStage == 4);
        CHECK(Harness::mechanism.stage == 4 && Harness::selectCount == 3);
    } else if (name == "stage3_park_retarget") {
        running(); button(11,700); CHECK(Harness::mechanism.stage==3);
        Harness::mechanism.phase="busy";
        button(10,750); CHECK(Harness::mechanism.pendingStage==2);
        button(13,800); CHECK(Harness::mechanism.pendingStage==5);
        button(6,850); CHECK(Harness::nextCount==0);
    } else if (name == "busy_other_stage_selection_queued_once") {
        running(); Harness::mechanism.phase = "busy";
        const unsigned selections = Harness::selectCount;
        button(SystemConfig::Rc::STAGE_CHANNELS[0], 700);
        CHECK(Harness::mechanism.stage == 4 && Harness::mechanism.pendingStage == 2);
        CHECK(RcController::status().selectedStage == 2 && Harness::selectCount == selections + 1);
        // Simulate the mechanism completing the queued return. The held selector is not replayed.
        Harness::mechanism.stage = Harness::mechanism.pendingStage;
        Harness::mechanism.pendingStage = 0; ready(); tick(800, 8); tick(850, 8);
        CHECK(Harness::mechanism.stage == 2 && Harness::selectCount == selections + 1);
        CHECK(Harness::nextCount == 0 && Harness::previousCount == 0);
    } else if (name == "debug_upload_locked_navigation") {
        running(); Harness::mechanism.debug = true;
        button(6, 700); ready(); Harness::mechanism.debug = false;
        button(6, 750); CHECK(Harness::nextCount == 0);
        Harness::mechanism.uploading = true; button(5, 800);
        CHECK(Harness::previousCount == 0);
    } else if (name == "clear_errors_locked") {
        tick(100); tick(140); Harness::sent.clear();
        radio().channels[SystemConfig::Rc::CLEAR_CHANNEL] = 2000; tick(150); tick(180);
        CHECK(commandCount(0x18) == 4);
        for (const auto &frame : Harness::sent) if ((frame.id & 31) == 0x18) CHECK(frame.bytes.empty());
        tick(190); CHECK(commandCount(0x18) == 4);
    } else if (name == "clear_errors_retry_fresh_truth") {
        running(); mockMillis = 681; ChassisController::receive(1, heartbeat(0, 8, 9));
        Harness::sent.clear(); Harness::failAt = Harness::attempts + 2;
        ChassisController::clearErrors(); CHECK(commandCount(0x18) == 3);
        CHECK(ChassisController::status().axes[0].error == 9);
        mockMillis = 691; ChassisController::poll(); CHECK(commandCount(0x18) == 4);
        CHECK(ChassisController::status().axes[0].error == 9);
    } else if (name == "clear_startup_held") {
        radio().channels[SystemConfig::Rc::CLEAR_CHANNEL] = 2000; tick(100); tick(140);
        CHECK(commandCount(0x18) == 0);
        radio().channels[SystemConfig::Rc::CLEAR_CHANNEL] = 1000; tick(150); tick(180);
        radio().channels[SystemConfig::Rc::CLEAR_CHANNEL] = 2000; tick(190); tick(220);
        CHECK(commandCount(0x18) == 4);
    } else if (name == "fresh_fault_blocks_navigation") {
        running(); radio().channels[6] = 2000; tick(700, 8);
        mockMillis = 730; ChassisController::receive(1, heartbeat(0, 8, 42));
        RcController::poll(); CHECK(Harness::nextCount == 0);
    } else if (name == "rc_snapshot_channels") {
        tick(100); const auto result = RcController::status();
        CHECK(result.link && result.valid && result.neutral && result.ageMs == 0);
        CHECK(result.channels[0] == ROTATE.middle && result.channels[1] == FORWARD.middle && result.channels[2] == STRAFE.middle);
        radio().linked = false; tick(150); CHECK(RcController::status().ageMs == 50);
    } else throw std::runtime_error("Unknown chassis/RC case");
}
int main(int argc, char **argv) {
    try { CHECK(argc == 2); initialize(); test(argv[1]); printf("PASS %s\n", argv[1]); return 0; }
    catch (const std::exception &error) { fprintf(stderr, "FAIL %s: %s\n", argc > 1 ? argv[1] : "?", error.what()); return 1; }
}
