#include <stdio.h>
#include <stdexcept>
#include "../src/main.cpp"

#define CHECK(expr) do { if (!(expr)) throw std::runtime_error(#expr); } while (0)
bool close(float a, float b) { return fabsf(a - b) < 0.001f; }
CAN_message_t hb(uint8_t i, uint8_t state = IDLE, uint32_t error = 0) {
    CAN_message_t msg{};
    msg.id = (uint32_t(Config::NODE_ID[i]) << 5) | HEARTBEAT;
    msg.len = 8;
    memcpy(msg.buf, &error, 4); msg.buf[4] = state;
    return msg;
}
void neutral() {
    crsf.linked = true;
    crsf.channels[Config::FORWARD.number] = Config::FORWARD.middle;
    crsf.channels[Config::STRAFE.number] = Config::STRAFE.middle;
    crsf.channels[Config::ROTATE.number] = Config::ROTATE.middle;
}
void tick(uint32_t at, uint8_t state = IDLE, bool fresh = true) {
    testNow = at;
    if (fresh) {
        for (uint8_t i = 0; i < 4; ++i) {
            if (Config::CAN_BUS[i] == 1) can1.incoming.push_back(hb(i, state));
            else can2.incoming.push_back(hb(i, state));
        }
    }
    loop();
}
void arming() {
    neutral(); tick(100); tick(350); tick(599);
    CHECK(mode == Mode::Locked);
    tick(600); CHECK(mode == Mode::Arming);
}
void running() {
    arming(); tick(615, CLOSED_LOOP); CHECK(mode == Mode::Running);
}
template<class Bus> void onlyStopCommands(const Bus &bus) {
    for (const auto &msg : bus.sent) {
        if ((msg.id & 0x1F) == SET_INPUT_VEL) {
            float v; memcpy(&v, msg.buf, 4); CHECK(v == 0);
        } else {
            uint32_t state; memcpy(&state, msg.buf, 4); CHECK(state == IDLE);
        }
    }
}
void zeroWheels() { for (float v : wheel) CHECK(v == 0); }

void test(const std::string &name) {
    if (name == "math") {
        for (Config::Channel c : {Config::FORWARD, Config::STRAFE, Config::ROTATE}) {
            CHECK(mapJoystick(c.minimum, c) == -1);
            CHECK(mapJoystick(c.maximum, c) == 1);
            CHECK(mapJoystick(900, c) == -1);
            CHECK(mapJoystick(2100, c) == 1);
            CHECK(mapJoystick(c.middle - Config::DEAD_ZONE, c) == 0);
            CHECK(mapJoystick(c.middle + Config::DEAD_ZONE, c) == 0);
            CHECK(mapJoystick(c.middle + Config::DEAD_ZONE + 1, c) > 0);
            CHECK(mapJoystick(c.middle - Config::DEAD_ZONE - 1, c) < 0);
        }
        float w[4];
        mixWheels(Config::FORWARD.maximum, Config::STRAFE.middle, Config::ROTATE.middle, w);
        CHECK(close(w[0], -90) && close(w[1], 90) && close(w[2], 90) && close(w[3], -90));
        mixWheels(Config::FORWARD.middle, Config::STRAFE.maximum, Config::ROTATE.middle, w);
        CHECK(close(w[0], -90) && close(w[1], -90) && close(w[2], -90) && close(w[3], -90));
        mixWheels(Config::FORWARD.middle, Config::STRAFE.middle, Config::ROTATE.maximum, w);
        CHECK(close(w[0], -90) && close(w[1], -90) && close(w[2], 90) && close(w[3], 90));
        mixWheels(Config::FORWARD.maximum, Config::STRAFE.maximum, Config::ROTATE.maximum, w);
        CHECK(close(w[0], -90) && close(w[1], -30) && close(w[2], 30) && close(w[3], -30));
        for (int f = 900; f <= 2100; f += 50)
            for (int s = 900; s <= 2100; s += 50)
                for (int r = 900; r <= 2100; r += 50) {
                    mixWheels(f, s, r, w);
                    for (float v : w) CHECK(fabsf(v) <= Config::MAX_SPEED + 0.001f);
                }
    } else if (name == "boot_routing") {
        CHECK(Serial.baud == 115200 && Serial2.baud == Config::CRSF_BAUD);
        CHECK(can1.baud == Config::CAN_BAUD && can2.baud == Config::CAN_BAUD);
        CHECK(can1.fifo && can2.fifo && can1.sent.size() == 4 && can2.sent.size() == 4);
        onlyStopCommands(can1); onlyStopCommands(can2);
        for (uint8_t i = 0; i < 4; ++i) {
            const auto &frames = Config::CAN_BUS[i] == 1 ? can1.sent : can2.sent;
            const auto &v = frames[(i % 2) * 2];
            CHECK(v.id == (uint32_t(Config::NODE_ID[i]) << 5 | SET_INPUT_VEL));
            CHECK(v.len == 8 && !v.flags.extended && !v.flags.remote);
            float torque; memcpy(&torque, v.buf + 4, 4); CHECK(torque == 0);
        }
        neutral(); tick(100, IDLE, false); tick(1000, IDLE, false);
        CHECK(mode == Mode::Locked); zeroWheels();
    } else if (name == "center_hold") {
        neutral(); crsf.channels[3] = Config::FORWARD.maximum;
        tick(100); tick(700); CHECK(mode == Mode::Locked);
        neutral(); tick(800); tick(1299); CHECK(mode == Mode::Locked);
        tick(1300); CHECK(mode == Mode::Arming);
        tick(1315, CLOSED_LOOP); CHECK(mode == Mode::Running); zeroWheels();
    } else if (name == "motion") {
        running(); crsf.channels[3] = Config::FORWARD.maximum;
        can1.sent.clear(); can2.sent.clear(); tick(630, CLOSED_LOOP);
        CHECK(mode == Mode::Running && close(wheel[0], -90) && close(wheel[1], 90));
        CHECK(can1.sent.size() == 2 && can2.sent.size() == 2);
        float v; memcpy(&v, can1.sent[0].buf, 4); CHECK(close(v, -90));
    } else if (name == "rc_loss" || name == "rc_invalid") {
        running(); crsf.channels[3] = Config::FORWARD.maximum; tick(630, CLOSED_LOOP);
        if (name == "rc_loss") crsf.linked = false;
        else crsf.channels[1] = 2101;
        can1.sent.clear(); can2.sent.clear(); tick(631, CLOSED_LOOP);
        CHECK(mode == Mode::Locked); zeroWheels(); onlyStopCommands(can1); onlyStopCommands(can2);
    } else if (name == "heartbeat_timeout") {
        running(); tick(1115, CLOSED_LOOP, false); CHECK(mode == Mode::Running);
        tick(1116, CLOSED_LOOP, false); CHECK(mode == Mode::Locked); zeroWheels();
    } else if (name == "axis_error") {
        running(); testNow = 630;
        can2.incoming.push_back(hb(2, CLOSED_LOOP, 1)); loop();
        CHECK(mode == Mode::Locked); zeroWheels();
    } else if (name == "wrong_frames") {
        testNow = 100;
        auto msg = hb(0); heartbeat(2, msg); CHECK(!motor[0].seen);
        msg.flags.extended = true; heartbeat(1, msg); CHECK(!motor[0].seen);
        msg.flags.extended = false; msg.flags.remote = true; heartbeat(1, msg); CHECK(!motor[0].seen);
        msg.flags.remote = false; msg.len = 4; heartbeat(1, msg); CHECK(!motor[0].seen);
        msg.len = 8; msg.id += 1; heartbeat(1, msg); CHECK(!motor[0].seen);
        msg = hb(0); heartbeat(1, msg); CHECK(motor[0].seen && motor[0].heartbeatAt == 100);
    } else if (name == "arm_movement") {
        arming(); crsf.channels[3] = Config::FORWARD.maximum; tick(615);
        CHECK(mode == Mode::Locked); zeroWheels();
    } else if (name == "arm_timeout") {
        arming();
        for (uint32_t at = 850; at <= 3600; at += 250) tick(at);
        CHECK(mode == Mode::Arming); tick(3601); CHECK(mode == Mode::Locked);
    } else if (name == "late_arm_reply") {
        arming(); for (uint32_t at = 850; at <= 3350; at += 250) tick(at);
        tick(3601, CLOSED_LOOP); CHECK(mode == Mode::Locked); zeroWheels();
    } else if (name == "arm_send_failure") {
        neutral(); tick(100); tick(350);
        can1.failAt = can1.writes + 2; tick(600);
        CHECK(mode == Mode::Locked); zeroWheels(); CHECK(!trackingCenter);
    } else if (name == "motion_send_failure") {
        running(); crsf.channels[3] = Config::FORWARD.maximum;
        can1.failAt = can1.writes + 2;
        can2.sent.clear(); tick(630, CLOSED_LOOP);
        CHECK(mode == Mode::Locked); zeroWheels(); onlyStopCommands(can2);
    } else if (name == "closed_loop_exit") {
        running(); tick(630, IDLE); CHECK(mode == Mode::Locked); zeroWheels();
    } else if (name == "recovery") {
        running(); crsf.linked = false; tick(630, CLOSED_LOOP);
        neutral(); tick(700); tick(1199); CHECK(mode == Mode::Locked);
        tick(1200); CHECK(mode == Mode::Arming);
        tick(1215, CLOSED_LOOP); CHECK(mode == Mode::Running);
    } else if (name == "rollover") {
        neutral(); tick(UINT32_MAX - 300); tick(UINT32_MAX - 50);
        tick(198); CHECK(mode == Mode::Locked);
        tick(199); CHECK(mode == Mode::Arming);
        tick(214, CLOSED_LOOP); CHECK(mode == Mode::Running);
    } else if (name == "without_usb") {
        Serial.connected = false; running(); CHECK(mode == Mode::Running);
    } else throw std::runtime_error("Unknown case");
}

int main(int argc, char **argv) {
    try {
        CHECK(argc == 2); setup(); test(argv[1]);
        ::printf("PASS %s\n", argv[1]); return 0;
    } catch (const std::exception &e) {
        ::fprintf(stderr, "FAIL: %s\n", e.what()); return 1;
    }
}
