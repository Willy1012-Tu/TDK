#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <AlfredoCRSF.h>
#include <string.h>
#include "chassis_math.h"

namespace {
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;
AlfredoCRSF crsf;

constexpr uint8_t HEARTBEAT = 0x01, SET_AXIS_STATE = 0x07, SET_INPUT_VEL = 0x0D;
constexpr uint32_t IDLE = 1, CLOSED_LOOP = 8;
enum class Mode { Locked, Arming, Running };
Mode mode = Mode::Locked;
struct Motor {
    bool seen = false;
    uint32_t heartbeatAt = 0, error = 0;
    uint8_t state = 0;
} motor[4];
bool trackingCenter = false;
uint32_t centerSince = 0, armSince = 0, lastSend = 0, lastIdle = 0, lastStatus = 0;
float wheel[4] = {};

bool send(uint8_t index, uint8_t command, const void *data, uint8_t size) {
    CAN_message_t msg{};
    msg.id = (uint32_t(Config::NODE_ID[index]) << 5) | command;
    msg.len = size;
    memcpy(msg.buf, data, size);
    return (Config::CAN_BUS[index] == 1 ? can1.write(msg) : can2.write(msg)) != 0;
}

bool velocity(uint8_t index, float value) {
    const float payload[2] = {value, 0.0f}; // turns/s、torque feedforward
    return send(index, SET_INPUT_VEL, payload, sizeof(payload));
}

bool axisState(uint8_t index, uint32_t state) {
    return send(index, SET_AXIS_STATE, &state, sizeof(state));
}

void idleAll() {
    // 即使某顆傳送失敗，仍逐一嘗試讓另外三顆停止。
    for (uint8_t i = 0; i < 4; ++i) {
        wheel[i] = 0;
        velocity(i, 0);
        axisState(i, IDLE);
    }
}

void lock(const char *reason) {
    trackingCenter = false;
    if (mode == Mode::Locked) return;
    mode = Mode::Locked;
    idleAll();
    if (Serial) Serial.printf("LOCKED: %s\n", reason);
}

void heartbeat(uint8_t bus, const CAN_message_t &msg) {
    if (msg.flags.extended || msg.flags.remote || msg.len < 5 ||
        msg.id > 0x7FF || (msg.id & 0x1F) != HEARTBEAT) return;
    for (uint8_t i = 0; i < 4; ++i) {
        if ((msg.id >> 5) != Config::NODE_ID[i] || bus != Config::CAN_BUS[i]) continue;
        memcpy(&motor[i].error, msg.buf, 4);
        motor[i].state = msg.buf[4];
        motor[i].heartbeatAt = millis();
        motor[i].seen = true;
        return;
    }
}

void pollCan() {
    can1.events();
    can2.events();
    CAN_message_t msg{};
    for (int n = 0; n < 32 && can1.read(msg); ++n) heartbeat(1, msg);
    for (int n = 0; n < 32 && can2.read(msg); ++n) heartbeat(2, msg);
}

bool healthy(uint32_t now) {
    for (const Motor &m : motor)
        if (!m.seen || m.error || now - m.heartbeatAt > Config::HEARTBEAT_TIMEOUT_MS)
            return false;
    return true;
}

bool allInState(uint8_t state) {
    for (const Motor &m : motor) if (!m.seen || m.state != state) return false;
    return true;
}

void arm(uint32_t now) {
    mode = Mode::Arming;
    armSince = now;
    for (uint8_t i = 0; i < 4; ++i) {
        if (!velocity(i, 0) || !axisState(i, CLOSED_LOOP)) {
            lock("CAN transmit failed");
            return;
        }
    }
    if (Serial) Serial.println("ARMING");
}

void updateMode(uint32_t now, bool rcOK, bool neutral) {
    if (!rcOK) { lock("ELRS link / channel invalid"); return; }
    if (!healthy(now)) { lock("ODrive error / heartbeat lost"); return; }
    if (mode == Mode::Locked) {
        // 故障恢復後，四輪 IDLE 且搖桿持續置中，才重新解鎖。
        if (!neutral || !allInState(IDLE)) { trackingCenter = false; return; }
        if (!trackingCenter) { trackingCenter = true; centerSince = now; }
        if (now - centerSince >= Config::CENTER_HOLD_MS) arm(now);
    } else if (mode == Mode::Arming) {
        if (!neutral) lock("Joystick moved during arming");
        else if (now - armSince > Config::ARM_TIMEOUT_MS) lock("Arming timeout");
        else if (allInState(CLOSED_LOOP)) {
            mode = Mode::Running;
            if (Serial) Serial.println("RUNNING");
        }
    } else if (!allInState(CLOSED_LOOP)) lock("ODrive left closed loop");
}
} // namespace

void setup() {
    Serial.begin(115200);
    Serial2.begin(Config::CRSF_BAUD); // ELRS：RX=7、TX=8
    crsf.begin(Serial2);
    can1.begin(); can1.setBaudRate(Config::CAN_BAUD); can1.enableFIFO();
    can2.begin(); can2.setBaudRate(Config::CAN_BAUD); can2.enableFIFO();
    idleAll();
    if (Serial) Serial.println("LOCKED: Waiting for ELRS, four ODrives and centered joystick");
}

void loop() {
    crsf.update();
    pollCan();
    const uint32_t now = millis();
    const int forward = crsf.getChannel(Config::FORWARD.number);
    const int strafe = crsf.getChannel(Config::STRAFE.number);
    const int rotate = crsf.getChannel(Config::ROTATE.number);
    const bool rcOK = crsf.isLinkUp() && channelValid(forward) &&
                      channelValid(strafe) && channelValid(rotate);
    const bool neutral = centered(forward, Config::FORWARD) &&
                         centered(strafe, Config::STRAFE) && centered(rotate, Config::ROTATE);
    updateMode(now, rcOK, neutral);

    if (now - lastSend >= Config::SEND_INTERVAL_MS) {
        lastSend = now;
        if (mode == Mode::Running) mixWheels(forward, strafe, rotate, wheel);
        else for (float &value : wheel) value = 0;
        for (uint8_t i = 0; i < 4; ++i) {
            if (!velocity(i, wheel[i])) {
                lock("CAN transmit failed");
                break; // 不再送出本輪剩下的非零輪速
            }
        }
    }
    if (mode == Mode::Locked && now - lastIdle >= Config::IDLE_RETRY_MS) {
        lastIdle = now;
        idleAll();
    }
    if (Config::PRINT_STATUS && now - lastStatus >= Config::STATUS_INTERVAL_MS) {
        lastStatus = now;
        if (Serial && Serial.availableForWrite() >= 128)
            Serial.printf("%s Forward:%d Strafe:%d Rotate:%d Wheel:%.1f %.1f %.1f %.1f\n",
                          mode == Mode::Running ? "RUNNING" : mode == Mode::Arming ? "ARMING" : "LOCKED",
                          forward, strafe, rotate, wheel[0], wheel[1], wheel[2], wheel[3]);
    }
}
