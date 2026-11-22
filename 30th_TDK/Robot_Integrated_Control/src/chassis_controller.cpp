#include "chassis_controller.h"
#include "chassis_math.h"
#include "chassis_ramp.h"
#include "can_transport.h"
#include "rc_controller.h"
#include "system_safety.h"
#include "mechanism_controller.h"
#include <Arduino.h>
#include <string.h>

namespace ChassisController {
namespace {
using namespace SystemConfig::Chassis;
constexpr uint8_t HEARTBEAT = 0x01, SET_AXIS_STATE = 0x07;
constexpr uint8_t SET_INPUT_VEL = 0x0D, CLEAR_ERRORS = 0x18;
constexpr uint32_t IDLE = 1, CLOSED_LOOP = 8;
struct Axis {
    bool seen = false;
    uint32_t heartbeatAt = 0, error = 0;
    uint8_t state = 0;
};
Axis axes[4];
Mode mode = Mode::Locked;
const char *reason = "Waiting for ELRS and four healthy ODrives";
bool trackingCenter = false, emergencyLatched = false;
uint32_t centerSince = 0, armSince = 0, lastSend = 0, lastIdle = 0;
uint8_t pendingIdle = 0, pendingClear = 0;
uint32_t lastRetry = 0;
float wheel[4] = {}, targetWheel[4] = {};
MotionSettings motionBanks[2] = {defaultMotionSettings(0), defaultMotionSettings(1)};
uint8_t motionProfile() { return RcController::status().processionMode ? 1 : 0; }
MotionSettings &motionSettings() { return motionBanks[motionProfile()]; }
ChassisMath::IcrGeometry geometry(int rotate) {
    const auto &motion = motionSettings();
    return ChassisMath::turnIcr(ChassisMath::mapJoystick(rotate, ROTATE),
        motion.wheelbaseMm, motion.trackWidthMm, motion.icrXMm, motion.icrLeftYMm, motion.icrRightYMm);
}
bool motionEditable() {
    if (emergencyLatched || SystemSafety::stopped() || mode == Mode::Arming) return false;
    const auto rc = RcController::status();
    // Invalid/disconnected sticks cannot authorize reconfiguration while Running.
    if (mode == Mode::Running && (!rc.link || !rc.valid || !rc.neutral)) return false;
    for (uint8_t i = 0; i < 4; ++i)
        if (wheel[i] != 0 || targetWheel[i] != 0) return false;
    return true;
}
uint8_t directionStage() {
    if (RcController::status().processionMode) return SystemConfig::Rc::PROCESSION_DIRECTION_STAGE;
    const auto mechanism = MechanismController::status();
    return mechanism.pendingStage ? mechanism.pendingStage : mechanism.stage;
}
bool send(uint8_t index, uint8_t command, const void *data, uint8_t size) {
    return CanTransport::send(CAN_BUS[index],
        (uint32_t(NODE_ID[index]) << 5) | command,
        static_cast<const uint8_t *>(data), size);
}
bool velocity(uint8_t index, float value) {
    const float payload[2] = {value, 0.0f};
    return send(index, SET_INPUT_VEL, payload, sizeof(payload));
}
bool axisState(uint8_t index, uint32_t state) {
    return send(index, SET_AXIS_STATE, &state, sizeof(state));
}
void serviceIdle() {
    for (uint8_t i = 0; i < 4; ++i) {
        if (!(pendingIdle & (1U << i))) continue;
        const bool zeroSent = velocity(i, 0);
        const bool idleSent = axisState(i, IDLE);
        if (zeroSent && idleSent) pendingIdle &= uint8_t(~(1U << i));
    }
}
void idleAll() {
    for (float &value : wheel) value = 0;
    for (float &value : targetWheel) value = 0;
    pendingIdle = 0x0F;
    serviceIdle();
}
void lock(const char *why) {
    const bool wasActive = mode != Mode::Locked;
    mode = Mode::Locked;
    trackingCenter = false;
    reason = why;
    if (wasActive) idleAll();
}
bool healthy(uint32_t now) {
    for (const Axis &axis : axes)
        if (!axis.seen || axis.error || now - axis.heartbeatAt > HEARTBEAT_TIMEOUT_MS)
            return false;
    return true;
}
bool allInState(uint8_t expected) {
    for (const Axis &axis : axes)
        if (!axis.seen || axis.state != expected) return false;
    return true;
}
void arm(uint32_t now) {
    mode = Mode::Arming;
    reason = "Waiting for all four ODrives to enter closed loop";
    armSince = now;
    trackingCenter = false;
    for (uint8_t i = 0; i < 4; ++i) {
        if (!velocity(i, 0) || !axisState(i, CLOSED_LOOP)) {
            lock("CAN transmit failed while arming");
            return;
        }
    }
}
void updateMode(uint32_t now, const RcController::Status &rc) {
    if (emergencyLatched || SystemSafety::stopped()) {
        lock("Global STOP latched; reboot to restart");
        return;
    }
    if (!rc.link || !rc.valid) { lock("ELRS link lost or joystick channel invalid"); return; }
    if (!healthy(now)) { lock("ODrive error or heartbeat timeout"); return; }
    if (mode == Mode::Locked) {
        if (!rc.neutral || !allInState(IDLE) || pendingIdle) { trackingCenter = false; return; }
        if (!trackingCenter) { trackingCenter = true; centerSince = now; }
        if (now - centerSince >= CENTER_HOLD_MS) arm(now);
    } else if (mode == Mode::Arming) {
        if (!rc.neutral) lock("Joystick moved while arming");
        else if (now - armSince > ARM_TIMEOUT_MS) lock("ODrive arming timeout");
        else if (allInState(CLOSED_LOOP)) {
            mode = Mode::Running;
            reason = "Ready";
        }
    } else if (!allInState(CLOSED_LOOP)) lock("ODrive left closed loop");
}
void serviceClear() {
    for (uint8_t i = 0; i < 4; ++i) {
        if (!(pendingClear & (1U << i))) continue;
        if (send(i, CLEAR_ERRORS, nullptr, 0)) pendingClear &= uint8_t(~(1U << i));
    }
}

} // namespace

void begin() {
    motionBanks[0] = defaultMotionSettings(0);
    motionBanks[1] = defaultMotionSettings(1);
    for (Axis &axis : axes) axis = Axis{};
    mode = Mode::Locked;
    reason = "Waiting for ELRS and four healthy ODrives";
    trackingCenter = emergencyLatched = false;
    centerSince = armSince = lastSend = lastIdle = lastRetry = millis();
    pendingClear = 0;
    idleAll();
}
void receive(uint8_t bus, const CAN_message_t &message) {
    if (message.flags.extended || message.flags.remote || message.len < 5 ||
        message.id > 0x7FF || (message.id & 0x1F) != HEARTBEAT) return;
    for (uint8_t i = 0; i < 4; ++i) {
        if ((message.id >> 5) != NODE_ID[i] || bus != CAN_BUS[i]) continue;
        memcpy(&axes[i].error, message.buf, sizeof(uint32_t));
        axes[i].state = message.buf[4];
        axes[i].heartbeatAt = millis();
        axes[i].seen = true;
        return;
    }
}
void poll() {
    const uint32_t now = millis();
    const RcController::Status rc = RcController::status();

    const auto &motion = motionSettings();
    updateMode(now, rc);
    if (now - lastSend >= SEND_INTERVAL_MS) {
        const uint32_t elapsed = now - lastSend;
        const uint32_t rampMs = elapsed > RAMP_MAX_DT_MS ? RAMP_MAX_DT_MS : elapsed;
        lastSend = now;
        if (mode == Mode::Running) {
            // The latest accepted target changes the driver frame immediately;
            // the mechanism still finishes its independent safe return barrier.
            const uint8_t stage = directionStage();
            const auto icr = geometry(rc.rotate);
            ChassisMath::mixWheels(rc.forward, rc.strafe, rc.rotate, targetWheel, stage, icr);
            ChassisMath::clampWheelSpeeds(targetWheel);
            if (motion.rampEnabled)
                ChassisRamp::advance(wheel, targetWheel, rampMs, motion.acceleration, motion.deceleration);
            else for (uint8_t i = 0; i < 4; ++i) wheel[i] = targetWheel[i];
        } else {
            for (float &value : wheel) value = 0;
            for (float &value : targetWheel) value = 0;
        }
        for (uint8_t i = 0; i < 4; ++i) {
            if (!velocity(i, wheel[i])) {
                lock("CAN velocity transmit failed");
                break;
            }
        }
    }
    if (mode == Mode::Locked && now - lastIdle >= IDLE_RETRY_MS) {
        lastIdle = now;
        idleAll();
    }
    if (now - lastRetry >= CLEAR_RETRY_MS) {
        lastRetry = now;
        serviceIdle();
        serviceClear();
    }
}
void emergencyStop() {
    emergencyLatched = true;
    mode = Mode::Locked;
    trackingCenter = false;
    reason = "Global STOP latched; reboot to restart";
    idleAll();
}
void clearErrors() {
    // Preserve heartbeat errors until a fresh drive heartbeat confirms recovery.
    pendingClear = 0x0F;
    serviceClear();
}
MotionSettings defaultMotionSettings(uint8_t profile) {
    if (profile == 1) return {Procession::RAMP_ENABLED, Procession::ACCEL_TURNS_S2, Procession::DECEL_TURNS_S2,
        WHEELBASE_MM, TRACK_WIDTH_MM, Procession::ICR_X_MM, Procession::ICR_LEFT_Y_MM, Procession::ICR_RIGHT_Y_MM};
    return {RAMP_ENABLED, ACCEL_TURNS_S2, DECEL_TURNS_S2,
            WHEELBASE_MM, TRACK_WIDTH_MM, ICR_X_MM, ICR_LEFT_Y_MM, ICR_RIGHT_Y_MM};
}
bool configureMotion(const MotionSettings &settings, uint8_t expectedProfile) {
    if (expectedProfile != 255 && expectedProfile != motionProfile()) return false;
    const ChassisMath::IcrGeometry left{settings.wheelbaseMm, settings.trackWidthMm, settings.icrXMm, settings.icrLeftYMm};
    const ChassisMath::IcrGeometry right{settings.wheelbaseMm, settings.trackWidthMm, settings.icrXMm, settings.icrRightYMm};
    if (!std::isfinite(settings.acceleration) || settings.acceleration < 1 || settings.acceleration > MAX_RAMP_RATE ||
        !std::isfinite(settings.deceleration) || settings.deceleration < 1 || settings.deceleration > MAX_RAMP_RATE ||
        !ChassisMath::validIcr(left) || !ChassisMath::validIcr(right) || !motionEditable()) return false;
    motionSettings() = settings;
    return true;
}
Status status() {
    Status result{};
    result.mode = mode;
    result.modeName = mode == Mode::Running ? "running" : mode == Mode::Arming ? "arming" : "locked";
    result.reason = reason;
    result.healthy = healthy(millis());
    result.emergencyLatched = emergencyLatched || SystemSafety::stopped();
    for (uint8_t i = 0; i < 4; ++i) {
        result.wheel[i] = wheel[i];
        result.targetWheel[i] = targetWheel[i];
        result.axes[i] = {axes[i].seen, axes[i].heartbeatAt,
            axes[i].seen ? millis() - axes[i].heartbeatAt : UINT32_MAX,
            axes[i].error, axes[i].state};
    }
    result.directionStage = directionStage();
    result.motion = motionSettings();
    result.processionMode = motionProfile() == 1;
    result.motionEditable = motionEditable();
    return result;
}
} // namespace ChassisController
