#include "rc_controller.h"
#include "chassis_controller.h"
#include "chassis_math.h"
#include "mechanism_controller.h"
#include "system_config.h"
#include "system_safety.h"
#include "pwm_controller.h"
#include <Arduino.h>
#include <AlfredoCRSF.h>
#include <string.h>

namespace RcController {
namespace {
using namespace SystemConfig::Rc;
AlfredoCRSF receiver;
Status snapshot{};
uint32_t lastLinkedAt = 0;
bool seenLink = false;
struct Button {
    bool armed = false, trackingPress = false, trackingRelease = false;
    uint32_t pressSince = 0, releaseSince = 0;
};
Button nextButton, previousButton, clearButton, propButtons[5];
struct StageSwitch {
    bool high = false, candidate = false;
    uint32_t since = 0;
};
StageSwitch stages[GENERAL_SELECTOR_COUNT];
bool stagesInitialized = false, stageLevelsReady = false, stageSelectionPending = false;
bool selectionRejectionReported = false;
uint32_t initializedAt = 0, switchSince = 0;
uint8_t desiredSelection = 0, guardedStageMask = 0;
uint8_t selectorPriority[GENERAL_SELECTOR_COUNT]{}, selectorPriorityCount = 0;
uint8_t switchCandidate = 255, stableSwitch = 255;
// Internal selector identity only: never passed to MechanismController.

uint8_t selectorIdentity(uint8_t index) {
    return index + 1 == GENERAL_SELECTOR_COUNT ? FRUIT_STAGE : STAGE_MAP[index];
}
bool selectorEnabled(uint8_t index) {
    return GENERAL_SELECTOR_CHANNELS[index] != LIGHT_CHANNEL &&
           MechanismConfig::stageEnabled(selectorIdentity(index));
}
enum class Context : uint8_t { Neutral, General, Stage1 };
Context context = Context::Neutral;
bool modeRequestPending = false, exitRequested = false;
bool lightAvailable = false, lightHigh = false, lightCandidate = false;
uint32_t lightSince = 0;

void updateLight(uint32_t now) {
    const int value = snapshot.channels[LIGHT_CHANNEL - 1];
    const bool allowed = context == Context::General && !snapshot.stage1Mode &&
        snapshot.switchPosition != 255 && snapshot.link && snapshot.valid &&
        ChassisMath::channelValid(value);
    if (!allowed) {
        if (lightAvailable || lightHigh) PwmController::setRcStrobe(false);
        lightAvailable = lightHigh = lightCandidate = false;
        snapshot.lightStrobeEnabled = false;
        lightSince = now;
        return;
    }
    if (!lightAvailable) {
        lightAvailable = true;
        lightHigh = false;
        lightCandidate = value > PRESS_US;
        lightSince = now;
        PwmController::setRcStrobe(false);
    }
    bool requested = lightHigh;
    if (value > PRESS_US) requested = true;
    else if (value < RELEASE_US) requested = false;
    if (requested != lightCandidate) { lightCandidate = requested; lightSince = now; }
    if (lightCandidate != lightHigh && uint32_t(now - lightSince) >= DEBOUNCE_MS) {
        lightHigh = lightCandidate;
        PwmController::setRcStrobe(lightHigh);
    }
    snapshot.lightStrobeEnabled = lightHigh;
}

bool press(Button &button, int value, bool allowed, uint32_t now) {
    if (!allowed || !ChassisMath::channelValid(value)) {
        button = Button{};
        return false;
    }
    if (value < RELEASE_US) {
        button.trackingPress = false;
        if (!button.trackingRelease) { button.trackingRelease = true; button.releaseSince = now; }
        if (now - button.releaseSince >= DEBOUNCE_MS) button.armed = true;
    } else if (value > PRESS_US) {
        button.trackingRelease = false;
        if (!button.armed) return false;
        if (!button.trackingPress) { button.trackingPress = true; button.pressSince = now; }
        if (now - button.pressSince >= DEBOUNCE_MS) {
            button.armed = button.trackingPress = false;
            return true;
        }
    } else button.trackingPress = button.trackingRelease = false;
    return false;
}
bool normalReady(const MechanismController::Status &mechanism) {
    return !mechanism.debug && !mechanism.uploading && strcmp(mechanism.phase, "ready") == 0;
}
bool competitionEnabled() {
    if (!snapshot.link || !snapshot.valid || SystemSafety::stopped()) return false;
    if constexpr (SystemConfig::BENCH_TEST_MODE) {
        // Mechanism-only bench tests do not require online, closed-loop ODrives.
        return true;
    } else {
        const auto chassis = ChassisController::status();
        if (chassis.mode != ChassisController::Mode::Running || !chassis.healthy ||
            chassis.emergencyLatched) return false;
        for (const auto &axis : chassis.axes) if (axis.state != 8) return false;
        return true;
    }
}
void resetContextInputs() {
    nextButton = previousButton = Button{};
    for (Button &button : propButtons) button = Button{};
    for (StageSwitch &stage : stages) stage = StageSwitch{};
    stagesInitialized = stageLevelsReady = stageSelectionPending = false;
    selectionRejectionReported = false;
    desiredSelection = guardedStageMask = selectorPriorityCount = 0;
    snapshot.selectedStage = snapshot.selectedProp = 0;
    snapshot.processionMode = false;
    snapshot.stage1TrimUs = 0;
    snapshot.stage4TrimPulses = 0;
    snapshot.stage2TrimPulses = 0;
}
void resetMode() {
    resetContextInputs();
    context = Context::Neutral;
    snapshot.stage1Mode = false;
    snapshot.switchPosition = switchCandidate = stableSwitch = 255;
    modeRequestPending = exitRequested = false;
}
void updateContext(uint32_t now) {
    const int value = snapshot.channels[STAGE1_SWITCH_CHANNEL - 1];
    if (!snapshot.link || !ChassisMath::channelValid(value)) { resetMode(); return; }
    const uint8_t position = value < STAGE1_LOW_US ? 0 : value > STAGE1_HIGH_US ? 2 : 1;
    snapshot.switchPosition = position;
    if (position != switchCandidate) { switchCandidate = position; switchSince = now; }
    if (position != stableSwitch && now - switchSince >= DEBOUNCE_MS) {
        stableSwitch = position;
        const auto mechanism = MechanismController::status();
        if (position == 2 && (context != Context::Stage1 || mechanism.stage != 1)) {
            resetContextInputs();
            context = Context::Stage1;
            snapshot.stage1Mode = true;
            modeRequestPending = true;
            exitRequested = false;
        } else if (position == 0 && (context != Context::General || mechanism.stage == 1)) {
            const bool leaving = context == Context::Stage1 || mechanism.stage == 1 || mechanism.pendingStage == 1;
            resetContextInputs();
            context = Context::General;
            snapshot.stage1Mode = false;
            exitRequested = modeRequestPending = leaving;
            // Shared prop keys held during LOW exit need a stable release before
            // they can select a normal stage; cold-start held selectors are allowed.
            guardedStageMask = leaving ? uint8_t((1U << GENERAL_SELECTOR_COUNT) - 1) : 0;
        }
        // MID deliberately retains the preceding context and sends no action.
    }
    if (modeRequestPending && competitionEnabled()) {
        const auto mechanism = MechanismController::status();
        if (mechanism.debug || mechanism.uploading || !strcmp(mechanism.phase, "booting") ||
            !strcmp(mechanism.phase, "fault")) return;
        const uint8_t prop = mechanism.stage == 1 && mechanism.prop ? mechanism.prop
                                      : SystemConfig::Mechanism::DEFAULT_STAGE1_PROP;
        const bool accepted = exitRequested ? MechanismController::exitStage1(4)
                                            : MechanismController::selectProfile(1, prop);
        if (accepted) {
            snapshot.selectedStage = exitRequested ? 4 : 1;
            snapshot.selectedProp = exitRequested ? 0 : prop;
            modeRequestPending = false;
            nextButton = previousButton = Button{};
        }
    }
}
void removeSelectorPriority(uint8_t index) {
    for (uint8_t i = 0; i < selectorPriorityCount; ++i) {
        if (selectorPriority[i] != index) continue;
        for (uint8_t j = i; j + 1 < selectorPriorityCount; ++j)
            selectorPriority[j] = selectorPriority[j + 1];
        --selectorPriorityCount;
        return;
    }
}
void rememberSelectorPriority(uint8_t index) {
    removeSelectorPriority(index);
    selectorPriority[selectorPriorityCount++] = index;
}
bool eligibleSelector(uint8_t index) {
    return selectorEnabled(index) &&
           stages[index].high && !(guardedStageMask & (1U << index));
}
uint8_t heldStage() {
    // Known press history wins; no fixed channel priority is imposed.
    for (uint8_t i = selectorPriorityCount; i; --i) {
        const uint8_t index = selectorPriority[i - 1];
        if (eligibleSelector(index)) return selectorIdentity(index);
    }
    uint8_t count = 0, index = 0;
    for (uint8_t i = 0; i < GENERAL_SELECTOR_COUNT; ++i)
        if (eligibleSelector(i)) { ++count; index = i; }
    // Multiple switches held at boot have no ordering information. A unique
    // stable level, or a later real LOW-to-HIGH press, resolves the ambiguity.
    if (count == 1) {
        rememberSelectorPriority(index);
        return selectorIdentity(index);
    }
    return 0;
}
void requestHeldStage(uint8_t stage) {
    if (stage == desiredSelection) return;
    desiredSelection = stage;
    stageSelectionPending = stage != 0;
    selectionRejectionReported = false;
    nextButton = previousButton = Button{};
    if (!stage) {
        snapshot.selectedStage = snapshot.selectedProp = 0;
        snapshot.processionMode = false;
    }
}
void deliverStageSelection() {
    if (!stageSelectionPending || modeRequestPending || !competitionEnabled()) return;
    const auto mechanism = MechanismController::status();
    if (mechanism.debug || mechanism.uploading || !strcmp(mechanism.phase, "booting") ||
        !strcmp(mechanism.phase, "fault") ||
        (mechanism.startupPhase && strcmp(mechanism.startupPhase, "ready"))) return;
    if (MechanismController::selectStage(desiredSelection)) {
        snapshot.processionMode = false;
        snapshot.selectedStage = desiredSelection;
        snapshot.selectedProp = 0;
        stageSelectionPending = false;
        selectionRejectionReported = false;
        nextButton = previousButton = Button{};
    } else if (!selectionRejectionReported) {
        // A transient rejection retains the level request. Do not flood the
        // rejection counter or replay an already accepted held selection.
        ++snapshot.rejectedActions;
        selectionRejectionReported = true;
    }
}
void updateStages(uint32_t now) {
    if (context != Context::General) return;
    bool allValid = snapshot.link;
    for (uint8_t i = 0; i < GENERAL_SELECTOR_COUNT; ++i)
        if (selectorEnabled(i))
            allValid = allValid && ChassisMath::channelValid(snapshot.channels[GENERAL_SELECTOR_CHANNELS[i] - 1]);
    if (!allValid) {
        stagesInitialized = stageLevelsReady = false;
        // An invalid selector cannot retain a chassis override. Recover a held CH14
        // through the same debounced request path without inventing a mechanism stage.
        if (snapshot.processionMode) stageSelectionPending = true;
        snapshot.processionMode = false;
        snapshot.selectedStage = 0;
        nextButton = previousButton = Button{};
        return;
    }
    if (!stagesInitialized) {
        stagesInitialized = true;
        initializedAt = now;
        for (uint8_t i = 0; i < GENERAL_SELECTOR_COUNT; ++i) {
            if (!selectorEnabled(i)) {
                stages[i] = StageSwitch{};
                continue;
            }
            const int value = snapshot.channels[GENERAL_SELECTOR_CHANNELS[i] - 1];
            bool high = stages[i].high;
            if (value > PRESS_US) high = true;
            else if (value < RELEASE_US) high = false;
            stages[i] = {high, high, now};
        }
        return;
    }
    bool allStable = true;
    for (uint8_t i = 0; i < GENERAL_SELECTOR_COUNT; ++i) {
        if (!selectorEnabled(i)) continue;
        const int value = snapshot.channels[GENERAL_SELECTOR_CHANNELS[i] - 1];
        bool target = stages[i].high;
        if (value > PRESS_US) target = true;
        else if (value < RELEASE_US) target = false;
        if (target != stages[i].candidate) { stages[i].candidate = target; stages[i].since = now; }
        const bool settled = now - stages[i].since >= DEBOUNCE_MS;
        if (stages[i].candidate != stages[i].high && settled) {
            stages[i].high = stages[i].candidate;
            if (eligibleSelector(i)) rememberSelectorPriority(i);
            else removeSelectorPriority(i);
        }
        if (!stages[i].high && value < RELEASE_US && settled)
            guardedStageMask &= static_cast<uint8_t>(~(1U << i));
        if (stages[i].candidate != stages[i].high || !settled) allStable = false;
    }
    if (!stageLevelsReady) {
        if (!allStable || now - initializedAt < DEBOUNCE_MS) return;
        stageLevelsReady = true;
        for (uint8_t i = 0; i < GENERAL_SELECTOR_COUNT; ++i)
            if (!eligibleSelector(i)) removeSelectorPriority(i);
    }
    requestHeldStage(heldStage());
    if (!stageSelectionPending && desiredSelection && !snapshot.selectedStage)
        snapshot.selectedStage = desiredSelection; // Valid levels recovered; no new request.
    deliverStageSelection();
}
void updateProps(uint32_t now) {
    const auto mechanism = MechanismController::status();
    const bool allowed = context == Context::Stage1 && !modeRequestPending &&
        competitionEnabled() && mechanism.stage == 1 && normalReady(mechanism);
    uint8_t requested = 0, count = 0;
    for (uint8_t i = 0; i < 5; ++i)
        if (press(propButtons[i], snapshot.channels[PROP_CHANNELS[i] - 1], allowed, now)) {
            requested = uint8_t(i + 1); ++count;
        }
    if (count > 1) {
        ++snapshot.rejectedActions;
        for (Button &button : propButtons) button = Button{};
    } else if (requested) {
        if (MechanismController::triggerProp(requested)) {
            snapshot.selectedStage = 1; snapshot.selectedProp = requested;
            nextButton = previousButton = Button{};
        } else ++snapshot.rejectedActions;
    }
}
void updateStage1Trim() {
    snapshot.stage1TrimUs = 0;
    snapshot.stage4TrimPulses = 0;
    snapshot.stage2TrimPulses = 0;
    if (context != Context::Stage1 || !snapshot.stage1Mode || !snapshot.link ||
        !snapshot.valid || SystemSafety::stopped()) return;
    const auto mechanism = MechanismController::status();
    if (mechanism.stage != 1 || !mechanism.stage1TrimAllowed) return;
    int value = snapshot.channels[STAGE1_TRIM_CHANNEL - 1];
    // A bad knob sample disables only trim, never the three chassis joystick axes.
    if (!ChassisMath::channelValid(value)) return;
    if (value < STAGE1_TRIM_MIN_US) value = STAGE1_TRIM_MIN_US;
    if (value > STAGE1_TRIM_MAX_US) value = STAGE1_TRIM_MAX_US;
    const int difference = value - STAGE1_TRIM_CENTER_US;
    const int magnitude = difference < 0 ? -difference : difference;
    if (magnitude <= STAGE1_TRIM_DEADZONE_US) return;
    const int span = difference > 0 ? STAGE1_TRIM_MAX_US - STAGE1_TRIM_CENTER_US
                                   : STAGE1_TRIM_CENTER_US - STAGE1_TRIM_MIN_US;
    const int offset = (magnitude - STAGE1_TRIM_DEADZONE_US) * STAGE1_TRIM_MAX_OFFSET_US /
                       (span - STAGE1_TRIM_DEADZONE_US);
    snapshot.stage1TrimUs = difference < 0 ? -offset : offset;
}
void updateStage2Trim() {
    snapshot.stage2TrimPulses = 0;
    if (context != Context::General || snapshot.stage1Mode || !snapshot.link ||
        !snapshot.valid || SystemSafety::stopped()) return;
    const auto mechanism = MechanismController::status();
    if (mechanism.stage != 2 || !mechanism.stage2TrimAllowed) return;
    int value = snapshot.channels[STAGE2_TRIM_CHANNEL - 1];
    if (!ChassisMath::channelValid(value)) return;
    if (value < STAGE4_TRIM_MIN_US) value = STAGE4_TRIM_MIN_US;
    if (value > STAGE4_TRIM_MAX_US) value = STAGE4_TRIM_MAX_US;
    const int difference = value - STAGE4_TRIM_CENTER_US;
    const int magnitude = difference < 0 ? -difference : difference;
    if (magnitude <= STAGE4_TRIM_DEADZONE_US) return;
    const int span = difference > 0 ? STAGE4_TRIM_MAX_US - STAGE4_TRIM_CENTER_US
                                   : STAGE4_TRIM_CENTER_US - STAGE4_TRIM_MIN_US;
    const int32_t offset = (magnitude - STAGE4_TRIM_DEADZONE_US) * STAGE2_TRIM_MAX_OFFSET_PULSES /
                           (span - STAGE4_TRIM_DEADZONE_US);
    snapshot.stage2TrimPulses = (difference < 0 ? -offset : offset) * STAGE2_TRIM_DIRECTION;
}
void updateStage4Trim() {
    snapshot.stage4TrimPulses = 0;
    if (context != Context::General || snapshot.stage1Mode || !snapshot.link ||
        !snapshot.valid || SystemSafety::stopped()) return;
    const auto mechanism = MechanismController::status();
    if (!MechanismConfig::heightTrimStage(mechanism.stage) || !mechanism.stage4TrimAllowed) return;
    int value = snapshot.channels[STAGE4_TRIM_CHANNEL - 1];
    if (!ChassisMath::channelValid(value)) return;
    if (value < STAGE4_TRIM_MIN_US) value = STAGE4_TRIM_MIN_US;
    if (value > STAGE4_TRIM_MAX_US) value = STAGE4_TRIM_MAX_US;
    const int difference = value - STAGE4_TRIM_CENTER_US;
    const int magnitude = difference < 0 ? -difference : difference;
    if (magnitude <= STAGE4_TRIM_DEADZONE_US) return;
    const int span = difference > 0 ? STAGE4_TRIM_MAX_US - STAGE4_TRIM_CENTER_US
                                   : STAGE4_TRIM_CENTER_US - STAGE4_TRIM_MIN_US;
    const int32_t offset = (magnitude - STAGE4_TRIM_DEADZONE_US) * MechanismConfig::heightTrimMaxOffset(mechanism.stage) /
                           (span - STAGE4_TRIM_DEADZONE_US);
    snapshot.stage4TrimPulses = (difference < 0 ? -offset : offset) * STAGE4_TRIM_DIRECTION;
}
} // namespace
void begin() {
    snapshot = Status{};
    snapshot.ageMs = UINT32_MAX;
    seenLink = false;
    clearButton = Button{};
    lightAvailable = lightHigh = lightCandidate = false;
    lightSince = millis();
    PwmController::setRcStrobe(false); // Whole-robot boot remains dark until a stable CH13 HIGH.
    resetMode();
    initializedAt = switchSince = lastLinkedAt = millis();
    Serial2.begin(CRSF_BAUD);
    receiver.begin(Serial2);
}
void poll() {
    receiver.update();
    const uint32_t now = millis();
    snapshot.link = receiver.isLinkUp();
    for (uint8_t i = 0; i < 16; ++i) snapshot.channels[i] = receiver.getChannel(i + 1);
    snapshot.forward = snapshot.channels[SystemConfig::Chassis::FORWARD.number - 1];
    snapshot.strafe = snapshot.channels[SystemConfig::Chassis::STRAFE.number - 1];
    snapshot.rotate = snapshot.channels[SystemConfig::Chassis::ROTATE.number - 1];
    snapshot.valid = ChassisMath::channelValid(snapshot.forward) &&
        ChassisMath::channelValid(snapshot.strafe) && ChassisMath::channelValid(snapshot.rotate);
    snapshot.neutral = snapshot.valid &&
        ChassisMath::centered(snapshot.forward, SystemConfig::Chassis::FORWARD) &&
        ChassisMath::centered(snapshot.strafe, SystemConfig::Chassis::STRAFE) &&
        ChassisMath::centered(snapshot.rotate, SystemConfig::Chassis::ROTATE);
    if (snapshot.link) { lastLinkedAt = now; seenLink = true; }
    snapshot.ageMs = seenLink ? now - lastLinkedAt : UINT32_MAX;
    if (press(clearButton, snapshot.channels[CLEAR_CHANNEL - 1], snapshot.link, now)) ChassisController::clearErrors();
    updateContext(now);
    updateLight(now);
    updateStages(now);
    updateProps(now);
    const auto mechanism = MechanismController::status();
    const bool selectedContext = context == Context::Stage1 ? mechanism.stage == 1
                                   : context == Context::General && mechanism.stage != 1;
    const bool allowed = competitionEnabled() && snapshot.selectedStage != 0 &&
        !modeRequestPending && !stageSelectionPending && selectedContext && normalReady(mechanism);
    const bool previousPressed = press(previousButton, snapshot.channels[PREV_CHANNEL - 1], allowed, now);
    const bool nextPressed = press(nextButton, snapshot.channels[NEXT_CHANNEL - 1],
        allowed, now);
    if (previousPressed && nextPressed) {
        ++snapshot.rejectedActions; nextButton = previousButton = Button{};
    } else if (previousPressed) {
        if (!MechanismController::previous()) ++snapshot.rejectedActions;
    } else if (nextPressed) {
        if (!MechanismController::next()) ++snapshot.rejectedActions;
    }
    // Context changes and accepted button actions must be visible before eligibility is read.
    updateStage1Trim();
    updateStage2Trim();
    updateStage4Trim();
}
Status status() {
    Status result = snapshot;
    // Do not call MechanismController::status() here: its output status reads this snapshot.
    if (!result.link || !result.valid || !result.stage1Mode || SystemSafety::stopped())
        result.stage1TrimUs = 0;
    if (!result.link || !result.valid || result.stage1Mode || !result.selectedStage ||
        result.switchPosition == 255 || SystemSafety::stopped())
        result.stage4TrimPulses = 0;
    if (!result.link || !result.valid || result.stage1Mode || !result.selectedStage ||
        result.switchPosition == 255 || SystemSafety::stopped())
        result.stage2TrimPulses = 0;
    return result;
}
} // namespace RcController