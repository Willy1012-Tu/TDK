#include "mechanism_controller.h"
#include "can_transport.h"
#include "system_safety.h"
#include "rc_controller.h"
#include <Arduino.h>
#include <Servo.h>
#include <stdio.h>
#include <string.h>

namespace {
namespace Config = SystemConfig::Mechanism;
using namespace MechanismConfig;
// Cooperative deadlines make motion, servo ramps and upload waits nonblocking.
struct Deadline {
  uint32_t since = 0, duration = 0;
  bool armed = false;
  void start(uint32_t ms) {
    since = millis();
    duration = ms;
    armed = true;
  }
  bool expired() const { return !armed || uint32_t(millis() - since) >= duration; }
  uint32_t elapsed() const {
    const uint32_t t = uint32_t(millis() - since);
    return t > duration ? duration : t;
  }
};
enum class Phase : uint8_t {
  BootDelay,
  FeedbackQueue,
  FeedbackWait,
  CurrentQueue,
  CurrentWait,
  EnableQueue,
  EnableWait,
  ZeroQueue,
  ZeroWait,
  ActionQueue,
  Moving,
  Pause,
  Ready,
  Fault
};
bool isStartupSettingQueue(Phase value) {
  return value == Phase::FeedbackQueue || value == Phase::CurrentQueue ||
         value == Phase::EnableQueue || value == Phase::ZeroQueue;
}
bool isStartupSettingWait(Phase value) {
  return value == Phase::FeedbackWait || value == Phase::CurrentWait ||
         value == Phase::EnableWait || value == Phase::ZeroWait;
}
bool isStartupSettingPhase(Phase value) {
  return isStartupSettingQueue(value) || isStartupSettingWait(value);
}
Phase startupQueueFor(uint8_t command) {
  return command == 0x8C ? Phase::FeedbackQueue
         : command == 0x83 ? Phase::CurrentQueue
         : command == 0xF3 ? Phase::EnableQueue : Phase::ZeroQueue;
}
Phase startupWaitFor(uint8_t command) {
  return command == 0x8C ? Phase::FeedbackWait
         : command == 0x83 ? Phase::CurrentWait
         : command == 0xF3 ? Phase::EnableWait : Phase::ZeroWait;
}
enum class Job : uint8_t { Startup, Next, Previous, Run, Home, AutoHome, EntryHome, Debug, DebugGoHome, Stage4Trim, Stage2Trim, LocalHold, SwitchStop, SwitchReturn };
Phase phase = Phase::BootDelay;
Job job = Job::Startup;
Deadline window, retry, pauseTime, uploadWindow, startupAckWindow, startupAckRetry, selectionWindow;
bool startupAckStarted = false, startupComplete = false;
// Preserve an offline warning for one main-loop turn before continuing setup.
bool benchOfflineNotice = false;
uint8_t startupNode = 0, startupCommand = 0;
uint16_t startupAttempts = 0;
uint16_t configuredCurrentMa[Config::MOTOR_COUNT]{};
char noticeBuffer[160]{}, faultBuffer[192]{}, localWarningBuffer[192]{};
const Action *actions = nullptr;
size_t actionCount = 0, actionIndex = 0;
uint16_t reportedBranch = 0;
uint8_t completed = 0, currentStep = 0, activeStage = Config::DEFAULT_STAGE, pendingStage = 0;
uint8_t activeProp = normalizedProp(Config::DEFAULT_STAGE), pendingProp = 0;
// Only CH7 LOW exit requests physical ID 6 HOME. Ordinary selections use FE.
bool returnWithHome = false, exitHomeSent = false;
uint64_t progress = 0, previousTarget = 0;
bool debugMode = false, uploading = false, pendingCommit = false, txAttempted = false;
bool stage1TrimSuspended = true; // HOME holds zero trim until a new first-stage action.
// Shared by Stage4 and Stage6; legacy internal names match retained USB telemetry.
bool stage4TrimSuspended = true; // Explicit NEXT/PREV/RUN arms trim after HOME or context loss.
int32_t stage4TrimBase = 0; // Last untrimmed ID6 target confirmed by the normal plan.
Action stage4TrimAction[1]{};
Deadline stage4TrimRate;
bool stage2TrimSuspended = true;
int32_t stage2TrimBase[2] = {};
Action stage2TrimActions[3]{};
Deadline stage2TrimRate;
uint8_t initMotor = 0, initZero = 0, ack = 0xFF, stopMask = 0, disableMask = 0;
uint16_t stagedStepCount = 0, stagedActionCount = 0;
uint8_t stagedStage = 4, stagedRepeatFrom = 1, stagedProp = 0;
uint32_t planVersion = 0;
constexpr uint8_t PROFILE_COUNT = Config::STAGE1_PROP_COUNT + Config::STAGE_COUNT - 1;
constexpr uint8_t BANKS = PROFILE_COUNT + 1, NO_BANK = 255;
// Ten retained profiles plus one transactional staging bank. A bank is only
// exchanged after validation and the outgoing profile return barrier.
Action runtimeActions[BANKS][MechanismController::MAX_RUNTIME_ACTIONS]{};
Step runtimeSteps[BANKS][254]{};
uint8_t profileBank[PROFILE_COUNT] = {NO_BANK, NO_BANK, NO_BANK, NO_BANK, NO_BANK,
                                      NO_BANK, NO_BANK, NO_BANK, NO_BANK, NO_BANK};
uint8_t uploadBank = PROFILE_COUNT;
uint16_t profileStepCount[PROFILE_COUNT]{};
uint8_t profileRepeatFrom[PROFILE_COUNT] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
static_assert(PROFILE_COUNT == 10, "Five props plus five normal profiles required");
bool actionWritten[MechanismController::MAX_RUNTIME_ACTIONS]{}, stepWritten[254]{};
constexpr Action DEBUG_WAIT[] = {waitAll()};
constexpr uint16_t CYLINDER_BIT = uint16_t(1U << (Config::MOTOR_COUNT + Config::SERVO_COUNT));
Action reverseActions[Config::MOTOR_COUNT + Config::SERVO_COUNT + 2]{};
Action returnActions[Config::MOTOR_COUNT + Config::SERVO_COUNT + 2]{};
struct MotorAxis {
  // Connectivity is independent of active (an in-flight motion).
  bool online = false, offline = false, recoveryRequired = false;
  bool active = false, started = false, done = false, positionKnown = false;
  uint8_t command = 0xFE;
  int32_t position = 0, target = 0;
  Deadline start, finish;
};
MotorAxis motors[Config::MOTOR_COUNT];
struct ServoAxis {
  Servo device;
  int pulse = 1500, from = 1500, target = 1500, output = 1500; // pulse/from/target remain untrimmed.
  uint32_t ms = 0;
  bool active = false;
  Deadline ramp, update;
};
ServoAxis servos[Config::SERVO_COUNT];
struct Snapshot {
  int32_t motor[Config::MOTOR_COUNT];
  int servo[Config::SERVO_COUNT];
  CylinderAction cylinderReference = CylinderAction::Stop;
  bool cylinderKnown = false;
};
Snapshot entry{};
const char *notice = "Waiting for MKS startup";
uint32_t noticeVersion = 1;
void report(const char *text) {
  notice = text;
  ++noticeVersion;
}
int motorIndex(uint32_t node) {
  for (uint8_t i = 0; i < Config::MOTOR_COUNT; ++i)
    if (Config::MOTOR_IDS[i] == node)
      return i;
  return -1;
}
int servoIndex(uint8_t pin) {
  for (uint8_t i = 0; i < Config::SERVO_COUNT; ++i)
    if (Config::SERVO_PINS[i] == pin)
      return i;
  return -1;
}
bool motorsBusy() {
  for (const auto &m : motors)
    if (m.active)
      return true;
  return false;
}
// Bench skipping never invents an FE reference. Explicit 0x91 is permitted
// only for an online, homing-capable axis to establish a missing reference.
bool usableMotor(uint8_t index, bool goHome = false) {
  const auto &motor = motors[index];
  // A local isolation is enforced in competition and bench modes alike.
  if (motor.recoveryRequired)
    return goHome && motor.online && !(stopMask & (1U << index));
  if constexpr (SystemConfig::BENCH_TEST_MODE) {
    const auto &axis = motors[index];
    return axis.online && (axis.positionKnown || goHome);
  } else {
    static_cast<void>(index);
    static_cast<void>(goHome);
    return true;
  }
}
void serviceStop();
void noteOffline(uint8_t index, uint8_t command, const char *detail) {
  auto &axis = motors[index];
  if (command == 0x91 && axis.active) {
    // Enqueued HOME may physically run even without a reply. Stop/disable only
    // this axis, retrying enqueue cooperatively without a global emergency.
    const uint8_t bit = static_cast<uint8_t>(1U << index);
    stopMask |= bit;
    disableMask |= bit;
    txAttempted = false;
    serviceStop();
  }
  axis.online = false;
  axis.offline = true;
  axis.active = axis.started = axis.done = axis.positionKnown = false;
  snprintf(noticeBuffer, sizeof(noticeBuffer),
           "Bench warning: MKS ID %u 0x%02X %s; offline/unreferenced, skipping axis",
           unsigned(Config::MOTOR_IDS[index]), unsigned(command), detail);
  report(noticeBuffer);
  benchOfflineNotice = true;
}
bool servosBusy() {
  for (const auto &s : servos)
    if (s.active)
      return true;
  return false;
}
uint8_t activeKey() { return profileKey(activeStage, activeProp); }
bool runtimePlan() { return profileBank[activeKey()] != NO_BANK; }
uint16_t builtinCount(uint8_t stage, uint8_t prop) {
  switch (profileKey(stage, prop)) {
  case 0: return sizeof(Stage1Basket::STEPS) / sizeof(Step);
  case 1: return sizeof(Stage1Vase::STEPS) / sizeof(Step);
  case 2: return sizeof(Stage1Tube::STEPS) / sizeof(Step);
  case 3: return sizeof(Stage1Chicken::STEPS) / sizeof(Step);
  case 4: return sizeof(Stage1Bird::STEPS) / sizeof(Step);
  case 5: return sizeof(Stage2::STEPS) / sizeof(Step);
  case 6: return sizeof(Stage3::STEPS) / sizeof(Step);
  case 7: return sizeof(Stage4::STEPS) / sizeof(Step);
  case 8: return sizeof(Stage5::STEPS) / sizeof(Step);
  default: return sizeof(Stage6::STEPS) / sizeof(Step);
  }
}
uint16_t stepCount() {
  return runtimePlan() ? profileStepCount[activeKey()] : builtinCount(activeStage, activeProp);
}
uint8_t repeatFrom() {
  if (runtimePlan())
    return profileRepeatFrom[activeKey()];
  switch (activeKey()) {
  case 0: return Stage1Basket::REPEAT_FROM;
  case 1: return Stage1Vase::REPEAT_FROM;
  case 2: return Stage1Tube::REPEAT_FROM;
  case 3: return Stage1Chicken::REPEAT_FROM;
  case 4: return Stage1Bird::REPEAT_FROM;
  case 5: return Stage2::REPEAT_FROM;
  case 6: return Stage3::REPEAT_FROM;
  case 7: return Stage4::REPEAT_FROM;
  case 8: return Stage5::REPEAT_FROM;
  default: return Stage6::REPEAT_FROM;
  }
}
const Step &profileStep(uint8_t index) {
  if (runtimePlan())
    return runtimeSteps[profileBank[activeKey()]][index - 1];
  switch (activeKey()) {
  case 0: return Stage1Basket::STEPS[index - 1];
  case 1: return Stage1Vase::STEPS[index - 1];
  case 2: return Stage1Tube::STEPS[index - 1];
  case 3: return Stage1Chicken::STEPS[index - 1];
  case 4: return Stage1Bird::STEPS[index - 1];
  case 5: return Stage2::STEPS[index - 1];
  case 6: return Stage3::STEPS[index - 1];
  case 7: return Stage4::STEPS[index - 1];
  case 8: return Stage5::STEPS[index - 1];
  default: return Stage6::STEPS[index - 1];
  }
}
uint8_t stepAt(uint64_t n) {
  if (!n)
    return 0;
  const uint16_t total = stepCount();
  if (n <= total)
    return static_cast<uint8_t>(n);
  if (activeStage == 1)
    return static_cast<uint8_t>(total);
  return static_cast<uint8_t>(repeatFrom() + (n - total - 1) % (total - repeatFrom() + 1));
}
void captureEntry() {
  for (uint8_t i = 0; i < Config::MOTOR_COUNT; ++i) {
    const uint8_t node = Config::MOTOR_IDS[i];
    entry.motor[i] = activeStage == 2 && (node == 4 || node == 5) ? stage2TrimBase[node - 4]
                    : heightTrimStage(activeStage) && node == SystemConfig::Rc::STAGE4_TRIM_NODE
                         ? stage4TrimBase : motors[i].position;
  }
  for (uint8_t i = 0; i < Config::SERVO_COUNT; ++i)
    entry.servo[i] = servos[i].pulse;
  const auto cylinderState = CylinderController::status();
  entry.cylinderReference = cylinderState.referenceAction;
  entry.cylinderKnown = cylinderState.referenceKnown;
  completed = currentStep = 0;
  reportedBranch = 0;
  progress = 0;
}
void applyState(Snapshot &s, uint8_t index) {
  const auto &step = profileStep(index);
  for (size_t i = 0; i < step.count; ++i) {
    const auto &a = step.actions[i];
    if (a.kind == ActionKind::Stepper)
      s.motor[motorIndex(a.node)] = boundedMotorPosition(activeStage, a.node, a.position);
    else if (a.kind == ActionKind::Servo)
      s.servo[servoIndex(a.pin)] = a.pulse;
    else if (a.kind == ActionKind::Cylinder && a.cylinderAct != CylinderAction::Stop) {
      s.cylinderReference = a.cylinderAct;
      s.cylinderKnown = true;
    }
  }
}
Snapshot stateAt(uint64_t n) {
  Snapshot result = entry;
  const uint16_t total = stepCount();
  const uint16_t initial = static_cast<uint16_t>(n < total ? n : total);
  for (uint16_t i = 1; i <= initial; ++i)
    applyState(result, static_cast<uint8_t>(i));
  if (n <= total)
    return result;
  const uint16_t first = repeatFrom(), period = total - first + 1;
  const uint64_t after = n - total;
  // Absolute assignments make one complete repeat cycle sufficient, regardless
  // of the number of cycles. This keeps PREV bounded without losing history.
  if (after >= period)
    for (uint16_t i = first; i <= total; ++i)
      applyState(result, static_cast<uint8_t>(i));
  const uint16_t tail = static_cast<uint16_t>(after % period);
  for (uint16_t i = 0; i < tail; ++i)
    applyState(result, static_cast<uint8_t>(first + i));
  return result;
}
bool send(uint8_t node, uint8_t command, const uint8_t *data = nullptr, uint8_t count = 0) {
  if (motorIndex(node) < 0)
    return false;
  uint8_t frame[8] = {};
  frame[0] = command;
  uint32_t sum = node + command;
  for (uint8_t i = 0; i < count; ++i) {
    frame[i + 1] = data[i];
    sum += data[i];
  }
  frame[count + 1] = static_cast<uint8_t>(sum);
  return CanTransport::send(1, node, frame, static_cast<uint8_t>(count + 2));
}
void serviceStop() {
  if (!stopMask && !disableMask)
    return;
  if (txAttempted && !retry.expired())
    return;
  txAttempted = true;
  retry.start(Config::TX_RETRY_MS);
  const uint8_t off = 0;
  for (uint8_t i = 0; i < Config::MOTOR_COUNT; ++i) {
    const uint8_t bit = static_cast<uint8_t>(1U << i);
    if ((stopMask & bit) && send(Config::MOTOR_IDS[i], 0xF7))
      stopMask &= static_cast<uint8_t>(~bit);
    if ((disableMask & bit) && send(Config::MOTOR_IDS[i], 0xF3, &off, 1))
      disableMask &= static_cast<uint8_t>(~bit);
  }
}
// Cancel the current plan without inventing a completed position or relaxing
// SystemSafety. F7 stops accepted motion while retaining the vertical holding torque.
void abortLocalPlan(int failedIndex, const char *reason) {
  CylinderController::stop();
  for (uint8_t i = 0; i < Config::MOTOR_COUNT; ++i) {
    auto &axis = motors[i];
    if (!axis.active && int(i) != failedIndex)
      continue;
    stopMask |= static_cast<uint8_t>(1U << i);
    axis.active = axis.started = axis.done = axis.positionKnown = false;
    axis.recoveryRequired = true;
  }
  for (auto &servoAxis : servos)
    servoAxis.active = false; // Hold the last output; do not complete a release ramp.
  stage2TrimSuspended = stage4TrimSuspended = stage1TrimSuspended = true;
  debugMode = false;
  actions = nullptr;
  actionCount = actionIndex = 0;
  reportedBranch = 0;
  currentStep = completed;
  selectionWindow.armed = false;
  job = Job::LocalHold;
  phase = Phase::Ready;
  snprintf(localWarningBuffer, sizeof(localWarningBuffer), "%s", reason);
  txAttempted = false;
  serviceStop();
  report(localWarningBuffer);
}
void fail(const char *reason, bool global = true) {
  CylinderController::stop();
  if (phase == Phase::Fault)
    return;
  phase = Phase::Fault;
  stage2TrimSuspended = true;
  stage4TrimSuspended = true;
  uploading = pendingCommit = false;
  pendingStage = pendingProp = 0;
  returnWithHome = exitHomeSent = false;
  selectionWindow.armed = false;
  for (auto &m : motors) {
    if (m.active)
      m.positionKnown = false; // A stopped, unfinished move has no confirmed target position.
    m.active = false;
  }
  for (auto &s : servos) {
    s.active = false;
    if (s.device.attached())
      s.device.detach();
  }
  stopMask = disableMask = static_cast<uint8_t>((1U << Config::MOTOR_COUNT) - 1U);
  txAttempted = false;
  serviceStop();
  report(reason);
  if (global)
    SystemSafety::stop(reason);
}
void formatStartup(char *buffer, size_t bytes, const char *detail) {
  if (startupCommand == 0x83)
    snprintf(buffer, bytes, "MKS startup ID %u 0x83 %u mA attempt %u: %s",
             unsigned(startupNode), unsigned(Config::startupCurrentMilliamps(startupNode)),
             unsigned(startupAttempts), detail);
  else
    snprintf(buffer, bytes, "MKS startup ID %u 0x%02X attempt %u: %s",
             unsigned(startupNode), unsigned(startupCommand), unsigned(startupAttempts), detail);
}
void reportStartup(const char *detail) {
  formatStartup(noticeBuffer, sizeof(noticeBuffer), detail);
  report(noticeBuffer);
}
void failStartup(const char *detail) {
  formatStartup(faultBuffer, sizeof(faultBuffer), detail);
  fail(faultBuffer);
}
void failMotion(uint8_t index, uint8_t command, const char *detail) {
  snprintf(faultBuffer, sizeof(faultBuffer), "MKS ID %u 0x%02X: %s",
           unsigned(Config::MOTOR_IDS[index]), unsigned(command), detail);
  const bool fruitAxis = Config::MOTOR_IDS[index] == 6 &&
      (activeStage == SystemConfig::Rc::FRUIT_STAGE || motors[index].recoveryRequired);
  if (startupComplete && fruitAxis && !SystemSafety::stopped())
    abortLocalPlan(index, faultBuffer);
  else
    fail(faultBuffer);
}
void queue(Phase next) {
  phase = next;
  txAttempted = false;
  ack = 0xFF;
  window.start(Config::TX_TIMEOUT_MS);
  if (isStartupSettingQueue(next)) {
    startupNode = next == Phase::ZeroQueue ? Config::STATIONARY_ZERO_IDS[initZero]
                                          : Config::MOTOR_IDS[initMotor];
    startupCommand = next == Phase::FeedbackQueue ? 0x8C
                     : next == Phase::CurrentQueue ? 0x83
                     : next == Phase::EnableQueue ? 0xF3 : 0x92;
    startupAttempts = 0;
    startupAckStarted = false;
    reportStartup("waiting for CAN enqueue");
  }
}
void finishStartupSetup();
void queueStationaryZero() {
  while (initZero < sizeof(Config::STATIONARY_ZERO_IDS)) {
    if constexpr (SystemConfig::BENCH_TEST_MODE) {
      const int index = motorIndex(Config::STATIONARY_ZERO_IDS[initZero]);
      if (index < 0 || !motors[index].online) {
        ++initZero;
        continue;
      }
    }
    queue(Phase::ZeroQueue);
    return;
  }
  finishStartupSetup();
}
void advanceStartupAxis() {
  if (startupCommand == 0x92) {
    ++initZero;
    queueStationaryZero();
  } else if (++initMotor < Config::MOTOR_COUNT)
    queue(Phase::FeedbackQueue);
  else {
    initZero = 0;
    queueStationaryZero();
  }
}
void skipStartupAxis(const char *detail) {
  const uint8_t index = static_cast<uint8_t>(motorIndex(startupNode));
  const uint8_t command = startupCommand;
  // Advance first: queue() would otherwise overwrite the offline warning.
  advanceStartupAxis();
  noteOffline(index, command, detail);
}
bool goHomeFor(const Action &a) {
  return homingMotor(a.node) &&
         (job == Job::Startup || job == Job::Home || job == Job::DebugGoHome ||
          (job == Job::SwitchReturn && returnWithHome && !exitHomeSent &&
           activeStage == 1 && a.node == 6));
}
bool txDue() {
  if (window.expired()) {
    if (isStartupSettingQueue(phase)) {
      if constexpr (SystemConfig::BENCH_TEST_MODE)
        skipStartupAxis("CAN enqueue timeout (500 ms)");
      else
        failStartup("CAN enqueue timeout (500 ms)");
    } else if (actions && actions[actionIndex].kind == ActionKind::Stepper) {
      const uint8_t index = static_cast<uint8_t>(motorIndex(actions[actionIndex].node));
      const uint8_t command = goHomeFor(actions[actionIndex]) ? 0x91 : 0xFE;
      if constexpr (SystemConfig::BENCH_TEST_MODE) {
        if (job == Job::Startup && command == 0x91) {
          // HOME never entered the CAN queue, so no physical motion needs
          // stopping. Preserve the warning; next poll skips this action.
          noteOffline(index, command, "CAN enqueue timeout (500 ms)");
          return false;
        }
      }
      failMotion(index, command, "CAN enqueue timeout (500 ms)");
    } else
      fail("MKS CAN transmit timeout");
    return false;
  }
  if (txAttempted && !retry.expired())
    return false;
  txAttempted = true;
  retry.start(Config::TX_RETRY_MS);
  return true;
}
void updateStage2Base(uint8_t i, int32_t position) {
  const uint8_t node = Config::MOTOR_IDS[i];
  if (activeStage == 2 && (node == 4 || node == 5) && job != Job::Stage2Trim)
    stage2TrimBase[node - 4] = position;
}
bool startMotor(uint8_t i, const Action &a, bool goHome) {
  auto &m = motors[i];
  if (m.active || (stopMask & (1U << i)) || (disableMask & (1U << i)))
    return false;
  const int32_t target = boundedMotorPosition(activeStage, Config::MOTOR_IDS[i], a.position);
  if (!usableMotor(i, goHome)) {
    if (goHome && returnWithHome && Config::MOTOR_IDS[i] == 6)
      exitHomeSent = true;
    return true;
  }
  if (!goHome && m.positionKnown && m.position == target) {
    updateStage2Base(i, target);
    if (heightTrimStage(activeStage) && Config::MOTOR_IDS[i] == SystemConfig::Rc::STAGE4_TRIM_NODE &&
        job != Job::Stage4Trim)
      stage4TrimBase = target;
    return true;
  }
  const uint8_t node = Config::MOTOR_IDS[i];
  if (goHome && !homingMotor(node))
    return false;
  if (goHome) {
    if (!send(node, 0x91))
      return false;
  } else {
    const uint32_t raw = static_cast<uint32_t>(target) & 0xFFFFFFU;
    const uint8_t data[6] = {static_cast<uint8_t>(a.rpm >> 8),
                             static_cast<uint8_t>(a.rpm),
                             a.acc,
                             static_cast<uint8_t>(raw >> 16),
                             static_cast<uint8_t>(raw >> 8),
                             static_cast<uint8_t>(raw)};
    if (!send(node, 0xFE, data, 6))
      return false;
  }
  if (goHome && returnWithHome && node == 6 &&
      (job == Job::Home || job == Job::SwitchReturn))
    exitHomeSent = true;
  m.active = true;
  m.started = m.done = false;
  m.command = goHome ? 0x91 : 0xFE;
  m.target = goHome ? 0 : target;
  m.start.start(SystemConfig::BENCH_TEST_MODE && job == Job::Startup && goHome
                    ? Config::STARTUP_ACK_TIMEOUT_MS : Config::START_TIMEOUT_MS);
  m.finish.start(startupComplete && node == 6 &&
                         (activeStage == SystemConfig::Rc::FRUIT_STAGE || m.recoveryRequired)
                     ? SystemConfig::Fruit::FINISH_TIMEOUT_MS : Config::FINISH_TIMEOUT_MS);
  return true;
}
bool stage2TrimContext() {
  const auto rc = RcController::status();
  const int knob = rc.channels[SystemConfig::Rc::STAGE2_TRIM_CHANNEL - 1];
  if (!startupComplete || phase == Phase::Fault || SystemSafety::stopped() || activeStage != 2 ||
      pendingStage || returnWithHome || debugMode || uploading || pendingCommit ||
      !rc.link || !rc.valid || rc.stage1Mode || !rc.selectedStage || rc.switchPosition == 255 ||
      knob < SystemConfig::Rc::CHANNEL_MIN_US || knob > SystemConfig::Rc::CHANNEL_MAX_US)
    return false;
  for (uint8_t node = 4; node <= 5; ++node) {
    const int i = motorIndex(node);
    if (i < 0 || !motors[i].online || !motors[i].positionKnown || motors[i].recoveryRequired)
      return false;
  }
  return true;
}
bool stage2TrimAllowed() { return !stage2TrimSuspended && stage2TrimContext(); }
bool stage4TrimContext() {
  const auto rc = RcController::status();
  const int index = motorIndex(SystemConfig::Rc::STAGE4_TRIM_NODE);
  const int knob = rc.channels[SystemConfig::Rc::STAGE4_TRIM_CHANNEL - 1];
  return startupComplete && phase != Phase::Fault && !SystemSafety::stopped() &&
         heightTrimStage(activeStage) && !pendingStage && !returnWithHome && !debugMode &&
         !uploading && !pendingCommit && rc.link && rc.valid && !rc.stage1Mode &&
         rc.selectedStage != 0 && rc.switchPosition != 255 &&
         knob >= SystemConfig::Rc::CHANNEL_MIN_US && knob <= SystemConfig::Rc::CHANNEL_MAX_US &&
         index >= 0 && motors[index].online && motors[index].positionKnown && !motors[index].recoveryRequired;
}
bool stage4TrimAllowed() {
  // Eligibility is retained through both STEP and trim moves; only scheduling
  // requires Ready, so a busy trim never feeds a false zero back to the RC module.
  return !stage4TrimSuspended && stage4TrimContext();
}
bool trimAllowed() {
  const auto rc = RcController::status();
  return startupComplete && phase != Phase::Fault && !SystemSafety::stopped() &&
         activeStage == 1 && !pendingStage && !returnWithHome && !stage1TrimSuspended &&
         rc.stage1Mode && rc.link && rc.valid;
}
void writeServoOutput(uint8_t i, bool force = false) {
  auto &s = servos[i];
  if (phase == Phase::Fault || SystemSafety::stopped() || !s.device.attached()) return;
  int pulse = s.pulse;
  if (Config::SERVO_PINS[i] == SystemConfig::Rc::STAGE1_TRIM_PIN && trimAllowed()) {
    const int requested = RcController::status().stage1TrimUs;
    const int maximum = SystemConfig::Rc::STAGE1_TRIM_MAX_OFFSET_US;
    const int offset = requested < -maximum ? -maximum : requested > maximum ? maximum : requested;
    pulse += offset;
    if (pulse < SystemConfig::Rc::STAGE1_TRIM_OUTPUT_MIN_US) pulse = SystemConfig::Rc::STAGE1_TRIM_OUTPUT_MIN_US;
    if (pulse > SystemConfig::Rc::STAGE1_TRIM_OUTPUT_MAX_US) pulse = SystemConfig::Rc::STAGE1_TRIM_OUTPUT_MAX_US;
  }
  if (pulse < Config::SERVO_MIN_US) pulse = Config::SERVO_MIN_US;
  if (pulse > Config::SERVO_MAX_US) pulse = Config::SERVO_MAX_US;
  if (force || pulse != s.output) {
    s.device.writeMicroseconds(pulse);
    s.output = pulse;
  }
}
void refreshTrimOutput() {
  const int index = servoIndex(SystemConfig::Rc::STAGE1_TRIM_PIN);
  if (index >= 0) writeServoOutput(static_cast<uint8_t>(index));
}
void servoMove(uint8_t i, int pulse, uint32_t ms) {
  auto &s = servos[i];
  if (!ms || pulse == s.pulse) {
    s.active = false;
    s.pulse = pulse;
    writeServoOutput(i, true);
    return;
  }
  s.from = s.pulse;
  s.target = pulse;
  s.ms = ms;
  s.active = true;
  s.ramp.start(ms);
  s.update.start(Config::SERVO_UPDATE_MS);
}
void updateServos() {
  for (uint8_t i = 0; i < Config::SERVO_COUNT; ++i) {
    auto &s = servos[i];
    bool updated = false;
    if (s.active && (s.ramp.expired() || s.update.expired())) {
      const uint32_t elapsed = s.ramp.elapsed();
      const bool done = s.ramp.expired();
      s.pulse =
          s.from + static_cast<int>((int32_t(s.target - s.from) * int32_t(elapsed)) / int32_t(s.ms));
      if (done) s.active = false;
      s.update.start(Config::SERVO_UPDATE_MS);
      updated = true;
    }
    // Refresh the trim target even while holding or between ramp update ticks.
    if (updated || Config::SERVO_PINS[i] == SystemConfig::Rc::STAGE1_TRIM_PIN)
      writeServoOutput(i, updated);
  }
}
void startPlan(const Action *plan, size_t count) {
  actions = plan;
  actionCount = count;
  actionIndex = 0;
  reportedBranch = 0;
  queue(Phase::ActionQueue);
}
void serviceStage4Trim() {
  if (!stage4TrimContext()) {
    stage4TrimSuspended = true;
    return; // Never create a compensating move on signal loss or invalid input.
  }
  if (!stage4TrimAllowed() || phase != Phase::Ready || motorsBusy() || servosBusy() ||
      CylinderController::status().active)
    return;
  const int index = motorIndex(SystemConfig::Rc::STAGE4_TRIM_NODE);
  const int32_t maximum = heightTrimMaxOffset(activeStage);
  const int32_t requested = RcController::status().stage4TrimPulses;
  const int32_t offset = requested < -maximum ? -maximum : requested > maximum ? maximum : requested;
  const int64_t raw = int64_t(stage4TrimBase) + offset;
  const int32_t target = boundedMotorPosition(activeStage, SystemConfig::Rc::STAGE4_TRIM_NODE, raw);
  if (motors[index].position == target || (stage4TrimRate.armed && !stage4TrimRate.expired()))
    return;
  stage4TrimAction[0] = stepperFor(SystemConfig::Rc::STAGE4_TRIM_NODE, target,
                                  SystemConfig::Rc::STAGE4_TRIM_RPM, SystemConfig::Rc::STAGE4_TRIM_ACC);
  job = Job::Stage4Trim;
  startPlan(stage4TrimAction, 1);
  stage4TrimRate.start(SystemConfig::Rc::STAGE4_TRIM_UPDATE_MS);
  report("ID6 CH4 trim: adjusting height");
}
void serviceStage2Trim() {
  if (!stage2TrimContext()) {
    stage2TrimSuspended = true;
    return;
  }
  if (!stage2TrimAllowed() || phase != Phase::Ready || motorsBusy() || servosBusy() ||
      CylinderController::status().active)
    return;
  const int left = motorIndex(4), right = motorIndex(5);
  const int32_t maximum = SystemConfig::Rc::STAGE2_TRIM_MAX_OFFSET_PULSES;
  const int32_t requested = RcController::status().stage2TrimPulses;
  int64_t offset = requested < -maximum ? -maximum : requested > maximum ? maximum : requested;
  // Clip one common displacement, preserving opposite, equal travel at both boundaries.
  const int64_t minimum = -int64_t(stage2TrimBase[0]) > int64_t(stage2TrimBase[1])
                              ? -int64_t(stage2TrimBase[0]) : int64_t(stage2TrimBase[1]);
  const int64_t maximumTravel = 8388607LL - stage2TrimBase[0] < int64_t(stage2TrimBase[1]) + 8388608LL
                                   ? 8388607LL - stage2TrimBase[0] : int64_t(stage2TrimBase[1]) + 8388608LL;
  if (offset < minimum) offset = minimum;
  if (offset > maximumTravel) offset = maximumTravel;
  const int32_t target4 = boundedMotorPosition(2, 4, int64_t(stage2TrimBase[0]) + offset);
  const int32_t target5 = boundedMotorPosition(2, 5, int64_t(stage2TrimBase[1]) - offset);
  if ((motors[left].position == target4 && motors[right].position == target5) ||
      (stage2TrimRate.armed && !stage2TrimRate.expired()))
    return;
  stage2TrimActions[0] = stepperFor(4, target4, SystemConfig::Rc::STAGE2_TRIM_RPM,
                                  SystemConfig::Rc::STAGE2_TRIM_ACC, false);
  stage2TrimActions[1] = stepperFor(5, target5, SystemConfig::Rc::STAGE2_TRIM_RPM,
                                  SystemConfig::Rc::STAGE2_TRIM_ACC, false);
  stage2TrimActions[2] = waitAll();
  job = Job::Stage2Trim;
  startPlan(stage2TrimActions, 3);
  stage2TrimRate.start(SystemConfig::Rc::STAGE2_TRIM_UPDATE_MS);
  report("Stage2 CH4 trim: adjusting ID4/5 together");
}
void startStep(uint8_t index) {
  currentStep = index;
  const auto &s = profileStep(index);
  startPlan(s.actions, s.count);
}
uint16_t actuatorMask(const Action *plan, size_t count) {
  uint16_t mask = 0;
  for (size_t i = 0; i < count; ++i) {
    const auto &a = plan[i];
    if (a.kind == ActionKind::Stepper)
      mask |= static_cast<uint16_t>(1U << motorIndex(a.node));
    else if (a.kind == ActionKind::Servo)
      mask |= static_cast<uint16_t>(1U << (Config::MOTOR_COUNT + servoIndex(a.pin)));
    else if (a.kind == ActionKind::Cylinder)
      mask |= CYLINDER_BIT;
  }
  return mask;
}
uint16_t profileActuatorMask() {
  // Stage2 borrows the paired lift and Pin3 even when its editable YAML omits them.
  uint16_t mask = activeStage == 2 ? actuatorMask(HOME2, sizeof(HOME2) / sizeof(Action)) : 0;
  for (uint16_t i = 1; i <= stepCount(); ++i) {
    const auto &s = profileStep(static_cast<uint8_t>(i));
    mask |= actuatorMask(s.actions, s.count);
  }
  return mask;
}
bool profileNeedsRecovery() {
  const uint16_t mask = profileActuatorMask();
  for (uint8_t i = 0; i < Config::MOTOR_COUNT; ++i)
    if (motors[i].recoveryRequired && (mask & (1U << i)))
      return true;
  return false;
}
bool selectionJob() {
  return job == Job::SwitchStop || job == Job::SwitchReturn || job == Job::EntryHome;
}
void startSelectionWindow() {
  if (!selectionWindow.armed)
    selectionWindow.start(Config::STAGE_RETURN_TIMEOUT_MS);
}
bool cylinderRetracted() {
  const auto state = CylinderController::status();
  // This is a completed timed-travel estimate, not feedback from a limit switch.
  return !state.active && !state.in1High && !state.in2High && state.fullyRetracted;
}
// Startup establishes the cylinder retract estimate. Stage2 only repeats the
// full retract when subsequent motion or interruption invalidated that estimate.
void activateSelection() {
  stage1TrimSuspended = activeStage != 1;
  stage2TrimSuspended = true;
  stage4TrimSuspended = true;
  if (heightTrimStage(activeStage))
    stage4TrimBase = motors[motorIndex(SystemConfig::Rc::STAGE4_TRIM_NODE)].position;
  if (activeStage == 2) {
    stage2TrimBase[0] = motors[motorIndex(4)].position;
    stage2TrimBase[1] = motors[motorIndex(5)].position;
    stage2TrimSuspended = false; // Entry STEP 0 is immediately eligible after its servo plan.
  }
  captureEntry();
  returnWithHome = exitHomeSent = false;
  if (activeStage == 2) {
    startSelectionWindow();
    job = Job::EntryHome;
    startPlan(ENTRY2, sizeof(ENTRY2) / sizeof(Action));
    report(cylinderRetracted()
               ? "Stage 2 entry: cylinder already retracted; restoring servos without retract wait"
               : "Stage 2 entry: retracting unreferenced cylinder and restoring servos before STEP 0");
  } else {
    selectionWindow.armed = false;
    phase = Phase::Ready;
    report("Stage selected: ready at STEP 0; no entry HOME");
  }
}
void startSwitchStop() {
  stage2TrimSuspended = true;
  stage4TrimSuspended = true;
  stage1TrimSuspended = true;
  refreshTrimOutput();
  startSelectionWindow();
  CylinderController::stop();
  // An interrupted STEP is not counted as complete. Discard unissued actions,
  // while already accepted MKS moves and servo ramps retain their deadlines.
  captureEntry();
  job = Job::SwitchStop;
  startPlan(DEBUG_WAIT, 1);
  report("Selection accepted: cylinder stopped; waiting for issued axes before return");
}
void startReturn() {
  stage2TrimSuspended = true;
  stage4TrimSuspended = true;
  stage1TrimSuspended = true;
  refreshTrimOutput();
  startSelectionWindow();
  const Step standard = activeStage == 6 ? step(HOME6)
                        : activeStage == 5 ? step(HOME5)
                        : activeStage == 3 ? step(PARK3)
                        : activeStage == 2 ? step(HOME2) : step(HOME14);
  // Include custom RAM/YAML actuators as well as the standard stage group.
  // Ordinary outgoing axes return with FE; Stage 1 LOW additionally homes ID 6.
  // Stage 5 has no standard hardware group and returns only its profile axes.
  // PARK3 uses 0/0 and waits for both motors before activating pendingStage.
  const uint16_t mask = profileActuatorMask() | actuatorMask(standard.actions, standard.count);
  size_t count = 0;
  for (uint8_t i = 0; i < Config::MOTOR_COUNT; ++i) {
    if (!(mask & (1U << i)))
      continue;
    const bool requestedHome = returnWithHome && activeStage == 1 && Config::MOTOR_IDS[i] == 6;
    if (motors[i].recoveryRequired) {
      if (requestedHome)
        exitHomeSent = true; // A context exit never implicitly re-homes an isolated axis.
      continue;
    }
    if (!usableMotor(i, requestedHome)) {
      if (requestedHome)
        exitHomeSent = true; // An unavailable axis must not requeue LOW-exit HOME forever.
      continue;
    }
    Action movement = stepperFor(Config::MOTOR_IDS[i], 0, 320, 0, false);
    for (size_t j = 0; j < standard.count; ++j)
      if (standard.actions[j].kind == ActionKind::Stepper &&
          standard.actions[j].node == Config::MOTOR_IDS[i])
        movement = standard.actions[j];
    returnActions[count++] = movement;
  }
  for (uint8_t i = 0; i < Config::SERVO_COUNT; ++i)
    if (mask & (1U << (Config::MOTOR_COUNT + i)))
      returnActions[count++] = servo(Config::SERVO_PINS[i], Config::SERVO_START_US[i], 0, false);
  if ((mask & CYLINDER_BIT) && !cylinderRetracted())
    returnActions[count++] = cylinder(CylinderAction::Retract, SystemConfig::Cylinder::HOME_RETRACT_MS, false);
  returnActions[count++] = waitAll();
  captureEntry();
  job = Job::SwitchReturn;
  startPlan(returnActions, count);
  report(returnWithHome ? "Exiting Stage 1: homing ID 6 before normal selection"
                        : "Returning current stage to initial state before selection");
}
void installRuntime() {
  const uint8_t slot = profileKey(stagedStage, stagedProp), old = profileBank[slot];
  profileBank[slot] = uploadBank;
  profileStepCount[slot] = stagedStepCount;
  profileRepeatFrom[slot] = stagedRepeatFrom;
  if (old != NO_BANK)
    uploadBank = old;
  else {
    for (uint8_t bank = 0; bank < BANKS; ++bank) {
      bool used = false;
      for (uint8_t mapped : profileBank)
        if (mapped == bank)
          used = true;
      if (!used) {
        uploadBank = bank;
        break;
      }
    }
  }
  activeStage = stagedStage;
  activeProp = stagedProp;
  ++planVersion;
  pendingCommit = false;
}
void finishAction() {
  if (++actionIndex < actionCount) {
    queue(Phase::ActionQueue);
    return;
  }
  if (job == Job::SwitchStop) {
    startReturn();
    return;
  }
  if (job == Job::SwitchReturn) {
    // LOW may arrive after an ordinary FE return was already accepted. Finish
    // that barrier, then issue the required HOME exactly once before switching.
    if (returnWithHome && !exitHomeSent) {
      startReturn();
      return;
    }
    if (pendingCommit)
      installRuntime();
    else {
      activeStage = pendingStage;
      activeProp = pendingProp;
    }
    pendingStage = pendingProp = 0;
    activateSelection();
    return;
  }
  if (job == Job::Stage4Trim || job == Job::Stage2Trim) {
    if (pendingStage) {
      startReturn();
      return;
    }
    phase = Phase::Ready;
    report(job == Job::Stage2Trim ? "Stage2 paired CH4 trim completed" : "ID6 CH4 trim completed");
    return; // A knob move is never a STEP or a PREV history entry.
  }
  if (job == Job::Debug || job == Job::DebugGoHome) {
    phase = Phase::Ready;
    report("Debug move completed");
    return;
  }
  if (job == Job::Home || job == Job::AutoHome || job == Job::EntryHome || job == Job::Startup)
    captureEntry();
  else if (job == Job::Previous) {
    progress = previousTarget;
    completed = currentStep = stepAt(progress);
  } else {
    ++progress;
    completed = currentStep;
  }
  if (pendingStage) {
    startReturn();
    return;
  }
  if (activeStage == SystemConfig::Rc::FRUIT_STAGE && (job == Job::Next || job == Job::Run) &&
      completed == stepCount()) {
    // The final fruit STEP must return to zero and stay there, even when CH4
    // remains off-centre. Only the next explicit cycle may re-arm height trim.
    stage2TrimSuspended = true;
    stage4TrimSuspended = true;
    job = Job::AutoHome;
    currentStep = 0;
    startPlan(HOME6, sizeof(HOME6) / sizeof(Action));
    report("Fruit complete: returning to zero and standby");
    return;
  }
  if (job == Job::Run && completed < stepCount()) {
    startStep(completed + 1);
    return;
  }
  phase = Phase::Ready;
  selectionWindow.armed = false;
  if (job == Job::Startup)
    startupComplete = true;
  report(job == Job::Startup    ? (SystemConfig::BENCH_TEST_MODE
                                     ? "Bench startup and cylinder retract completed; unavailable axes remain unreferenced"
                                     : "MKS startup and cylinder retract completed")
         : (job == Job::Home || job == Job::AutoHome || job == Job::EntryHome) ? "Home completed"
         : job == Job::Previous ? "Previous state restored"
         : job == Job::Run      ? "Run completed"
                                : "Step completed");
}
void startAction() {
  if (stopMask || disableMask)
    return; // Keep retrying F7 before any new motion, including a recovery HOME.
  if (job == Job::Stage4Trim && !stage4TrimAllowed()) {
    if (pendingStage)
      startReturn();
    else {
      phase = Phase::Ready;
      report("ID6 CH4 trim cancelled");
    }
    return; // Cancel an unissued trim when its input/context has become invalid.
  }
  // Validate the entire pair before its first send. Once either axis is issued,
  // complete the pair and its barrier even if RC context changes in between.
  if (job == Job::Stage2Trim && actionIndex == 0 && !stage2TrimAllowed()) {
    if (pendingStage)
      startReturn();
    else {
      phase = Phase::Ready;
      report("Stage2 paired trim cancelled before motion");
    }
    return;
  }
  const auto &a = actions[actionIndex];
  if (!valid(a)) {
    fail("Invalid mechanism action");
    return;
  }
  if (a.substep && a.substep != reportedBranch)
    reportedBranch = a.substep;
  if (a.kind == ActionKind::Delay) {
    if (!a.ms) {
      finishAction();
      return;
    }
    pauseTime.start(a.ms);
    phase = Phase::Pause;
    return;
  }
  if (a.kind == ActionKind::Stepper) {
    const uint8_t i = static_cast<uint8_t>(motorIndex(a.node));
    const bool goHome = goHomeFor(a);
    if (!usableMotor(i, goHome)) {
      if (goHome && returnWithHome && a.node == 6)
        exitHomeSent = true;
      // Skip before txDue(): disconnected/unknown axes create no CAN traffic
      // and no start/completion wait, including custom RAM plans.
      finishAction();
      return;
    }
    if (motors[i].active) {
      fail("Overlapping actions on one stepper");
      return;
    }
    if (!txDue())
      return;
    if (!startMotor(i, a, goHomeFor(a))) {
      if (!a.wait && motorsBusy())
        failMotion(i, goHomeFor(a) ? 0x91 : 0xFE,
                   "partial parallel CAN send failed");
      return;
    }
  } else if (a.kind == ActionKind::Servo)
    servoMove(static_cast<uint8_t>(servoIndex(a.pin)), a.pulse, a.ms);
  else if (a.kind == ActionKind::Cylinder) {
    if ((job == Job::SwitchReturn || job == Job::EntryHome) &&
        a.cylinderAct == CylinderAction::Retract && cylinderRetracted()) {
      finishAction(); // Automatic selection does not repeat an already completed safe retract.
      return;
    }
    if (CylinderController::status().active || !CylinderController::start(a.cylinderAct, a.ms)) {
      fail("Overlapping or invalid cylinder action");
      return;
    }
  }
  if (a.wait)
    phase = Phase::Moving;
  else
    finishAction();
}
void finishStartupSetup() {
  if (Config::STARTUP_RETURN_ZERO) {
    currentStep = 0;
    startupNode = 0; // Group homing; positionKnownMask identifies each completed axis.
    startupCommand = 0x91;
    startupAttempts = 1;
    startPlan(STARTUP_HOME, sizeof(STARTUP_HOME) / sizeof(Action));
    report(SystemConfig::BENCH_TEST_MODE
               ? "Bench startup: referencing online MKS axes; waiting for startup cylinder retract"
               : "ID 4/5 zeroed in place; homing ID 6 with 0x91; waiting for startup cylinder retract");
  } else {
    // Disabling MKS startup HOME must not bypass the independent cylinder barrier.
    job = Job::Startup;
    currentStep = 0;
    startPlan(DEBUG_WAIT, 1);
    report(SystemConfig::BENCH_TEST_MODE
               ? "Bench startup: MKS homing disabled; waiting for startup cylinder retract"
               : "ID 4/5 zeroed in place; MKS homing disabled; waiting for startup cylinder retract");
  }
}
void serviceState() {
  if (phase == Phase::BootDelay) {
    if (!window.expired())
      return;
    queue(Phase::FeedbackQueue);
    report("Configuring MKS feedback, startup working current and enable");
  }
  const bool setupQueue = isStartupSettingQueue(phase);
  const bool setupWait = isStartupSettingWait(phase);
  if (setupQueue || setupWait) {
    // One deadline per setting begins at its first successful enqueue and is
    // never extended by retransmission, CAN congestion or a delayed reply.
    if (ack == 0) {
      failStartup("setting rejected (ACK 0)");
      return;
    }
    if (startupAckStarted && startupAckWindow.expired()) {
      if constexpr (SystemConfig::BENCH_TEST_MODE)
        skipStartupAxis("ACK timeout (10000 ms total)");
      else
        failStartup("ACK timeout (10000 ms total)");
      return;
    }
    if (ack == 1) {
      reportStartup("ACK accepted");
      if (startupCommand == 0x8C) {
        motors[motorIndex(startupNode)].online = true;
        queue(Config::startupCurrentMilliamps(startupNode) ? Phase::CurrentQueue
                                                         : Phase::EnableQueue);
      } else if (startupCommand == 0x83) {
        // CAN manual 5.2.3: DLC4 [83, mA high, mA low, checksum].
        // Only ACK 1 confirms this saved working-current setting. Do not enable
        // or home the axis while 0x83 is pending or rejected.
        configuredCurrentMa[motorIndex(startupNode)] = Config::startupCurrentMilliamps(startupNode);
        queue(Phase::EnableQueue);
      } else if (startupCommand == 0x92) {
        // Makerbase CAN manual V1.0.6, p30: 0x92 sets the current driver axis
        // to zero without running the motor. Only a successful ACK establishes
        // the matching logical FE zero; no physical homing is sent to ID 4/5.
        auto &axis = motors[motorIndex(startupNode)];
        axis.position = axis.target = 0;
        axis.positionKnown = true;
        advanceStartupAxis();
      } else
        advanceStartupAxis();
      return;
    }
    if (setupWait) {
      if (!startupAckRetry.expired())
        return;
      // Preserve the outstanding ACK/deadline while retrying CAN enqueue. A
      // late valid ACK is accepted in this queue phase, before another send.
      phase = startupQueueFor(startupCommand);
      txAttempted = false;
      window.start(Config::TX_TIMEOUT_MS);
      reportStartup("retry waiting for CAN enqueue");
    }
    if (!txDue())
      return;
    uint8_t data[2] = {1, 1};
    const uint8_t bytes = (startupCommand == 0x8C || startupCommand == 0x83) ? 2
                         : startupCommand == 0xF3 ? 1 : 0;
    if (startupCommand == 0x83) {
      const uint16_t milliamps = Config::startupCurrentMilliamps(startupNode);
      data[0] = static_cast<uint8_t>(milliamps >> 8);
      data[1] = static_cast<uint8_t>(milliamps);
    }
    if (!send(startupNode, startupCommand, data, bytes))
      return;
    ++startupAttempts;
    if (!startupAckStarted) {
      startupAckStarted = true;
      startupAckWindow.start(Config::STARTUP_ACK_TIMEOUT_MS);
    }
    startupAckRetry.start(Config::STARTUP_ACK_RETRY_MS);
    phase = startupWaitFor(startupCommand);
    reportStartup("waiting for ACK");
  }
  if (phase == Phase::ActionQueue)
    startAction();
  if (phase == Phase::Moving) {
    const auto &a = actions[actionIndex];
    const bool done = a.kind == ActionKind::Stepper ? !motors[motorIndex(a.node)].active
                      : a.kind == ActionKind::Servo ? !servos[servoIndex(a.pin)].active
                      : a.kind == ActionKind::Cylinder ? !CylinderController::status().active
                                                    : !motorsBusy() && !servosBusy() && !CylinderController::status().active;
    if (done)
      finishAction();
  }
  if (phase == Phase::Pause && pauseTime.expired())
    finishAction();
}
bool available() {
  return phase == Phase::Ready && !uploading && !pendingCommit && !stopMask && !disableMask && !SystemSafety::stopped();
}
bool remainingCylinderActions() {
  if (phase == Phase::Ready || !actions)
    return false;
  for (size_t i = actionIndex; i < actionCount; ++i)
    if (actions[i].kind == ActionKind::Cylinder)
      return true;
  return false;
}
void serviceSelectionRequest() {
  // Fruit selections may abort the outgoing move immediately. Unknown axes are
  // isolated and excluded from the return plan; late FE ACKs are discarded.
  if (activeStage == SystemConfig::Rc::FRUIT_STAGE && phase != Phase::Ready && !selectionJob())
    abortLocalPlan(-1, "Fruit action cancelled for stage selection; interrupted axes require manual HOME");
  if (phase != Phase::Ready && selectionJob()) {
    // The outgoing safe posture is independent of the destination. Retarget
    // only the selection; retain already issued MKS moves and cylinder clocks.
    report("Selection target updated; current safe return continues");
    return;
  }
  const bool interruptCylinder = CylinderController::status().active || remainingCylinderActions();
  CylinderController::stop(); // Every accepted different-profile request is immediately LOW/LOW.
  if (interruptCylinder)
    startSwitchStop();
  else if (available())
    startReturn();
  else
    report("Selection pending current action completion");
}
bool uploadActive() {
  if (uploading && uploadWindow.expired())
    uploading = false;
  return uploading;
}
} // namespace

namespace MechanismController {
void begin() {
  stage2TrimSuspended = true;
  stage4TrimSuspended = true;
  stage1TrimSuspended = true;
  CylinderController::begin();
  for (uint8_t i = 0; i < Config::SERVO_COUNT; ++i) {
    servos[i].device.attach(Config::SERVO_PINS[i], Config::SERVO_MIN_US, Config::SERVO_MAX_US);
    if (!servos[i].device.attached()) {
      fail("Servo attachment failed");
      return;
    }
    servos[i].pulse = Config::SERVO_START_US[i];
    writeServoOutput(i, true);
  }
  // The cylinder's own deadtime/timer runs beside MKS startup; no blocking wait.
  if (!CylinderController::start(CylinderAction::Retract, SystemConfig::Cylinder::HOME_RETRACT_MS)) {
    fail("Startup cylinder retract rejected");
    return;
  }
  window.start(Config::STARTUP_DELAY_SECONDS * 1000U);
}
void receive(uint8_t bus, const CAN_message_t &m) {
  const int i = motorIndex(m.id);
  if (bus != 1 || i < 0 || m.flags.extended || m.flags.remote || m.len != 3 ||
      phase == Phase::Fault)
    return;
  if (m.buf[2] != static_cast<uint8_t>(m.id + m.buf[0] + m.buf[1]))
    return;
  const uint8_t value = m.buf[1];
  const bool settingPending = isStartupSettingPhase(phase);
  if (settingPending && startupAckStarted && m.id == startupNode && m.buf[0] == startupCommand) {
    if (value == 0) {
      failStartup("setting rejected (ACK 0)");
      return;
    }
    if (value == 1 && !startupAckWindow.expired())
      ack = value;
    return;
  }
  auto &axis = motors[i];
  if (axis.offline || !axis.active || m.buf[0] != axis.command)
    return;
  if (value == 0 || (axis.command == 0xFE && (value == 3 || value == 5))) {
    failMotion(static_cast<uint8_t>(i), axis.command, "move rejected or limit reached");
    return;
  }
  if (value == 1 && !axis.started) {
    if (axis.start.expired()) {
      if constexpr (SystemConfig::BENCH_TEST_MODE) {
        if (job == Job::Startup && axis.command == 0x91) {
          noteOffline(static_cast<uint8_t>(i), axis.command, "home start ACK timeout (10000 ms)");
          return;
        }
      }
      failMotion(static_cast<uint8_t>(i), axis.command, "waiting for start ACK timed out");
      return;
    }
    axis.started = true;
  } else if (value == 2 && axis.started) {
    if (axis.finish.expired()) {
      failMotion(static_cast<uint8_t>(i), axis.command, "waiting for completion ACK timed out");
      return;
    }
    axis.done = true;
  }
}
void poll() {
  serviceStop();
  if (phase == Phase::Fault)
    return;
  if (SystemSafety::stopped()) {
    fail("Global emergency stop", false);
    return;
  }
  if (uploading && uploadWindow.expired()) {
    uploading = false;
    report("RAM upload timed out; retained previous plan");
  }
  for (uint8_t i = 0; i < Config::MOTOR_COUNT; ++i) {
    auto &m = motors[i];
    if (!m.active)
      continue;
    if ((!m.started && m.start.expired()) || m.finish.expired()) {
      if constexpr (SystemConfig::BENCH_TEST_MODE) {
        if (job == Job::Startup && m.command == 0x91 && !m.started) {
          noteOffline(i, m.command, "home start ACK timeout (10000 ms)");
          continue;
        }
      }
      failMotion(i, m.command,
                 !m.started ? "waiting for start ACK timed out"
                            : "waiting for completion ACK timed out");
      return;
    }
    if (m.done) {
      m.position = m.target;
      m.positionKnown = true;
      updateStage2Base(i, m.position);
      if (m.command == 0x91 && m.recoveryRequired) {
        m.recoveryRequired = false;
        bool isolated = false;
        for (const auto &axis : motors)
          isolated = isolated || axis.recoveryRequired;
        if (!isolated)
          localWarningBuffer[0] = '\0';
      }
      if (heightTrimStage(activeStage) && Config::MOTOR_IDS[i] == SystemConfig::Rc::STAGE4_TRIM_NODE &&
          job != Job::Stage4Trim)
        stage4TrimBase = m.position;
      m.active = false;
    }
  }
  if (selectionWindow.armed && selectionJob() && selectionWindow.expired()) {
    if (activeStage == SystemConfig::Rc::FRUIT_STAGE ||
        motors[motorIndex(6)].recoveryRequired)
      abortLocalPlan(-1, "Stage return timed out; interrupted axes isolated for manual HOME");
    else
      fail("Stage selection return deadline exceeded");
    return;
  }
  if (phase == Phase::Ready && pendingStage && !uploading && !stopMask && !disableMask)
    startReturn(); // Resume a pending hand-off after local isolation or delayed STOP enqueue.
  updateServos();
  CylinderController::poll();
  if (benchOfflineNotice) {
    benchOfflineNotice = false;
    return; // Give the serial gateway one loop turn to print this warning.
  }
  serviceStage2Trim();
  serviceStage4Trim();
  for (uint8_t n = 0; n < 16; ++n) {
    const Phase before = phase;
    const size_t beforeAction = actionIndex;
    const uint8_t beforeStep = currentStep;
    serviceState();
    if (benchOfflineNotice) {
      benchOfflineNotice = false;
      break;
    }
    if (phase == Phase::Fault ||
        (before == phase && beforeAction == actionIndex && beforeStep == currentStep))
      break;
  }
}
bool next() {
  if (!available() || debugMode || progress == UINT64_MAX || profileNeedsRecovery())
    return false;
  // A separate NEXT homes a completed Stage 1 prop. HOME completion resets
  // progress to STEP 0; only a fresh subsequent NEXT starts the next cycle.
  if (activeStage == 1 && progress >= stepCount())
    return home();
  stage1TrimSuspended = false;
  stage2TrimSuspended = activeStage != 2;
  stage4TrimSuspended = !heightTrimStage(activeStage);
  job = Job::Next;
  startStep(stepAt(progress + 1));
  return true;
}
bool previous() {
  if (!available() || debugMode || !progress || profileNeedsRecovery())
    return false;
  stage1TrimSuspended = false;
  stage2TrimSuspended = activeStage != 2;
  stage4TrimSuspended = !heightTrimStage(activeStage);
  previousTarget = progress - 1;
  const Snapshot target = stateAt(previousTarget);
  const uint16_t mask = profileActuatorMask();
  size_t count = 0;
  for (uint8_t i = 0; i < Config::MOTOR_COUNT; ++i)
    if ((mask & (1U << i)) && usableMotor(i))
      reverseActions[count++] = stepperFor(Config::MOTOR_IDS[i], target.motor[i], 320, 0, false);
  for (uint8_t i = 0; i < Config::SERVO_COUNT; ++i)
    if (mask & (1U << (Config::MOTOR_COUNT + i)))
      reverseActions[count++] = servo(Config::SERVO_PINS[i], target.servo[i], 300, false);
  if (mask & CYLINDER_BIT) {
    if (!target.cylinderKnown)
      CylinderController::invalidateReference();
    reverseActions[count++] = cylinder(target.cylinderKnown ? target.cylinderReference : CylinderAction::Stop,
                                      target.cylinderKnown ? SystemConfig::Cylinder::DEFAULT_TIMEOUT_MS : 0, false);
  }
  reverseActions[count++] = waitAll();
  job = Job::Previous;
  currentStep = stepAt(previousTarget);
  startPlan(reverseActions, count);
  return true;
}
bool runAll() {
  if (!available() || debugMode || profileNeedsRecovery())
    return false;
  stage1TrimSuspended = false;
  stage2TrimSuspended = activeStage != 2;
  stage4TrimSuspended = !heightTrimStage(activeStage);
  captureEntry();
  job = Job::Run;
  startStep(1);
  return true;
}
bool home() {
  if (!available() || debugMode)
    return false;
  stage2TrimSuspended = true;
  stage4TrimSuspended = true;
  stage1TrimSuspended = true;
  refreshTrimOutput();
  captureEntry();
  job = Job::Home;
  if (activeStage == 6)
    startPlan(HOME6, sizeof(HOME6) / sizeof(Action));
  else if (activeStage == 5)
    startPlan(HOME5, sizeof(HOME5) / sizeof(Action));
  else if (activeStage == 3)
    startPlan(HOME3, sizeof(HOME3) / sizeof(Action));
  else if (activeStage == 2)
    startPlan(HOME2, sizeof(HOME2) / sizeof(Action));
  else
    startPlan(HOME14, sizeof(HOME14) / sizeof(Action));
  report(activeStage == 5 ? "Stage 5 HOME: standby without hardware movement"
         : activeStage == 3 ? "Returning stage 3 to stationary startup zero with 0xFE"
         : activeStage == 2 ? "Retracting stage 2 cylinder and returning servos to standby"
                            : "Homing selected stage with 0x91");
  return true;
}
bool selectProfile(uint8_t stage, uint8_t prop) {
  if (!validProfile(stage, prop) || uploading || pendingCommit || debugMode || phase == Phase::Fault ||
      SystemSafety::stopped() || phase == Phase::BootDelay || isStartupSettingPhase(phase) ||
      (job == Job::Startup && phase != Phase::Ready))
    return false;
  prop = normalizedProp(stage, prop);
  const bool same = stage == activeStage && prop == activeProp;
  if (pendingStage) {
    // Pure MKS/servo selections retain complete-STEP and cancellation behavior.
    // An already interrupted cylinder STEP cannot be resumed by cancellation.
    if (!selectionJob() && !returnWithHome && same) {
      pendingStage = pendingProp = 0;
      selectionWindow.armed = false;
      CylinderController::stop();
    } else {
      pendingStage = stage;
      pendingProp = prop;
      serviceSelectionRequest();
    }
    refreshTrimOutput();
    return true;
  }
  if (same)
    return true;
  pendingStage = stage;
  pendingProp = prop;
  serviceSelectionRequest();
  refreshTrimOutput();
  return true;
}
bool selectStage(uint8_t stage) { return selectProfile(stage); }
bool triggerProp(uint8_t prop) {
  if (!available() || debugMode || activeStage != 1 || prop < 1 ||
      prop > Config::STAGE1_PROP_COUNT || motors[motorIndex(6)].recoveryRequired)
    return false;
  activeProp = prop;
  captureEntry();
  stage1TrimSuspended = false;
  stage2TrimSuspended = activeStage != 2;
  stage4TrimSuspended = !heightTrimStage(activeStage);
  job = Job::Next;
  startStep(1);
  report("Stage 1 prop triggered: executing STEP 1");
  return true;
}
bool exitStage1(uint8_t destination) {
  if (destination < 2 || !stageEnabled(destination))
    return false;
  // A HIGH selection not yet activated may be cancelled by LOW without homing
  // an unrelated outgoing stage.
  if (activeStage != 1)
    return selectProfile(destination);
  if (uploading || pendingCommit || debugMode || phase == Phase::Fault ||
      SystemSafety::stopped() || phase == Phase::BootDelay || isStartupSettingPhase(phase) ||
      (job == Job::Startup && phase != Phase::Ready))
    return false;
  if (!returnWithHome) {
    const int index = motorIndex(6);
    // An in-flight explicit ID 6 HOME can satisfy LOW; do not send it twice.
    exitHomeSent = index >= 0 && motors[index].active && motors[index].command == 0x91;
  }
  returnWithHome = true;
  pendingStage = destination;
  pendingProp = 0;
  serviceSelectionRequest();
  refreshTrimOutput();
  return true;
}
void stop() { fail("Global emergency stop", false); }
Status status() {
  const auto pulse = [](uint8_t pin) {
    const int index = servoIndex(pin);
    return index < 0 ? 0 : servos[index].output;
  };
  const auto position = [](uint8_t node) -> int32_t {
    const int index = motorIndex(node);
    return index < 0 ? 0 : motors[index].position;
  };
  uint8_t knownMask = 0, onlineMask = 0, offlineMask = 0, recoveryMask = 0;
  for (uint8_t i = 0; i < Config::MOTOR_COUNT; ++i) {
    const uint8_t bit = static_cast<uint8_t>(1U << (Config::MOTOR_IDS[i] - 4));
    if (motors[i].positionKnown)
      knownMask |= bit;
    if (motors[i].online)
      onlineMask |= bit;
    if (motors[i].offline)
      offlineMask |= bit;
    if (motors[i].recoveryRequired)
      recoveryMask |= bit;
  }
  const auto current = [](uint8_t node) -> uint16_t {
    const int index = motorIndex(node);
    return index < 0 ? 0 : configuredCurrentMa[index];
  };
  uint8_t currentMask = 0;
  for (uint8_t i = 0; i < Config::MOTOR_COUNT; ++i)
    if (configuredCurrentMa[i] && Config::MOTOR_IDS[i] >= 4 && Config::MOTOR_IDS[i] <= 6)
      currentMask |= static_cast<uint8_t>(1U << (Config::MOTOR_IDS[i] - 4));
  const char *startupPhase = phase == Phase::Fault           ? "fault"
                             : startupComplete               ? "ready"
                             : phase == Phase::BootDelay     ? "boot_delay"
                             : phase == Phase::FeedbackQueue ? "feedback_queue"
                             : phase == Phase::FeedbackWait  ? "feedback_wait"
                             : phase == Phase::CurrentQueue  ? "current_queue"
                             : phase == Phase::CurrentWait   ? "current_wait"
                             : phase == Phase::EnableQueue   ? "enable_queue"
                             : phase == Phase::EnableWait    ? "enable_wait"
                             : phase == Phase::ZeroQueue     ? "zero_queue"
                             : phase == Phase::ZeroWait      ? "zero_wait"
                             : phase == Phase::ActionQueue   ? "home_queue"
                                                             : "home_wait";
  const int trimIndex = motorIndex(SystemConfig::Rc::STAGE4_TRIM_NODE);
  const int32_t trimPosition = motors[trimIndex].position;
  const int32_t trimTarget = motors[trimIndex].active ? motors[trimIndex].target : trimPosition;
  const char *state =
      phase == Phase::Ready   ? "ready"
      : phase == Phase::Fault ? "fault"
      : (phase == Phase::BootDelay || isStartupSettingPhase(phase))
          ? "booting"
          : (job == Job::Home || job == Job::AutoHome || job == Job::EntryHome) ? "home_wait"
          : (job == Job::Stage4Trim || job == Job::Stage2Trim) ? "trim_wait"
          : job == Job::SwitchStop ? "return_wait"
          : job == Job::SwitchReturn ? (returnWithHome ? "home_wait" : "return_wait")
                                    : "busy";
  return {debugMode,          uploading,
          runtimePlan(),      state,
          completed,          currentStep,
          reportedBranch,     stepCount(),
          planVersion,        pulse(2),
          pulse(5),           pulse(3),
          pulse(4),           pulse(6),
          pulse(9),           pulse(10),
          pulse(11),          pulse(12),
          activeStage,
          pendingStage,       position(4),
          position(5),        position(6),
          available() && !debugMode && progress != 0,
          progress,           notice,
          noticeVersion,      startupNode,
          startupCommand,     startupAttempts,
          startupPhase,       knownMask,
          static_cast<uint16_t>(startupCommand == 0x83 ? Config::startupCurrentMilliamps(startupNode) : 0),
          current(4),         current(5),
          current(6),
          currentMask,       activeProp,
          pendingProp,       activeStage == 1 && progress >= stepCount(),
          CylinderController::status(), onlineMask, offlineMask,
          servos[servoIndex(SystemConfig::Rc::STAGE1_TRIM_PIN)].pulse,
          trimAllowed() ? pulse(2) - servos[servoIndex(SystemConfig::Rc::STAGE1_TRIM_PIN)].pulse : 0,
          trimAllowed(),
          heightTrimStage(activeStage) ? stage4TrimBase : trimPosition,
          heightTrimStage(activeStage) ? trimPosition - stage4TrimBase : 0,
          trimTarget, stage4TrimAllowed(),
          job == Job::Stage4Trim && phase != Phase::Ready && phase != Phase::Fault,
          recoveryMask, profileNeedsRecovery(), localWarningBuffer,
          activeStage == 2 ? stage2TrimBase[0] : position(4),
          activeStage == 2 ? stage2TrimBase[1] : position(5),
          activeStage == 2 ? position(4) - stage2TrimBase[0] : 0,
          motors[motorIndex(4)].active ? motors[motorIndex(4)].target : position(4),
          motors[motorIndex(5)].active ? motors[motorIndex(5)].target : position(5),
          stage2TrimAllowed(),
          job == Job::Stage2Trim && phase != Phase::Ready && phase != Phase::Fault};
}
bool setDebug(bool enabled) {
  if (!available())
    return false;
  if (enabled != debugMode)
    captureEntry();
  debugMode = enabled;
  stage2TrimSuspended = true;
  stage4TrimSuspended = true;
  if (!enabled)
    captureEntry();
  return true;
}
bool debugStepper(int32_t position, uint16_t rpm, uint8_t acc, bool goHome, uint8_t node) {
  const auto a = stepperFor(node, position, rpm, acc, false);
  if (!debugMode || uploading || phase == Phase::Fault || SystemSafety::stopped() || !valid(a) ||
      (activeStage == 3 && !Config::STAGE3_ENABLED) || (goHome && !homingMotor(node)))
    return false;
  if (!available() && !(job == Job::Debug || job == Job::DebugGoHome))
    return false;
  const uint8_t i = static_cast<uint8_t>(motorIndex(node));
  if (!usableMotor(i, goHome) || motors[i].active || !startMotor(i, a, goHome))
    return false;
  if (goHome && activeStage == 1) {
    stage1TrimSuspended = true;
    refreshTrimOutput();
  }
  if (phase == Phase::Ready) {
    job = Job::Debug;
    currentStep = 0;
    startPlan(DEBUG_WAIT, 1);
  }
  return true;
}
bool debugServo(uint8_t pin, uint16_t angleTenths, uint32_t durationMs) {
  const int i = servoIndex(pin);
  if (!debugMode || uploading || phase == Phase::Fault || SystemSafety::stopped() ||
      angleTenths > 1800 || durationMs > 60000 || i < 0 ||
      (activeStage == 3 && !Config::STAGE3_ENABLED))
    return false;
  if (phase != Phase::Ready && !(job == Job::Debug || job == Job::DebugGoHome))
    return false;
  const int pulse =
      Config::SERVO_MIN_US +
      (int32_t(angleTenths) * (Config::SERVO_MAX_US - Config::SERVO_MIN_US) + 900) / 1800;
  servoMove(static_cast<uint8_t>(i), pulse, durationMs);
  if (phase == Phase::Ready) {
    job = Job::Debug;
    currentStep = 0;
    startPlan(DEBUG_WAIT, 1);
  }
  return true;
}
bool debugCylinder(CylinderAction action, uint32_t durationMs) {
  if (!debugMode || uploading || phase == Phase::Fault || SystemSafety::stopped() ||
      !valid(cylinder(action, durationMs)))
    return false;
  if (!available() && !(job == Job::Debug || job == Job::DebugGoHome))
    return false;
  if (action != CylinderAction::Stop && CylinderController::status().active)
    return false;
  if (!CylinderController::start(action, durationMs))
    return false;
  if (phase == Phase::Ready) {
    job = Job::Debug;
    currentStep = 0;
    startPlan(DEBUG_WAIT, 1);
  }
  return true;
}
bool planBegin(uint16_t steps, uint16_t count, uint8_t stage, uint8_t repeatStart, uint8_t prop) {
  if (!available() || debugMode || steps < 1 || steps > 254 || count < 1 ||
      count > MAX_RUNTIME_ACTIONS || !validProfile(stage, prop) || repeatStart < 1 ||
      repeatStart > steps)
    return false;
  stagedStepCount = steps;
  stagedActionCount = count;
  stagedStage = stage;
  stagedProp = normalizedProp(stage, prop);
  stagedRepeatFrom = repeatStart;
  memset(actionWritten, 0, sizeof(actionWritten));
  memset(stepWritten, 0, sizeof(stepWritten));
  uploading = true;
  uploadWindow.start(10000);
  return true;
}
bool planAction(uint16_t index, const Action &a) {
  if (!uploadActive() || index >= stagedActionCount || !valid(a) || a.substep < 1 ||
      a.substep > 256 ||
      (stagedStage == 3 && !Config::STAGE3_ENABLED &&
       (a.kind != ActionKind::Delay || a.ms != 0)))
    return false;
  runtimeActions[uploadBank][index] = a;
  actionWritten[index] = true;
  uploadWindow.start(10000);
  return true;
}
bool planStep(uint16_t index, uint16_t start, uint16_t count) {
  if (!uploadActive() || index >= stagedStepCount || !count ||
      uint32_t(start) + count > stagedActionCount)
    return false;
  runtimeSteps[uploadBank][index] = {runtimeActions[uploadBank] + start, count};
  stepWritten[index] = true;
  uploadWindow.start(10000);
  return true;
}
bool planCommit() {
  if (!uploadActive() || phase != Phase::Ready || SystemSafety::stopped())
    return false;
  size_t expected = 0;
  for (uint16_t i = 0; i < stagedActionCount; ++i)
    if (!actionWritten[i])
      return false;
  for (uint16_t i = 0; i < stagedStepCount; ++i) {
    const auto &s = runtimeSteps[uploadBank][i];
    if (!stepWritten[i] || s.actions != runtimeActions[uploadBank] + expected)
      return false;
    uint16_t previousBranch = 0;
    uint16_t pending = 0;
    for (size_t j = 0; j < s.count; ++j) {
      const auto &a = s.actions[j];
      if (a.substep != previousBranch) {
        if (pending || a.substep != previousBranch + 1)
          return false;
        previousBranch = a.substep;
      } else if (!pending)
        return false;
      if (a.kind == ActionKind::WaitAll) {
        if (!pending || !a.wait)
          return false;
        pending = 0;
      } else if (!a.wait) {
        if (a.kind != ActionKind::Stepper && a.kind != ActionKind::Servo &&
            a.kind != ActionKind::Cylinder)
          return false;
        const uint16_t bit =
            a.kind == ActionKind::Stepper
                ? static_cast<uint16_t>(1U << motorIndex(a.node))
                : a.kind == ActionKind::Servo
                    ? static_cast<uint16_t>(1U << (Config::MOTOR_COUNT + servoIndex(a.pin)))
                    : CYLINDER_BIT;
        if (pending & bit)
          return false;
        pending |= bit;
      } else if (pending)
        return false;
    }
    if (pending)
      return false;
    expected += s.count;
  }
  if (expected != stagedActionCount)
    return false;
  uploading = false;
  // Keep the current profile installed until its own return barrier completes.
  pendingStage = stagedStage;
  pendingProp = stagedProp;
  pendingCommit = true;
  startReturn();
  return true;
}
void planAbort() {
  uploading = false;
  if (pendingCommit) {
    pendingCommit = false;
    pendingStage = activeStage;
    pendingProp = activeProp;
  }
}
} // namespace MechanismController
