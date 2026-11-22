#pragma once
#include "mechanism_config.h"
#include "cylinder_controller.h"
#include <FlexCAN_T4.h>
#include <stdint.h>

namespace MechanismController {
void begin();
void poll();
void receive(uint8_t bus, const CAN_message_t &message);
bool next();
bool previous();
bool runAll();
bool home();
bool selectProfile(uint8_t stage, uint8_t prop = 0);
bool selectStage(uint8_t stage);
bool triggerProp(uint8_t prop);
bool exitStage1(uint8_t destination = 4);
void stop();
struct Status {
  bool debug, uploading, runtimePlan;
  const char *phase;
  uint8_t completed, step;
  uint16_t branch, steps;
  uint32_t planVersion;
  int pulse2, pulse5, pulse3;
  int pulse4, pulse6, pulse9;
  int pulse10, pulse11, pulse12;
  uint8_t stage, pendingStage;
  int32_t position4, position5, position6;
  bool previousAvailable;
  uint64_t progress;
  const char *notice;
  uint32_t noticeVersion;
  uint8_t startupNode, startupCommand;
  uint16_t startupAttempts;
  const char *startupPhase;
  uint8_t positionKnownMask; // Bit 0/1/2 corresponds to ID 4/5/6.
  uint16_t startupCurrentMa; // Current 0x83 target; retained on startup fault.
  uint16_t current4Ma, current5Ma; // Accepted startup settings, not measured current.
  uint16_t current6Ma;
  uint8_t currentConfiguredMask; // ID 4/5/6 bits; all three accepted settings = 7.
  uint8_t prop, pendingProp; // Stage 1: 1..5; normal stages: 0.
  bool propComplete; // Stage 1 one-shot terminal state.
  CylinderController::Status cylinder;
  // ID 4/5/6 bits. Unknown at boot; offline is final until restart.
  uint8_t motorOnlineMask, motorOfflineMask;
  int pulse2Base, stage1TrimUs; // Untrimmed base and the applied (possibly clipped) offset.
  bool stage1TrimAllowed;
  // Stage4/6 share ID6 height trim; legacy stage4 field names preserve USB compatibility.
  int32_t position6Base, stage4TrimPulses, stage4TrimTarget;
  bool stage4TrimAllowed, stage4TrimMoving; // Moving waits for MKS completion, preserving STEP history.
  uint8_t motorRecoveryMask; // ID4/5/6 bits: interrupted/failed axes need explicit HOME.
  bool stageRecoveryRequired;
  const char *localWarning; // Local isolation leaves unrelated stages and chassis operational.
  int32_t position4Base, position5Base, stage2TrimPulses;
  int32_t stage2TrimTarget4, stage2TrimTarget5;
  bool stage2TrimAllowed, stage2TrimMoving; // A paired trim waits for both FE completions.
};
Status status();
bool setDebug(bool enabled);
bool debugStepper(int32_t position, uint16_t rpm, uint8_t acc, bool goHome = false,
                  uint8_t node = SystemConfig::Mechanism::MOTOR_ID);
bool debugServo(uint8_t pin, uint16_t angleTenths, uint32_t durationMs);
bool debugCylinder(MechanismConfig::CylinderAction action, uint32_t durationMs);
constexpr uint16_t MAX_RUNTIME_ACTIONS = SystemConfig::Mechanism::RAM_ACTIONS;
bool planBegin(uint16_t steps, uint16_t actions, uint8_t stage = 4, uint8_t repeatFrom = 1,
               uint8_t prop = 0);
bool planAction(uint16_t index, const MechanismConfig::Action &action);
bool planStep(uint16_t index, uint16_t start, uint16_t count);
bool planCommit();
void planAbort();
} // namespace MechanismController
