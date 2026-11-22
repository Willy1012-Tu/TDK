#pragma once
#include <stdint.h>
#include "stage4_config.h"
namespace Stage4Test {
void begin();
void loop();
// 初始化及動作期間拒收新動作；stop 隨時有效，停止後須重啟。
bool next();
bool runAll();
bool home();
bool selectStage(uint8_t stage);
void stop();
void poll(); // 核心執行緒及主機測試共用的輪詢入口。
struct Status {
  bool debug, uploading, runtimePlan;
  const char *phase;
  uint8_t completed, step;
  uint16_t branch, steps;
  uint32_t planVersion;
  int pulse2, pulse5;
  int pulse3;
  uint8_t stage, pendingStage;
  int32_t position4, position5, position6;
  const char *startupPhase;
  uint8_t startupNode, startupCommand;
  uint16_t startupAttempts;
  uint8_t positionKnownMask; // ID6 bit2、ID7 bit3；實際完成回報才設置。
  int pulse4, pulse6, pulse9;
  int32_t position7;
};
Status status();
bool setDebug(bool enabled);
bool debugStepper(int32_t position, uint16_t rpm, uint8_t acc, bool goHome=false,
                  uint8_t node=TestConfig::MOTOR_ID);
bool debugServo(uint8_t pin, uint16_t angleTenths, uint32_t durationMs);
// Upload 使用另一個 RAM bank；commit 前不覆寫有效設定。
constexpr uint16_t MAX_RUNTIME_ACTIONS=2048;
bool planBegin(uint16_t steps, uint16_t actions, uint8_t stage=4, uint8_t repeatFrom=1);
bool planAction(uint16_t index, const Stage4::Action &action);
bool planStep(uint16_t index, uint16_t start, uint16_t count);
bool planCommit();
void planAbort();
}
