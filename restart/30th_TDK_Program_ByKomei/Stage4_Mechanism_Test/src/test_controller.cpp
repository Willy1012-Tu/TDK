#include "test_controller.h"
#include "test_config.h"
#include "stage4_config.h"
#include "serial_protocol.h"
#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <Servo.h>
#include <IntervalTimer.h>
#include <TeensyThreads.h>
#include <string.h>
#include <stdio.h>

namespace {
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> canBus;
Threads::Mutex stateMutex;
IntervalTimer clockTimer;
// 每顆馬達及伺服有獨立倒數；中斷只更新計時，不執行 CAN 或 USB。
enum Timer : uint8_t { Window, Retry, MoveStart,
  MoveFinish=MoveStart+TestConfig::MOTOR_COUNT,
  PauseTime=MoveFinish+TestConfig::MOTOR_COUNT, Ramp,
  Update=Ramp+TestConfig::SERVO_COUNT,
  UploadWindow=Update+TestConfig::SERVO_COUNT, TimerCount };
volatile uint32_t remaining[TimerCount]{};
void tick() { for (auto &left:remaining) if (left) --left; }
void startTimer(Timer t, uint32_t ms) { remaining[t]=ms==UINT32_MAX?ms:ms+1; }
bool expired(Timer t) { return remaining[t]==0; }
Timer motorStartTimer(uint8_t i) { return static_cast<Timer>(MoveStart+i); }
Timer motorFinishTimer(uint8_t i) { return static_cast<Timer>(MoveFinish+i); }
Timer servoRampTimer(uint8_t i) { return static_cast<Timer>(Ramp+i); }
Timer servoUpdateTimer(uint8_t i) { return static_cast<Timer>(Update+i); }
enum class Phase : uint8_t { BootDelay, FeedbackQueue, FeedbackWait, EnableQueue,
  EnableWait, ActionQueue, Moving, Pause, Ready, Fault };
enum class Job : uint8_t { Startup, Next, Run, Home, Debug, DebugGoHome };
Phase phase=Phase::BootDelay;
Job job=Job::Startup;
const Stage4::Action *actions=nullptr;
size_t actionCount=0, actionIndex=0;
uint16_t reportedBranch=0;
uint8_t completed=0, currentStep=0, activeStage=TestConfig::DEFAULT_STAGE, pendingStage=0;
bool debugMode=false, uploading=false, runtimePlan=false, pendingCommit=false;
uint8_t activeBank=0, initMotor=0, ack=0xFF;
uint16_t runtimeStepCount=0, stagedStepCount=0, stagedActionCount=0;
uint8_t runtimeRepeatFrom=1, stagedStage=4, stagedRepeatFrom=1;
uint32_t planVersion=0;
Stage4::Action runtimeActions[2][Stage4Test::MAX_RUNTIME_ACTIONS]{};
Stage4::Step runtimeSteps[2][254]{};
bool actionWritten[Stage4Test::MAX_RUNTIME_ACTIONS]{}, stepWritten[254]{};
constexpr Stage4::Action DEBUG_WAIT[]={Stage4::waitAll()};
bool timersReady=false, workerReady=false, txAttempted=false;
uint8_t stopMask=0, disableMask=0;
struct MotorAxis {
  bool active=false, started=false, done=false;
  bool positionKnown=false;
  uint8_t command=0xFE;
  int32_t position=0, target=0;
};
MotorAxis motors[TestConfig::MOTOR_COUNT];
struct ServoAxis {
  Servo device;
  int pulse=1500, from=1500, target=1500;
  uint32_t ms=0;
  bool active=false;
};
ServoAxis servos[TestConfig::SERVO_COUNT];
const char *notice="開機等待中。";
uint32_t noticeVersion=1;
const char *startupPhase="delay";
uint8_t startupNode=0, startupCommand=0;
uint16_t startupAttempts=0;
char detailNotice[224]{};
void report(const char *message) { notice=message; ++noticeVersion; }
struct Progress { uint8_t step; uint16_t branch; };
constexpr uint8_t PROGRESS_CAPACITY=32;
Progress progress[PROGRESS_CAPACITY]{};
uint8_t progressHead=0, progressCount=0;
void reportProgress(uint8_t step, uint16_t branch) {
  if (!Serial) return;
  if (progressCount==PROGRESS_CAPACITY) {
    progressHead=(progressHead+1)%PROGRESS_CAPACITY; --progressCount;
  }
  progress[(progressHead+progressCount)%PROGRESS_CAPACITY]={step,branch}; ++progressCount;
}
int motorIndex(uint32_t node) {
  for (uint8_t i=0;i<TestConfig::MOTOR_COUNT;++i) if (TestConfig::MOTOR_IDS[i]==node) return i;
  return -1;
}
int servoIndex(uint8_t pin) {
  for (uint8_t i=0;i<TestConfig::SERVO_COUNT;++i) if (TestConfig::SERVO_PINS[i]==pin) return i;
  return -1;
}
int pulseAt(uint8_t pin) {const int i=servoIndex(pin); return i<0?0:servos[i].pulse;}
int32_t positionAt(uint8_t node) {const int i=motorIndex(node); return i<0?0:motors[i].position;}
uint8_t knownMask() {
  uint8_t result=0;
  for(uint8_t i=0;i<TestConfig::MOTOR_COUNT;++i) {
    const uint8_t node=TestConfig::MOTOR_IDS[i];
    if(motors[i].positionKnown && node>=4 && node<=7) result|=static_cast<uint8_t>(1U<<(node-4));
  }
  return result;
}
bool motorsBusy() { for (const auto &m:motors) if (m.active) return true; return false; }
bool servosBusy() { for (const auto &s:servos) if (s.active) return true; return false; }
uint16_t stepCount() {
  return runtimePlan?runtimeStepCount:static_cast<uint16_t>(
    activeStage==2?Stage4::STAGE2_STEP_COUNT:activeStage==3?Stage4::STAGE3_STEP_COUNT:Stage4::STEP_COUNT);
}
uint8_t repeatFrom() {
  return runtimePlan?runtimeRepeatFrom:(activeStage==2?Stage2::REPEAT_FROM:activeStage==3?Stage3::REPEAT_FROM:Stage4::REPEAT_FROM);
}
bool send(uint8_t node, uint8_t command, const uint8_t *data=nullptr, uint8_t count=0) {
  if (motorIndex(node)<0) return false;
  CAN_message_t m{}; m.id=node; m.len=count+2; m.buf[0]=command;
  uint32_t sum=node+command;
  for (uint8_t i=0;i<count;++i) { m.buf[i+1]=data[i]; sum+=data[i]; }
  m.buf[count+1]=static_cast<uint8_t>(sum);
  return canBus.write(m)!=0;
}
void serviceStop() {
  if (!stopMask && !disableMask) return;
  if (timersReady && txAttempted && !expired(Retry)) return;
  txAttempted=true; if (timersReady) startTimer(Retry,TestConfig::TX_RETRY_MS);
  const uint8_t off=0;
  for (uint8_t i=0;i<TestConfig::MOTOR_COUNT;++i) {
    const uint8_t bit=static_cast<uint8_t>(1U<<i);
    if ((stopMask&bit) && send(TestConfig::MOTOR_IDS[i],0xF7)) stopMask&=static_cast<uint8_t>(~bit);
    if ((disableMask&bit) && send(TestConfig::MOTOR_IDS[i],0xF3,&off,1)) disableMask&=static_cast<uint8_t>(~bit);
  }
}
void fail(const char *reason) {
  if (phase==Phase::Fault) return;
  phase=Phase::Fault; uploading=pendingCommit=false; pendingStage=0;
  progressHead=progressCount=0;
  for (auto &m:motors) m.active=false;
  for (auto &s:servos) {s.active=false;if(s.device.attached())s.device.detach();}
  stopMask=disableMask=static_cast<uint8_t>((1U<<TestConfig::MOTOR_COUNT)-1U);
  txAttempted=false; serviceStop(); report(reason);
}
void reportStartup(const char *reason) {
  snprintf(detailNotice,sizeof(detailNotice),"Startup ID%u 0x%02X attempt %u: %s",
    unsigned(startupNode),unsigned(startupCommand),unsigned(startupAttempts),reason);
  report(detailNotice);
}
void failStartup(const char *reason) {
  snprintf(detailNotice,sizeof(detailNotice),"Startup ID%u 0x%02X attempt %u: %s; STOP all motors, restart required.",
    unsigned(startupNode),unsigned(startupCommand),unsigned(startupAttempts),reason);
  fail(detailNotice);
}
void failMotor(uint8_t index, const char *reason) {
  snprintf(detailNotice,sizeof(detailNotice),"ID%u 0x%02X: %s; STOP all motors, restart required.",
    unsigned(TestConfig::MOTOR_IDS[index]),unsigned(motors[index].command),reason);
  fail(detailNotice);
}
void queue(Phase next) {
  phase=next; txAttempted=false; ack=0xFF; startTimer(Window,TestConfig::TX_TIMEOUT_MS);
  if (next==Phase::FeedbackQueue || next==Phase::EnableQueue) {
    startupPhase=next==Phase::FeedbackQueue?"feedback_queue":"enable_queue";
    startupNode=TestConfig::MOTOR_IDS[initMotor];
    startupCommand=next==Phase::FeedbackQueue?0x8C:0xF3;
    startupAttempts=0;
  }
}
bool txDue() {
  if (expired(Window)) {
    if (phase==Phase::FeedbackQueue || phase==Phase::EnableQueue) failStartup("CAN 指令佇列逾時");
    else fail("CAN 指令佇列逾時，已停止；請重啟。");
    return false;
  }
  if (txAttempted && !expired(Retry)) return false;
  txAttempted=true; startTimer(Retry,TestConfig::TX_RETRY_MS); return true;
}
void receive(const CAN_message_t &m) {
  const int index=motorIndex(m.id);
  if (index<0 || m.flags.extended || m.flags.remote || m.len!=3) return;
  if (m.buf[2]!=static_cast<uint8_t>(m.id+m.buf[0]+m.buf[1])) return;
  const uint8_t value=m.buf[1];
  if (index==initMotor && ((phase==Phase::FeedbackWait && m.buf[0]==0x8C)
      || (phase==Phase::EnableWait && m.buf[0]==0xF3))) {
    if (value==0) { ack=0; failStartup("驅動器明確拒絕設定（ACK 00）"); }
    else if (value==1 && ack!=0) ack=value;
    return;
  }
  auto &axis=motors[index];
  if (!axis.active || m.buf[0]!=axis.command) return;
  if (value==0 || (axis.command==0xFE && (value==3 || value==5))) {
    failMotor(static_cast<uint8_t>(index),"移動／尋原點回報失敗或限位"); return;
  }
  if (value==1 && !axis.started) {
    if (expired(motorStartTimer(static_cast<uint8_t>(index)))) {
      failMotor(static_cast<uint8_t>(index),"等待開始回報逾時"); return;
    }
    axis.started=true;
  } else if (value==2 && axis.started) axis.done=true;
}
bool startMotor(uint8_t i, const Stage4::Action &a, bool goHome) {
  auto &m=motors[i];
  if (m.active) return false;
  // 已完成回報的位置等於目標時維持原位；91 尋零仍一律實際執行。
  if (!goHome && m.positionKnown && m.position==a.position) return true;
  const uint8_t node=TestConfig::MOTOR_IDS[i];
  if (goHome) {
    if (!send(node,0x91)) return false;
  } else {
    // FE 坐標為帶符號 24 位二補數；負數不能用 FD 的方向位與距離編碼。
    const uint32_t raw=static_cast<uint32_t>(a.position)&0xFFFFFFU;
    const uint8_t data[6]={static_cast<uint8_t>(a.rpm>>8),static_cast<uint8_t>(a.rpm),a.acc,
      static_cast<uint8_t>(raw>>16),static_cast<uint8_t>(raw>>8),static_cast<uint8_t>(raw)};
    if (!send(node,0xFE,data,6)) return false;
  }
  m.active=true; m.started=m.done=false; m.command=goHome?0x91:0xFE;
  m.target=goHome?0:a.position;
  startTimer(motorStartTimer(i),TestConfig::START_TIMEOUT_MS);
  startTimer(motorFinishTimer(i),TestConfig::FINISH_TIMEOUT_MS);
  return true;
}
void servoMove(uint8_t i, int pulse, uint32_t ms) {
  auto &s=servos[i];
  if (!ms || pulse==s.pulse) { s.active=false; s.pulse=pulse; s.device.writeMicroseconds(pulse); return; }
  s.from=s.pulse; s.target=pulse; s.ms=ms; s.active=true;
  startTimer(servoRampTimer(i),ms); startTimer(servoUpdateTimer(i),TestConfig::SERVO_UPDATE_MS);
}
void updateServos() {
  for (uint8_t i=0;i<TestConfig::SERVO_COUNT;++i) {
    auto &s=servos[i]; if (!s.active) continue;
    const uint32_t left=remaining[servoRampTimer(i)];
    const uint32_t elapsed=left>=s.ms?0:s.ms-left;
    const bool done=elapsed>=s.ms;
    if (!done && !expired(servoUpdateTimer(i))) continue;
    s.pulse=s.from+static_cast<int>((static_cast<int32_t>(s.target-s.from)
      *static_cast<int32_t>(elapsed))/static_cast<int32_t>(s.ms));
    s.device.writeMicroseconds(s.pulse); if (done) s.active=false;
    startTimer(servoUpdateTimer(i),TestConfig::SERVO_UPDATE_MS);
  }
}
void startPlan(const Stage4::Action *plan, size_t count) {
  actions=plan; actionCount=count; actionIndex=0; reportedBranch=0; queue(Phase::ActionQueue);
}
void startStep(uint8_t step) {
  currentStep=step;
  const auto &s=runtimePlan?runtimeSteps[activeBank][step-1]:
    (activeStage==2?Stage2::STEPS[step-1]:activeStage==3?Stage3::STEPS[step-1]:Stage4::STEPS[step-1]);
  startPlan(s.actions,s.count); reportProgress(step,0);
}
void installRuntime() {
  activeBank=1-activeBank; runtimeStepCount=stagedStepCount; runtimeRepeatFrom=stagedRepeatFrom;
  activeStage=stagedStage; runtimePlan=true; ++planVersion; pendingCommit=false;
}
void finishAction() {
  if (++actionIndex<actionCount) { queue(Phase::ActionQueue); return; }
  if (job==Job::Debug || job==Job::DebugGoHome) {
    phase=Phase::Ready; report("Debug 動作完成。"); return;
  }
  completed=currentStep;
  // 第三關待機沒有離場停放致動器。
  if (job==Job::Run && completed<stepCount()) { startStep(completed+1); return; }
  phase=Phase::Ready;
  if (job==Job::Startup) startupPhase="ready";
  report(job==Job::Startup?"啟用軸尋原點完成，已建立 0。":
    job==Job::Home?((activeStage==2 || activeStage==4)?"尋原點完成，段數已歸零。":"待機設定完成，段數已歸零。"):job==Job::Run?"整關執行完成。":"本段完成。");
}
void startAction() {
  const auto &a=actions[actionIndex];
  if (!Stage4::validForStage(activeStage,a)) { fail("動作設定超出範圍，已停止；請修正設定後重啟。"); return; }
  if (a.substep && a.substep!=reportedBranch) {
    reportedBranch=a.substep; reportProgress(currentStep,a.substep);
  }
  if (a.kind==Stage4::ActionKind::Delay) {
    if (!a.ms) { finishAction(); return; }
    startTimer(PauseTime,a.ms); phase=Phase::Pause; return;
  }
  if (a.kind==Stage4::ActionKind::Stepper) {
    const uint8_t index=static_cast<uint8_t>(motorIndex(a.node));
    if (motors[index].active) { fail("同一顆步進的上一動作尚未完成，已停止。"); return; }
    if (!txDue()) return;
    const bool goHome=job==Job::Startup || job==Job::Home || job==Job::DebugGoHome;
    if (job==Job::Startup) {
      startupPhase="homing"; startupNode=a.node; startupCommand=0x91; startupAttempts=1;
    }
    if (!startMotor(index,a,goHome)) {
      // 一組成對移動只能全組開始；避免另一顆已移動時繼續重送落後馬達。
      if (!a.wait && motorsBusy()) fail("平行步進指令僅部分送出，已停止全部馬達。");
      return;
    }
    if (job==Job::Startup) reportStartup("尋原點已入列，等待 01 開始／02 完成回報");
  } else if (a.kind==Stage4::ActionKind::Servo) {
    servoMove(static_cast<uint8_t>(servoIndex(a.pin)),a.pulse,a.ms);
  }
  if (a.wait) phase=Phase::Moving;
  else finishAction();
}
void serviceState() {
  if (phase==Phase::BootDelay) {
    if (!expired(Window)) return;
    queue(Phase::FeedbackQueue); report("確認各 MKS 完成回報與啟用設定。");
  }
  if (phase==Phase::FeedbackQueue || phase==Phase::EnableQueue) {
    if (!txDue()) return;
    const bool feedback=phase==Phase::FeedbackQueue;
    const uint8_t data[2]={1,1};
    if (startupAttempts<UINT16_MAX) ++startupAttempts;
    if (!send(TestConfig::MOTOR_IDS[initMotor],feedback?0x8C:0xF3,data,feedback?2:1)) {
      reportStartup("CAN 佇列拒收，等待重試"); return;
    }
    phase=feedback?Phase::FeedbackWait:Phase::EnableWait;
    startupPhase=feedback?"feedback_wait":"enable_wait";
    startTimer(Window,TestConfig::STARTUP_ACK_TIMEOUT_MS);
    startTimer(Retry,TestConfig::STARTUP_ACK_RETRY_MS);
    reportStartup("設定已入列，等待 ACK 01");
  }
  if (phase==Phase::FeedbackWait || phase==Phase::EnableWait) {
    if (expired(Window)) { failStartup("等待設定 ACK 總期限逾時"); return; }
    if (ack==0) { failStartup("驅動器明確拒絕設定（ACK 00）"); return; }
    if (ack!=1) {
      if (!expired(Retry)) return;
      startTimer(Retry,TestConfig::STARTUP_ACK_RETRY_MS);
      if (startupAttempts<UINT16_MAX) ++startupAttempts;
      const bool feedback=phase==Phase::FeedbackWait;
      const uint8_t data[2]={1,1};
      const bool accepted=send(startupNode,startupCommand,data,feedback?2:1);
      reportStartup(accepted?"ACK 尚未回報，已重送設定":"ACK 尚未回報，重送遭 CAN 佇列拒收");
      return;
    }
    if (phase==Phase::FeedbackWait) queue(Phase::EnableQueue);
    else if (++initMotor<TestConfig::MOTOR_COUNT) queue(Phase::FeedbackQueue);
    else if constexpr (TestConfig::STARTUP_RETURN_ZERO) {
      currentStep=0; startPlan(Stage4::STARTUP_HOME,
        sizeof(Stage4::STARTUP_HOME)/sizeof(Stage4::STARTUP_HOME[0]));
      report("開機 91 尋原點，等待全部馬達開始與完成回報。");
    } else { phase=Phase::Ready; startupPhase="ready";
      report("啟用軸設定完成；開機不尋零，位置尚未確認。"); }
  }
  if (phase==Phase::ActionQueue) startAction();
  if (phase==Phase::Moving) {
    const auto &a=actions[actionIndex];
    const bool done=a.kind==Stage4::ActionKind::Stepper?!motors[motorIndex(a.node)].active:
      a.kind==Stage4::ActionKind::Servo?!servos[servoIndex(a.pin)].active:
      !motorsBusy() && !servosBusy();
    if (done) finishAction();
  }
  if (phase==Phase::Pause && expired(PauseTime)) finishAction();
}
void worker() { while (true) { Stage4Test::poll(); threads.delay(1); } }
bool available() { return phase==Phase::Ready && !uploading && !pendingCommit; }
bool uploadActive() {
  if (uploading && expired(UploadWindow)) uploading=false;
  return uploading;
}
void serviceSerial() {
  static char line[160]; static uint8_t used=0; static bool discard=false;
  for (uint8_t n=0;n<64 && Serial.available();++n) {
    const int c=Serial.read(); if (c<0) break;
    if (c=='\r' || c=='\n') {
      if (!discard && used) {
        line[used]=0;
        if (line[0]=='@') { handleSerialRequest(line); used=0; discard=false; continue; }
        char *command=strtok(line," \t");
        if (command && !strtok(nullptr," \t")) {
          for (char *p=command;*p;++p) if (*p>='A' && *p<='Z') *p+='a'-'A';
          bool accepted=true;
          if (!strcmp(command,"next")) accepted=Stage4Test::next();
          else if (!strcmp(command,"run")) accepted=Stage4Test::runAll();
          else if (!strcmp(command,"home")) accepted=Stage4Test::home();
          else if (!strcmp(command,"stop")) Stage4Test::stop();
          else if (!strcmp(command,"stage2")) accepted=Stage4Test::selectStage(2);
          else if (!strcmp(command,"stage3")) accepted=Stage4Test::selectStage(3);
          else if (!strcmp(command,"stage4")) accepted=Stage4Test::selectStage(4);
          else { Serial.println("指令：next / run / home / stop / stage2 / stage3 / stage4。"); command=nullptr; }
          if (command && !accepted) Serial.println("初始化、忙碌或停止中，未接受新動作。");
        } else Serial.println("指令格式錯誤。");
      } else if (discard) Serial.println("輸入無效或過長，已忽略。");
      used=0; discard=false;
    } else if (!discard) {
      if ((c<32 && c!='\t') || c>126 || used>=sizeof(line)-1) discard=true;
      else line[used++]=static_cast<char>(c);
    }
  }
}
}
namespace Stage4Test {
void begin() {
  Serial.begin(TestConfig::SERIAL_BAUD); canBus.begin(); canBus.setBaudRate(TestConfig::CAN_BAUD); canBus.enableFIFO();
  const auto &initial=TestConfig::SERVO_START_US;
  for (uint8_t i=0;i<TestConfig::SERVO_COUNT;++i) {
    servos[i].device.attach(TestConfig::SERVO_PINS[i],500,2500);
    if (!servos[i].device.attached()) { fail("伺服初始化失敗，已停止；請檢查後重啟。"); return; }
    servos[i].pulse=initial[i]; servos[i].device.writeMicroseconds(initial[i]);
  }
  timersReady=clockTimer.begin(tick,1000);
  if (!timersReady) { fail("計時器建立失敗，已停止；請重啟。"); return; }
  startTimer(Window,TestConfig::STARTUP_DELAY_SECONDS*1000U);
  {
    Threads::Suspend pause;
    threads.setDefaultTimeSlice(1); threads.setTimeSlice(0,1);
    workerReady=threads.addThread(worker,0,8192)>=0;
    if (!workerReady) { fail("執行緒啟動失敗，已停止；請重啟。"); return; }
  }
  threads.start();
}
void poll() {
  Threads::Scope lock(stateMutex); serviceStop(); if (phase==Phase::Fault) return;
  if (uploading && expired(UploadWindow)) { uploading=false; report("設定傳輸逾時，保留原設定。"); }
  canBus.events(); CAN_message_t m{};
  for (uint8_t n=0;n<32 && canBus.read(m);++n) receive(m);
  if (phase==Phase::Fault) return;
  for (uint8_t i=0;i<TestConfig::MOTOR_COUNT;++i) {
    auto &axis=motors[i]; if (!axis.active) continue;
    if ((!axis.started && expired(motorStartTimer(i))) || expired(motorFinishTimer(i))) {
      failMotor(i,!axis.started?"等待開始回報逾時":"等待完成回報逾時"); return;
    }
    if (axis.done) { axis.position=axis.target; axis.positionKnown=true; axis.active=false; }
  }
  updateServos();
  for (uint8_t n=0;n<16;++n) {
    const Phase before=phase; const size_t beforeAction=actionIndex; const uint8_t beforeStep=currentStep;
    serviceState();
    if (phase==Phase::Fault || (before==phase && beforeAction==actionIndex && beforeStep==currentStep)) break;
  }
}
void loop() {
  serviceSerial(); if (!workerReady) poll();
  static bool banner=false; static uint32_t printed=0;
  if (Serial) {
    if (!banner) {
      banner=true; Serial.println("第二／三／四關測試：啟用 ID6/7，開機等待雙軸尋原點完成。");
    }
    for (uint8_t n=0;n<16;++n) {
      Progress item{}; bool present;
      { Threads::Scope lock(stateMutex); present=progressCount!=0;
        if (present) { item=progress[progressHead]; progressHead=(progressHead+1)%PROGRESS_CAPACITY; --progressCount; }
      }
      if (!present) break;
      if (item.branch) Serial.printf("STEP%u-%u\n",unsigned(item.step),unsigned(item.branch));
      else Serial.printf("STEP%u\n",unsigned(item.step));
    }
    char message[sizeof(detailNotice)]{}; uint32_t version;
    { Threads::Scope lock(stateMutex);
      snprintf(message,sizeof(message),"%s",notice); version=noticeVersion;
    }
    if (version!=printed) { printed=version; Serial.println(message); }
  }
  threads.delay(1);
}
bool next() {
  Threads::Scope lock(stateMutex); if (!available() || debugMode) return false;
  job=Job::Next; startStep(completed==stepCount()?repeatFrom():completed+1); return true;
}
bool runAll() {
  Threads::Scope lock(stateMutex); if (!available() || debugMode) return false;
  completed=0; job=Job::Run; startStep(1); return true;
}
bool home() {
  Threads::Scope lock(stateMutex); if (!available() || debugMode) return false;
  job=Job::Home; currentStep=0;
  if (activeStage==2) startPlan(Stage4::HOME2,sizeof(Stage4::HOME2)/sizeof(Stage4::HOME2[0]));
  else if (activeStage==3) startPlan(Stage4::HOME3,sizeof(Stage4::HOME3)/sizeof(Stage4::HOME3[0]));
  else startPlan(Stage4::HOME,sizeof(Stage4::HOME)/sizeof(Stage4::HOME[0]));
  report(activeStage==4?"ID6 91 重新尋原點，完成後重設本關段數。":activeStage==2?"ID7 91 重新尋原點，完成後重設本關段數。":"第三關待機，沒有致動器動作。"); return true;
}
bool selectStage(uint8_t stage) {
  Threads::Scope lock(stateMutex);
  if ((stage!=2 && stage!=3 && stage!=4) || !available() || debugMode) return false;
  if (stage==activeStage && (stage==2 || !runtimePlan)) return true;
  activeStage=stage; runtimePlan=false; completed=currentStep=0; reportedBranch=0;
  pendingStage=0; progressHead=progressCount=0;
  if (activeStage==2) {job=Job::Next;startStep(1);report("第二關已啟用，自動執行第一段。");}
  else report("關卡已切換；NEXT 才開始動作。");
  return true;
}
void stop() { Threads::Scope lock(stateMutex); fail("已停止並停用全部馬達，取消後續動作；檢查後重啟。"); }
Status status() {
  Threads::Scope lock(stateMutex);
  const char *state=phase==Phase::Ready?"ready":phase==Phase::Fault?"fault":
    (phase==Phase::BootDelay || phase==Phase::FeedbackQueue || phase==Phase::FeedbackWait
      || phase==Phase::EnableQueue || phase==Phase::EnableWait)?"booting":"busy";
  return {debugMode,uploading,runtimePlan,state,completed,currentStep,reportedBranch,stepCount(),planVersion,
    pulseAt(2),pulseAt(5),pulseAt(3),activeStage,pendingStage,
    positionAt(4),positionAt(5),positionAt(6),
    startupPhase,startupNode,startupCommand,startupAttempts,knownMask(),
    pulseAt(4),pulseAt(6),pulseAt(9),positionAt(7)};
}
bool setDebug(bool enabled) {
  Threads::Scope lock(stateMutex); if (!available()) return false;
  if (enabled!=debugMode) { completed=currentStep=0; reportedBranch=0; progressHead=progressCount=0; }
  debugMode=enabled; return true;
}
bool debugStepper(int32_t position, uint16_t rpm, uint8_t acc, bool goHome, uint8_t node) {
  Threads::Scope lock(stateMutex);
  const auto a=Stage4::stepperFor(node,position,rpm,acc,false);
  if (!debugMode || activeStage==3 || uploading || phase==Phase::Fault || !Stage4::valid(a)) return false;
  if (!available() && !(job==Job::Debug || job==Job::DebugGoHome)) return false;
  const uint8_t i=static_cast<uint8_t>(motorIndex(node));
  if (motors[i].active || !startMotor(i,a,goHome)) return false;
  if (phase==Phase::Ready) { job=Job::Debug; currentStep=0; startPlan(DEBUG_WAIT,1); }
  return true;
}
bool debugServo(uint8_t pin, uint16_t angleTenths, uint32_t durationMs) {
  Threads::Scope lock(stateMutex);
  const int i=servoIndex(pin);
  if (!debugMode || activeStage==3 || uploading || phase==Phase::Fault || angleTenths>1800 || durationMs>60000 || i<0) return false;
  if (phase!=Phase::Ready && !(job==Job::Debug || job==Job::DebugGoHome)) return false;
  const int pulse=TestConfig::SERVO_MIN_US+(int32_t(angleTenths)*(TestConfig::SERVO_MAX_US-TestConfig::SERVO_MIN_US)+900)/1800;
  servoMove(static_cast<uint8_t>(i),pulse,durationMs);
  if (phase==Phase::Ready) { job=Job::Debug; currentStep=0; startPlan(DEBUG_WAIT,1); }
  return true;
}
bool planBegin(uint16_t steps, uint16_t count, uint8_t stage, uint8_t repeatStart) {
  Threads::Scope lock(stateMutex);
  if (!available() || debugMode || steps<1 || steps>254 || count<1 || count>MAX_RUNTIME_ACTIONS
      || (stage!=2 && stage!=3 && stage!=4) || repeatStart<1 || repeatStart>steps) return false;
  stagedStepCount=steps; stagedActionCount=count; stagedStage=stage; stagedRepeatFrom=repeatStart;
  memset(actionWritten,0,sizeof(actionWritten)); memset(stepWritten,0,sizeof(stepWritten));
  uploading=true; startTimer(UploadWindow,10000); return true;
}
bool planAction(uint16_t index, const Stage4::Action &a) {
  Threads::Scope lock(stateMutex);
  if (!uploadActive() || index>=stagedActionCount || !Stage4::validForStage(stagedStage,a) || a.substep<1 || a.substep>256) return false;
  runtimeActions[1-activeBank][index]=a; actionWritten[index]=true; startTimer(UploadWindow,10000); return true;
}
bool planStep(uint16_t index, uint16_t start, uint16_t count) {
  Threads::Scope lock(stateMutex);
  if (!uploadActive() || index>=stagedStepCount || !count || uint32_t(start)+count>stagedActionCount) return false;
  runtimeSteps[1-activeBank][index]={runtimeActions[1-activeBank]+start,count}; stepWritten[index]=true;
  startTimer(UploadWindow,10000); return true;
}
bool planCommit() {
  Threads::Scope lock(stateMutex); if (!uploadActive() || phase!=Phase::Ready) return false;
  size_t expected=0;
  for (uint16_t i=0;i<stagedActionCount;++i) if (!actionWritten[i]) return false;
  for (uint16_t i=0;i<stagedStepCount;++i) {
    const auto &s=runtimeSteps[1-activeBank][i];
    if (!stepWritten[i] || s.actions!=runtimeActions[1-activeBank]+expected) return false;
    uint16_t previous=0; uint8_t pending=0;
    for (size_t j=0;j<s.count;++j) {
      const auto &a=s.actions[j];
      if (!Stage4::validForStage(stagedStage,a)) return false;
      if (a.substep!=previous) {
        if (pending || a.substep!=previous+1) return false;
        previous=a.substep;
      } else if (!pending) return false;
      if (a.kind==Stage4::ActionKind::WaitAll) {
        if (!pending || !a.wait) return false;
        pending=0;
      } else if (!a.wait) {
        if (a.kind!=Stage4::ActionKind::Stepper && a.kind!=Stage4::ActionKind::Servo) return false;
        const uint8_t bit=a.kind==Stage4::ActionKind::Stepper?
          static_cast<uint8_t>(1U<<motorIndex(a.node)):
          static_cast<uint8_t>(1U<<(TestConfig::MOTOR_COUNT+servoIndex(a.pin)));
        if (pending&bit) return false;
        pending|=bit;
      } else if (pending) return false;
    }
    if (pending) return false;
    expected+=s.count;
  }
  if (expected!=stagedActionCount) return false;
  uploading=false;
  installRuntime(); completed=currentStep=0; reportedBranch=0; progressHead=progressCount=0;
  if (activeStage==2) {job=Job::Next;startStep(1);report("第二關 Web 設定已套用，自動執行第一段。");}
  else report("Web 設定已套用；NEXT / RUN 才啟動。");
  return true;
}
void planAbort() {
  Threads::Scope lock(stateMutex); uploading=false;
  if (pendingCommit) { pendingCommit=false; pendingStage=activeStage; }
}
}


