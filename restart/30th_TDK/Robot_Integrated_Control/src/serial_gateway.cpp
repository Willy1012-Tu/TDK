#include "serial_gateway.h"
#include "system_config.h"
#include "system_safety.h"
#include "mechanism_controller.h"
#include "chassis_controller.h"
#include "rc_controller.h"
#include "pwm_controller.h"
#include <Arduino.h>
#include <errno.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace {
char outgoing[SystemConfig::USB_QUEUE_BYTES]{};
uint16_t outputHead=0,outputCount=0;
char input[SystemConfig::USB_LINE_BYTES]{},pendingStopReply[80]{};
uint16_t inputCount=0;
bool inputOverflow=false;
uint32_t lastNotice=0;
uint8_t lastStage=0,lastStep=0;
uint16_t lastBranch=0;

size_t freeOutput() { return sizeof(outgoing)-outputCount; }
bool enqueue(const char *text) {
  const size_t length=strlen(text);
  if(length>freeOutput()) return false;
  for(size_t i=0;i<length;++i) outgoing[(outputHead+outputCount+i)%sizeof(outgoing)]=text[i];
  outputCount=static_cast<uint16_t>(outputCount+length);
  return true;
}
void flushOutput() {
  if(!Serial || !outputCount) return;
  const int available=Serial.availableForWrite();
  if(available<=0) return;
  size_t length=outputCount;
  if(length>SystemConfig::USB_TX_BUDGET) length=SystemConfig::USB_TX_BUDGET;
  if(length>static_cast<size_t>(available)) length=static_cast<size_t>(available);
  const size_t contiguous=sizeof(outgoing)-outputHead;
  if(length>contiguous) length=contiguous;
  const size_t written=Serial.write(reinterpret_cast<const uint8_t*>(outgoing+outputHead),length);
  outputHead=static_cast<uint16_t>((outputHead+written)%sizeof(outgoing));
  outputCount=static_cast<uint16_t>(outputCount-written);
}
struct JsonBuffer {
  char text[SystemConfig::USB_REPLY_BYTES]{};
  size_t used=0;
  bool valid=true;
  void add(const char *format,...) {
    if(!valid) return;
    va_list args;va_start(args,format);
    const int length=vsnprintf(text+used,sizeof(text)-used,format,args);
    va_end(args);
    if(length<0 || static_cast<size_t>(length)>=sizeof(text)-used) { valid=false;return; }
    used+=static_cast<size_t>(length);
  }
  void quoted(const char *value) {
    add("\"");
    for(const unsigned char *p=reinterpret_cast<const unsigned char*>(value?value:"");*p;++p) {
      if(*p=='"' || *p=='\\') add("\\%c",*p);
      else if(*p<0x20) add("\\u%04x",unsigned(*p));
      else add("%c",*p);
    }
    add("\"");
  }
};
const char *boolean(bool value) { return value?"true":"false"; }
void reply(long request,bool accepted,const char *message) {
  char text[128]{};
  snprintf(text,sizeof(text),"@%ld %s %s\n",request,accepted?"OK":"ERR",message);
  enqueue(text);
}
void statusReply(long request) {
  const auto m=MechanismController::status();
  const auto c=ChassisController::status();
  const auto r=RcController::status();
  const auto pwm=PwmController::status();
  JsonBuffer out;
  out.add("@%ld OK {\"protocol\":1,\"mode\":\"%s\",\"phase\":\"%s\",",request,m.debug?"debug":"normal",m.phase);
  out.add("\"firmware_id\":\"Robot_Integrated_Control\",\"firmware_revision\":\"bench-test-v7\",");
  out.add("\"bench_test_mode\":%s,\"motor_online_mask\":%u,\"motor_offline_mask\":%u,",
      boolean(SystemConfig::BENCH_TEST_MODE),unsigned(m.motorOnlineMask),unsigned(m.motorOfflineMask));
  out.add("\"startup_return_zero\":%s,\"startup_phase\":\"%s\",\"startup_node\":%u,\"startup_command\":%u,\"startup_attempts\":%u,\"position_known_mask\":%u,",
      boolean(SystemConfig::Mechanism::STARTUP_RETURN_ZERO),m.startupPhase,unsigned(m.startupNode),
      unsigned(m.startupCommand),unsigned(m.startupAttempts),unsigned(m.positionKnownMask));
  out.add("\"startup_current_ma\":%u,\"current4_ma\":%u,\"current5_ma\":%u,\"current6_ma\":%u,\"current_configured_mask\":%u,",
      unsigned(m.startupCurrentMa),unsigned(m.current4Ma),unsigned(m.current5Ma),
      unsigned(m.current6Ma),unsigned(m.currentConfiguredMask));
  out.add("\"startup_current_settings\":[");
  for(uint8_t i=0;i<SystemConfig::Mechanism::STARTUP_CURRENT_COUNT;++i) {
    const auto setting=SystemConfig::Mechanism::STARTUP_CURRENT_SETTINGS[i];
    out.add(i?",{\"node\":%u,\"milliamps\":%u}":"{\"node\":%u,\"milliamps\":%u}",unsigned(setting.node),unsigned(setting.milliamps));
  }
  out.add("],\"startup_home_ids\":[6],\"startup_hold_ids\":[4,5],");
  out.add("\"completed\":%u,\"step\":%u,\"branch\":%u,\"steps\":%u,",unsigned(m.completed),unsigned(m.step),unsigned(m.branch),unsigned(m.steps));
  out.add("\"uploading\":%s,\"runtime_plan\":%s,\"plan_version\":%lu,",boolean(m.uploading),boolean(m.runtimePlan),static_cast<unsigned long>(m.planVersion));
  out.add("\"pulse2\":%d,\"pulse3\":%d,\"pulse5\":%d,\"pulse4\":%d,\"pulse6\":%d,\"pulse9\":%d,",m.pulse2,m.pulse3,m.pulse5,m.pulse4,m.pulse6,m.pulse9);
  out.add("\"pulse2_base\":%d,\"stage1_trim_us\":%d,\"stage1_trim_allowed\":%s,",
      m.pulse2Base,m.stage1TrimUs,boolean(m.stage1TrimAllowed));
  out.add("\"position6_base\":%ld,\"stage4_trim_pulses\":%ld,\"stage4_trim_target\":%ld,"
      "\"stage4_trim_allowed\":%s,\"stage4_trim_moving\":%s,",
      static_cast<long>(m.position6Base),static_cast<long>(m.stage4TrimPulses),
      static_cast<long>(m.stage4TrimTarget),boolean(m.stage4TrimAllowed),boolean(m.stage4TrimMoving));
  out.add("\"position4_base\":%ld,\"position5_base\":%ld,\"stage2_trim_pulses\":%ld,"
      "\"stage2_trim_target4\":%ld,\"stage2_trim_target5\":%ld,"
      "\"stage2_trim_allowed\":%s,\"stage2_trim_moving\":%s,",
      static_cast<long>(m.position4Base),static_cast<long>(m.position5Base),static_cast<long>(m.stage2TrimPulses),
      static_cast<long>(m.stage2TrimTarget4),static_cast<long>(m.stage2TrimTarget5),
      boolean(m.stage2TrimAllowed),boolean(m.stage2TrimMoving));
  out.add("\"motor_recovery_mask\":%u,\"stage_recovery_required\":%s,\"mechanism_warning\":",
      unsigned(m.motorRecoveryMask),boolean(m.stageRecoveryRequired));
  out.quoted(m.localWarning);
  out.add(",\"id6_position_max\":%ld,\"fruit_position_min\":%ld,\"fruit_position_max\":%ld,",
      static_cast<long>(SystemConfig::ID6_POSITION_MAX),
      static_cast<long>(SystemConfig::Fruit::POSITION_MIN),static_cast<long>(SystemConfig::Fruit::POSITION_MAX));
  out.add("\"pulse10\":%d,\"pulse11\":%d,\"pulse12\":%d,",m.pulse10,m.pulse11,m.pulse12);
  out.add("\"pin2\":2,\"pin3\":3,\"pin5\":5,\"pin4\":4,\"pin6\":6,\"pin9\":9,\"pin10\":10,\"pin11\":11,\"pin12\":12,\"servo_pins\":[");
  for(uint8_t i=0;i<SystemConfig::Mechanism::SERVO_COUNT;++i)
    out.add(i?",%u":"%u",unsigned(SystemConfig::Mechanism::SERVO_PINS[i]));
  out.add("],\"servo_min_us\":%d,\"servo_max_us\":%d,\"motor_id\":%u,\"motor_ids\":[",SystemConfig::Mechanism::SERVO_MIN_US,SystemConfig::Mechanism::SERVO_MAX_US,unsigned(SystemConfig::Mechanism::MOTOR_ID));
  for(uint8_t i=0;i<SystemConfig::Mechanism::MOTOR_COUNT;++i)
    out.add(i?",%u":"%u",unsigned(SystemConfig::Mechanism::MOTOR_IDS[i]));
  out.add("],\"stage3_enabled\":%s,",boolean(SystemConfig::Mechanism::STAGE3_ENABLED));
  out.add("\"stage\":%u,\"pending_stage\":%u,\"stage_ids\":[",unsigned(m.stage),unsigned(m.pendingStage));
  bool firstStage = true;
  for (uint8_t stage = 1; stage <= SystemConfig::Mechanism::STAGE_COUNT; ++stage) {
    if (!MechanismConfig::stageEnabled(stage)) continue;
    out.add(firstStage ? "%u" : ",%u",unsigned(stage));
    firstStage = false;
  }
  out.add("],\"stage1_prop_ids\":[1,2,3,4,5],");
  out.add("\"prop\":%u,\"pending_prop\":%u,\"prop_complete\":%s,",unsigned(m.prop),unsigned(m.pendingProp),boolean(m.propComplete));
  out.add("\"max_runtime_actions\":%u,\"previous_available\":%s,\"progress\":%llu,",unsigned(MechanismController::MAX_RUNTIME_ACTIONS),boolean(m.previousAvailable),static_cast<unsigned long long>(m.progress));
  out.add("\"position4\":%ld,\"position5\":%ld,\"position6\":%ld,",static_cast<long>(m.position4),static_cast<long>(m.position5),static_cast<long>(m.position6));
  out.add("\"cylinder\":{\"available\":true,\"action\":%u,\"active\":%s,"
      "\"duration_ms\":%lu,\"remaining_ms\":%lu,\"in1_pin\":%u,\"in2_pin\":%u,"
      "\"in1_high\":%s,\"in2_high\":%s,\"reference_action\":%u,\"reference_known\":%s,\"fully_retracted\":%s},",
      unsigned(m.cylinder.action),boolean(m.cylinder.active),
      static_cast<unsigned long>(m.cylinder.durationMs),static_cast<unsigned long>(m.cylinder.remainingMs),
      unsigned(SystemConfig::Cylinder::IN1_PIN),unsigned(SystemConfig::Cylinder::IN2_PIN),
      boolean(m.cylinder.in1High),boolean(m.cylinder.in2High),
      unsigned(m.cylinder.referenceAction),boolean(m.cylinder.referenceKnown),boolean(m.cylinder.fullyRetracted));
  out.add("\"pwm\":{\"available\":true,\"pin\":%u,\"value\":%u,\"max_value\":%u,"
      "\"step_interval_ms\":%lu,\"default_max_value\":%u,\"default_step_interval_ms\":%lu,"
      "\"min_step_interval_ms\":%lu,\"max_step_interval_ms\":%lu,"
      "\"strobe_enabled\":%s,\"rc_controlled\":%s,\"strobe_interval_ms\":%lu,"
      "\"strobe_value\":%u,\"rc_channel\":%u},",
      unsigned(pwm.pin),unsigned(pwm.value),unsigned(pwm.maxValue),
      static_cast<unsigned long>(pwm.stepIntervalMs),unsigned(SystemConfig::Pwm::MAX_VALUE),
      static_cast<unsigned long>(SystemConfig::Pwm::STEP_INTERVAL_MS),
      static_cast<unsigned long>(SystemConfig::Pwm::MIN_STEP_INTERVAL_MS),
      static_cast<unsigned long>(SystemConfig::Pwm::MAX_STEP_INTERVAL_MS),
      boolean(pwm.strobeEnabled),boolean(pwm.rcControlled),
      static_cast<unsigned long>(SystemConfig::Pwm::STROBE_INTERVAL_MS),
      unsigned(SystemConfig::Pwm::STROBE_VALUE),unsigned(SystemConfig::Rc::LIGHT_CHANNEL));
  out.add("\"stopped\":%s,\"stop_reason\":",boolean(SystemSafety::stopped()));
  out.quoted(SystemSafety::reason());
  out.add(",\"chassis\":{\"state\":\"%s\",\"healthy\":%s,\"emergency\":%s,\"reason\":",c.modeName,boolean(c.healthy),boolean(c.emergencyLatched));
  out.quoted(c.reason);
  out.add(",\"direction_stage\":%u,\"wheels\":[",unsigned(c.directionStage));
  for(uint8_t i=0;i<4;++i) {
    if(i) out.add(",");
    out.add("{\"node\":%u,\"bus\":%u,\"heartbeat_age_ms\":%lu,\"error\":%lu,\"axis_state\":%u,\"velocity\":%.3f}",
      unsigned(SystemConfig::Chassis::NODE_ID[i]),unsigned(SystemConfig::Chassis::CAN_BUS[i]),
      static_cast<unsigned long>(c.axes[i].heartbeatAge),static_cast<unsigned long>(c.axes[i].error),unsigned(c.axes[i].state),static_cast<double>(c.wheel[i]));
  }
  out.add("]");
  out.add(",\"motion\":{\"available\":true,\"editable\":%s,\"ramp_enabled\":%s,\"profiles\":true,\"profile_id\":%u,"
      "\"accel\":%.3f,\"decel\":%.3f,\"wheelbase_mm\":%.3f,\"track_mm\":%.3f,"
      "\"icr_mode\":\"directional\",\"icr_x_mm\":%.3f,\"icr_left_y_mm\":%.3f,\"icr_right_y_mm\":%.3f,\"target\":[%.3f,%.3f,%.3f,%.3f]}",
      boolean(c.motionEditable),boolean(c.motion.rampEnabled),unsigned(c.processionMode ? 1 : 0),
      static_cast<double>(c.motion.acceleration),static_cast<double>(c.motion.deceleration),
      static_cast<double>(c.motion.wheelbaseMm),static_cast<double>(c.motion.trackWidthMm),
      static_cast<double>(c.motion.icrXMm),static_cast<double>(c.motion.icrLeftYMm),static_cast<double>(c.motion.icrRightYMm),
      static_cast<double>(c.targetWheel[0]),static_cast<double>(c.targetWheel[1]),
      static_cast<double>(c.targetWheel[2]),static_cast<double>(c.targetWheel[3]));
  out.add("},\"rc\":{\"connected\":%s,\"valid\":%s,\"age_ms\":%lu,\"stage\":%u,"
      "\"light_strobe_enabled\":%s,\"stage1_mode\":%s,\"stage1_trim_us\":%d,\"stage4_trim_pulses\":%ld,\"stage2_trim_pulses\":%ld,\"procession_mode\":%s,\"prop\":%u,\"switch_position\":%u,\"channels\":[",
      boolean(r.link),boolean(r.valid),static_cast<unsigned long>(r.ageMs),unsigned(r.selectedStage),
      boolean(r.lightStrobeEnabled),boolean(r.stage1Mode),m.stage1TrimAllowed ? r.stage1TrimUs : 0,
      static_cast<long>(m.stage4TrimAllowed ? r.stage4TrimPulses : 0),
      static_cast<long>(m.stage2TrimAllowed ? r.stage2TrimPulses : 0),boolean(r.processionMode),unsigned(r.selectedProp),unsigned(r.switchPosition));
  for(uint8_t i=0;i<16;++i) out.add(i?",%d":"%d",r.channels[i]);
  out.add("]}}\n");
  if(out.valid) enqueue(out.text);else reply(request,false,"status_too_large");
}
bool number(const char *text,long low,long high,long &value) {
  if(!text || !*text) return false;
  for(const char *p=text;*p;++p) {
    if((*p<'0' || *p>'9') && !(p==text && *p=='-')) return false;
  }
  errno=0;char *end=nullptr;value=strtol(text,&end,10);
  return errno!=ERANGE && end && !*end && value>=low && value<=high;
}
void request(char *line) {
  char *tokens[15]{};size_t count=0;bool tooMany=false;
  for(char *p=strtok(line," \t");p;p=strtok(nullptr," \t")) {
    if(count==15) {tooMany=true;break;}
    tokens[count++]=p;
  }
  long id=0;
  if(count<2 || tokens[0][0]!='@' || !number(tokens[0]+1,1,1000000000,id)) return;
  const char *command=tokens[1];
  if(!tooMany && count==2 && !strcmp(command,"stop")) {
    SystemSafety::stop("USB STOP");
    char ack[80]{};
    snprintf(ack,sizeof(ack),"@%ld OK accepted\n",id);
    if(!enqueue(ack)) memcpy(pendingStopReply,ack,sizeof(ack));
    return;
  }
  if(freeOutput()<SystemConfig::USB_REPLY_BYTES) { reply(id,false,"usb_backpressure");return; }
  if(tooMany) { reply(id,false,"invalid_arguments");return; }
  if((!strcmp(command,"status") || !strcmp(command,"hello")) && count==2) {statusReply(id);return;}
  bool accepted=false;
  long values[12]{};
  auto args=[&](size_t expected,const long *low,const long *high) {
    if(count!=expected+2) return false;
    for(size_t i=0;i<expected;++i) if(!number(tokens[i+2],low[i],high[i],values[i])) return false;
    return true;
  };
  if(!strcmp(command,"chassis.motion")) {
    // enabled accel decel wheelbase_mm track_mm icr_x_mm left_y right_y [expected_profile].
    // Legacy 8-field clients may modify normal only. GUI appends explicit bank0/1.
    const long low[]={0,1,1,1,1,-5000,-5000,-5000,0},high[]={1,10000,10000,5000,5000,5000,5000,5000,1};
    if(args(8,low,high) || args(9,low,high)) accepted=ChassisController::configureMotion({
        values[0]!=0,static_cast<float>(values[1]),static_cast<float>(values[2]),
        static_cast<float>(values[3]),static_cast<float>(values[4]),
        static_cast<float>(values[5]),static_cast<float>(values[6]),static_cast<float>(values[7])},
        count == 11 ? static_cast<uint8_t>(values[8]) : uint8_t(0));
  } else if(!strcmp(command,"pwm.set")) {
    // PWM tuning is independent of mechanism locks and latched motion STOP.
    // Parse and range-check both integers before narrowing to the controller types.
    const long low[]={0,SystemConfig::Pwm::MIN_STEP_INTERVAL_MS},high[]={255,SystemConfig::Pwm::MAX_STEP_INTERVAL_MS};
    if(args(2,low,high)) accepted=PwmController::configure(
        static_cast<uint16_t>(values[0]),static_cast<uint32_t>(values[1]));
  } else if(!strcmp(command,"mode") && count==3 && (!strcmp(tokens[2],"normal") || !strcmp(tokens[2],"debug")))
    accepted=MechanismController::setDebug(!strcmp(tokens[2],"debug"));
  else if(!strcmp(command,"next") && count==2) accepted=MechanismController::next();
  else if(!strcmp(command,"prev") && count==2) accepted=MechanismController::previous();
  else if(!strcmp(command,"run") && count==2) accepted=MechanismController::runAll();
  else if(!strcmp(command,"home") && count==2) accepted=MechanismController::home();
  else if(!strcmp(command,"clear") && count==2) {ChassisController::clearErrors();accepted=true;}
  else if(!strcmp(command,"stage")) {
    const long low[]={1,0},high[]={6,5};
    if(count==3 && args(1,low,high)) accepted=MechanismController::selectStage(static_cast<uint8_t>(values[0]));
    else if(args(2,low,high)) accepted=MechanismController::selectProfile(static_cast<uint8_t>(values[0]),static_cast<uint8_t>(values[1]));
  } else if(!strcmp(command,"prop")) {
    const long low[]={1},high[]={5};
    if(args(1,low,high)) accepted=MechanismController::triggerProp(static_cast<uint8_t>(values[0]));
  } else if(!strcmp(command,"stage1.exit")) {
    const long low[]={2},high[]={6};
    if(count==2) accepted=MechanismController::exitStage1(4);
    else if(args(1,low,high)) accepted=MechanismController::exitStage1(static_cast<uint8_t>(values[0]));
  } else if(!strcmp(command,"debug.gohome")) {
    const long low[]={6},high[]={6};
    if(count==2) accepted=MechanismController::debugStepper(0,SystemConfig::Mechanism::HOME_RPM,SystemConfig::Mechanism::HOME_ACC,true);
    else if(args(1,low,high)) accepted=MechanismController::debugStepper(0,SystemConfig::Mechanism::HOME_RPM,SystemConfig::Mechanism::HOME_ACC,true,static_cast<uint8_t>(values[0]));
  } else if(!strcmp(command,"debug.move")) {
    const long low[]={-8388608,1,0},high[]={8388607,3000,255};
    const long nodeLow[]={4,-8388608,1,0},nodeHigh[]={6,8388607,3000,255};
    if(count==5 && args(3,low,high)) accepted=MechanismController::debugStepper(static_cast<int32_t>(values[0]),static_cast<uint16_t>(values[1]),static_cast<uint8_t>(values[2]));
    else if(args(4,nodeLow,nodeHigh)) accepted=MechanismController::debugStepper(static_cast<int32_t>(values[1]),static_cast<uint16_t>(values[2]),static_cast<uint8_t>(values[3]),false,static_cast<uint8_t>(values[0]));
  } else if(!strcmp(command,"debug.servo")) {
    const long low[]={0,0,0},high[]={39,1800,60000};
    if(args(3,low,high)) accepted=MechanismController::debugServo(static_cast<uint8_t>(values[0]),static_cast<uint16_t>(values[1]),static_cast<uint32_t>(values[2]));
  } else if(!strcmp(command,"debug.cylinder")) {
    const long low[]={0,0},high[]={2,SystemConfig::Cylinder::MAX_DURATION_MS};
    if(args(2,low,high)) accepted=MechanismController::debugCylinder(
        static_cast<MechanismConfig::CylinderAction>(values[0]),static_cast<uint32_t>(values[1]));
  } else if(!strcmp(command,"plan.begin")) {
    const long low[]={1,1},high[]={254,MechanismController::MAX_RUNTIME_ACTIONS};
    const long profileLow[]={1,1,1,1,0},profileHigh[]={254,MechanismController::MAX_RUNTIME_ACTIONS,6,254,5};
    if(count==4 && args(2,low,high)) accepted=MechanismController::planBegin(static_cast<uint16_t>(values[0]),static_cast<uint16_t>(values[1]));
    else if(count==6 && args(4,profileLow,profileHigh)) accepted=MechanismController::planBegin(static_cast<uint16_t>(values[0]),static_cast<uint16_t>(values[1]),static_cast<uint8_t>(values[2]),static_cast<uint8_t>(values[3]));
    else if(args(5,profileLow,profileHigh)) accepted=MechanismController::planBegin(static_cast<uint16_t>(values[0]),static_cast<uint16_t>(values[1]),static_cast<uint8_t>(values[2]),static_cast<uint8_t>(values[3]),static_cast<uint8_t>(values[4]));
  } else if(!strcmp(command,"plan.action")) {
    // index kind position rpm acc pin pulse ms wait substep [node [cylinder_action]]
    // Existing kinds keep their old format; Cylinder requires the explicit enum field.
    const long low[]={0,0,-8388608,0,0,0,0,0,0,1,0,0};
    const long high[]={MechanismController::MAX_RUNTIME_ACTIONS-1,4,8388607,3000,255,39,2500,60000,1,256,6,2};
    const bool parsed=(count==12 && args(10,low,high)) ||
                      (count==13 && args(11,low,high)) ||
                      (count==14 && args(12,low,high));
    const bool cylinderKind=values[1]==static_cast<long>(MechanismConfig::ActionKind::Cylinder);
    if(parsed && (cylinderKind ? count==14 : count!=14)) {
      MechanismConfig::Action action{static_cast<MechanismConfig::ActionKind>(values[1]),
        static_cast<int32_t>(values[2]),static_cast<uint16_t>(values[3]),static_cast<uint8_t>(values[4]),
        static_cast<uint8_t>(values[5]),static_cast<int>(values[6]),static_cast<uint32_t>(values[7]),
        values[8]!=0,static_cast<uint16_t>(values[9]),static_cast<uint8_t>(count>=13?values[10]:6),
        cylinderKind ? static_cast<MechanismConfig::CylinderAction>(values[11]) : MechanismConfig::CylinderAction::Stop};
      accepted=MechanismController::planAction(static_cast<uint16_t>(values[0]),action);
    }
  } else if(!strcmp(command,"plan.step")) {
    const long low[]={0,0,1},high[]={253,MechanismController::MAX_RUNTIME_ACTIONS-1,MechanismController::MAX_RUNTIME_ACTIONS};
    if(args(3,low,high)) accepted=MechanismController::planStep(static_cast<uint16_t>(values[0]),static_cast<uint16_t>(values[1]),static_cast<uint16_t>(values[2]));
  } else if(!strcmp(command,"plan.commit") && count==2) accepted=MechanismController::planCommit();
  else if(!strcmp(command,"plan.abort") && count==2) {MechanismController::planAbort();accepted=true;}
  reply(id,accepted,accepted?"accepted":"invalid_or_busy");
}
void plainCommand(char *line) {
  if(!strcmp(line,"stop")) {SystemSafety::stop("Serial STOP");return;}
  if(freeOutput()<SystemConfig::USB_REPLY_BYTES) return;
  if(!strcmp(line,"next")) MechanismController::next();
  else if(!strcmp(line,"prev")) MechanismController::previous();
  else if(!strcmp(line,"run")) MechanismController::runAll();
  else if(!strcmp(line,"home")) MechanismController::home();
  else if(!strcmp(line,"clear")) ChassisController::clearErrors();
  else if(strlen(line)==6 && !memcmp(line,"stage",5) && line[5]>='1' && line[5]<='6')
    MechanismController::selectStage(static_cast<uint8_t>(line[5]-'0'));
}
void emitNotice() {
  if(!Serial || freeOutput()<SystemConfig::USB_REPLY_BYTES+256U) return;
  const auto state=MechanismController::status();
  if(state.noticeVersion!=lastNotice) {
    char line[256]{};snprintf(line,sizeof(line),"# %s\n",state.notice);
    if(enqueue(line)) lastNotice=state.noticeVersion;
  }

  if(state.stage!=lastStage || state.step!=lastStep || state.branch!=lastBranch) {
    char line[80]{};
    snprintf(line,sizeof(line),"# STAGE%u STEP%u-%u\n",unsigned(state.stage),unsigned(state.step),unsigned(state.branch));
    if(enqueue(line)) {lastStage=state.stage;lastStep=state.step;lastBranch=state.branch;}
  }
}
}
namespace SerialGateway {
void begin() {
  outputHead=outputCount=inputCount=0;inputOverflow=false;
  pendingStopReply[0]=0;lastNotice=0;lastStage=lastStep=0;lastBranch=0;
}
void poll() {
  flushOutput();
  if(pendingStopReply[0] && enqueue(pendingStopReply)) pendingStopReply[0]=0;
  // Read a bounded number of bytes even under USB backpressure; STOP still parses.
  for(uint16_t n=0;n<SystemConfig::USB_RX_BUDGET && Serial.available()>0;++n) {
    const int raw=Serial.read();if(raw<0) break;
    const char value=static_cast<char>(raw);
    if(value=='\r' || value=='\n') {
      if(inputCount && !inputOverflow) {
        input[inputCount]=0;
        if(input[0]=='@') request(input);else plainCommand(input);
      }
      inputCount=0;inputOverflow=false;
    } else if(!inputOverflow) {
      if(inputCount+1U>=sizeof(input)) inputOverflow=true;
      else input[inputCount++]=value;
    }
  }
  emitNotice();
}
}
