#include "serial_protocol.h"
#include "test_controller.h"
#include <Arduino.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

namespace {
bool number(const char *s, long low, long high, long &value) {
  if (!s || !*s) return false;
  for (const char *p=s; *p; ++p) if ((*p<'0' || *p>'9') && !(p==s && *p=='-')) return false;
  char *end=nullptr; value=strtol(s,&end,10);
  return end && !*end && value>=low && value<=high;
}
}
void handleSerialRequest(char *line) {
  char *tokens[15]{}; size_t count=0;
  for (char *p=strtok(line," \t"); p && count<15; p=strtok(nullptr," \t")) tokens[count++]=p;
  long request=0;
  if (count<2 || tokens[0][0]!='@' || !number(tokens[0]+1,1,1000000000,request)) return;
  const char *command=tokens[1]; bool accepted=false; const char *error="invalid_or_busy";
  long values[11]{};
  auto args=[&](size_t expected, const long *low, const long *high) {
    if (count!=expected+2) return false;
    for (size_t i=0; i<expected; ++i) if (!number(tokens[i+2],low[i],high[i],values[i])) return false;
    return true;
  };
  if ((!strcmp(command,"status") || !strcmp(command,"hello")) && count==2) {
    const auto s=Stage4Test::status();
    Serial.printf("@%ld OK {\"protocol\":1,\"mode\":\"%s\",\"phase\":\"%s\",",request,s.debug?"debug":"normal",s.phase);
    Serial.printf("\"firmware_id\":\"Stage4_Mechanism_Test\",\"firmware_revision\":\"stage2-id67-home-v1\",");
    Serial.printf("\"completed\":%u,\"step\":%u,\"branch\":%u,\"steps\":%u,\"uploading\":%s,\"runtime_plan\":%s,\"plan_version\":%lu,",
      unsigned(s.completed),unsigned(s.step),unsigned(s.branch),unsigned(s.steps),
      s.uploading?"true":"false",s.runtimePlan?"true":"false",static_cast<unsigned long>(s.planVersion));
    Serial.printf("\"pulse2\":%d,\"pulse3\":%d,\"pulse5\":%d,\"pulse4\":%d,\"pulse6\":%d,\"pulse9\":%d,",
      s.pulse2,s.pulse3,s.pulse5,s.pulse4,s.pulse6,s.pulse9);
    Serial.printf("\"pin2\":2,\"pin5\":5,\"pin4\":4,\"pin6\":6,\"pin9\":9,\"servo_pins\":[");
    for(uint8_t i=0;i<TestConfig::SERVO_COUNT;++i)
      Serial.printf(i?",%u":"%u",unsigned(TestConfig::SERVO_PINS[i]));
    Serial.printf("],\"servo_min_us\":%d,\"servo_max_us\":%d,\"motor_id\":%u,\"motor_ids\":[",
      TestConfig::SERVO_MIN_US,TestConfig::SERVO_MAX_US,unsigned(TestConfig::MOTOR_ID));
    for(uint8_t i=0;i<TestConfig::MOTOR_COUNT;++i)
      Serial.printf(i?",%u":"%u",unsigned(TestConfig::MOTOR_IDS[i]));
    Serial.printf("],\"stage_ids\":[2,3,4],\"stage3_enabled\":%s,\"stage\":%u,\"pending_stage\":%u,",
      TestConfig::STAGE3_ENABLED?"true":"false",unsigned(s.stage),unsigned(s.pendingStage));
    Serial.printf("\"position4\":%ld,\"position5\":%ld,\"position6\":%ld,\"position7\":%ld,",
      static_cast<long>(s.position4),static_cast<long>(s.position5),static_cast<long>(s.position6),static_cast<long>(s.position7));
    Serial.printf("\"startup_return_zero\":%s,\"startup_phase\":\"%s\",\"startup_node\":%u,\"startup_command\":%u,\"startup_attempts\":%u,\"position_known_mask\":%u}\n",
      TestConfig::STARTUP_RETURN_ZERO?"true":"false",s.startupPhase,unsigned(s.startupNode),
      unsigned(s.startupCommand),unsigned(s.startupAttempts),unsigned(s.positionKnownMask));
    return;
  }
  if (!strcmp(command,"mode") && count==3 && (!strcmp(tokens[2],"normal") || !strcmp(tokens[2],"debug"))) accepted=Stage4Test::setDebug(!strcmp(tokens[2],"debug"));
  else if (!strcmp(command,"next") && count==2) accepted=Stage4Test::next();
  else if (!strcmp(command,"run") && count==2) accepted=Stage4Test::runAll();
  else if (!strcmp(command,"home") && count==2) accepted=Stage4Test::home();
  else if (!strcmp(command,"stop") && count==2) { Stage4Test::stop(); accepted=true; }
  else if (!strcmp(command,"stage")) {
    const long low[]={2},high[]={4};
    if (args(1,low,high)) accepted=Stage4Test::selectStage(uint8_t(values[0]));
  } else if (!strcmp(command,"debug.gohome")) {
    const long low[]={6},high[]={7};
    if (count==2) accepted=Stage4Test::debugStepper(0,TestConfig::HOME_RPM,TestConfig::HOME_ACC,true);
    else if (args(1,low,high)) accepted=Stage4Test::debugStepper(0,TestConfig::HOME_RPM,TestConfig::HOME_ACC,true,uint8_t(values[0]));
  }
  else if (!strcmp(command,"debug.move")) {
    const long low[]={-8388607,1,0},high[]={8388607,3000,255};
    const long nodeLow[]={6,-8388607,1,0},nodeHigh[]={7,8388607,3000,255};
    if (count==5 && args(3,low,high)) accepted=Stage4Test::debugStepper(int32_t(values[0]),uint16_t(values[1]),uint8_t(values[2]));
    else if (args(4,nodeLow,nodeHigh)) accepted=Stage4Test::debugStepper(int32_t(values[1]),uint16_t(values[2]),uint8_t(values[3]),false,uint8_t(values[0]));
  } else if (!strcmp(command,"debug.servo")) {
    const long low[]={0,0,0},high[]={39,1800,60000};
    if (args(3,low,high)) accepted=Stage4Test::debugServo(uint8_t(values[0]),uint16_t(values[1]),uint32_t(values[2]));
  } else if (!strcmp(command,"plan.begin")) {
    const long low[]={1,1},high[]={254,Stage4Test::MAX_RUNTIME_ACTIONS};
    const long profileLow[]={1,1,2,1},profileHigh[]={254,Stage4Test::MAX_RUNTIME_ACTIONS,4,254};
    if (count==4 && args(2,low,high)) accepted=Stage4Test::planBegin(uint16_t(values[0]),uint16_t(values[1]));
    else if (args(4,profileLow,profileHigh)) accepted=Stage4Test::planBegin(uint16_t(values[0]),uint16_t(values[1]),uint8_t(values[2]),uint8_t(values[3]));
  } else if (!strcmp(command,"plan.action")) {
    // index kind position rpm acc pin pulse ms wait substep [node]
    const long low[]={0,0,-8388607,0,0,0,0,0,0,1,0},high[]={Stage4Test::MAX_RUNTIME_ACTIONS-1,3,8388607,3000,255,39,2500,60000,1,256,7};
    if ((count==12 && args(10,low,high)) || args(11,low,high)) {
      Stage4::Action a{static_cast<Stage4::ActionKind>(values[1]),int32_t(values[2]),uint16_t(values[3]),uint8_t(values[4]),uint8_t(values[5]),int(values[6]),uint32_t(values[7]),values[8]!=0,uint16_t(values[9]),uint8_t(count==13?values[10]:6)};
      accepted=Stage4Test::planAction(uint16_t(values[0]),a);
    }
  } else if (!strcmp(command,"plan.step")) {
    const long low[]={0,0,1},high[]={253,Stage4Test::MAX_RUNTIME_ACTIONS-1,Stage4Test::MAX_RUNTIME_ACTIONS};
    if (args(3,low,high)) accepted=Stage4Test::planStep(uint16_t(values[0]),uint16_t(values[1]),uint16_t(values[2]));
  } else if (!strcmp(command,"plan.commit") && count==2) accepted=Stage4Test::planCommit();
  else if (!strcmp(command,"plan.abort") && count==2) { Stage4Test::planAbort(); accepted=true; }
  Serial.printf("@%ld %s %s\n",request,accepted?"OK":"ERR",accepted?"accepted":error);
}
