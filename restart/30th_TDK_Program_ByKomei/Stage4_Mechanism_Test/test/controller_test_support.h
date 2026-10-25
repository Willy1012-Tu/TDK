#pragma once
#include "test_controller.h"
#include "test_config.h"
#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <IntervalTimer.h>
#include <Servo.h>
#include <TeensyThreads.h>
#include <iostream>
#include <stdexcept>
#include <string>
#define CHECK(x) do { if (!(x)) throw std::runtime_error(std::to_string(__LINE__) + ": " + #x); } while (0)
namespace TestSupport {
using namespace Stage4Test;
inline void pump(unsigned n=8) { while(n--) poll(); }
inline void advance(unsigned ms) { Mock::advance(ms); pump(); }
inline void reply(int command,int status,uint8_t node=6,int bus=1,bool bad=false) {
  CAN_message_t f{}; f.id=node; f.len=3; f.buf[0]=uint8_t(command); f.buf[1]=uint8_t(status);
  f.buf[2]=uint8_t(node+command+status+(bad?1:0)); Mock::incoming[bus].push_back(f);
}
inline void finish(uint8_t node=6,int command=0xFE) { reply(command,1,node); reply(command,2,node); pump(); }
inline int count(int command,int node=-1) {
  int n=0; for(const auto &f:Mock::sent) if(f.msg.buf[0]==command && (node<0 || f.msg.id==uint32_t(node))) ++n; return n;
}
inline int disabled(int node=-1) {
  int n=0; for(const auto &f:Mock::sent) if(f.msg.buf[0]==0xF3 && f.msg.buf[1]==0 && (node<0 || f.msg.id==uint32_t(node))) ++n; return n;
}
inline CAN_message_t lastMove(int node=6) {
  for(auto f=Mock::sent.rbegin();f!=Mock::sent.rend();++f) if(f->msg.buf[0]==0xFE && f->msg.id==uint32_t(node)) return f->msg;
  throw std::runtime_error("missing FE position frame");
}
inline int target(int node=6) {
  const auto f=lastMove(node); const int raw=(f.buf[4]<<16)|(f.buf[5]<<8)|f.buf[6]; return (raw&0x800000)?raw-0x1000000:raw;
}
inline void checkFrames() {
  for(const auto &f:Mock::sent) {
    CHECK(f.bus==1 && (f.msg.id==6 || f.msg.id==7) && !f.msg.flags.extended && !f.msg.flags.remote);
    uint8_t sum=uint8_t(f.msg.id); for(int i=0;i<f.msg.len-1;++i) sum+=f.msg.buf[i]; CHECK(sum==f.msg.buf[f.msg.len-1]);
  }
}
inline void continueStartup() {
  size_t read=0;
  for(int round=0;round<100;++round) {
    while(read<Mock::sent.size()) {
      const auto f=Mock::sent[read++].msg;
      if(f.buf[0]==0x8C || (f.buf[0]==0xF3 && f.buf[1]==1)) reply(f.buf[0],1,uint8_t(f.id));
      else if(f.buf[0]==0x91) {reply(0x91,1,uint8_t(f.id));reply(0x91,2,uint8_t(f.id));}
    }
    pump();
    if(std::string(status().phase)=="ready") return;
  }
  throw std::runtime_error("startup did not reach dual-HOME Ready");
}
inline void startup() {
  begin(); pump(); advance(TestConfig::STARTUP_DELAY_SECONDS*1000U+1U); continueStartup();
}
inline void ready() {
  startup(); CHECK(Mock::workers.size()==1 && Mock::started && Mock::baud[1]==1000000 && Mock::baud[2]==0);
  CHECK(count(0x8C,6)==1 && count(0xF3,6)==1 && count(0x8C,7)==1 && count(0xF3,7)==1);
  CHECK(count(0x91,6)==1 && count(0x91,7)==1 && count(0xFE)==0 && Mock::sent.size()==6);
  for(uint8_t i=0;i<TestConfig::SERVO_COUNT;++i) {
    const auto pin=TestConfig::SERVO_PINS[i];
    CHECK(Mock::attachAttempts[pin]==1 && Mock::servo[pin]==TestConfig::SERVO_START_US[i]);
  }
  CHECK(Mock::attachAttempts[3]==0 && Mock::servo[3]==0);
  CHECK(status().position4==0 && status().position5==0 && status().position6==0 && status().position7==0 && status().positionKnownMask==12);
  CHECK(status().pulse3==0 && status().pulse4==2000 && status().pulse6==1400 && status().pulse9==1800);
  checkFrames(); Mock::sent.clear(); Mock::servoWrites.clear();
}
inline void nextStep() { CHECK(next()); pump(); }
inline void serial(const std::string &s) { Serial.input=s; while(!Serial.input.empty()) Stage4Test::loop(); pump(); }
inline void request(const std::string &s,bool accepted=true) {
  Serial.output.clear(); serial("@17 "+s+"\n"); CHECK(Serial.output.find(accepted?"@17 OK":"@17 ERR")!=std::string::npos);
}
inline void fault() {
  CHECK(!next() && !runAll() && !home());
  const int moves=count(0xFE); Mock::rejectNode=-1; advance(11);
  CHECK(count(0xF7,6)>=1 && count(0xF7,7)>=1 && disabled(6)>=1 && disabled(7)>=1 && count(0xFE)==moves);
  Stage4Test::loop(); CHECK(std::string(status().phase)=="fault"); checkFrames(); CHECK(Mock::attachAttempts[3]==0);
  for(auto pin:TestConfig::SERVO_PINS) CHECK(!Mock::servoAttached[pin]);
}
}
