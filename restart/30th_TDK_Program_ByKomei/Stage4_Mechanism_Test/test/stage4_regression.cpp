// Each scenario runs in a fresh process against actual controller sources.
#include "controller_test_support.h"
using namespace Stage4Test;
using namespace TestSupport;
void toMotor() { nextStep(); CHECK(target()==-600); finish(); nextStep(); CHECK(Mock::servo[2]==1120); nextStep(); CHECK(target()==-8100); }
void toRise() { toMotor(); finish(); CHECK(!next()); advance(2001); nextStep(); CHECK(target()==-600 && Mock::servo[5]==1400); }
void fullCycle() {
  toRise(); finish(); CHECK(target()==-2600 && Mock::servo[5]==1800); finish();
  nextStep(); CHECK(Mock::servo[2]==1500 && Mock::servo[5]==1800); nextStep(); CHECK(target()==0); finish();
  CHECK(Mock::servo[2]==1150 && Mock::servo[5]==2100 && status().completed==6);
}
void startBoot() { begin(); pump(); CHECK(count(0x8C)==0); advance(TestConfig::STARTUP_DELAY_SECONDS*1000U+1U); CHECK(count(0x8C,6)==1); }
void enableBoot(bool requireAccepted=true) {
  startBoot(); size_t read=0;
  for(int n=0;n<100 && count(0xF3,6)==0;++n) {
    while(read<Mock::sent.size()) {const auto f=Mock::sent[read++].msg;if(f.buf[0]==0x8C) reply(0x8C,1,uint8_t(f.id));} pump();
  }
  if(requireAccepted) CHECK(count(0xF3,6)==1);
}
void bootToHome() {
  startBoot();reply(0x8C,1,6);pump();reply(0xF3,1,6);pump();
  CHECK(count(0x8C,7)==1 && count(0x91)==0);
  reply(0x8C,1,7);pump();CHECK(count(0xF3,7)==1 && count(0x91)==0);
  reply(0xF3,1,7);pump();
}
void runCase(const std::string &name) {
  if(name=="boot_dual_ack_barrier" || name=="boot_second_enable_retry") {
    startBoot();reply(0x8C,1,6);pump();CHECK(count(0x8C,7)==0 && count(0x91)==0);
    reply(0xF3,1,6);pump();CHECK(count(0x8C,7)==1 && count(0x91)==0);
    reply(0x8C,1,7);pump();CHECK(count(0xF3,7)==1 && count(0x91)==0);
    reply(0xF3,1,6);reply(0x91,1,6);reply(0x91,2,7);pump();CHECK(count(0x91)==0 && status().positionKnownMask==0);
    if(name=="boot_second_enable_retry") {advance(1501);CHECK(count(0xF3,7)>1 && count(0x91)==0 && status().startupNode==7);}
    reply(0xF3,1,7);pump();CHECK(count(0x91,6)==1 && count(0x91,7)==1 && std::string(status().phase)=="busy");
    finish(7,0x91);CHECK(!next() && status().positionKnownMask==8);finish(6,0x91);CHECK(status().positionKnownMask==12 && std::string(status().phase)=="ready");checkFrames();return;
  }
  if(name=="boot_dual_home_6_first" || name=="boot_dual_home_7_first") {
    bootToHome();const uint8_t first=name=="boot_dual_home_6_first"?6:7,second=first==6?7:6;
    CHECK(count(0x91)==2 && !next() && !runAll() && !home() && !selectStage(2));
    reply(0x91,2,first);pump();CHECK(status().positionKnownMask==0);finish(first,0x91);
    CHECK(status().positionKnownMask==(first==6?4:8) && std::string(status().phase)=="busy" && !next());
    reply(0xFE,1,second);reply(0xFE,2,second);pump();CHECK(!next());finish(second,0x91);
    CHECK(status().positionKnownMask==12 && std::string(status().phase)=="ready" && count(0x91)==2);return;
  }
  if(name=="boot_dual_home_partial_send") {
    Mock::rejectNode=7;Mock::rejectCommand=0x91;bootToHome();CHECK(count(0x91,6)==1 && count(0x91,7)==0);
    Mock::rejectNode=-1;advance(501);CHECK(count(0x91,7)==0);fault();return;
  }
  if(name=="boot_dual_home_queue_retry") {
    Mock::rejectNode=6;Mock::rejectCommand=0x91;bootToHome();CHECK(count(0x91)==0 && !next());
    Mock::rejectNode=-1;advance(11);CHECK(count(0x91,6)==1 && count(0x91,7)==1);
    finish(6,0x91);finish(7,0x91);CHECK(status().positionKnownMask==12 && std::string(status().phase)=="ready");return;
  }
  if(name=="boot_dual_home_start_timeout" || name=="boot_dual_home_finish_timeout") {
    bootToHome();finish(6,0x91);if(name=="boot_dual_home_finish_timeout"){reply(0x91,1,7);pump();}
    advance(name=="boot_dual_home_finish_timeout"?30001:1501);CHECK(status().positionKnownMask==4 && count(0x91)==2);
    Stage4Test::loop();CHECK(Serial.output.find("ID7 0x91")!=std::string::npos);fault();return;
  }
  if(name=="boot_dual_home_stop") {
    bootToHome();stop();finish(6,0x91);finish(7,0x91);advance(30001);CHECK(status().positionKnownMask==0 && count(0x91)==2);fault();return;
  }
  if(name=="servo_failure" || name=="timer_failure" || name=="thread_failure") {
    if(name=="servo_failure") Mock::rejectServoPin=9; else if(name=="timer_failure") Mock::timerOK=false; else Mock::rejectThread=1;
    begin(); pump(); CHECK(count(0x91)==0 && count(0xFE)==0); fault(); return;
  }
  if(name=="boot_delay" || name=="boot_delay_stop") {
    begin(); pump(); CHECK(count(0x8C)==0 && count(0xF3)==0 && !next() && !home());
    if(name=="boot_delay_stop") {stop(); advance(10001); CHECK(count(0x8C)==0 && count(0x91)==0); fault(); return;}
    advance(TestConfig::STARTUP_DELAY_SECONDS*1000U); CHECK(count(0x8C)==0); advance(1); CHECK(count(0x8C,6)==1); return;
  }
  if(name=="boot_first_ack_retry") {
    startBoot(); advance(1501);
    CHECK(std::string(status().phase)=="booting" && count(0x8C,6)>1);
    CHECK(count(0x91)==0 && count(0xF3,6)==0);
    continueStartup();
    CHECK(std::string(status().phase)=="ready" && count(0x91)==2 && status().positionKnownMask==12);
    checkFrames(); return;
  }
  if(name=="boot_late_enable_ack") {
    enableBoot(); advance(2001);
    CHECK(std::string(status().phase)=="booting" && count(0xF3,6)>1 && count(0x91)==0);
    CHECK(status().startupNode==6 && status().startupCommand==0xF3 && status().startupAttempts>1);
    reply(0xF3,1,5); pump(); CHECK(count(0x8C,5)==0 && count(0x91)==0);
    reply(0xF3,1,6); pump(); CHECK(std::string(status().phase)=="booting" && count(0x91)==0 && count(0x8C,7)==1);
    continueStartup(); CHECK(count(0x91)==2 && std::string(status().phase)=="ready"); return;
  }
  if(name=="boot_ack_fixed_deadline") {
    startBoot();
    for(int n=0;n<19;++n) {
      advance(501); CHECK(std::string(status().phase)=="booting" && count(0x91)==0);
    }
    advance(481); CHECK(std::string(status().phase)=="booting" && count(0x8C,6)==20);
    CHECK(status().startupAttempts==20 && count(0xF3,6)==0);
    advance(1); CHECK(std::string(status().phase)=="fault");
    CHECK(std::string(status().startupPhase)=="feedback_wait" && status().startupNode==6 && status().startupCommand==0x8C);
    const int sent=count(0x8C,6); reply(0x8C,1,6); advance(10001);
    CHECK(count(0x8C,6)==sent && count(0x91)==0 && count(0xF3,6)==1);
    Stage4Test::loop(); CHECK(Serial.output.find("ID6 0x8C")!=std::string::npos);
    CHECK(Serial.output.find("ACK")!=std::string::npos); fault(); return;
  }
  if(name=="boot_ack_last_millisecond") {
    startBoot(); advance(10000); CHECK(std::string(status().phase)=="booting");
    reply(0x8C,1,6); pump(); CHECK(count(0xF3,6)==1 && std::string(status().phase)=="booting");
    continueStartup(); CHECK(std::string(status().phase)=="ready"); return;
  }
  if(name=="boot_retry_rejected_ack") {
    startBoot(); advance(501); CHECK(count(0x8C,6)==2);
    reply(0x8C,0,6); reply(0x8C,1,6); pump();
    CHECK(std::string(status().phase)=="fault" && count(0x91)==0 && count(0xF3,6)==1);
    Stage4Test::loop(); CHECK(Serial.output.find("ID6 0x8C")!=std::string::npos);
    CHECK(Serial.output.find("ACK 00")!=std::string::npos); fault(); return;
  }
  if(name=="boot_retry_stop") {
    enableBoot(); advance(501); CHECK(count(0xF3,6)==2);
    stop(); const int before=count(0xF3,6);
    reply(0xF3,1,6); reply(0x8C,1,5); advance(20001);
    CHECK(count(0xF3,6)==before && count(0x8C,5)==0 && count(0x91)==0); fault(); return;
  }
  if(name=="boot_retry_queue_rejected") {
    startBoot(); Mock::rejectNode=6; Mock::rejectCommand=0x8C;
    advance(501); advance(501);
    CHECK(count(0x8C,6)==1 && status().startupAttempts==3 && std::string(status().phase)=="booting");
    Mock::rejectNode=-1; advance(501);
    CHECK(count(0x8C,6)==2 && status().startupAttempts==4);
    continueStartup(); CHECK(count(0x91)==2 && std::string(status().phase)=="ready"); return;
  }
  if(name=="boot_diagnostics") {
    startBoot(); CHECK(std::string(status().startupPhase)=="feedback_wait" && status().startupNode==6);
    CHECK(status().startupCommand==0x8C && status().startupAttempts==1 && status().positionKnownMask==0);
    advance(500); CHECK(count(0x8C,6)==1); advance(1); CHECK(status().startupAttempts==2);
    reply(0x8C,1,4); reply(0x8C,1,5); pump(); CHECK(count(0xF3,6)==0);
    reply(0x8C,1,6); pump();
    CHECK(std::string(status().startupPhase)=="enable_wait" && status().startupCommand==0xF3 && status().startupAttempts==1);
    reply(0xF3,1,5); reply(0xF3,1,6,1,true); advance(501);
    CHECK(count(0x91)==0 && status().startupAttempts==2 && status().positionKnownMask==0);
    reply(0xF3,1,6); pump(); CHECK(std::string(status().startupPhase)=="feedback_wait" && status().startupNode==7 && count(0x91)==0);
    continueStartup(); CHECK(status().positionKnownMask==12);
    nextStep(); CHECK(target()==-600 && status().positionKnownMask==12);
    reply(0xFE,2,6); pump(); CHECK(!next() && status().positionKnownMask==12);
    finish(); CHECK(status().positionKnownMask==12); return;
  }
  if(name=="home_motion_not_retried") {
    ready(); CHECK(home()); pump(); reply(0x91,1,6); pump();
    for(int n=0;n<6;++n) advance(501);
    CHECK(count(0x91)==1 && count(0xFE)==0 && std::string(status().phase)=="busy");
    finish(6,0x91); CHECK(std::string(status().phase)=="ready" && status().positionKnownMask==12); return;
  }
  if(name=="boot_disabled_absent") {
    Mock::rejectServoPin=3;Mock::rejectNode=4;Mock::rejectCommand=0x8C;ready();
    CHECK(std::string(status().phase)=="ready" && Mock::attachAttempts[3]==0);return;
  }
  if(name=="boot_only_active_hardware" || name=="boot_ready_dual_home" || name=="boot_without_usb") {
    if(name=="boot_without_usb") Serial.connected=false;
    startup(); CHECK(Mock::sent.size()==6 && count(0x8C)==2 && count(0xF3)==2);
    CHECK(count(0x91)==2 && count(0xFE)==0 && count(0x92)==0 && count(0x80)==0);
    CHECK(status().positionKnownMask==12 && Mock::attachAttempts[3]==0);
    CHECK(Mock::servo[4]==2000 && Mock::servo[6]==1400 && Mock::servo[9]==1800);
    advance(31001); CHECK(std::string(status().phase)=="ready" && Mock::sent.size()==6);
    reply(0x91,0,4); reply(0x91,2,5); reply(0x91,2,6); pump();
    CHECK(std::string(status().phase)=="ready" && status().positionKnownMask==12); checkFrames(); return;
  }
  if(name=="boot_feedback_timeout" || name=="boot_feedback_failure" || name=="boot_feedback_failure_latched") {
    startBoot(); if(name=="boot_feedback_timeout") advance(TestConfig::STARTUP_ACK_TIMEOUT_MS+1U);
    else {reply(0x8C,0,6); if(name=="boot_feedback_failure_latched") reply(0x8C,1,6); pump();}
    CHECK(count(0x91)==0); fault(); return;
  }
  if(name=="boot_enable_failure" || name=="boot_enable_failure_latched" || name=="boot_enable_timeout") {
    enableBoot(); if(name=="boot_enable_timeout") advance(TestConfig::STARTUP_ACK_TIMEOUT_MS+1U);
    else {reply(0xF3,0,6); if(name=="boot_enable_failure_latched") reply(0xF3,1,6); pump();}
    CHECK(count(0x91)==0); fault(); return;
  }
  if(name=="boot_bad_reply") {
    startBoot(); reply(0x8C,1,6,2); reply(0x8C,1,6,1,true); pump(); CHECK(count(0xF3,6)==0);
    reply(0x8C,1,6); pump(); CHECK(count(0xF3,6)==1); return;
  }
  if(name.rfind("boot_feedback_queue_",0)==0 || name.rfind("boot_enable_queue_",0)==0) {
    const int command=name.rfind("boot_feedback_",0)==0?0x8C:0xF3; Mock::rejectNode=6; Mock::rejectCommand=command;
    if(command==0x8C) {begin(); advance(1001);} else enableBoot(false);
    CHECK(count(command,6)==0);
    if(name.find("timeout")!=std::string::npos) {advance(501); fault(); return;}
    Mock::rejectNode=-1; advance(11); CHECK(count(command,6)==1); return;
  }
  if(name=="home_queue_retry" || name=="home_queue_timeout") {
    ready(); Mock::rejectNode=6; Mock::rejectCommand=0x91; CHECK(home()); pump();
    CHECK(count(0x91)==0);
    if(name=="home_queue_timeout") {advance(501); fault(); return;}
    Mock::rejectNode=-1; advance(11); CHECK(count(0x91,6)==1); return;
  }
  if(name.rfind("home_",0)==0) {
    ready(); CHECK(home()); pump(); CHECK(count(0x91)==1 && count(0xFE)==0 && !next());
    if(name=="home_frame") {
      for(const auto &f:Mock::sent) if(f.msg.buf[0]==0x91) CHECK(f.msg.len==2 && f.msg.buf[1]==uint8_t(f.msg.id+0x91));
      reply(0x91,2,6); pump(); CHECK(!next()); finish(6,0x91); CHECK(next()); return;
    }
    if(name=="home_failure") {
      reply(0x91,0,6); pump(); Stage4Test::loop(); CHECK(Serial.output.find("ID6 0x91")!=std::string::npos); fault(); return;
    }
    if(name=="home_stop") {stop(); finish(6,0x91); advance(10000); CHECK(count(0xFE)==0); fault(); return;}
    if(name=="home_finish_timeout") {reply(0x91,1,6); pump();}
    Mock::advance(name=="home_finish_timeout"?30001:1501); if(name=="home_late_start") finish(6,0x91); else pump();
    Stage4Test::loop(); CHECK(Serial.output.find("ID6 0x91")!=std::string::npos && count(0x91)==1); fault(); return;
  }
  ready();
  if(name=="only_stage4") {toMotor(); checkFrames(); CHECK(count(0xFE,4)==0 && count(0xFE,5)==0 && Mock::servo[3]==0); return;}
  if(name=="full_cycle") {fullCycle(); CHECK(count(0xFE)==5); nextStep(); CHECK(target()==-600 && count(0xFE)==6); return;}
  if(name=="run_all" || name=="no_usb") {
    if(name=="no_usb") {Serial.connected=false; Serial.output.clear();}
    CHECK(runAll()); pump(); CHECK(target()==-600); finish(); CHECK(target()==-8100); finish(); advance(2001);
    CHECK(target()==-600); finish(); CHECK(target()==-2600); finish(); CHECK(target()==0); finish();
    CHECK(status().completed==6 && std::string(status().phase)=="ready" && count(0xFE)==5);
    advance(10000); CHECK(count(0xFE)==5); if(name=="no_usb") CHECK(Serial.output.empty()); return;
  }
  if(name=="wait_start_order") {
    toRise(); const int n=count(0xFE); reply(0xFE,2); reply(0xFD,1); reply(0xFD,2); reply(0xFE,1,6,2); pump(); CHECK(count(0xFE)==n);
    reply(0xFE,1); pump(); CHECK(count(0xFE)==n); reply(0xFE,2); pump(); CHECK(count(0xFE)==n+1 && target()==-2600);
    reply(0xFE,2); pump(); CHECK(count(0xFE)==n+1 && !next()); return;
  }
  if(name=="start_timeout" || name=="late_start" || name=="finish_timeout" || name=="late_finish" || name=="limit_failure" || name=="limit_failure_5" || name=="move_failure") {
    toMotor(); if(name=="finish_timeout" || name=="late_finish") {reply(0xFE,1); pump();}
    if(name=="limit_failure" || name=="limit_failure_5" || name=="move_failure") {reply(0xFE,name=="move_failure"?0:name=="limit_failure"?3:5); pump();}
    else {Mock::advance(name=="finish_timeout" || name=="late_finish"?30001:1501); if(name=="late_start" || name=="late_finish") finish(); else pump();} fault(); return;
  }
  if(name=="bad_move_reply") {
    nextStep(); reply(0xFE,1,6,1,true); reply(0xFE,2,6,1,true); pump(); CHECK(!next());
    for(int kind=0;kind<4;++kind) {reply(0xFE,1); auto &f=Mock::incoming[1].back(); if(kind==0) f.id=0x106; else if(kind==1) f.len=2; else if(kind==2) f.flags.extended=true; else f.flags.remote=true; pump();}
    reply(0xFE,2); pump(); CHECK(!next()); finish(); CHECK(next()); return;
  }
  if(name=="queue_retry" || name=="queue_timeout") {
    Mock::rejectNode=6; Mock::rejectCommand=0xFE; nextStep(); CHECK(count(0xFE)==0 && Mock::servo[2]==1150 && Mock::servo[5]==2100);
    if(name=="queue_timeout") {advance(501); fault(); return;}
    Mock::rejectNode=-1; advance(11); CHECK(count(0xFE)==1 && Mock::servo[2]==1500 && Mock::servo[5]==1400); pump(); CHECK(count(0xFE)==1); return;
  }
  if(name=="home") {fullCycle(); CHECK(home()); pump(); CHECK(count(0x91,6)==1 && count(0xFE)==5); finish(6,0x91); CHECK(status().position6==0 && next()); return;}
  if(name=="busy") {toMotor(); CHECK(!next() && !runAll() && !home()); finish(); CHECK(!next()); advance(2001); CHECK(next()); return;}
  if(name=="stop_and_no_resume") {CHECK(runAll()); pump(); const int n=count(0xFE); stop(); finish(); advance(10000); CHECK(count(0xFE)==n); fault(); return;}
  if(name=="stop_ramp") {toMotor(); reply(0xFE,1); pump(); advance(1001); CHECK(Mock::servo[5]==1475); stop(); finish(); advance(5000); CHECK(Mock::servo[5]==1475); fault(); return;}
  if(name=="stop_retry" || name=="disable_retry") {
    nextStep(); const bool disable=name=="disable_retry"; Mock::rejectNode=6; Mock::rejectCommand=disable?0xF3:0xF7; stop(); CHECK(disable?disabled(6)==0:count(0xF7,6)==0);
    Mock::rejectNode=-1; advance(11); CHECK(count(0xF7)==2 && disabled()==2); fault(); return;
  }
  if(name=="serial_valid") {serial(" NEXT \r\n"); CHECK(target()==-600 && !home()); finish(); serial("home\n"); CHECK(count(0x91,6)==1); finish(6,0x91); serial("run\n"); CHECK(target()==-600); serial("stop\n"); fault(); return;}
  if(name=="serial_removed") {serial("prev\np 100\nrpm 600\nacc 3\nservo 5 1400\nstatus\n"); CHECK(count(0xFE)==0 && Mock::servo[2]==1150 && Mock::servo[5]==2100); CHECK(next()); return;}
  if(name=="serial_invalid") {serial("next extra\n"+std::string(140,'x')+"\nnext\x01\n"); CHECK(count(0xFE)==0 && Mock::servo[2]==1150); CHECK(next()); return;}
  if(name=="ramp_finishes_first") {toMotor(); reply(0xFE,1); pump(); advance(2001); CHECK(Mock::servo[5]==1550 && !next()); reply(0xFE,2); pump(); CHECK(next()); return;}
  if(name=="negative_position_bytes") {
    nextStep(); const auto f=lastMove(); CHECK(f.buf[4]==0xFF && f.buf[5]==0xFD && f.buf[6]==0xA8); checkFrames();
    finish(); nextStep(); nextStep(); const auto d=lastMove(); CHECK(d.buf[4]==0xFF && d.buf[5]==0xE0 && d.buf[6]==0x5C); return;
  }
  throw std::runtime_error("unknown scenario");
}
int main(int argc,char **argv) {try {CHECK(argc==2); runCase(argv[1]); std::cout<<"PASS "<<argv[1]<<"\n"; return 0;} catch(const std::exception &e) {std::cerr<<"FAIL "<<(argc>1?argv[1]:"")<<": "<<e.what()<<"\n"; return 1;}}

