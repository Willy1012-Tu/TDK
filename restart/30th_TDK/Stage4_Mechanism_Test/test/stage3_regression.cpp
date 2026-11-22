#include "controller_test_support.h"
using namespace Stage4Test; using namespace TestSupport;
void noTraffic(size_t writes=0) {CHECK(Mock::sent.empty() && Mock::servoWrites.size()==writes && Mock::attachAttempts[3]==0);}
void selectSecond() {
  CHECK(selectStage(2)); CHECK(status().stage==2 && status().steps==4 && std::string(status().phase)=="busy");
  pump();CHECK(target(7)==-8000 && count(0xFE,6)==0 && status().completed==0);
  CHECK(Mock::servo[9]==1800 && Mock::servo[4]==1300 && Mock::servo[6]==1400 && !next());
  finish(7);CHECK(status().completed==1 && status().position7==-8000 && std::string(status().phase)=="ready");
}
void selectThird() {CHECK(selectStage(3));pump();CHECK(status().stage==3 && status().steps==1);}
void secondStep() {nextStep();CHECK(status().completed==2 && Mock::servo[9]==1100 && Mock::servo[4]==2000);}
void thirdStep() {nextStep();CHECK(target(7)==-3000 && Mock::servo[6]==2200 && !next());finish(7);CHECK(status().completed==3);}
void fourthStep() {nextStep();CHECK(target(7)==-8000 && Mock::servo[6]==1600 && Mock::servo[9]==1800 && Mock::servo[4]==1300);finish(7);CHECK(status().completed==4);}
void runCase(const std::string &name) {
  ready();
  if(name=="standby_next" || name=="standby_run" || name=="standby_home") {
    selectThird();for(int n=0;n<4;++n){CHECK(name=="standby_next"?next():name=="standby_run"?runAll():home());pump();CHECK(std::string(status().phase)=="ready");}
    noTraffic();CHECK(status().positionKnownMask==12);return;
  }
  if(name=="standby_switch") {
    selectThird();nextStep();CHECK(selectStage(4));pump();noTraffic();selectSecond();
    CHECK(selectStage(3));pump();const auto writes=Mock::servoWrites.size();const auto frames=Mock::sent.size();
    nextStep();CHECK(Mock::servoWrites.size()==writes && Mock::sent.size()==frames && status().position7==-8000);return;
  }
  if(name=="standby_debug_reject") {
    selectThird();CHECK(setDebug(true));for(auto node:TestConfig::MOTOR_IDS){CHECK(!debugStepper(10,320,0,false,node) && !debugStepper(0,320,0,true,node));}
    for(auto pin:TestConfig::SERVO_PINS)CHECK(!debugServo(pin,1800,0));noTraffic();return;
  }
  if(name=="standby_ram_reject" || name=="standby_ram_noop") {
    CHECK(planBegin(1,1,3,1));for(auto node:TestConfig::MOTOR_IDS)CHECK(!planAction(0,Stage4::branch(Stage4::stepperFor(node,10,320),1)));
    for(auto pin:TestConfig::SERVO_PINS)CHECK(!planAction(0,Stage4::branch(Stage4::servo(pin,1800),1)));
    CHECK(!planAction(0,Stage4::branch(Stage4::delay(10),1)) && !planAction(0,Stage4::branch(Stage4::waitAll(),1)));
    if(name=="standby_ram_reject"){CHECK(!planCommit());planAbort();CHECK(status().stage==4);noTraffic();return;}
    CHECK(planAction(0,Stage4::branch(Stage4::delay(0),1)));CHECK(planStep(0,0,1));CHECK(planCommit());
    nextStep();CHECK(status().stage==3 && status().runtimePlan && status().completed==1);noTraffic();return;
  }
  if(name=="disabled_debug") {
    CHECK(setDebug(true));for(auto node:{4,5,8}){CHECK(!debugStepper(10,320,0,false,uint8_t(node)));CHECK(!debugStepper(0,320,0,true,uint8_t(node)));}
    CHECK(!debugServo(3,900,0));noTraffic();return;
  }
  if(name=="disabled_runtime") {
    CHECK(planBegin(1,1,4,1));for(auto node:{4,5,8})CHECK(!planAction(0,Stage4::branch(Stage4::stepperFor(uint8_t(node),1,320),1)));
    CHECK(!planAction(0,Stage4::branch(Stage4::servo(3,1800),1)));planAbort();noTraffic();return;
  }
  if(name=="stage2_steps" || name=="stage2_repeat") {
    selectSecond();for(int cycle=0;cycle<(name=="stage2_repeat"?4:1);++cycle){secondStep();thirdStep();fourthStep();}
    CHECK(count(0xFE,6)==0 && Mock::servo[2]==1150 && Mock::servo[5]==2100 && Mock::attachAttempts[3]==0);checkFrames();return;
  }
  if(name=="stage2_run") {
    selectSecond();CHECK(runAll());pump();CHECK(status().completed==2 && target(7)==-3000 && !next());
    finish(7);CHECK(target(7)==-8000 && !next());finish(7);
    CHECK(status().completed==4 && std::string(status().phase)=="ready" && Mock::servo[6]==1600);return;
  }
  if(name=="stage2_home") {
    selectSecond();secondStep();thirdStep();CHECK(home());pump();
    CHECK(count(0x91,7)==1 && count(0x91,6)==0 && Mock::servo[4]==2000 && Mock::servo[6]==1400 && Mock::servo[9]==1800 && !next());
    reply(0x91,2,7);pump();CHECK(!next());finish(7,0x91);CHECK(status().completed==0 && status().position7==0);
    Stage4Test::loop();CHECK(Serial.output.find("尋原點完成")!=std::string::npos);
    nextStep();CHECK(target(7)==-8000 && Mock::servo[4]==1300);return;
  }
  if(name=="five_servo_timers") {
    CHECK(setDebug(true));for(uint8_t i=0;i<TestConfig::SERVO_COUNT;++i)CHECK(debugServo(TestConfig::SERVO_PINS[i],1800,1000U+500U*i));pump();
    advance(1001);CHECK(Mock::servo[2]==2500 && Mock::servo[5]<2500 && Mock::servo[4]<2500 && Mock::servo[6]<2500 && Mock::servo[9]<2500);
    advance(1000);CHECK(Mock::servo[4]==2500 && Mock::servo[6]<2500 && Mock::servo[9]<2500);advance(1000);
    for(auto pin:TestConfig::SERVO_PINS)CHECK(Mock::servo[pin]==2500);CHECK(std::string(status().phase)=="ready" && Mock::sent.empty());return;
  }
  if(name=="seven_device_barrier") {
    CHECK(planBegin(1,8,4,1));for(uint8_t i=0;i<TestConfig::MOTOR_COUNT;++i)CHECK(planAction(i,Stage4::branch(Stage4::stepperFor(TestConfig::MOTOR_IDS[i],-50-i,320,0,false),1)));
    for(uint8_t i=0;i<TestConfig::SERVO_COUNT;++i)CHECK(planAction(i+2,Stage4::branch(Stage4::servo(TestConfig::SERVO_PINS[i],1900,1000U+100U*i,false),1)));
    CHECK(planAction(7,Stage4::branch(Stage4::waitAll(),1)));CHECK(planStep(0,0,8));CHECK(planCommit());nextStep();finish(6);
    advance(1401);CHECK(!next() && Mock::servo[9]==1900);finish(7);CHECK(std::string(status().phase)=="ready" && status().position6==-50 && status().position7==-51);checkFrames();return;
  }
  if(name=="stage2_ram") {
    CHECK(planBegin(1,3,2,1));CHECK(planAction(0,Stage4::branch(Stage4::servo(4,1800,0,false),1)));
    CHECK(planAction(1,Stage4::branch(Stage4::servo(9,1800,0,false),1)));CHECK(planAction(2,Stage4::branch(Stage4::waitAll(),1)));
    CHECK(planStep(0,0,3));CHECK(planCommit());CHECK(std::string(status().phase)=="busy" && !next());pump();
    CHECK(status().stage==2 && status().completed==1 && Mock::servo[4]==1800 && Mock::servo[9]==1800 && Mock::sent.empty());
    const auto writes=Mock::servoWrites.size();CHECK(selectStage(2));pump();CHECK(status().runtimePlan && Mock::servoWrites.size()==writes);return;
  }
  if(name=="stage2_duplicate_parallel") {
    CHECK(planBegin(1,3,2,1));for(uint8_t i=0;i<2;++i)CHECK(planAction(i,Stage4::branch(Stage4::servo(9,1800,0,false),1)));
    CHECK(planAction(2,Stage4::branch(Stage4::waitAll(),1)));CHECK(planStep(0,0,3));CHECK(!planCommit());planAbort();noTraffic();return;
  }
  if(name=="stop_five_pwm") {
    CHECK(setDebug(true));for(auto pin:TestConfig::SERVO_PINS)CHECK(debugServo(pin,1800,2000));
    CHECK(debugStepper(-20,320,0));CHECK(debugStepper(-30,320,0,false,7));pump();advance(501);const auto writes=Mock::servoWrites.size();
    stop();CHECK(Mock::detachedPins.size()==5);for(auto pin:TestConfig::SERVO_PINS)CHECK(!Mock::servoAttached[pin]);
    finish(6);finish(7);advance(5001);CHECK(Mock::servoWrites.size()==writes);fault();return;
  }
  if(name=="standby_stop"){selectThird();stop();fault();CHECK(count(0xF7,6)==1 && count(0xF7,7)==1 && disabled(6)==1 && disabled(7)==1);return;}
  if(name=="stage2_activation_once") {
    selectSecond();secondStep();const int moves=count(0xFE,7);const auto writes=Mock::servoWrites.size();CHECK(selectStage(2));pump();
    CHECK(status().completed==2 && count(0xFE,7)==moves && Mock::servoWrites.size()==writes);
    CHECK(selectStage(4));pump();CHECK(selectStage(2));pump();CHECK(status().completed==1 && Mock::servo[4]==1300 && Mock::servo[9]==1800);return;
  }
  if(name=="stage2_auto_stop") {
    CHECK(selectStage(2));CHECK(std::string(status().phase)=="busy" && count(0xFE)==0);pump();CHECK(count(0xFE,7)==1 && !next());
    stop();finish(7);advance(30001);CHECK(count(0xFE,7)==1 && status().completed==0);fault();return;
  }
  if(name=="debug_id7_independent") {
    CHECK(setDebug(true));CHECK(debugStepper(-20,320,0,false,6));CHECK(debugStepper(-30,500,2,false,7));
    CHECK(!debugStepper(-40,320,0,false,7));pump();finish(7);CHECK(status().position7==-30 && status().position6==0 && !setDebug(false));
    finish(6);CHECK(status().position6==-20 && std::string(status().phase)=="ready" && status().positionKnownMask==12);checkFrames();return;
  }
  if(name=="stage2_ram_auto_motor") {
    CHECK(planBegin(2,2,2,2));CHECK(planAction(0,Stage4::branch(Stage4::stepperFor(7,-300,320),1)));
    CHECK(planAction(1,Stage4::branch(Stage4::servo(6,2200),1)));CHECK(planStep(0,0,1));CHECK(planStep(1,1,1));CHECK(planCommit());
    CHECK(status().completed==0 && !next());pump();CHECK(target(7)==-300 && !next());finish(7);CHECK(status().completed==1);nextStep();CHECK(status().completed==2 && Mock::servo[6]==2200);
    const int moves=count(0xFE,7);nextStep();CHECK(status().completed==2 && count(0xFE,7)==moves);return;
  }
  throw std::runtime_error("unknown case");
}
int main(int argc,char **argv){try{CHECK(argc==2);runCase(argv[1]);std::cout<<"PASS "<<argv[1]<<"\n";return 0;}catch(const std::exception &e){std::cerr<<"FAIL "<<(argc>1?argv[1]:"")<<": "<<e.what()<<"\n";return 1;}}

