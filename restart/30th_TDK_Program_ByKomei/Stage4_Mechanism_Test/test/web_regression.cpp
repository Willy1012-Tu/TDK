#include "controller_test_support.h"
using namespace Stage4Test;
using namespace TestSupport;
void loadDelay(uint32_t ms) {
  CHECK(planBegin(1,1)); CHECK(planAction(0,Stage4::branch(Stage4::delay(ms),1)));
  CHECK(planStep(0,0,1)); CHECK(planCommit());
}
void runCase(const std::string &name) {
  ready();
  if (name=="web_status") {
    request("status"); CHECK(Serial.output.find("\"protocol\":1")!=std::string::npos);
    CHECK(Serial.output.find("\"steps\":6")!=std::string::npos && count(0xFE)==0);
    CHECK(Serial.output.find("\"firmware_id\":\"Stage4_Mechanism_Test\"")!=std::string::npos);
    CHECK(Serial.output.find("\"firmware_revision\":\"stage2-id67-home-v1\"")!=std::string::npos);
    CHECK(Serial.output.find("\"startup_return_zero\":true")!=std::string::npos);
    CHECK(Serial.output.find("\"startup_phase\":\"ready\"")!=std::string::npos);
    CHECK(Serial.output.find("\"position_known_mask\":12")!=std::string::npos);
    const std::string prefix="@17 OK ";
    const size_t replyBegin=Serial.output.find(prefix+"{"); CHECK(replyBegin!=std::string::npos);
    const size_t jsonBegin=replyBegin+prefix.size(), jsonEnd=Serial.output.find('\n',jsonBegin);
    CHECK(jsonEnd!=std::string::npos && jsonEnd>jsonBegin && Serial.output[jsonEnd-1]=='}');
    FILE *fixture=fopen(".pio/host-tests/status_fixture.json","wb"); CHECK(fixture!=nullptr);
    const std::string json=Serial.output.substr(jsonBegin,jsonEnd-jsonBegin);
    CHECK(fwrite(json.data(),1,json.size(),fixture)==json.size()); CHECK(fclose(fixture)==0);
    request("hello"); CHECK(Serial.output.find("\"firmware_id\":\"Stage4_Mechanism_Test\"")!=std::string::npos);
    CHECK(!status().debug && status().completed==0); return;
  }
  if (name=="debug_validation") {
    request("debug.move 0 320 5",false); request("mode debug");
    request("debug.move 0 0 5",false); request("debug.move 8388608 320 5",false);
    request("debug.move 0 320 256",false); request("debug.move 0 320 5 extra",false);
    request("debug.servo 3 900 1000",false); request("debug.servo 2 1801 1000",false);
    request("debug.servo 2 900 60001",false); CHECK(count(0xFE)==0 && Mock::servo[2]==1150);
    request("next",false); request("run",false); request("home",false); return;
  }
  if (name=="debug_stepper") {
    request("mode debug"); request("debug.move -400 200 7");
    CHECK(count(0xFE)==1 && status().debug); const auto f=Mock::sent.back().msg;
    CHECK(f.buf[1]==0 && f.buf[2]==200 && f.buf[3]==7 && f.buf[4]==0xFF && f.buf[5]==0xFE && f.buf[6]==0x70);
    request("mode normal",false); request("debug.move 0 320 5",false);
    finish(); CHECK(status().completed==0); request("mode normal"); CHECK(next()); return;
  }
  if (name=="debug_gohome") {
    request("mode debug"); request("debug.gohome"); CHECK(count(0x91)==1 && count(0xFE)==0);
    finish(6,0x91); CHECK(std::string(status().phase)=="ready" && status().completed==0); return;
  }
  if (name=="debug_independent_servos") {
    CHECK(setDebug(true)); CHECK(debugServo(2,900,1000)); poll();
    CHECK(debugServo(5,1800,1000)); CHECK(!setDebug(false)); advance(501);
    CHECK(Mock::servo[2]==1325 && Mock::servo[5]==2300);
    advance(500); CHECK(Mock::servo[2]==1500 && Mock::servo[5]==2500 && std::string(status().phase)=="ready");
    CHECK(debugServo(2,0,0)); poll(); CHECK(Mock::servo[2]==500 && Mock::servo[5]==2500); return;
  }
  if (name=="debug_motor_and_servo") {
    CHECK(setDebug(true)); CHECK(debugStepper(100,320,5)); poll(); CHECK(debugServo(2,900,1000));
    finish(); CHECK(std::string(status().phase)=="busy"); advance(1001);
    CHECK(Mock::servo[2]==1500 && std::string(status().phase)=="ready"); return;
  }
  if (name=="plan_transaction") {
    loadDelay(1000); CHECK(status().runtimePlan && status().steps==1 && status().planVersion==1);
    CHECK(planBegin(1,2)); CHECK(planAction(0,Stage4::branch(Stage4::servo(2,2000),1)));
    CHECK(planStep(0,0,2)); CHECK(!planCommit()); CHECK(status().planVersion==1 && !next());
    planAbort(); CHECK(next()); poll(); CHECK(Mock::servo[2]==1150); advance(1001);
    loadDelay(500); CHECK(status().planVersion==2 && status().steps==1); CHECK(runAll()); poll(); advance(501);
    CHECK(std::string(status().phase)=="ready" && status().completed==1); return;
  }
  if (name=="plan_parallel_validation") {
    CHECK(planBegin(1,3)); CHECK(planStep(0,0,3));
    CHECK(planAction(0,Stage4::branch(Stage4::servo(2,1500,1000,false),1)));
    CHECK(planAction(1,Stage4::branch(Stage4::servo(2,1600,1000,false),1)));
    CHECK(planAction(2,Stage4::branch(Stage4::waitAll(),1))); CHECK(!planCommit());
    CHECK(planAction(1,Stage4::branch(Stage4::servo(5,1600,1000,false),1))); CHECK(planCommit());
    CHECK(runAll()); poll(); advance(1001); CHECK(Mock::servo[2]==1500 && Mock::servo[5]==1600 && status().completed==1); return;
  }
  if (name=="plan_incomplete_barrier") {
    CHECK(planBegin(1,1)); CHECK(planStep(0,0,1)); CHECK(planAction(0,Stage4::branch(Stage4::stepper(0,320,0,false),1)));
    CHECK(!planCommit()); planAbort(); CHECK(planBegin(1,1)); CHECK(planStep(0,0,1));
    CHECK(planAction(0,Stage4::branch(Stage4::delay(1),2))); CHECK(!planCommit()); planAbort(); CHECK(status().steps==6); return;
  }
  if (name=="plan_timeout_stop") {
    CHECK(planBegin(1,1)); CHECK(!setDebug(true) && !next()); advance(10001); CHECK(!status().uploading && status().steps==6);
    CHECK(planBegin(1,1)); Mock::advance(10001); CHECK(!planAction(0,Stage4::branch(Stage4::delay(1),1)) && !planCommit());
    CHECK(planBegin(1,1)); stop(); CHECK(!planCommit() && !status().uploading && !next()); return;
  }
  if (name=="plan_protocol") {
    request("plan.begin 2 2"); request("plan.action 0 1 0 0 0 2 1500 0 1 1");
    request("plan.action 1 2 0 0 0 0 0 500 1 1"); request("plan.step 0 0 1"); request("plan.step 1 1 1");
    request("plan.commit"); CHECK(status().steps==2 && status().planVersion==1); request("run");
    CHECK(Mock::servo[2]==1500); advance(501); CHECK(status().completed==2 && std::string(status().phase)=="ready"); return;
  }
  if (name=="plan_254_steps") {
    Serial.connected=false; CHECK(planBegin(254,254));
    for (uint16_t i=0;i<254;++i) { CHECK(planAction(i,Stage4::branch(Stage4::delay(0),1))); CHECK(planStep(i,i,1)); }
    CHECK(planCommit() && runAll()); for (int i=0;i<40;++i) poll();
    CHECK(status().steps==254 && status().completed==254 && std::string(status().phase)=="ready"); CHECK(next()); poll(); CHECK(status().completed==1); return;
  }
  if (name=="debug_three_axes") {
    CHECK(setDebug(true));
    CHECK(!debugStepper(-100,200,7,false,4) && !debugStepper(100,320,5,false,5));
    CHECK(!debugServo(3,1062,0)); CHECK(debugStepper(-600,320,5,false,6)); CHECK(debugStepper(-3000,320,2,false,7));
    CHECK(debugServo(4,1170,1000) && debugServo(6,1350,1000) && debugServo(9,720,1000));
    pump(); finish(6); CHECK(std::string(status().phase)=="busy"); finish(7); advance(1001);
    CHECK(Mock::servo[4]==1800 && Mock::servo[6]==2000 && Mock::servo[9]==1300);
    CHECK(std::string(status().phase)=="ready" && status().position4==0 && status().position5==0 && status().position6==-600 && status().position7==-3000);
    CHECK(Mock::attachAttempts[3]==0); checkFrames(); return;
  }
  if (name=="debug_node_protocol") {
    request("mode debug"); request("debug.move 4 -4200 320 5",false); request("debug.move 5 4200 320 5",false);
    request("debug.move 8 0 320 5",false); request("debug.gohome 4",false); request("debug.gohome 5",false);
    request("debug.servo 3 1062 0",false); CHECK(count(0xFE)==0 && count(0x91)==0);
    request("debug.move 6 -4200 320 5"); CHECK(target(6)==-4200); finish();
    request("debug.gohome 6"); CHECK(count(0x91,6)==1); finish(6,0x91);
    request("debug.move 7 -8000 320 2"); CHECK(target(7)==-8000); finish(7);
    request("debug.gohome 7"); CHECK(count(0x91,7)==1); finish(7,0x91);
    request("debug.servo 4 1170 0"); request("debug.servo 6 1350 0"); request("debug.servo 9 720 0");
    CHECK(Mock::servo[4]==1800 && Mock::servo[6]==2000 && Mock::servo[9]==1300);
    request("status"); CHECK(Serial.output.find("\"motor_ids\":[6,7]")!=std::string::npos);
    CHECK(Serial.output.find("\"servo_pins\":[2,5,4,6,9]")!=std::string::npos);
    CHECK(Serial.output.find("\"pulse4\":1800")!=std::string::npos && Serial.output.find("\"position4\":0")!=std::string::npos); return;
  }
  if (name=="plan_three_devices") {
    CHECK(!planBegin(1,4,1,1) && !planBegin(1,4,2,2)); CHECK(planBegin(1,4,2,1)); CHECK(planStep(0,0,4));
    CHECK(!planAction(0,Stage4::branch(Stage4::stepperFor(4,-4200,320,5,false),1)));
    CHECK(planAction(0,Stage4::branch(Stage4::servo(4,1800,0,false),1)));
    CHECK(planAction(1,Stage4::branch(Stage4::servo(4,1300,0,false),1)));
    CHECK(planAction(2,Stage4::branch(Stage4::servo(6,2000,0,false),1)));
    CHECK(planAction(3,Stage4::branch(Stage4::waitAll(),1))); CHECK(!planCommit());
    CHECK(!planAction(1,Stage4::branch(Stage4::servo(3,1680,0,false),1)));
    CHECK(planAction(1,Stage4::branch(Stage4::servo(9,1800,0,false),1))); CHECK(planCommit());
    pump(); CHECK(status().stage==2 && status().runtimePlan && status().completed==1 && runAll()); pump();
    CHECK(Mock::servo[4]==1800 && Mock::servo[9]==1800 && Mock::servo[6]==2000 && status().completed==1);
    CHECK(count(0xFE)==0 && count(0x91)==0); return;
  }
  if (name=="plan_switch_parks") {
    CHECK(selectStage(3)); nextStep(); CHECK(count(0xFE)==0);
    CHECK(setDebug(true)); CHECK(!debugStepper(-100,320,0) && !debugServo(4,900,0)); CHECK(setDebug(false));
    CHECK(planBegin(1,1,3,1)); CHECK(!planAction(0,Stage4::branch(Stage4::servo(4,1800),1))); planAbort();
    CHECK(planBegin(1,1,2,1)); CHECK(planAction(0,Stage4::branch(Stage4::servo(4,1800),1))); CHECK(planStep(0,0,1)); CHECK(planCommit()); pump();
    CHECK(status().stage==2 && status().pendingStage==0 && status().runtimePlan && status().steps==1 && status().planVersion==1);
    CHECK(next()); pump(); CHECK(Mock::servo[4]==1800 && status().completed==1);
    CHECK(Mock::attachAttempts[3]==0 && count(0xFE)==0 && count(0x91)==0); return;
  }
  if (name=="plan_profile_protocol") {
    request("plan.begin 4 15 2 2");
    request("plan.action 0 0 -8000 320 2 0 0 0 0 1 7");
    request("plan.action 1 1 0 0 0 9 1800 0 0 1 0"); request("plan.action 2 1 0 0 0 4 1300 0 0 1 0");
    request("plan.action 3 3 0 0 0 0 0 0 1 1 0");
    request("plan.action 4 1 0 0 0 9 1100 0 0 1 0"); request("plan.action 5 1 0 0 0 4 2000 0 0 1 0");
    request("plan.action 6 3 0 0 0 0 0 0 1 1 0");
    request("plan.action 7 0 -3000 320 2 0 0 0 0 1 7"); request("plan.action 8 1 0 0 0 6 2200 0 0 1 0");
    request("plan.action 9 3 0 0 0 0 0 0 1 1 0");
    request("plan.action 10 0 -8000 320 2 0 0 0 0 1 7"); request("plan.action 11 1 0 0 0 6 1600 0 0 1 0");
    request("plan.action 12 1 0 0 0 9 1800 0 0 1 0"); request("plan.action 13 1 0 0 0 4 1300 0 0 1 0");
    request("plan.action 14 3 0 0 0 0 0 0 1 1 0");
    request("plan.step 0 0 4"); request("plan.step 1 4 3"); request("plan.step 2 7 3"); request("plan.step 3 10 5"); request("plan.commit");
    CHECK(status().stage==2 && status().runtimePlan && status().steps==4 && std::string(status().phase)=="busy");
    CHECK(target(7)==-8000 && Mock::servo[9]==1800 && Mock::servo[4]==1300);
    request("next",false); finish(7); CHECK(status().completed==1 && status().position7==-8000);
    request("next"); CHECK(status().completed==2 && Mock::servo[9]==1100 && Mock::servo[4]==2000);
    request("next"); CHECK(target(7)==-3000 && Mock::servo[6]==2200); finish(7); CHECK(status().completed==3);
    request("next"); CHECK(target(7)==-8000 && Mock::servo[6]==1600 && Mock::servo[9]==1800 && Mock::servo[4]==1300);
    finish(7); CHECK(status().completed==4);
    const int beforeRepeat=count(0xFE,7); request("next");
    CHECK(status().completed==2 && Mock::servo[9]==1100 && Mock::servo[4]==2000 && count(0xFE,7)==beforeRepeat);
    CHECK(count(0xFE,6)==0 && count(0x91)==0); return;
  }
  if (name=="serial_stage_aliases") {
    serial("stage3\n"); CHECK(status().stage==3 && count(0xFE)==0); nextStep();
    serial("stage2\n"); CHECK(status().stage==2 && status().pendingStage==0 && target(7)==-8000);
    CHECK(std::string(status().phase)=="busy" && !next()); finish(7);
    CHECK(status().completed==1 && Mock::servo[4]==1300 && Mock::servo[9]==1800);
    nextStep(); CHECK(status().completed==2 && Mock::servo[4]==2000 && Mock::servo[9]==1100);
    serial("stage4\n"); CHECK(status().stage==4 && status().pendingStage==0 && count(0xFE,6)==0 && count(0x91)==0); return;
  }
  throw std::runtime_error("unknown case");
}
int main(int argc,char **argv) { try { CHECK(argc==2); runCase(argv[1]); std::cout<<"PASS "<<argv[1]<<"\n"; return 0; }
  catch (const std::exception &e) { std::cerr<<"FAIL "<<(argc>1?argv[1]:"")<<": "<<e.what()<<"\n"; return 1; } }
