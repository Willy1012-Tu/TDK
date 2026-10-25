#include "controller_test_support.h"
using namespace Stage4Test;
using namespace TestSupport;
void firstStep() {
  CHECK(next()); poll(); CHECK(Mock::servo[2]==1150 && Mock::servo[5]==2100 && count(0xFE)==0);
  advance(501); CHECK(Mock::servo[2]==1325 && Mock::servo[5]==2100 && count(0xFE)==0);
  advance(500); CHECK(Mock::servo[2]==1500 && count(0xFE)==0);
  advance(500); CHECK(count(0xFE)==0 && !next()); advance(1); CHECK(count(0xFE)==1);
  const auto f=Mock::sent.back().msg;
  CHECK(f.buf[1]==0 && f.buf[2]==200 && f.buf[3]==7);
  CHECK(f.buf[4]==0xFF && f.buf[5]==0xFE && f.buf[6]==0x70);
  CHECK(Mock::servo[5]==2100); finish(); CHECK(!next());
  advance(501); CHECK(Mock::servo[5]==1900 && Mock::servo[2]==1500);
  advance(500); CHECK(Mock::servo[5]==1700);
}
void thirdStep() {
  firstStep(); CHECK(next()); poll(); advance(60000); CHECK(!next());
  advance(1); CHECK(Mock::servo[2]==1300); CHECK(next()); poll(); CHECK(count(0xFE)==2);
}
void runCase(const std::string &name) {
  ready();
  if (name=="single_actions_progress") {
    firstStep(); Stage4Test::loop();
    size_t previous=Serial.output.find("STEP1\n"); CHECK(previous!=std::string::npos);
    for (unsigned i=1;i<=5;++i) {
      const auto position=Serial.output.find("STEP1-"+std::to_string(i)+"\n");
      CHECK(position!=std::string::npos && position>previous); previous=position;
    }
    CHECK(Serial.output.find("首段移動")==std::string::npos && Serial.output.find("已完成")==std::string::npos);
    CHECK(home()); return;
  }
  if (name=="delay_serial_stop") {
    CHECK(next()); poll(); advance(1001); CHECK(!next() && count(0xFE)==0);
    Serial.input="stop\n"; Stage4Test::loop(); advance(60001);
    CHECK(count(0xF7)==2 && count(0xFE)==0 && !next() && !runAll() && !home());
    CHECK(Serial.output.find("已停止")!=std::string::npos); return;
  }
  if (name=="parallel_waits_all") {
    thirdStep(); finish(); CHECK(!next());
    advance(251); CHECK(Mock::servo[5]==1800); advance(250); CHECK(Mock::servo[5]==1900 && !next());
    advance(101); CHECK(Mock::servo[2]==1300 && Mock::servo[5]==1900 && !next());
    advance(251); CHECK(Mock::servo[2]==1550 && Mock::servo[5]==1700 && !next());
    advance(250); CHECK(Mock::servo[2]==1800 && Mock::servo[5]==1500 && next()); return;
  }
  if (name=="parallel_motor_timeout") {
    thirdStep(); advance(1501); CHECK(count(0xF7)==2 && !next() && !home()); return;
  }
  throw std::runtime_error("unknown case");
}
int main(int argc, char **argv) {
  try { CHECK(argc==2); runCase(argv[1]); std::cout<<"PASS "<<argv[1]<<"\n"; return 0; }
  catch (const std::exception &e) { std::cerr<<"FAIL "<<(argc>1?argv[1]:"")<<": "<<e.what()<<"\n"; return 1; }
}
