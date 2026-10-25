#include "servoxxd_can.h"
#include "trajectory.h"
#include "alg_pid.h"
#include <IntervalTimer.h>
#include <iostream>
#include <stdexcept>
void servoxxd_onFeedbackMessage(const CAN_message_t &);
#define CHECK(x) do{if(!(x))throw std::runtime_error(std::to_string(__LINE__)+": "+#x);}while(0)
int main(){try{
 servoxxd_can_init();Mock::sent.clear();
 for(uint8_t id: {uint8_t(0),uint8_t(7),uint8_t(255)}){
  servoxxd_motor_gohome(id);servoxxd_set_motor_rpm(id,false,320,0);
  servoxxd_set_motor_abs_pos(id,320,0,1000);servoxxd_set_abs_pos_zero(id);
  servoxxd_set_motor_gohome(id,false,false,320);
 }
 CHECK(Mock::sent.empty());servoxxd_motor_gohome(6);CHECK(servoxxd_motor[5].state==MOTOR_BUZY);
 CHECK(Mock::sent.back().msg.id==6 && Mock::sent.back().msg.buf[1]==0x97);
 servoxxd_set_motor_abs_pos(6,320,5,-600);const auto f=Mock::sent.back().msg;
 CHECK(f.len==8 && f.buf[0]==0xF5 && f.buf[4]==0xFF && f.buf[5]==0xFD && f.buf[6]==0xA8);
 CAN_message_t position{};position.id=6;position.len=8;position.buf[0]=0x31;
 for(int i=1;i<=6;++i)position.buf[i]=0xFF;
 uint8_t sum=6;for(int i=0;i<7;++i)sum+=position.buf[i];position.buf[7]=sum;
 servoxxd_onFeedbackMessage(position);CHECK(servoxxd_motor[5].abs_pos==-1);
 servoxxd_motor[5].abs_pos=123;position.len=3;servoxxd_onFeedbackMessage(position);
 CHECK(servoxxd_motor[5].abs_pos==123);position.len=8;position.buf[7]++;
 servoxxd_onFeedbackMessage(position);CHECK(servoxxd_motor[5].abs_pos==123);
 position.buf[7]--;position.flags.extended=true;servoxxd_onFeedbackMessage(position);
 CHECK(servoxxd_motor[5].abs_pos==123);
 CHECK(!Traj_Engine_Start(0) && !Traj_Engine_Start(-1));
 TrajGen_t axis{};Traj_Init(&axis,0,10,20);Traj_SetTarget(&axis,1);
 CHECK(Traj_Engine_Start(0.001f));Mock::advance(2000);
 CHECK(std::fabs(Traj_GetPosition(&axis)-1)<0.0001f && Traj_GetVelocity(&axis)==0);
 Traj_SetCurrentPos(&axis,2);CHECK(Traj_GetPosition(&axis)==2 && Traj_GetVelocity(&axis)==0);
 Class_PID pid;pid.Init(2,0,0);pid.Set_Target(3);pid.Set_Now(1);
 pid.TIM_Adjust_PeriodElapsedCallback();CHECK(pid.Get_Out()==4);
 Class_PID integral;integral.Init(0,1,0,0,0,0,1,0,10,2);
 integral.Set_Target(7);integral.TIM_Adjust_PeriodElapsedCallback();
 CHECK(std::fabs(integral.Get_Out()-3.5f)<0.0001f);
 Class_PID threshold;threshold.Init(0,1,0,0,0,0,1,0,0,2);
 threshold.Set_Target(2);threshold.TIM_Adjust_PeriodElapsedCallback();CHECK(threshold.Get_Out()==2);
 threshold.Set_Target(3);threshold.TIM_Adjust_PeriodElapsedCallback();CHECK(threshold.Get_Out()==2);
 std::cout<<"PASS library_regression (CAN ID bounds/checksum, trajectory, PID)\n";return 0;
}catch(const std::exception &e){std::cerr<<"FAIL library_regression: "<<e.what()<<"\n";return 1;}}
