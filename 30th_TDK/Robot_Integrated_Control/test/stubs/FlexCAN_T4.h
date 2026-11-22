#pragma once
#include "Arduino.h"
#include <vector>
#include <deque>
struct CAN_message_t {uint32_t id=0;uint8_t len=0;struct{bool extended=false,remote=false;}flags;uint8_t buf[8]={};};
constexpr int CAN1=1,CAN2=2,RX_SIZE_256=256,TX_SIZE_16=16;
namespace Mock {
struct Frame{int bus;CAN_message_t msg;};
inline std::vector<Frame> sent;
inline std::deque<CAN_message_t> incoming[3];
inline int rejectNode=-1,rejectCommand=-1;
inline uint32_t baud[3]={};
}
template<int Bus,int Rx,int Tx> class FlexCAN_T4 {
public:
 void begin(){} void setBaudRate(uint32_t b){Mock::baud[Bus]=b;} void enableFIFO(){} void events(){}
 void enableMBInterrupts(){} void onReceive(void(*)(const CAN_message_t&)){}
 int write(const CAN_message_t&m){if(int(m.id)==Mock::rejectNode && m.buf[0]==Mock::rejectCommand)return 0;Mock::sent.push_back({Bus,m});return 1;}
 int read(CAN_message_t&m){auto&q=Mock::incoming[Bus];if(q.empty())return 0;m=q.front();q.pop_front();return 1;}
};
