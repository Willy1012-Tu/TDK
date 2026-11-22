#pragma once
namespace Mock {inline void(*tick)()=nullptr;inline bool timerOK=true;inline void advance(unsigned n){while(n--)if(tick)tick();}}
class IntervalTimer{public:bool begin(void(*fn)(),unsigned){Mock::tick=fn;return Mock::timerOK;}};
