#pragma once
#include <vector>
#include <stdexcept>
#include "IntervalTimer.h"
namespace Mock {
inline bool started=false; inline int rejectThread=-1, lockDepth=0;
inline std::vector<void(*)()> workers; inline std::vector<int> killed;
inline std::vector<unsigned> delays; inline void(*delayHook)(unsigned)=nullptr;
}
class Threads {
public:
 class Mutex {bool held=false;public:void lock(){if(held)throw std::runtime_error("互斥鎖重複取得");held=true;++Mock::lockDepth;}void unlock(){held=false;--Mock::lockDepth;}};
 class Scope{Mutex&m;public:Scope(Mutex&v):m(v){m.lock();}~Scope(){m.unlock();}};
 class Suspend{bool previous;public:Suspend():previous(Mock::started){Mock::started=false;}~Suspend(){Mock::started=previous;}};
 int addThread(void(*fn)(),int,int){Mock::workers.push_back(fn);int id=int(Mock::workers.size());return id==Mock::rejectThread?-1:id;}
 void setDefaultTimeSlice(unsigned){}void setTimeSlice(int,unsigned){}void start(){Mock::started=true;}
 void kill(int id){Mock::killed.push_back(id);}
 void delay(unsigned ms){if(Mock::lockDepth)throw std::runtime_error("持鎖時呼叫 threads.delay");Mock::delays.push_back(ms);Mock::advance(ms);if(Mock::delayHook)Mock::delayHook(ms);}
};
inline Threads threads;
