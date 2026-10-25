#pragma once
#include <vector>
#include <utility>
namespace Mock {
inline int servo[64]={}, attachAttempts[64]={}; inline int rejectServoPin=-1;
inline std::vector<std::pair<int,int>> servoWrites;
inline bool servoAttached[64]={};inline std::vector<int> detachedPins;
}
class Servo {
 int pin=0; bool connected=false;
public:
 void attach(int p,int=500,int=2500){pin=p;++Mock::attachAttempts[p];connected=p!=Mock::rejectServoPin;Mock::servoAttached[p]=connected;}
 bool attached(){return connected;}
 void detach(){if(connected)Mock::detachedPins.push_back(pin);connected=false;Mock::servoAttached[pin]=false;}
 void writeMicroseconds(int value){if(connected){Mock::servo[pin]=value;Mock::servoWrites.push_back({pin,value});}}
};
