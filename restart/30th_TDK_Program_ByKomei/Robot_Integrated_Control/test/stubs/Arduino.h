#pragma once
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <string>
#include <cstdio>
#include <algorithm>
#include <vector>
constexpr int HEX = 16;
inline uint32_t mockMillis = 0;
inline uint32_t millis() { return mockMillis; }
inline uint32_t micros() { return mockMillis * 1000U; }
constexpr uint8_t LOW = 0, HIGH = 1, INPUT = 0, OUTPUT = 1, INPUT_PULLUP = 2;
namespace Mock {
struct GpioWrite { uint32_t at; uint8_t pin, value; };
struct GpioModeWrite { uint32_t at; uint8_t pin, mode, value; };
struct PwmWrite { uint32_t at; uint8_t pin; int value; };
struct PwmResolutionWrite { uint32_t at; uint32_t bits; };
inline uint8_t gpioState[256]{};
inline uint8_t gpioMode[256]{};
inline std::vector<GpioWrite> gpioWrites;
inline std::vector<GpioModeWrite> gpioModes;
// PWM duty is separate from digital GPIO states to expose H-bridge pin conflicts.
inline int pwmDuty[256]{};
inline uint32_t pwmResolution = 0;
inline std::vector<PwmWrite> pwmWrites;
inline std::vector<PwmResolutionWrite> pwmResolutions;
}
inline void pinMode(uint8_t pin, uint8_t mode) {
    Mock::gpioMode[pin] = mode; Mock::gpioModes.push_back({millis(),pin,mode,Mock::gpioState[pin]});
}
inline void digitalWrite(uint8_t pin, uint8_t value) {
    Mock::gpioState[pin] = value; Mock::gpioWrites.push_back({millis(),pin,value});
}
inline int digitalRead(uint8_t pin) { return Mock::gpioState[pin]; }
inline void analogWriteResolution(uint32_t bits) {
    Mock::pwmResolution = bits; Mock::pwmResolutions.push_back({millis(),bits});
}
inline void analogWrite(uint8_t pin, int value) {
    Mock::pwmDuty[pin] = value; Mock::pwmWrites.push_back({millis(),pin,value});
}
template<class T> T constrain(T x,T a,T b){return std::min(b,std::max(a,x));}
class String {
 std::string value;
public:
 String(const char *s):value(s){}
 String(uint32_t n,int){char b[16];snprintf(b,sizeof b,"%lx",static_cast<unsigned long>(n));value=b;}
 String& operator+=(const char*s){value+=s;return *this;}
 String& operator+=(const String&s){value+=s.value;return *this;}
 const char*c_str()const{return value.c_str();}
};
struct MockSerial {
 bool connected=true; int capacity=128; uint32_t beginBaud=0;
 std::string output,input;
 int available(){return static_cast<int>(input.size());}
 int read(){if(input.empty())return -1;unsigned char c=input.front();input.erase(0,1);return c;}
 void begin(uint32_t baud){beginBaud=baud;}
 void println(const char *s){output+=s;output+="\n";}
 template<class...Args>void printf(const char*format,Args...args){char b[4096];snprintf(b,sizeof b,format,args...);output+=b;}
 explicit operator bool()const{return connected;}
 int availableForWrite(){return capacity;}
 size_t write(const uint8_t*p,size_t n){
   const size_t accepted=capacity>0?std::min(n,static_cast<size_t>(capacity)):0;
   output.append(reinterpret_cast<const char*>(p),accepted);return accepted;
 }
};
inline MockSerial Serial,Serial2;