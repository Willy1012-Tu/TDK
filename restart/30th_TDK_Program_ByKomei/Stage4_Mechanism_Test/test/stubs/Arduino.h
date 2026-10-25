#pragma once
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <string>
#include <cstdio>
#include <algorithm>
constexpr int HEX = 16;
template<class T> T constrain(T x,T a,T b){return std::min(b,std::max(a,x));}
class String {
 std::string value;
public:
 String(const char *s):value(s){}
 String(uint32_t n,int){char b[16]; snprintf(b,sizeof b,"%lx",static_cast<unsigned long>(n));value=b;}
 String& operator+=(const char*s){value+=s;return *this;}
 String& operator+=(const String&s){value+=s.value;return *this;}
 const char*c_str()const{return value.c_str();}
};
struct MockSerial {
 bool connected=true; int capacity=128; std::string output, input;
 int available(){return static_cast<int>(input.size());}
 int read(){if(input.empty())return -1;unsigned char c=input.front();input.erase(0,1);return c;}
 void begin(uint32_t){}
 void println(const char *s){output+=s;output+="\n";}
 template<class... Args>void printf(const char *format,Args... args){char b[2048];snprintf(b,sizeof b,format,args...);output+=b;}
 explicit operator bool()const{return connected;}
 int availableForWrite(){return capacity;}
 size_t write(const uint8_t*p,size_t n){output.append(reinterpret_cast<const char*>(p),n);return n;}
};
inline MockSerial Serial,Serial2;
