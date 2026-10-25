#pragma once
#include "Arduino.h"
struct AlfredoCRSF {
  inline static AlfredoCRSF *instance=nullptr;
  int channels[17]={};
  bool linked=false;
  uint32_t updateCount=0;
  AlfredoCRSF(){instance=this;}
  void begin(MockSerial&){}
  void update(){++updateCount;}
  int getChannel(unsigned int number)const{return channels[number];}
  bool isLinkUp()const{return linked;}
};
