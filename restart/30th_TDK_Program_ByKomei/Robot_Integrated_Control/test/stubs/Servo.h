#pragma once
#include <stdint.h>
#include <vector>
namespace Mock {
inline int servo[64] = {};
inline bool servoAttached[64] = {};
inline int rejectServoPin = -1;
struct ServoWrite { uint8_t pin; int pulse; };
inline std::vector<ServoWrite> servoWrites;
}
class Servo {
  int pin = 0;
  bool connected = false;
public:
  void attach(int p, int = 500, int = 2500) {
    pin = p; connected = p != Mock::rejectServoPin; Mock::servoAttached[pin] = connected;
  }
  bool attached() { return connected; }
  void detach() { connected = false; Mock::servoAttached[pin] = false; }
  void writeMicroseconds(int value) {
    if (connected) { Mock::servo[pin] = value; Mock::servoWrites.push_back({uint8_t(pin), value}); }
  }
};
