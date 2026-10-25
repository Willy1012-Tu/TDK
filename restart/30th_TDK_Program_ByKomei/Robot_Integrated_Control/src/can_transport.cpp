#include "can_transport.h"
#include "system_config.h"
#include "chassis_controller.h"
#include "mechanism_controller.h"
#include <string.h>

namespace {
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;
void dispatch(uint8_t bus, const CAN_message_t &frame) {
  if (frame.flags.extended || frame.flags.remote || frame.id>0x7FF || frame.len>8) return;
  if (bus==1) {
    for (uint8_t id:SystemConfig::Mechanism::MOTOR_IDS) {
      if (frame.id==id) { MechanismController::receive(bus,frame); return; }
    }
  }
  ChassisController::receive(bus,frame);
}
}
namespace CanTransport {
void begin() {
  can1.begin(); can1.setBaudRate(SystemConfig::CAN_BAUD); can1.enableFIFO();
  can2.begin(); can2.setBaudRate(SystemConfig::CAN_BAUD); can2.enableFIFO();
}
void poll() {
  can1.events(); can2.events();
  CAN_message_t frame{};
  // Each bus has its own budget so CAN1 traffic cannot starve CAN2 heartbeats.
  for (uint16_t n=0;n<SystemConfig::CAN_RX_BUDGET && can1.read(frame);++n) dispatch(1,frame);
  for (uint16_t n=0;n<SystemConfig::CAN_RX_BUDGET && can2.read(frame);++n) dispatch(2,frame);
}
bool send(uint8_t bus, uint32_t id, const uint8_t *data, uint8_t length) {
  if ((bus!=1 && bus!=2) || id>0x7FF || length>8 || (length && !data)) return false;
  CAN_message_t frame{}; frame.id=id; frame.len=length;
  if (length) memcpy(frame.buf,data,length);
  return (bus==1?can1.write(frame):can2.write(frame))!=0;
}
}
