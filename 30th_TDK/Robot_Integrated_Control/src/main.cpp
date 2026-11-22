// Cooperative scheduling: no motion waits, USB waits, or delay() in the main loop.
#include <Arduino.h>
#include "system_config.h"
#include "can_transport.h"
#include "chassis_controller.h"
#include "mechanism_controller.h"
#include "rc_controller.h"
#include "serial_gateway.h"
#include "pwm_controller.h"
#include "cylinder_controller.h"

void setup() {
  // Reassert STOP before USB Serial/CAN/UART/Servo initialization.
  CylinderController::preparePins();
  Serial.begin(SystemConfig::SERIAL_BAUD);
  PwmController::begin();
  CanTransport::begin();
  RcController::begin();
  ChassisController::begin();
  MechanismController::begin();
  SerialGateway::begin();
}
void loop() {
  CanTransport::poll();
  SerialGateway::poll();
  RcController::poll();
  ChassisController::poll();
  MechanismController::poll();
  PwmController::poll();
}
