#include "system_safety.h"
#include "chassis_controller.h"
#include "mechanism_controller.h"

namespace {
bool emergency=false;
const char *emergencyReason="Ready";
}
namespace SystemSafety {
void stop(const char *message) {
  if (emergency) return;
  emergency=true;
  emergencyReason=message?message:"Operator emergency stop";
  ChassisController::emergencyStop();
  MechanismController::stop();
}
bool stopped() { return emergency; }
const char *reason() { return emergencyReason; }
}
