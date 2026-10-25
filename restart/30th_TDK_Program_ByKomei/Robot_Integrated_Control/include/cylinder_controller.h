#pragma once
#include "cylinder_types.h"
namespace CylinderController {
struct Status {
  MechanismConfig::CylinderAction action;
  bool active;
  uint32_t durationMs, remainingMs;
  bool in1High, in2High;
  MechanismConfig::CylinderAction referenceAction;
  bool referenceKnown; // Completed timed command estimate for PREV; no limit-switch feedback.
  bool fullyRetracted; // A full HOME_RETRACT_MS retract completed; still only a timed estimate.
};
// Hardware only: no timers, state, Serial or C++ objects; safe before constructors.
void preparePins();
void begin();
void poll();
bool start(MechanismConfig::CylinderAction action, uint32_t durationMs);
void stop();
void invalidateReference();
Status status();
} // namespace CylinderController
