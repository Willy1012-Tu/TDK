#pragma once
#include <stdint.h>
namespace MechanismConfig {
enum class CylinderAction : uint8_t { Stop = 0, Extend = 1, Retract = 2 };
struct CylinderCommand {
  CylinderAction action = CylinderAction::Stop;
  uint32_t duration_ms = 0;
};
} // namespace MechanismConfig
