#pragma once

// Global STOP is latched until the controller is restarted.
namespace SystemSafety {
void stop(const char *reason = "Operator emergency stop");
bool stopped();
const char *reason();
}
