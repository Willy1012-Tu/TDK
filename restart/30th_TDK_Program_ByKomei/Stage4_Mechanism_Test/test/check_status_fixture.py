"""Parse the complete status JSON emitted by the real C++ serial handler."""
import json
from pathlib import Path

fixture = Path(__file__).resolve().parents[1] / ".pio" / "host-tests" / "status_fixture.json"
status = json.loads(fixture.read_text(encoding="utf-8"))
assert status["protocol"] == 1 and status["motor_ids"] == [6, 7]
assert status["firmware_id"] == "Stage4_Mechanism_Test"
assert status["firmware_revision"] == "stage2-id67-home-v1"
assert status["startup_return_zero"] is True
assert status["phase"] == "ready" and status["startup_phase"] == "ready"
assert status["startup_command"] == 0x91 and status["startup_node"] in (0, 6, 7)
assert status["startup_attempts"] >= 1 and status["position_known_mask"] == 12
assert [status[f"position{node}"] for node in (4, 5, 6, 7)] == [0, 0, 0, 0]
print("PASS complete startup diagnostic wire JSON", flush=True)

assert status["servo_pins"] == [2, 5, 4, 6, 9] and status["stage_ids"] == [2, 3, 4]
assert status["stage3_enabled"] is False
assert [status[f"pulse{pin}"] for pin in (4, 6, 9)] == [2000, 1400, 1800]
