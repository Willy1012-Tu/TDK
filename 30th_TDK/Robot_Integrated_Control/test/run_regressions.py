"""Run each stateful C++ scenario in its own process and validate real wire JSON."""
import json
import math
from pathlib import Path
import re
import subprocess

project = Path(__file__).resolve().parents[1]
total = 0
for suite in ("mechanism_regression", "chassis_rc_regression", "system_regression", "bench_regression", "pwm_regression", "chassis_motion_regression"):
    source = (project / "test" / (suite + ".cpp")).read_text(encoding="utf-8-sig")
    cases = list(dict.fromkeys(re.findall(r'name\s*==\s*"([a-z0-9_]+)"', source)))
    if not cases:
        raise RuntimeError("No scenarios found in " + suite)
    for case in cases:
        subprocess.run([str(project / ".pio" / "host-tests" / (suite + ".exe")), case],
                       cwd=project, check=True)
    total += len(cases)
    print(f"{suite}: {len(cases)} scenarios PASS", flush=True)
fixture = project / ".pio" / "host-tests" / "status_fixture.json"
def reject_nonfinite(value):
    raise ValueError("Non-finite JSON constant: " + value)


def load_wire(path):
    return json.loads(path.read_text(encoding="utf-8"), parse_constant=reject_nonfinite)


# Read configurable field tuning independently of firmware telemetry.
config_source = (project / "include" / "system_config.h").read_text(encoding="utf-8-sig")
def configured_float(name):
    match = re.search(r"\bconstexpr\s+float\s+" + re.escape(name) +
                      r"\s*=\s*([-+]?(?:\d+(?:\.\d*)?|\.\d+))[fF]?\s*;", config_source)
    if not match:
        raise RuntimeError("Missing numeric configuration " + name)
    return float(match.group(1))
joystick_speed = configured_float("JOYSTICK_MAX_SPEED")
safety_speed = configured_float("SAFETY_MAX_SPEED")
assert 0 < joystick_speed <= safety_speed <= 115
status = load_wire(fixture)
assert status["protocol"] == 1 and status["stage_ids"] == [1, 2, 3, 4, 5]
assert status["firmware_id"] == "Robot_Integrated_Control"
assert status["firmware_revision"] == "bench-test-v7"
assert status["bench_test_mode"] is False
assert status["motor_online_mask"] == 7 and status["motor_offline_mask"] == 0
assert status["startup_return_zero"] is True and status["startup_phase"] == "ready"
assert status["position_known_mask"] == 7
assert status["startup_current_settings"] == [{"node": 4, "milliamps": 3000}, {"node": 5, "milliamps": 3000}, {"node": 6, "milliamps": 5200}]
assert status["startup_current_ma"] == 0 and status["current_configured_mask"] == 7
assert status["current4_ma"] == status["current5_ma"] == 3000 and status["current6_ma"] == 5200
assert all(isinstance(status[key], int) for key in ("startup_node", "startup_command", "startup_attempts"))
assert status["motor_ids"] == [4, 5, 6] and status["max_runtime_actions"] == 512
assert len(status["rc"]["channels"]) == 16 and len(status["chassis"]["wheels"]) == 4
assert status["rc"]["channels"][:3] == [1454, 1416, 1500]  # CH1 rotate, CH2 forward, CH3 strafe.
assert status["rc"]["channels"][3] == 1500  # CH4 trim starts centered for legacy scenarios.
assert status["rc"]["channels"][8] == 1000  # CH9 clear-errors is released at startup.
assert status["rc"]["channels"][9:14] == [1000, 1000, 2000, 1000, 1000]  # Normal CH12 selects Stage4; CH14 is General procession / Stage1 bird.
assert status["rc"]["channels"][6] == 1000  # CH7 LOW, normal mode.
assert status["rc"]["stage1_mode"] is False and status["rc"]["prop"] == 0 and status["rc"]["switch_position"] == 0
assert status["stage1_prop_ids"] == [1,2,3,4,5]
assert status["prop"] == 0 and status["pending_prop"] == 0 and status["prop_complete"] is False
assert status["rc"]["stage"] == 4 and status["chassis"]["state"] == "running"
assert status["chassis"]["direction_stage"] == 4

assert [(w["node"], w["bus"]) for w in status["chassis"]["wheels"]] == [(10,1),(11,1),(12,2),(13,2)]
assert isinstance(status["previous_available"], bool) and isinstance(status["stopped"], bool)
assert status["servo_pins"] == [2, 3, 4, 5, 6, 9, 10, 11, 12] and status["stage3_enabled"] is True
assert [status[f"pulse{pin}"] for pin in (2, 3, 4, 5, 6, 9, 10, 11, 12)] == [1150, 2250, 2000, 2100, 1400, 1100, 1500, 1500, 1500]

# Validate CH4 telemetry from the actual CRSF -> main loop -> PWM -> USB path.
trim = load_wire(project / ".pio" / "host-tests" / "stage1_trim_status_fixture.json")
assert trim["stage"] == 1 and trim["rc"]["stage1_mode"] is True
assert trim["rc"]["channels"][3] == 2000 and trim["rc"]["stage1_trim_us"] == 150
assert trim["pulse2_base"] == 1150 and trim["pulse2"] == 1300
assert trim["stage1_trim_us"] == 150 and trim["stage1_trim_allowed"] is True
home_trim = load_wire(project / ".pio" / "host-tests" / "stage1_trim_home_status_fixture.json")
assert home_trim["stage"] == 1 and home_trim["pulse2_base"] == home_trim["pulse2"] == 1150
assert home_trim["stage1_trim_us"] == 0 and home_trim["stage1_trim_allowed"] is False
assert status["rc"]["stage1_trim_us"] == status["stage1_trim_us"] == 0
assert status["pulse2_base"] == status["pulse2"] == 1150 and status["stage1_trim_allowed"] is False
print("PASS CH4 trim base/effective PWM telemetry and explicit HOME suspension", flush=True)

# Validate real serial dispatch and independent channel telemetry, not GUI fixtures.
servo9 = load_wire(project / ".pio" / "host-tests" / "servo9_debug_status_fixture.json")
assert servo9["servo_pins"] == [2, 3, 4, 5, 6, 9, 10, 11, 12]
assert [servo9[f"pulse{pin}"] for pin in (10,11,12)] == [500,2500,1000]
assert [servo9[f"pin{pin}"] for pin in (10,11,12)] == [10,11,12]
assert [servo9[f"pulse{pin}"] for pin in (2,3,4,5,6,9)] == [1150,2250,2000,2100,1400,1100]
assert servo9["mode"] == "debug" and servo9["stopped"] is False
assert len((project / ".pio" / "host-tests" / "servo9_debug_status_fixture.json").read_bytes()) + len("@1000000000 OK \\n") < 4096
print("PASS nine servo capabilities, independent Pin10/11/12 debug endpoints and wire telemetry", flush=True)

assert status["position6"] == 0
assert "position7" not in status and "current7_ma" not in status

assert status["pulse3"] == 2250
assert status["startup_home_ids"] == [6] and status["startup_hold_ids"] == [4,5]
assert not any(key in status for key in ("imu", "yaw", "pitch", "roll"))
assert "yaw_compensation" not in status["chassis"]

for node, expected_mask, expected_currents in (
    (4, 0, [0, 0, 0]),
    (5, 1, [3000, 0, 0]),
    (6, 3, [3000, 3000, 0]),
):
    pending = load_wire(project / ".pio" / "host-tests" / f"current_wait{node}_status_fixture.json")
    assert pending["startup_phase"] == "current_wait" and pending["startup_node"] == node
    assert pending["startup_command"] == 0x83 and pending["startup_current_ma"] == (5200 if node == 6 else 3000)
    assert pending["current_configured_mask"] == expected_mask
    assert [pending[f"current{axis}_ma"] for axis in (4, 5, 6)] == expected_currents
    assert pending["position_known_mask"] == 0 and pending["stopped"] is False
    assert pending["chassis"]["state"] == "running" and len(pending["rc"]["channels"]) == 16
    assert pending["startup_current_settings"] == status["startup_current_settings"]
print("PASS three-axis current telemetry: pending masks 0/1/3, ready mask7", flush=True)


# Parse actual cooperative-loop output, independent of mocked GUI protocol replies.
props = load_wire(project / ".pio" / "host-tests" / "stage1_props_status_fixture.json")
assert props["stage"] == 1 and props["prop"] == 5 and props["pending_prop"] == 0
assert props["prop_complete"] is True and props["completed"] == 4 and props["steps"] == 4
assert props["position6"] == 4000 and props["pulse2"] == 1667 and props["pulse5"] == 1944
assert props["rc"]["stage1_mode"] is True and props["rc"]["prop"] == 5 and props["rc"]["switch_position"] == 2
assert props["stage_ids"] == [1,2,3,4,5] and props["stage1_prop_ids"] == [1,2,3,4,5]
ram = load_wire(project / ".pio" / "host-tests" / "stage1_ram_status_fixture.json")
assert ram["stage"] == 1 and ram["prop"] == 1 and ram["runtime_plan"] is True
assert ram["prop_complete"] is True and ram["position6"] == 123 and ram["plan_version"] == 5
spare = load_wire(project / ".pio" / "host-tests" / "stage5_status_fixture.json")
assert spare["stage"] == 5 and spare["prop"] == 0 and spare["pending_prop"] == 0
assert spare["completed"] == 1 and spare["prop_complete"] is False
assert spare["rc"]["stage"] == 5 and spare["rc"]["stage1_mode"] is False
assert spare["rc"]["switch_position"] == 0 and spare["rc"]["prop"] == 0
assert [spare[f"position{node}"] for node in (4,5,6)] == [0,0,0]
assert [spare[f"pulse{pin}"] for pin in (2,3,4,5,6,9,10,11,12)] == [1150,2250,2000,2100,1400,1100,1500,1500,1500]
print("PASS five-prop identity, bird lift/release completion, independent RAM prop banks, and Stage5 standby wire JSON", flush=True)


def cylinder_status(wire):
    cylinder = wire["cylinder"]
    assert cylinder["available"] is True
    assert cylinder["action"] in (0,1,2) and cylinder["reference_action"] in (0,1,2)
    assert isinstance(cylinder["active"],bool) and isinstance(cylinder["reference_known"],bool)
    assert cylinder["in1_pin"] == 17 and cylinder["in2_pin"] == 16
    assert isinstance(cylinder["in1_high"],bool) and isinstance(cylinder["in2_high"],bool)
    assert not (cylinder["in1_high"] and cylinder["in2_high"])
    assert 0 <= cylinder["remaining_ms"] <= cylinder["duration_ms"] <= 60000
    assert "position7" not in wire and "current7_ma" not in wire and wire["motor_ids"] == [4,5,6]
    return cylinder
boot_cylinder = cylinder_status(status)
assert boot_cylinder["action"] == 0 and boot_cylinder["active"] is False
assert boot_cylinder["in1_high"] is False and boot_cylinder["in2_high"] is False and boot_cylinder["reference_known"] is False
travel = load_wire(project / ".pio" / "host-tests" / "cylinder_live_status_fixture.json")
live_cylinder = cylinder_status(travel)
assert live_cylinder["action"] == 1 and live_cylinder["active"] is True
assert live_cylinder["in1_high"] is True and live_cylinder["in2_high"] is False
assert live_cylinder["reference_known"] is False and live_cylinder["duration_ms"] == 6000
assert 0 < live_cylinder["remaining_ms"] <= 6000
print("PASS timed-cylinder GPIO/status JSON and complete ID7 removal", flush=True)


# The same actual modules run in strict competition mode, explicit bench mode,
# and once without a test override to catch an accidentally disabled production default.
for filename, online, known, stage in (
    ("bench_offline_status_fixture.json", 0, 0, 4),
    ("bench_id6_status_fixture.json", 4, 4, 4),
    ("bench_pair_status_fixture.json", 3, 3, 3),
    ("bench_stage2_status_fixture.json", 0, 0, 2),
):
    wire = load_wire(project / ".pio" / "host-tests" / filename)
    assert wire["firmware_revision"] == "bench-test-v7" and wire["bench_test_mode"] is True
    assert wire["motor_online_mask"] == online and wire["motor_offline_mask"] == (7 ^ online)
    assert wire["position_known_mask"] == known
    assert wire["stopped"] is False and wire["phase"] == "ready"
    assert wire["stage"] == stage and wire["motor_ids"] == [4, 5, 6]
    assert wire["startup_home_ids"] == [6] and wire["startup_hold_ids"] == [4, 5]
    assert wire["servo_pins"] == [2, 3, 4, 5, 6, 9, 10, 11, 12]
    assert wire["chassis"]["state"] == "locked"
    assert all(wheel["velocity"] == 0 for wheel in wire["chassis"]["wheels"])
    assert wire["rc"]["connected"] is True and wire["rc"]["valid"] is True
    cylinder_status(wire)
bench_stage2 = load_wire(project / ".pio" / "host-tests" / "bench_stage2_status_fixture.json")
assert bench_stage2["completed"] == 2 and bench_stage2["pulse4"] == 2100 and bench_stage2["pulse9"] == 1000
assert bench_stage2["cylinder"]["reference_known"] is True
assert bench_stage2["cylinder"]["reference_action"] == 2
subprocess.run([str(project / ".pio" / "host-tests" / "bench_default_regression.exe"), "bench_default_cylinder_only"],
               cwd=project, check=True)
total += 1
print("PASS strict-mode and bench-mode wire masks, offline chassis, isolated mechanisms, and production-default bench smoke", flush=True)

def pwm_status(wire):
    pwm = wire["pwm"]
    assert set(pwm) == {"available", "pin", "value", "max_value", "step_interval_ms",
                        "default_max_value", "default_step_interval_ms",
                        "min_step_interval_ms", "max_step_interval_ms"}
    assert pwm["available"] is True and pwm["pin"] == 14
    assert all(type(pwm[field]) is int for field in set(pwm) - {"available"})
    assert 0 <= pwm["value"] <= pwm["max_value"] <= 255
    assert 1 <= pwm["step_interval_ms"] <= 1000
    assert pwm["default_max_value"] == 55 and pwm["default_step_interval_ms"] == 5
    assert pwm["min_step_interval_ms"] == 1 and pwm["max_step_interval_ms"] == 1000
    return pwm

for filename in ("status_fixture.json", "pwm_default_status_fixture.json",
                 "bench_offline_status_fixture.json", "bench_stage2_status_fixture.json"):
    pwm = pwm_status(load_wire(project / ".pio" / "host-tests" / filename))
    assert pwm["max_value"] == 55 and pwm["step_interval_ms"] == 5
hot = load_wire(project / ".pio" / "host-tests" / "pwm_live_status_fixture.json")
assert pwm_status(hot)["max_value"] == 120 and hot["pwm"]["step_interval_ms"] == 10
assert hot["stopped"] is False and hot["phase"] == "busy"
assert hot["cylinder"]["active"] is True and hot["chassis"]["state"] == "running"
disabled = load_wire(project / ".pio" / "host-tests" / "pwm_stopped_status_fixture.json")
assert pwm_status(disabled)["max_value"] == disabled["pwm"]["value"] == 0
assert disabled["pwm"]["step_interval_ms"] == 1000 and disabled["stopped"] is True
assert disabled["phase"] == "fault" and disabled["cylinder"]["active"] is False
assert all(wheel["velocity"] == 0 for wheel in disabled["chassis"]["wheels"])
extreme_path = project / ".pio" / "host-tests" / "pwm_extreme_status_fixture.json"
extreme = load_wire(extreme_path)
assert pwm_status(extreme)["max_value"] == 255 and extreme["pwm"]["step_interval_ms"] == 1000
assert len(extreme_path.read_bytes()) + len("@1000000000 OK \n") < 4096
assert [extreme[f"position{node}"] for node in (4, 5, 6)] == [-8388608] * 3
assert extreme["rc"]["channels"] == [-2147483648] * 16
assert all(wheel["error"] == 4294967295 and wheel["axis_state"] == 255
           and wheel["heartbeat_age_ms"] > 4000000000 for wheel in extreme["chassis"]["wheels"])
assert extreme["cylinder"]["active"] is True and extreme["cylinder"]["duration_ms"] == 60000
assert extreme["stopped"] is False
print("PASS PWM default/live/STOP capability JSON, RAM settings, and numeric status-size bounds", flush=True)


# Real cooperative-loop pending direction telemetry remains distinct from mechanical active stage.
switch_pending = load_wire(project / ".pio" / "host-tests" / "stage_switch_pending_status_fixture.json")
assert switch_pending["stage"] == 2 and switch_pending["pending_stage"] == 1
assert switch_pending["phase"] == "return_wait" and switch_pending["chassis"]["direction_stage"] == 1
assert switch_pending["chassis"]["state"] == "running" and switch_pending["stopped"] is False
assert switch_pending["cylinder"]["active"] is True and switch_pending["cylinder"]["action"] == 2
assert [wheel["velocity"] for wheel in switch_pending["chassis"]["wheels"]] == [-joystick_speed] * 4
print("PASS immediate pending-stage chassis direction while mechanical return remains ACK/time gated", flush=True)

print(f"PASS wire JSON compatibility; {total} C++ scenarios total", flush=True)

motion_wire=load_wire(project / ".pio" / "host-tests" / "chassis_motion_status_fixture.json")
motion=motion_wire["chassis"]["motion"]
assert motion["available"] and motion["editable"] and motion["ramp_enabled"]
assert (motion["accel"],motion["decel"],motion["wheelbase_mm"],motion["track_mm"])==(90,180,260,320)
assert (motion["icr_x_mm"],motion["icr_left_y_mm"],motion["icr_right_y_mm"])==(0,0,0)
assert motion["icr_mode"]=="directional"
assert motion["target"]==[0,0,0,0]
print("PASS production-default chassis ramp/ICR status wire JSON",flush=True)

procession_wire=load_wire(project / ".pio" / "host-tests" / "procession_status_fixture.json")
procession=procession_wire["chassis"]["motion"]
assert procession["profiles"] and procession["profile_id"]==1
assert procession_wire["rc"]["procession_mode"] is True
assert procession_wire["chassis"]["direction_stage"]==2
assert procession_wire["stage"]==4 and procession_wire["pending_stage"]==0
assert (procession["accel"],procession["decel"],procession["wheelbase_mm"],procession["track_mm"])==(90,180,260,320)
assert (procession["icr_x_mm"],procession["icr_left_y_mm"],procession["icr_right_y_mm"])==(0,0,0)
print("PASS CH14 isolated procession bank, zero-default centers, CH10 direction and unchanged mechanism stage",flush=True)
