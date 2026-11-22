"""Validate YAML and generate the firmware action table (also used by host tests)."""
import argparse
import sys
from pathlib import Path

DEPENDENCIES = Path(__file__).resolve().parents[1] / ".pio" / "python-deps"
sys.path.insert(0, str(DEPENDENCIES))
try:
    import yaml
except ImportError:
    raise SystemExit("Missing PyYAML. Run PlatformIO Build first, or install tools/requirements-yaml.txt into .pio/python-deps.")


class ConfigError(ValueError):
    pass


MOTOR_IDS = (6, 7)
SERVO_PINS = (2, 5, 4, 6, 9)
STAGE_IDS = (2, 3, 4)
MAX_ACTIONS = 2048


class StrictLoader(yaml.SafeLoader):
    """Reject duplicate keys instead of silently discarding the first value."""


def mapping(loader, node):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node)
        if not isinstance(key, str):
            raise ConfigError(f"line {key_node.start_mark.line + 1}: keys must be strings")
        if key in result:
            raise ConfigError(f"line {key_node.start_mark.line + 1}: duplicate key {key!r}")
        result[key] = loader.construct_object(value_node)
    return result


StrictLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, mapping)


def keys(value, allowed, required, location):
    if not isinstance(value, dict):
        raise ConfigError(f"{location}: expected a mapping")
    unknown, missing = set(value) - set(allowed), set(required) - set(value)
    if unknown or missing:
        raise ConfigError(f"{location}: unknown keys {sorted(unknown)}, missing keys {sorted(missing)}")


def integer(value, low, high, location):
    if type(value) is not int or not low <= value <= high:
        raise ConfigError(f"{location}: expected integer {low}..{high}, got {value!r}")
    return value


def action(value, location, wait=True):
    if not isinstance(value, dict) or len(value) != 1:
        raise ConfigError(f"{location}: each action must have exactly one type")
    kind, params = next(iter(value.items()))
    flag = "true" if wait else "false"
    if kind == "stepper":
        keys(params, ("node", "position", "rpm", "acc"), ("position", "rpm"), location)
        node = integer(params.get("node", 6), 4, 7, location + ".node")
        if node not in MOTOR_IDS:
            raise ConfigError(f"{location}.node: expected a configured motor ID {MOTOR_IDS}")
        position = integer(params["position"], -8388607, 8388607, location + ".position")
        rpm = integer(params["rpm"], 1, 3000, location + ".rpm")
        acc = integer(params.get("acc", 0), 0, 255, location + ".acc")
        helper = f"Stage4::stepperFor({node}, " if "node" in params else "Stage4::stepper("
        return [f"{helper}{position}, {rpm}, {acc}, {flag})"], f"stepper:{node}"
    if kind == "servo":
        keys(params, ("pin", "pulse_us", "duration_ms"), ("pin", "pulse_us"), location)
        pin = integer(params["pin"], 0, 39, location + ".pin")
        if pin not in SERVO_PINS:
            raise ConfigError(f"{location}.pin: expected a configured servo pin {SERVO_PINS}")
        pulse = integer(params["pulse_us"], 500, 2500, location + ".pulse_us")
        ms = integer(params.get("duration_ms", 0), 0, 60000, location + ".duration_ms")
        return [f"Stage4::servo({pin}, {pulse}, {ms}, {flag})"], f"servo:{pin}"
    if not wait:
        raise ConfigError(f"{location}: parallel supports only stepper/servo actions")
    if kind == "delay":
        keys(params, ("ms",), ("ms",), location)
        return [f"Stage4::delay({integer(params['ms'], 0, 60000, location + '.ms')})"], None
    if kind == "parallel":
        if not isinstance(params, list) or not 1 <= len(params) <= 7:
            raise ConfigError(f"{location}: parallel requires 1..7 distinct actuators")
        compiled, devices = [], set()
        for index, child in enumerate(params):
            code, device = action(child, f"{location}.parallel[{index + 1}]", wait=False)
            if device in devices:
                raise ConfigError(f"{location}: actuator {device} appears twice in parallel")
            devices.add(device)
            compiled.append((device, code))
        # Queue the motor first: if CAN rejects it, no servo starts prematurely.
        compiled.sort(key=lambda item: not item[0].startswith("stepper:"))
        return [code for _, codes in compiled for code in codes] + ["Stage4::waitAll()"], None
    raise ConfigError(f"{location}: unknown action {kind!r}; use stepper, servo, delay, parallel")


def compile_document(document):
    keys(document, ("stage", "repeat_from", "steps"), ("steps",), "root")
    stage = integer(document.get("stage", 4), 2, 4, "root.stage")
    steps = document["steps"]
    if not isinstance(steps, list) or not 1 <= len(steps) <= 254:
        raise ConfigError("steps: expected 1..254 steps")
    repeat_from = integer(document.get("repeat_from", 1), 1, len(steps), "root.repeat_from")
    if stage == 3 and (repeat_from != 1 or steps != [{"actions": [{"delay": {"ms": 0}}]}]):
        raise ConfigError("stage 3 is disconnected: use exactly one delay 0 standby step, repeat_from 1")
    lines = [f"// Generated stage {stage} action profile. DO NOT EDIT.", "#pragma once", f"namespace Stage{stage} {{"]
    if stage != 4:
        lines.append("using namespace Stage4;")
    lines.extend([f"constexpr uint8_t STAGE_ID = {stage};", f"constexpr uint8_t REPEAT_FROM = {repeat_from};"])
    total = 0
    for index, entry in enumerate(steps):
        location = f"steps[{index + 1}]"
        keys(entry, ("actions",), ("actions",), location)
        actions = entry["actions"]
        if not isinstance(actions, list) or not 1 <= len(actions) <= 256:
            raise ConfigError(f"{location}.actions: expected 1..256 actions")
        code = []
        for number, item in enumerate(actions, 1):
            compiled, _ = action(item, f"{location}.actions[{number}]")
            code.extend(f"Stage4::branch({item}, {number})" for item in compiled)
        total += len(code)
        if total > MAX_ACTIONS:
            raise ConfigError(f"steps: exceeds {MAX_ACTIONS} generated actions (RAM capacity)")
        lines.append(f"constexpr Action YAML_STEP_{index + 1}[] = {{")
        lines.extend(f"  {item}," for item in code)
        lines.append("};")
    lines.append("constexpr Step STEPS[] = {")
    lines.extend(f"  Stage4::step(YAML_STEP_{index + 1})," for index in range(len(steps)))
    lines.extend(["};", "}", ""])
    return "\n".join(lines)


def generate(source, target, expected_stage=None):
    document=yaml.load(Path(source).read_text(encoding="utf-8-sig"), Loader=StrictLoader)
    code=compile_document(document)
    if expected_stage is not None and document.get("stage",4)!=expected_stage:
        raise ConfigError(f"root.stage: expected {expected_stage} for {Path(source).name}")
    target = Path(target)
    target.parent.mkdir(parents=True, exist_ok=True)
    if not target.exists() or target.read_text(encoding="utf-8") != code:
        target.write_text(code, encoding="utf-8")


def wire_document(document):
    """Use the same validation as Build, then expand to the serial RAM format."""
    compile_document(document)
    actions, steps = [], []
    def emit(item, branch, wait):
        kind, params = next(iter(item.items()))
        if kind == "parallel":
            ordered=sorted(params, key=lambda child: "stepper" not in child)
            for child in ordered:
                emit(child, branch, False)
            actions.append([3,0,0,0,0,0,0,1,branch,0])
        elif kind == "stepper":
            actions.append([0,params["position"],params["rpm"],params.get("acc",0),0,0,0,int(wait),branch,params.get("node",6)])
        elif kind == "servo":
            actions.append([1,0,0,0,params["pin"],params["pulse_us"],params.get("duration_ms",0),int(wait),branch,0])
        elif kind == "delay":
            actions.append([2,0,0,0,0,0,params["ms"],1,branch,0])
    for entry in document["steps"]:
        start=len(actions)
        for branch, item in enumerate(entry["actions"],1):
            emit(item, branch, True)
        steps.append([start,len(actions)-start])
    return actions, steps


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("target", type=Path)
    parser.add_argument("--expected-stage",type=int,choices=STAGE_IDS)
    args = parser.parse_args()
    try:
        generate(args.source, args.target,args.expected_stage)
    except (ConfigError, yaml.YAMLError, OSError) as exc:
        parser.exit(1, f"YAML configuration error in {args.source}: {exc}\n")
