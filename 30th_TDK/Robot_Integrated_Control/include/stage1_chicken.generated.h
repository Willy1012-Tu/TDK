// Generated stage 1 action profile. DO NOT EDIT.
#pragma once
namespace Stage1Chicken {
using namespace MechanismConfig;
constexpr uint8_t STAGE_ID = 1;
constexpr uint8_t PROP_ID = 4;
constexpr uint8_t REPEAT_FROM = 1;
constexpr Action YAML_STEP_1[] = {
  MechanismConfig::branch(MechanismConfig::stepperFor(6, 9200, 1000, 230, false), 1),
  MechanismConfig::branch(MechanismConfig::servo(5, 1722, 0, false), 1),
  MechanismConfig::branch(MechanismConfig::servo(2, 1611, 0, false), 1),
  MechanismConfig::branch(MechanismConfig::waitAll(), 1),
};
constexpr Action YAML_STEP_2[] = {
  MechanismConfig::branch(MechanismConfig::servo(2, 889, 0, true), 1),
};
constexpr Action YAML_STEP_3[] = {
  MechanismConfig::branch(MechanismConfig::stepperFor(6, -1300, 1500, 240, true), 1),
  MechanismConfig::branch(MechanismConfig::servo(5, 1333, 1000, true), 2),
};
constexpr Action YAML_STEP_4[] = {
  MechanismConfig::branch(MechanismConfig::servo(2, 1611, 0, true), 1),
  MechanismConfig::branch(MechanismConfig::servo(5, 1444, 0, true), 2),
};
constexpr Step STEPS[] = {
  MechanismConfig::step(YAML_STEP_1),
  MechanismConfig::step(YAML_STEP_2),
  MechanismConfig::step(YAML_STEP_3),
  MechanismConfig::step(YAML_STEP_4),
};
constexpr uint8_t STEP_COUNT = 4;
}
