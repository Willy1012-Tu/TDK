// Generated stage 1 action profile. DO NOT EDIT.
#pragma once
namespace Stage1Basket {
using namespace MechanismConfig;
constexpr uint8_t STAGE_ID = 1;
constexpr uint8_t PROP_ID = 1;
constexpr uint8_t REPEAT_FROM = 1;
constexpr Action YAML_STEP_1[] = {
  MechanismConfig::branch(MechanismConfig::stepperFor(6, 8000, 1000, 230, false), 1),
  MechanismConfig::branch(MechanismConfig::servo(5, 2056, 0, false), 1),
  MechanismConfig::branch(MechanismConfig::servo(2, 1650, 0, false), 1),
  MechanismConfig::branch(MechanismConfig::waitAll(), 1),
};
constexpr Action YAML_STEP_2[] = {
  MechanismConfig::branch(MechanismConfig::servo(2, 2100, 1000, true), 1),
  MechanismConfig::branch(MechanismConfig::stepperFor(6, 3000, 500, 150, true), 2),
};
constexpr Action YAML_STEP_3[] = {
  MechanismConfig::branch(MechanismConfig::servo(2, 1300, 1000, true), 1),
};
constexpr Step STEPS[] = {
  MechanismConfig::step(YAML_STEP_1),
  MechanismConfig::step(YAML_STEP_2),
  MechanismConfig::step(YAML_STEP_3),
};
constexpr uint8_t STEP_COUNT = 3;
}
