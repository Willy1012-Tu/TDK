#pragma once
#include <stddef.h>
#include <stdint.h>

namespace SystemConfig {
// Bench mode allows linked RC mechanism commands while ODrive is offline/IDLE.
// Missing MKS startup acknowledgements mark the axis offline; real faults still stop.
// The override is used by host regressions to verify both bench and competition modes.
#ifdef ROBOT_BENCH_TEST_MODE
static_assert(ROBOT_BENCH_TEST_MODE == 0 || ROBOT_BENCH_TEST_MODE == 1,
              "ROBOT_BENCH_TEST_MODE must be 0 or 1");
constexpr bool BENCH_TEST_MODE = ROBOT_BENCH_TEST_MODE != 0;
#else
constexpr bool BENCH_TEST_MODE = true;
#endif
constexpr uint32_t CAN_BAUD = 1000000, SERIAL_BAUD = 115200;
// Shared ID6 lift upper limit in every stage, FE plan, trim and Debug command.
constexpr int32_t ID6_POSITION_MAX = 10500;
static_assert(ID6_POSITION_MAX >= 0 && ID6_POSITION_MAX <= 8388607,
              "Invalid shared ID6 upper position limit");
constexpr uint8_t CAN1_RX_PIN = 23, CAN1_TX_PIN = 22;
constexpr uint8_t CAN2_RX_PIN = 0, CAN2_TX_PIN = 1;
constexpr uint16_t CAN_RX_BUDGET = 32;
constexpr uint16_t USB_RX_BUDGET = 96, USB_TX_BUDGET = 256;
constexpr uint16_t USB_LINE_BYTES = 256, USB_REPLY_BYTES = 4096, USB_QUEUE_BYTES = 16384;

namespace Chassis {
// Front left, front right, rear left, rear right. Original wheel direction preserved.
constexpr uint8_t NODE_ID[4] = {10, 11, 12, 13}, CAN_BUS[4] = {1, 1, 2, 2};
constexpr uint8_t MOTOR_COUNT = 4;
constexpr uint32_t CAN_BAUD = SystemConfig::CAN_BAUD;
// Wheel-speed budget (ODrive turns/s): manual input <= 90; absolute safety ceiling 115.
constexpr float JOYSTICK_MAX_SPEED = 90.0f;
constexpr float SAFETY_MAX_SPEED = 115.0f;
// Wheel-command acceleration/deceleration (turns/s per second).
// Default: 0 -> 90 takes 1 s; 90 -> 0 takes 0.5 s. STOP/fault bypasses the ramp.
constexpr bool RAMP_ENABLED = true;
constexpr float ACCEL_TURNS_S2 = 90.0f, DECEL_TURNS_S2 = 180.0f;
constexpr float MAX_RAMP_RATE = 10000.0f;
constexpr uint32_t RAMP_MAX_DT_MS = 50; // A delayed loop cannot cause a large catch-up jump.
// Measured wheel-center spacing, not outside chassis dimensions (millimeters).
constexpr float WHEELBASE_MM = 260.0f, TRACK_WIDTH_MM = 320.0f;
// Body-fixed rotation center: +X forward, +Y right, origin at chassis center.
// Examples: center (0,0), front-left (130,-160), front-right (130,160).
constexpr float ICR_X_MM = 0.0f; // Shared X for both manual turn directions.
constexpr float ICR_LEFT_Y_MM = 0.0f, ICR_RIGHT_Y_MM = 0.0f;
// Existing convention: positive normalized rotation is left/CCW.
// Set -1 only if the physical rotation direction is reversed.
constexpr int ICR_LEFT_ROTATE_SIGN = 1;
// Dedicated CH14 procession bank; normal-stage settings above remain separate.
// Uses the measured chassis; both turn centers default to the chassis origin.
namespace Procession {
constexpr bool RAMP_ENABLED = true;
constexpr float ACCEL_TURNS_S2 = 90.0f, DECEL_TURNS_S2 = 180.0f;
constexpr float ICR_X_MM = 0.0f, ICR_LEFT_Y_MM = 0.0f, ICR_RIGHT_Y_MM = 0.0f;
} // namespace Procession
constexpr float MAX_GEOMETRY_MM = 5000.0f, MAX_ICR_OFFSET_MM = 5000.0f;
constexpr int DEAD_ZONE = 30;
struct Channel {
  uint8_t number;
  int minimum, middle, maximum;
};
// RC sticks: CH2 forward, CH3 strafe, CH1 rotate; CH4 trims Stage1 Pin2 only.
constexpr Channel FORWARD = {2, 1129, 1416, 1717};
constexpr Channel STRAFE = {3, 1241, 1500, 1747};
constexpr Channel ROTATE = {1, 1182, 1454, 1739};
constexpr uint32_t CENTER_HOLD_MS = 500, SEND_INTERVAL_MS = 15;
constexpr uint32_t HEARTBEAT_TIMEOUT_MS = 500, ARM_TIMEOUT_MS = 3000;
constexpr uint32_t IDLE_RETRY_MS = 250, CLEAR_RETRY_MS = 10;
constexpr bool PRINT_STATUS = false;
constexpr uint32_t STATUS_INTERVAL_MS = 500;
} // namespace Chassis
namespace Rc {
constexpr uint32_t CRSF_BAUD = 420000;
constexpr uint8_t CRSF_RX_PIN = 7, CRSF_TX_PIN = 8;
constexpr int CHANNEL_MIN_US = 900, CHANNEL_MAX_US = 2100;
constexpr int PRESS_US = 1700, RELEASE_US = 1600;
constexpr uint32_t DEBOUNCE_MS = 30;
// Stage1-only continuous knob offset; base YAML/ramp pulses are never modified.
constexpr uint8_t STAGE1_TRIM_CHANNEL = 4, STAGE1_TRIM_PIN = 2;
constexpr int STAGE1_TRIM_MIN_US = 1000, STAGE1_TRIM_CENTER_US = 1500, STAGE1_TRIM_MAX_US = 2000;
constexpr int STAGE1_TRIM_DEADZONE_US = 30, STAGE1_TRIM_MAX_OFFSET_US = 150;
constexpr int STAGE1_TRIM_OUTPUT_MIN_US = 500, STAGE1_TRIM_OUTPUT_MAX_US = 2500;
// Stage2 CH4 paired lift: ID4 += offset, ID5 -= offset, preserving the untrimmed base.
constexpr uint8_t STAGE2_TRIM_CHANNEL = 4;
constexpr int32_t STAGE2_TRIM_MAX_OFFSET_PULSES = 1600;
constexpr int8_t STAGE2_TRIM_DIRECTION = 1;
constexpr uint16_t STAGE2_TRIM_RPM = 100;
constexpr uint8_t STAGE2_TRIM_ACC = 5;
constexpr uint32_t STAGE2_TRIM_UPDATE_MS = 100;
// Shared Stage4 (toss) / Stage6 (fruit) CH4 height trim for ID6.
// STAGE4_TRIM_* names are retained for configuration and telemetry compatibility.
// Normal STEP paths keep their YAML targets.
// The slow trim job starts only after the entire STEP has completed.
constexpr uint8_t STAGE4_TRIM_CHANNEL = 4, STAGE4_TRIM_NODE = 6;
constexpr int STAGE4_TRIM_MIN_US = 1000, STAGE4_TRIM_CENTER_US = 1500, STAGE4_TRIM_MAX_US = 2000;
constexpr int STAGE4_TRIM_DEADZONE_US = 30;
constexpr int32_t STAGE4_TRIM_MAX_OFFSET_PULSES = 200; // Toss Stage4 only.
constexpr int32_t FRUIT_TRIM_MAX_OFFSET_PULSES = 2000;  // Fruit Stage6 only.
constexpr int8_t STAGE4_TRIM_DIRECTION = 1; // -1 reverses the knob direction.
constexpr uint16_t STAGE4_TRIM_RPM = 100;
constexpr uint8_t STAGE4_TRIM_ACC = 5;
constexpr uint32_t STAGE4_TRIM_UPDATE_MS = 100;
// CH7 latches the control context: HIGH enters stage 1, LOW exits to stage 4.
// MID retains the previous context; shared CH10..13 have one meaning at a time.
constexpr uint8_t STAGE1_SWITCH_CHANNEL = 7;
constexpr int STAGE1_LOW_US = 1300, STAGE1_HIGH_US = 1700;
constexpr uint8_t PROP_BASKET_CH = 10, PROP_VASE_CH = 11, PROP_TUBE_CH = 12;
constexpr uint8_t PROP_CHICKEN_CH = 13, PROP_BIRD_CH = 14;
constexpr uint8_t PROP_CHANNELS[5] = {PROP_BASKET_CH, PROP_VASE_CH, PROP_TUBE_CH,
                                    PROP_CHICKEN_CH, PROP_BIRD_CH};
constexpr uint8_t STAGE_CHANNELS[4] = {10, 11, 12, 13};
constexpr uint8_t STAGE_MAP[4] = {2, 3, 4, 5};
// General CH14 selects fruit Stage 6; Stage1 CH14 still selects the bird.
constexpr uint8_t FRUIT_CHANNEL = 14, FRUIT_STAGE = 6;
// Retained legacy motion-bank constants for USB compatibility; RC no longer selects this bank.
constexpr uint8_t PROCESSION_CHANNEL = 14, PROCESSION_DIRECTION_STAGE = STAGE_MAP[0];
constexpr uint8_t GENERAL_SELECTOR_CHANNELS[] = {10, 11, 12, 13, PROCESSION_CHANNEL};
constexpr uint8_t GENERAL_SELECTOR_COUNT = sizeof(GENERAL_SELECTOR_CHANNELS) / sizeof(GENERAL_SELECTOR_CHANNELS[0]);
// General CH13 owns the Pin14 strobe; Stage1 CH13 retains the chicken prop.
// Stage5 remains available from GUI/USB and is no longer an RC selector.
constexpr uint8_t LIGHT_CHANNEL = 13;
constexpr uint8_t NEXT_CHANNEL = 6, PREV_CHANNEL = 5, CLEAR_CHANNEL = 9;
} // namespace Rc
namespace Stage2 {
// Entry pose is independent of global startup/HOME standby and editable STEP YAML.
constexpr int SERVO9_ENTRY_US = 1800, SERVO4_ENTRY_US = 1300, SERVO3_ENTRY_US = 1580;
} // namespace Stage2
namespace Fruit {
// Absolute FE travel limits apply to every Stage6 ID6 target, including RAM/Debug/trim.
constexpr int32_t POSITION_MIN = -1400, POSITION_MAX = SystemConfig::ID6_POSITION_MAX;
constexpr uint32_t FINISH_TIMEOUT_MS = 10000; // Missing completion isolates ID6, not the whole robot.
// FE return to established zero after the final step; manual HOME still uses 0x91.
constexpr uint16_t RETURN_RPM = 320;
constexpr uint8_t RETURN_ACC = 200;
constexpr int SERVO2_HOME_US = 1150, SERVO5_HOME_US = 2100;
constexpr uint32_t SERVO_HOME_DURATION_MS = 500;
} // namespace Fruit
static_assert(Fruit::POSITION_MIN >= -8388608 && Fruit::POSITION_MIN <= 0 &&
              Fruit::POSITION_MAX <= 8388607 && Fruit::POSITION_MAX >= 0 &&
              Fruit::POSITION_MIN < Fruit::POSITION_MAX &&
              Fruit::FINISH_TIMEOUT_MS > 1500 && Fruit::FINISH_TIMEOUT_MS < 0x80000000UL,
              "Invalid fruit travel limits or motion deadline");
static_assert(Fruit::RETURN_RPM >= 1 && Fruit::RETURN_RPM <= 3000, "Invalid fruit return speed");
static_assert(Fruit::SERVO2_HOME_US >= 500 && Fruit::SERVO2_HOME_US <= 2500 &&
              Fruit::SERVO5_HOME_US >= 500 && Fruit::SERVO5_HOME_US <= 2500 &&
              Fruit::SERVO_HOME_DURATION_MS <= 60000, "Invalid fruit standby servos");
namespace Mechanism {
// ID 4/5 lock in place and establish the driver zero without motion (0x92).
// Only ID 6 may use physical 0x91 homing. Stage 2 uses the GPIO cylinder driver.
constexpr uint8_t MOTOR_ID = 6, MOTOR_IDS[] = {4, 5, 6};
constexpr uint8_t STATIONARY_ZERO_IDS[] = {4, 5}, HOMING_IDS[] = {6};
// Saved startup working current: SERVO42D ID 4/5 = 3000 mA, SERVO57D ID 6 = 5200 mA.
// Each active axis waits for its 0x83 ACK 1 before F3; homing starts after all settings.
struct StartupCurrentSetting {
  uint8_t node;
  uint16_t milliamps;
};
constexpr StartupCurrentSetting STARTUP_CURRENT_SETTINGS[] = {
    {4, 3000}, {5, 3000}, {6, 5200}};
constexpr uint8_t STARTUP_CURRENT_COUNT =
    sizeof(STARTUP_CURRENT_SETTINGS) / sizeof(STARTUP_CURRENT_SETTINGS[0]);
constexpr uint16_t MAX_CURRENT_MA = 5200; // MKS SERVO57D 0x83 working-current limit.
constexpr uint16_t startupCurrentMilliamps(uint8_t node) {
  for (const auto &setting : STARTUP_CURRENT_SETTINGS)
    if (setting.node == node)
      return setting.milliamps;
  return 0; // No startup current command for this axis.
}
constexpr uint8_t SERVO_PINS[] = {2, 3, 4, 5, 6, 9, 10, 11, 12};
constexpr int SERVO_START_US[] = {1150, 2250, 2000, 2100, 1400, 1100, 1500, 1500, 1500};
 // Pins10/11/12 are independent Debug servos; existing six competition defaults stay fixed.
constexpr uint8_t MOTOR_COUNT = sizeof(MOTOR_IDS) / sizeof(MOTOR_IDS[0]);
constexpr uint8_t SERVO_COUNT = sizeof(SERVO_PINS) / sizeof(SERVO_PINS[0]);
constexpr uint8_t DEFAULT_STAGE = 4;
constexpr uint8_t STAGE_COUNT = 6, STAGE1_PROP_COUNT = 5, DEFAULT_STAGE1_PROP = 2;
// Disable only the Stage3 profile/selector; ID4/5 and Pin3 remain initialized for Stage2/Debug.
constexpr bool STAGE3_ENABLED = false;
constexpr uint32_t CAN_BAUD = SystemConfig::CAN_BAUD, SERIAL_BAUD = SystemConfig::SERIAL_BAUD;
constexpr int SERVO_MIN_US = 500, SERVO_MAX_US = 2500;
constexpr bool STARTUP_RETURN_ZERO = true;
constexpr uint32_t STARTUP_DELAY_SECONDS = 1;
constexpr uint16_t HOME_RPM = 320;
constexpr uint8_t HOME_ACC = 5;
// Nonmoving 0x8C/0x83/0xF3/0x92 settings may be retried; accepted 0x91/0xFE motion never is.
constexpr uint32_t STARTUP_ACK_TIMEOUT_MS = 10000, STARTUP_ACK_RETRY_MS = 500;
constexpr uint32_t START_TIMEOUT_MS = 1500, FINISH_TIMEOUT_MS = 30000;
constexpr uint32_t TX_TIMEOUT_MS = 500, TX_RETRY_MS = 10, SERVO_UPDATE_MS = 20;
// One complete selection: an issued 60 s servo ramp, two 30 s MKS barriers,
// 5 s cylinder entry and enqueue margin. Retargeting never extends this limit.
constexpr uint32_t STAGE_RETURN_TIMEOUT_MS = 130000;
constexpr uint16_t RAM_ACTIONS = 512;
} // namespace Mechanism
namespace Cylinder {
// XY-160D MOTOR1: HIGH/LOW extends, LOW/HIGH retracts; LOW/LOW stops.
// Swap logical drive pins to match the installed motor polarity; YAML meanings stay unchanged.
constexpr uint8_t IN1_PIN = 17, IN2_PIN = 16;
// Full retract is performed once at startup; later entry repeats it only if no full-home estimate remains.
constexpr uint32_t DEFAULT_TIMEOUT_MS = 6000, HOME_RETRACT_MS = 5000;
constexpr uint32_t MAX_DURATION_MS = 60000, REVERSAL_DEADTIME_MS = 50;
} // namespace Cylinder
namespace Pwm {
// Pin14 supports manual breathing and CH13 0/255 strobe; 16/17 drive the cylinder.
constexpr uint8_t PIN = 14, RESOLUTION_BITS = 8, MAX_VALUE = 55;
// Boot defaults; the GUI may change the peak and interval in RAM.
constexpr uint32_t STEP_INTERVAL_MS = 5;
constexpr uint8_t STROBE_VALUE = 255;
constexpr uint32_t STROBE_INTERVAL_MS = 50; // 50 ms on + 50 ms off = 10 flashes/s.
constexpr uint32_t MIN_STEP_INTERVAL_MS = 1, MAX_STEP_INTERVAL_MS = 1000;
} // namespace Pwm

static_assert(Rc::LIGHT_CHANNEL == Rc::PROP_CHICKEN_CH &&
              Pwm::STROBE_VALUE == 255 && Pwm::STROBE_INTERVAL_MS >= 1 &&
              Pwm::STROBE_INTERVAL_MS <= 1000, "Invalid CH13 strobe configuration");
constexpr bool validChannel(Chassis::Channel c) {
  return c.number >= 1 && c.number <= 16 && c.minimum >= Rc::CHANNEL_MIN_US &&
         c.maximum <= Rc::CHANNEL_MAX_US && c.minimum < c.middle - Chassis::DEAD_ZONE &&
         c.maximum > c.middle + Chassis::DEAD_ZONE;
}
// The five shared switches intentionally alias props and General stage/chassis selectors.
constexpr bool validRcMapping() {
  const uint8_t channels[] = {
      Chassis::FORWARD.number, Chassis::STRAFE.number, Chassis::ROTATE.number,
      Rc::NEXT_CHANNEL, Rc::PREV_CHANNEL, Rc::CLEAR_CHANNEL, Rc::STAGE1_SWITCH_CHANNEL, Rc::STAGE1_TRIM_CHANNEL,
      Rc::PROP_CHANNELS[0], Rc::PROP_CHANNELS[1], Rc::PROP_CHANNELS[2],
      Rc::PROP_CHANNELS[3], Rc::PROP_CHANNELS[4]};
  for (size_t i = 0; i < sizeof(channels) / sizeof(channels[0]); ++i) {
    if (channels[i] < 1 || channels[i] > 16) return false;
    for (size_t j = 0; j < i; ++j) if (channels[i] == channels[j]) return false;
  }
  for (uint8_t i = 0; i < 4; ++i) {
    if (Rc::STAGE_CHANNELS[i] != Rc::PROP_CHANNELS[i] ||
        Rc::STAGE_MAP[i] < 2 || Rc::STAGE_MAP[i] > Mechanism::STAGE_COUNT) return false;
    for (uint8_t j = 0; j < i; ++j) if (Rc::STAGE_MAP[i] == Rc::STAGE_MAP[j]) return false;
  }
  for (uint8_t i = 0; i < Rc::GENERAL_SELECTOR_COUNT; ++i)
    if (Rc::GENERAL_SELECTOR_CHANNELS[i] != Rc::PROP_CHANNELS[i]) return false;
  return Rc::GENERAL_SELECTOR_COUNT == 5 && Rc::PROCESSION_CHANNEL == Rc::PROP_BIRD_CH &&
         Rc::PROCESSION_DIRECTION_STAGE == Rc::STAGE_MAP[0] &&
         Rc::STAGE1_LOW_US < Rc::STAGE1_HIGH_US;
}
constexpr bool validStartupCurrents() {
  for (uint8_t i = 0; i < Mechanism::STARTUP_CURRENT_COUNT; ++i) {
    const auto setting = Mechanism::STARTUP_CURRENT_SETTINGS[i];
    if (!setting.milliamps || setting.milliamps > Mechanism::MAX_CURRENT_MA)
      return false;
    bool active = false;
    for (uint8_t node : Mechanism::MOTOR_IDS)
      if (node == setting.node)
        active = true;
    if (!active)
      return false;
    for (uint8_t j = 0; j < i; ++j)
      if (Mechanism::STARTUP_CURRENT_SETTINGS[j].node == setting.node)
        return false;
  }
  return true;
}
constexpr bool distinctHardware() {
  for (uint8_t i = 0; i < 4; ++i) {
    if (Chassis::NODE_ID[i] >= 63 || (Chassis::CAN_BUS[i] != 1 && Chassis::CAN_BUS[i] != 2))
      return false;
    for (uint8_t j = 0; j < i; ++j)
      if (Chassis::CAN_BUS[i] == Chassis::CAN_BUS[j] && Chassis::NODE_ID[i] == Chassis::NODE_ID[j])
        return false;
    // ODrive CANSimple IDs encode node<<5, separate from bare MKS IDs.
    for (uint8_t j = 0; j < Mechanism::MOTOR_COUNT; ++j)
      if (Chassis::CAN_BUS[i] == 1 &&
          ((static_cast<uint32_t>(Chassis::NODE_ID[i]) << 5) <= Mechanism::MOTOR_IDS[j] &&
           Mechanism::MOTOR_IDS[j] <= (static_cast<uint32_t>(Chassis::NODE_ID[i]) << 5) + 31))
        return false;
  }
  for (uint8_t i = 0; i < Mechanism::MOTOR_COUNT; ++i) {
    if (!Mechanism::MOTOR_IDS[i] || Mechanism::MOTOR_IDS[i] > 127)
      return false;
    for (uint8_t j = 0; j < i; ++j)
      if (Mechanism::MOTOR_IDS[i] == Mechanism::MOTOR_IDS[j])
        return false;
  }
  for (uint8_t i = 0; i < Mechanism::SERVO_COUNT; ++i) {
    if (Mechanism::SERVO_PINS[i] >= 40 || Mechanism::SERVO_START_US[i] < 500 ||
        Mechanism::SERVO_START_US[i] > 2500)
      return false;
    for (uint8_t j = 0; j < i; ++j)
      if (Mechanism::SERVO_PINS[i] == Mechanism::SERVO_PINS[j])
        return false;
  }
  return true;
}
static_assert(validChannel(Chassis::FORWARD) && validChannel(Chassis::STRAFE) &&
                  validChannel(Chassis::ROTATE),
              "Invalid joystick calibration");
static_assert(validRcMapping(), "Duplicate or invalid RC control channel");
static_assert(Rc::STAGE1_TRIM_PIN == Mechanism::SERVO_PINS[0] &&
                  Rc::STAGE1_TRIM_MIN_US >= Rc::CHANNEL_MIN_US &&
                  Rc::STAGE1_TRIM_MAX_US <= Rc::CHANNEL_MAX_US &&
                  Rc::STAGE1_TRIM_DEADZONE_US >= 0 &&
                  Rc::STAGE1_TRIM_CENTER_US - Rc::STAGE1_TRIM_MIN_US > Rc::STAGE1_TRIM_DEADZONE_US &&
                  Rc::STAGE1_TRIM_MAX_US - Rc::STAGE1_TRIM_CENTER_US > Rc::STAGE1_TRIM_DEADZONE_US &&
                  Rc::STAGE1_TRIM_MAX_OFFSET_US >= 0 &&
                  Rc::STAGE1_TRIM_MAX_OFFSET_US <= Mechanism::SERVO_MAX_US - Mechanism::SERVO_MIN_US &&
                  Rc::STAGE1_TRIM_OUTPUT_MIN_US >= Mechanism::SERVO_MIN_US &&
                  Rc::STAGE1_TRIM_OUTPUT_MAX_US <= Mechanism::SERVO_MAX_US &&
                  Rc::STAGE1_TRIM_OUTPUT_MIN_US < Rc::STAGE1_TRIM_OUTPUT_MAX_US,
              "Invalid Stage1 CH4 servo trim configuration");
static_assert(Rc::STAGE4_TRIM_CHANNEL == Rc::STAGE1_TRIM_CHANNEL &&
                  Rc::STAGE4_TRIM_NODE == 6 &&
                  Rc::STAGE4_TRIM_MIN_US >= Rc::CHANNEL_MIN_US &&
                  Rc::STAGE4_TRIM_MAX_US <= Rc::CHANNEL_MAX_US &&
                  Rc::STAGE4_TRIM_DEADZONE_US >= 0 &&
                  Rc::STAGE4_TRIM_CENTER_US - Rc::STAGE4_TRIM_MIN_US > Rc::STAGE4_TRIM_DEADZONE_US &&
                  Rc::STAGE4_TRIM_MAX_US - Rc::STAGE4_TRIM_CENTER_US > Rc::STAGE4_TRIM_DEADZONE_US &&
                  Rc::STAGE4_TRIM_MAX_OFFSET_PULSES >= 0 &&
                  Rc::STAGE4_TRIM_MAX_OFFSET_PULSES <= 10000 &&
                  Rc::FRUIT_TRIM_MAX_OFFSET_PULSES >= 0 &&
                  Rc::FRUIT_TRIM_MAX_OFFSET_PULSES <= 10000 &&
                  (Rc::STAGE4_TRIM_DIRECTION == 1 || Rc::STAGE4_TRIM_DIRECTION == -1) &&
                  Rc::STAGE4_TRIM_RPM >= 1 && Rc::STAGE4_TRIM_RPM <= 3000 &&
                  Rc::STAGE4_TRIM_UPDATE_MS >= 20 && Rc::STAGE4_TRIM_UPDATE_MS <= 1000,
              "Invalid ID6 CH4 stepper trim configuration");
static_assert(Rc::STAGE2_TRIM_CHANNEL == Rc::STAGE4_TRIM_CHANNEL &&
                  Rc::STAGE2_TRIM_MAX_OFFSET_PULSES >= 0 && Rc::STAGE2_TRIM_MAX_OFFSET_PULSES <= 10000 &&
                  (Rc::STAGE2_TRIM_DIRECTION == 1 || Rc::STAGE2_TRIM_DIRECTION == -1) &&
                  Rc::STAGE2_TRIM_RPM >= 1 && Rc::STAGE2_TRIM_RPM <= 3000 &&
                  Rc::STAGE2_TRIM_UPDATE_MS >= 20 && Rc::STAGE2_TRIM_UPDATE_MS <= 1000 &&
                  Stage2::SERVO9_ENTRY_US >= 500 && Stage2::SERVO9_ENTRY_US <= 2500 &&
                  Stage2::SERVO4_ENTRY_US >= 500 && Stage2::SERVO4_ENTRY_US <= 2500 &&
                  Stage2::SERVO3_ENTRY_US >= 500 && Stage2::SERVO3_ENTRY_US <= 2500,
              "Invalid Stage2 entry pose or paired CH4 trim configuration");
static_assert(Mechanism::MOTOR_COUNT + Mechanism::SERVO_COUNT + 1 <= 16,
              "Actuator masks support at most sixteen axes");
static_assert(Mechanism::MOTOR_COUNT <= 8, "Motor stop masks support at most eight axes");
static_assert(sizeof(Mechanism::SERVO_START_US) / sizeof(int) == Mechanism::SERVO_COUNT,
              "Every active servo requires a startup pulse");
// Competition standby values are fixed; STEP YAML pulses remain independently editable.
static_assert(Mechanism::SERVO_COUNT == 9 &&
                  Mechanism::SERVO_START_US[0] == 1150 &&
                  Mechanism::SERVO_START_US[1] == 2250 &&
                  Mechanism::SERVO_START_US[2] == 2000 &&
                  Mechanism::SERVO_START_US[3] == 2100 &&
                  Mechanism::SERVO_START_US[4] == 1400 &&
                  Mechanism::SERVO_START_US[5] == 1100,
              "Standby servo pulses are locked to 1150/2250/2000/2100/1400/1100 us");
constexpr bool validCylinderPins() {
  if constexpr (Cylinder::IN1_PIN >= 40 || Cylinder::IN2_PIN >= 40 ||
      Cylinder::IN1_PIN == Cylinder::IN2_PIN) return false;
  const uint8_t reserved[] = {CAN1_RX_PIN, CAN1_TX_PIN, CAN2_RX_PIN, CAN2_TX_PIN,
                             Rc::CRSF_RX_PIN, Rc::CRSF_TX_PIN, 18, 19, Pwm::PIN};
  for (uint8_t pin : reserved)
    if (pin == Cylinder::IN1_PIN || pin == Cylinder::IN2_PIN) return false;
  for (uint8_t pin : Mechanism::SERVO_PINS)
    if (pin == Cylinder::IN1_PIN || pin == Cylinder::IN2_PIN) return false;
  return true;
}
constexpr bool validPwmPin() {
  if constexpr (Pwm::PIN >= 40) return false;
  const uint8_t reserved[] = {CAN1_RX_PIN, CAN1_TX_PIN, CAN2_RX_PIN, CAN2_TX_PIN,
                             Rc::CRSF_RX_PIN, Rc::CRSF_TX_PIN, 18, 19,
                             Cylinder::IN1_PIN, Cylinder::IN2_PIN};
  for (uint8_t pin : reserved) if (pin == Pwm::PIN) return false;
  for (uint8_t pin : Mechanism::SERVO_PINS) if (pin == Pwm::PIN) return false;
  return true;
}
static_assert(validCylinderPins(), "Cylinder GPIO conflicts with CAN/UART/I2C/servo/PWM pins");
static_assert(validPwmPin(), "Breathing PWM pin conflicts with an existing actuator or bus");
static_assert(Pwm::RESOLUTION_BITS == 8 && Pwm::MAX_VALUE > 0 &&
                  Pwm::MIN_STEP_INTERVAL_MS > 0 &&
                  Pwm::STEP_INTERVAL_MS >= Pwm::MIN_STEP_INTERVAL_MS &&
                  Pwm::STEP_INTERVAL_MS <= Pwm::MAX_STEP_INTERVAL_MS &&
                  Pwm::MAX_STEP_INTERVAL_MS < 0x80000000UL,
              "Invalid breathing PWM resolution or interval");
static_assert(Cylinder::DEFAULT_TIMEOUT_MS > 0 &&
                  Cylinder::DEFAULT_TIMEOUT_MS <= Cylinder::MAX_DURATION_MS &&
                  Cylinder::HOME_RETRACT_MS > 0 &&
                  Cylinder::HOME_RETRACT_MS <= Cylinder::MAX_DURATION_MS &&
                  Cylinder::MAX_DURATION_MS <= 60000 && Cylinder::REVERSAL_DEADTIME_MS > 0,
              "Invalid cylinder duration/deadtime configuration");
static_assert(distinctHardware(), "Duplicate or invalid actuator CAN IDs / servo pins");
static_assert(validStartupCurrents(), "Invalid or duplicate MKS startup working-current setting");
static_assert(Rc::RELEASE_US < Rc::PRESS_US && Rc::DEBOUNCE_MS > 0, "Invalid RC button hysteresis");
static_assert(Mechanism::STARTUP_ACK_RETRY_MS > 0 &&
                  Mechanism::STARTUP_ACK_TIMEOUT_MS > Mechanism::STARTUP_ACK_RETRY_MS,
              "Invalid MKS startup acknowledgement timing");
static_assert(Mechanism::STAGE_RETURN_TIMEOUT_MS > Mechanism::FINISH_TIMEOUT_MS &&
                  Mechanism::STAGE_RETURN_TIMEOUT_MS < 0x80000000UL,
              "Invalid overall stage-return deadline");
static_assert(Mechanism::SERVO_MIN_US >= 500 && Mechanism::SERVO_MAX_US <= 2500,
              "Invalid PWM limits");
static_assert(Chassis::JOYSTICK_MAX_SPEED > 0 &&
                  Chassis::SAFETY_MAX_SPEED >= Chassis::JOYSTICK_MAX_SPEED &&
                  Chassis::SAFETY_MAX_SPEED <= 115.0f,
              "Invalid chassis wheel-speed budget");
static_assert(Chassis::ACCEL_TURNS_S2 >= 1 && Chassis::ACCEL_TURNS_S2 <= Chassis::MAX_RAMP_RATE &&
                  Chassis::DECEL_TURNS_S2 >= 1 && Chassis::DECEL_TURNS_S2 <= Chassis::MAX_RAMP_RATE &&
                  Chassis::WHEELBASE_MM >= 1 && Chassis::WHEELBASE_MM <= Chassis::MAX_GEOMETRY_MM &&
                  Chassis::TRACK_WIDTH_MM >= 1 && Chassis::TRACK_WIDTH_MM <= Chassis::MAX_GEOMETRY_MM &&
                  Chassis::ICR_X_MM >= -Chassis::MAX_ICR_OFFSET_MM && Chassis::ICR_X_MM <= Chassis::MAX_ICR_OFFSET_MM &&
                  Chassis::ICR_LEFT_Y_MM >= -Chassis::MAX_ICR_OFFSET_MM && Chassis::ICR_LEFT_Y_MM <= Chassis::MAX_ICR_OFFSET_MM &&
                  Chassis::ICR_RIGHT_Y_MM >= -Chassis::MAX_ICR_OFFSET_MM && Chassis::ICR_RIGHT_Y_MM <= Chassis::MAX_ICR_OFFSET_MM &&
                  (Chassis::ICR_LEFT_ROTATE_SIGN == 1 || Chassis::ICR_LEFT_ROTATE_SIGN == -1) &&
                  Chassis::RAMP_MAX_DT_MS >= Chassis::SEND_INTERVAL_MS && Chassis::RAMP_MAX_DT_MS <= 100,
              "Invalid chassis ramp or ICR geometry");
static_assert(Chassis::Procession::ACCEL_TURNS_S2 >= 1 &&
                  Chassis::Procession::ACCEL_TURNS_S2 <= Chassis::MAX_RAMP_RATE &&
                  Chassis::Procession::DECEL_TURNS_S2 >= 1 &&
                  Chassis::Procession::DECEL_TURNS_S2 <= Chassis::MAX_RAMP_RATE &&
                  Chassis::Procession::ICR_X_MM >= -Chassis::MAX_ICR_OFFSET_MM &&
                  Chassis::Procession::ICR_X_MM <= Chassis::MAX_ICR_OFFSET_MM &&
                  Chassis::Procession::ICR_LEFT_Y_MM >= -Chassis::MAX_ICR_OFFSET_MM &&
                  Chassis::Procession::ICR_LEFT_Y_MM <= Chassis::MAX_ICR_OFFSET_MM &&
                  Chassis::Procession::ICR_RIGHT_Y_MM >= -Chassis::MAX_ICR_OFFSET_MM &&
                  Chassis::Procession::ICR_RIGHT_Y_MM <= Chassis::MAX_ICR_OFFSET_MM,
              "Invalid CH14 procession motion settings");
static_assert(Chassis::CENTER_HOLD_MS > 0 && Chassis::SEND_INTERVAL_MS > 0,
              "Invalid chassis timing");
} // namespace SystemConfig
