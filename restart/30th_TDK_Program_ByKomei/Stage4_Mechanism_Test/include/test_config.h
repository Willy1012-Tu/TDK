#pragma once
#include <stdint.h>
#include <stddef.h>
namespace TestConfig {
constexpr uint8_t MOTOR_ID = 6;
constexpr uint8_t MOTOR_IDS[] = {6, 7};
constexpr uint8_t MOTOR_COUNT = sizeof(MOTOR_IDS)/sizeof(MOTOR_IDS[0]);
constexpr uint8_t SERVO_PINS[] = {2, 5, 4, 6, 9};
constexpr uint8_t SERVO_COUNT = sizeof(SERVO_PINS)/sizeof(SERVO_PINS[0]);
constexpr int SERVO_START_US[] = {1150, 2100, 2000, 1400, 1800};
constexpr uint8_t DEFAULT_STAGE = 4;
constexpr uint32_t CAN_BAUD = 1000000, SERIAL_BAUD = 115200;
constexpr int SERVO_MIN_US = 500, SERVO_MAX_US = 2500;
constexpr bool STARTUP_RETURN_ZERO = true;
constexpr bool STAGE3_ENABLED = false;
constexpr uint32_t STARTUP_DELAY_SECONDS = 1;
constexpr uint32_t STARTUP_ACK_TIMEOUT_MS = 10000, STARTUP_ACK_RETRY_MS = 500;
constexpr uint16_t HOME_RPM = 320;
constexpr uint8_t HOME_ACC = 5;
constexpr uint32_t START_TIMEOUT_MS = 1500, FINISH_TIMEOUT_MS = 30000;
constexpr uint32_t TX_TIMEOUT_MS = 500, TX_RETRY_MS = 10;
constexpr uint32_t SERVO_UPDATE_MS = 20;
template<size_t N> constexpr bool unique(const uint8_t (&values)[N]) {
  for (size_t i=0;i<N;++i) for(size_t j=i+1;j<N;++j) if(values[i]==values[j]) return false;
  return true;
}
constexpr bool validHardware() {
  for(auto id:MOTOR_IDS) if(id<1 || id>127) return false;
  for(auto pin:SERVO_PINS) if(pin>=40) return false;
  for(auto pulse:SERVO_START_US) if(pulse<500 || pulse>2500) return false;
  return true;
}
static_assert(MOTOR_COUNT>0 && MOTOR_COUNT+SERVO_COUNT<=8,"硬體數量超出狀態mask容量");
static_assert(unique(MOTOR_IDS) && unique(SERVO_PINS) && validHardware(),"啟用硬體設定無效");
static_assert(sizeof(SERVO_START_US)/sizeof(SERVO_START_US[0])==SERVO_COUNT,"待機脈寬數量不符");
static_assert(HOME_RPM>=1 && HOME_RPM<=3000,"尋零速度無效");
static_assert(STARTUP_DELAY_SECONDS<=UINT32_MAX/1000U,"開機等待秒數無效");
static_assert(SERVO_MIN_US>=500 && SERVO_MAX_US<=2500 && SERVO_MIN_US<SERVO_MAX_US,"伺服範圍無效");
static_assert(CAN_BAUD>0 && SERIAL_BAUD>0,"通訊速率無效");
static_assert(START_TIMEOUT_MS>0 && FINISH_TIMEOUT_MS>=START_TIMEOUT_MS && FINISH_TIMEOUT_MS<UINT32_MAX,"動作逾時無效");
static_assert(STARTUP_ACK_RETRY_MS>0 && STARTUP_ACK_TIMEOUT_MS>=STARTUP_ACK_RETRY_MS && STARTUP_ACK_TIMEOUT_MS<UINT32_MAX,"開機ACK逾時無效");
static_assert(TX_RETRY_MS>0 && TX_TIMEOUT_MS>=TX_RETRY_MS && TX_TIMEOUT_MS<UINT32_MAX,"CAN佇列逾時無效");
static_assert(SERVO_UPDATE_MS>0 && SERVO_UPDATE_MS<=60000,"伺服更新週期無效");
}
