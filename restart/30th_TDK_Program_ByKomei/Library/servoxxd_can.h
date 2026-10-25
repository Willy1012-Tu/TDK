#include <stdint.h>
#ifndef __SERVOXXD_CAN
#define __SERVOXXD_CAN

#include <Arduino.h>
#include <FlexCAN_T4.h>

#define SERVOXXD_SET_GOHOME_MOD 0x90
#define SERVOXXD_GOHOME_MOD     0x91
#define SERVOXXD_ABS_POS_MOD    0xF5 // absPos = 16385 / rev(360度)
#define SERVOXXD_SPEED_MOD      0xF6
#define SERVOXXD_GET_ABS_POS    0x31
#define SERVOXXD_SET_POS_ZERO   0x92

#define MOTOR_CAN_ID_BASE    0x01
#define MOTOR_MAX_NUM        6

 #define MOTOR_FINISH_TIME    500
#define MOTOR_STATIC_THRESHOLD 2
#define MOTOR_STATIC_TIME      50

typedef enum
{
  MOTOR_IDLE,
  MOTOR_BUZY
}servoxxd_motor_state_t;

typedef struct servoxxd_motor_t {
  uint8_t finish_flag = 0;  
  uint16_t finish_time = 0;
  int16_t static_time = 0;
  int64_t abs_pos;
  int64_t abs_pos_last;
  uint8_t state = MOTOR_IDLE;
}servoxxd_motor_t;

extern servoxxd_motor_t servoxxd_motor[MOTOR_MAX_NUM];

void servoxxd_can_init(void);
void servoxxd_set_motor_gohome(uint8_t servoxxd_id, bool homeTrig, bool homeDir, uint16_t homeSpeed);
void servoxxd_motor_gohome(uint8_t servoxxd_id);
void servoxxd_set_motor_rpm(uint8_t servoxxd_id, bool dir, uint16_t v, uint8_t acc);
void servoxxd_set_motor_abs_pos(uint8_t servoxxd_id, uint16_t v, uint8_t acc, int32_t absPos);
void servoxxd_set_abs_pos_zero(uint8_t servoxxd_id);
#endif
