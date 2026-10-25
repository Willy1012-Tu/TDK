#include <stdint.h>
#include "servoxxd_can.h"
#include <FlexCAN_T4.h>
#include <IntervalTimer.h>

servoxxd_motor_t servoxxd_motor[MOTOR_MAX_NUM];

IntervalTimer servoxxd_timer;

static bool validMotorId(uint8_t id) {
  return id >= MOTOR_CAN_ID_BASE && id < MOTOR_CAN_ID_BASE + MOTOR_MAX_NUM;
}

uint8_t calculateCRC(uint8_t can_id, uint8_t *data, uint16_t dataLength) {
    uint8_t crc = 0;  // 初始化 CRC 值

    crc += can_id;  // 先將 CAN ID 累加進 CRC

    for (uint16_t i = 0; i < dataLength; i++) {
        crc += data[i];  // 將每個資料字節與 CRC 值累加
    }

    return crc;  // 返回最終的 CRC 值，並自動取低 8 位
}

void servoxxd_onFeedbackMessage(const CAN_message_t &msg) {
  if (msg.flags.extended || msg.flags.remote || msg.len < 3 || msg.len > 8) return;
  uint8_t checksum = static_cast<uint8_t>(msg.id);
  for (uint8_t i = 0; i < msg.len - 1; ++i) checksum += msg.buf[i];
  if (checksum != msg.buf[msg.len - 1]) return;
  if (msg.buf[0] == SERVOXXD_GET_ABS_POS ? msg.len != 8 : msg.len != 3) return;
  if(msg.id >= MOTOR_CAN_ID_BASE && msg.id < (MOTOR_CAN_ID_BASE + MOTOR_MAX_NUM))
  {
    for(uint8_t index = 0; index < MOTOR_MAX_NUM; index++)
    {
      if(index == (msg.id - MOTOR_CAN_ID_BASE))
      {
        switch(msg.buf[0])
        {
            case SERVOXXD_GOHOME_MOD:
              switch (msg.buf[1]) 
              {
                case 2:
                  // servoxxd_motor[index].finish_flag = 1;
                break;

                default:
                  servoxxd_motor[index].state = MOTOR_BUZY;
                break;
              }
            break;

            case SERVOXXD_ABS_POS_MOD:
              switch (msg.buf[1]) 
              {
                case 2:
                  // servoxxd_motor[index].finish_flag = 1;
                break;

                default:
                  servoxxd_motor[index].state = MOTOR_BUZY;
                break;
              }
            break;

            case SERVOXXD_GET_ABS_POS:
            {
                uint64_t u48 = (uint64_t)msg.buf[6] |
                              ((uint64_t)msg.buf[5] << 8)  |
                              ((uint64_t)msg.buf[4] << 16) |
                              ((uint64_t)msg.buf[3] << 24) |
                              ((uint64_t)msg.buf[2] << 32) |
                              ((uint64_t)msg.buf[1] << 40);
                servoxxd_motor[index].abs_pos = (int64_t)((int64_t)(u48 << 16) >> 16);
            }
            break; 
        }
      }
    }
  }
}

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>servoxxd_can;

void servoxxd_timer_callback(void)
{
    for(uint8_t index = 0; index < MOTOR_MAX_NUM; index++)
    {
      CAN_message_t msg;
      uint8_t crc;

      msg.id = index + MOTOR_CAN_ID_BASE;
      msg.flags.extended = false;   // STD ID
      msg.flags.remote   = false;   // Data frame
      msg.len = 2;

      msg.buf[0] = SERVOXXD_GET_ABS_POS;
      crc = calculateCRC(static_cast<uint8_t>(msg.id), msg.buf, 1);
      msg.buf[1] = crc;

      servoxxd_can.write(msg);
    }

    for(uint8_t index = 0; index < MOTOR_MAX_NUM; index++)
    { 
      if(servoxxd_motor[index].state != MOTOR_IDLE)
      {
        if(abs(servoxxd_motor[index].abs_pos_last - servoxxd_motor[index].abs_pos) >= MOTOR_STATIC_THRESHOLD)
        {
          servoxxd_motor[index].static_time--;
          servoxxd_motor[index].static_time = (servoxxd_motor[index].static_time <= 0) ? 0 : servoxxd_motor[index].static_time;
        }
        else
        {
            if(servoxxd_motor[index].static_time++ >= MOTOR_STATIC_TIME)
            {
              servoxxd_motor[index].state = MOTOR_IDLE;
              servoxxd_motor[index].static_time = 0;
            }
        }
      }

      servoxxd_motor[index].abs_pos_last = servoxxd_motor[index].abs_pos;
    }

    for(uint8_t index = 0; index < MOTOR_MAX_NUM; index++)
    {
      if(servoxxd_motor[index].finish_flag == 1)
      {
        if(servoxxd_motor[index].finish_time++ >= MOTOR_FINISH_TIME)
        {
          servoxxd_motor[index].finish_flag = 0;
          servoxxd_motor[index].finish_time = 0;
          servoxxd_motor[index].state = MOTOR_IDLE;
        }
      }
    }
}  


void servoxxd_can_init(void) {
  servoxxd_can.begin();
  servoxxd_can.setBaudRate(1000000);

  servoxxd_can.enableMBInterrupts();
  servoxxd_can.onReceive(servoxxd_onFeedbackMessage);  // 註冊同一支 callback

  servoxxd_timer.begin(servoxxd_timer_callback, 2500);
}


void servoxxd_set_motor_gohome(uint8_t servoxxd_id, bool homeTrig, bool homeDir, uint16_t homeSpeed) {
  if (!validMotorId(servoxxd_id)) return;
  CAN_message_t msg;
  uint8_t crc;

  msg.id = servoxxd_id;
  msg.flags.extended = false;   // STD ID
  msg.flags.remote   = false;   // Data frame
  msg.len = 6;

  msg.buf[0] = SERVOXXD_SET_GOHOME_MOD;
  msg.buf[1] = static_cast<uint8_t>(homeTrig);
  msg.buf[2] = static_cast<uint8_t>(homeDir);
  msg.buf[3] = (homeSpeed >> 8) & 0xFF;
  msg.buf[4] = homeSpeed  & 0xFF;

  crc = calculateCRC(servoxxd_id, msg.buf, 5);
  msg.buf[5] = crc;

  servoxxd_can.write(msg);
}

void servoxxd_motor_gohome(uint8_t servoxxd_id) {
  if (!validMotorId(servoxxd_id)) return;
  CAN_message_t msg;
  uint8_t crc;

  servoxxd_motor[servoxxd_id - MOTOR_CAN_ID_BASE].state = MOTOR_BUZY;

  msg.id = servoxxd_id;
  msg.flags.extended = false;   // STD ID
  msg.flags.remote   = false;   // Data frame
  msg.len = 2;

  msg.buf[0] = SERVOXXD_GOHOME_MOD;

  crc = calculateCRC(servoxxd_id, msg.buf, 1);
  msg.buf[1] = crc;

  servoxxd_can.write(msg);
}

void servoxxd_set_motor_rpm(uint8_t servoxxd_id, bool dir, uint16_t v, uint8_t acc) {
  if (!validMotorId(servoxxd_id)) return;
  CAN_message_t msg;
  uint8_t crc;

  servoxxd_motor[servoxxd_id - MOTOR_CAN_ID_BASE].state = MOTOR_BUZY;

  msg.id = servoxxd_id;
  msg.flags.extended = false;   // STD ID
  msg.flags.remote   = false;   // Data frame
  msg.len = 5;

  msg.buf[0] = SERVOXXD_SPEED_MOD;
  msg.buf[1] = (dir << 7) | ((v >> 8) & 0xFF);
  msg.buf[2] = v & 0xFF;
  msg.buf[3] = acc & 0xFF;

  crc = calculateCRC(servoxxd_id, msg.buf, 4);
  msg.buf[4] = crc;

  servoxxd_can.write(msg);
}

void servoxxd_set_motor_abs_pos(uint8_t servoxxd_id, uint16_t v, uint8_t acc, int32_t absPos) {
  if (!validMotorId(servoxxd_id)) return;
  CAN_message_t msg;
  uint8_t crc;

  servoxxd_motor[servoxxd_id - MOTOR_CAN_ID_BASE].state = MOTOR_BUZY;

  msg.id = servoxxd_id;
  msg.flags.extended = false;   // STD ID
  msg.flags.remote   = false;   // Data frame
  msg.len = 8;

  msg.buf[0] = SERVOXXD_ABS_POS_MOD;
  msg.buf[1] = (v >> 8) & 0xFF;
  msg.buf[2] = v & 0xFF;
  msg.buf[3] = acc & 0xFF;
  msg.buf[4] = (absPos >> 16) & 0xFF;
  msg.buf[5] = (absPos >> 8) & 0xFF;
  msg.buf[6] = absPos & 0xFF;

  crc = calculateCRC(servoxxd_id, msg.buf, 7);
  msg.buf[7] = crc;

  servoxxd_can.write(msg);
}

void servoxxd_set_abs_pos_zero(uint8_t servoxxd_id)
{
  if (!validMotorId(servoxxd_id)) return;
  CAN_message_t msg;
  uint8_t crc;

  msg.id = servoxxd_id;
  msg.flags.extended = false;   // STD ID
  msg.flags.remote   = false;   // Data frame
  msg.len = 2;

  msg.buf[0] = SERVOXXD_SET_POS_ZERO;

  crc = calculateCRC(servoxxd_id, msg.buf, 1);
  msg.buf[1] = crc;

  servoxxd_can.write(msg);
}
