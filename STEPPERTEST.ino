#include <Arduino.h>
#include <FlexCAN_T4.h>

// ============================================================================
// STEPPERTEST.ino — Teensy 4.0 / 4.1 MKS CAN 閉迴路步進馬達獨立快速測試程式
// ============================================================================
// 硬體接線：
//   Teensy 4.0/4.1 CAN1 收發腳：Pin 22 (CTX1), Pin 23 (CRX1)
//   需接 CAN 收發器（如 SN65HVD230 / VP230 / TJA1050 等）至 MKS 驅動器 CAN 介面
//
// 功能說明：
//   1. 開機自動使能馬達，並立即發送「正轉 2000 脈衝」。
//   2. 支援 Serial Monitor (115200 baud) 互動指令：
//        - 按 Enter 或輸入 'g' / '1' : 再次正轉 2000 脈衝
//        - 輸入 'b' / '2'           : 反轉 2000 脈衝
//        - 輸入 's'                 : 緊急停止 (Stop)
//        - 輸入 '4', '5', '6', '7'  : 切換目標馬達 CAN ID (預設 ID 4)
//        - 輸入數字 (例如 5000 或 -1000) : 移動指定自訂脈衝數
// ============================================================================

// CAN 鮑率設定（請依你的 MKS 驅動器設定選擇）：
// 機器人整合程式使用 1,000,000 (1Mbps)；部分舊版或單機可能為 250,000 (250Kbps)
constexpr uint32_t CAN_BAUD = 1000000; // 若沒反應可改為 250000 測試

// 預設測試參數
uint8_t targetNodeId = 4;        // 預設測試 MKS ID 4 (可隨時切換 4, 5, 6, 7)
constexpr uint16_t DEFAULT_RPM = 320; // 轉速 (RPM)
constexpr uint8_t DEFAULT_ACC  = 0;   // 加速度 (0 = 預設平滑加速)
constexpr int32_t TEST_PULSES  = 2000; // 測試脈衝數 (正轉 2000)

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

// --- 發送 MKS CAN 協定指令 ---
bool sendMksCommand(uint8_t node, uint8_t cmd, const uint8_t *payload, uint8_t payload_len) {
  CAN_message_t msg{};
  msg.id = node;
  msg.len = payload_len + 2;
  msg.flags.extended = false;
  msg.flags.remote = false;
  msg.buf[0] = cmd;

  uint32_t sum = node + cmd;
  for (uint8_t i = 0; i < payload_len; i++) {
    msg.buf[i + 1] = payload[i];
    sum += payload[i];
  }
  msg.buf[payload_len + 1] = sum & 0xFF; // Checksum
  return can1.write(msg) != 0;
}

// 使能 / 釋放馬達 (0xF3)
void enableMotor(uint8_t node, bool enable) {
  uint8_t payload[1] = { enable ? (uint8_t)0x01 : (uint8_t)0x00 };
  sendMksCommand(node, 0xF3, payload, 1);
  Serial.printf("[CAN TX] ID %u -> %s (0xF3)\n", node, enable ? "使能 (Enable)" : "釋放 (Disable)");
}

// 相對位置移動 (0xFD)
bool moveRelative(uint8_t node, int32_t pulses, uint16_t rpm = DEFAULT_RPM, uint8_t acc = DEFAULT_ACC) {
  uint8_t payload[6];
  uint8_t dir = (pulses < 0) ? 1 : 0; // 0 = 正轉, 1 = 反轉
  uint32_t abs_pulses = abs(pulses);
  if (abs_pulses > 0xFFFFFF) abs_pulses = 0xFFFFFF;
  if (rpm > 3000) rpm = 3000;

  payload[0] = (dir << 7) | ((rpm >> 8) & 0x0F);
  payload[1] = rpm & 0xFF;
  payload[2] = acc;
  payload[3] = (abs_pulses >> 16) & 0xFF;
  payload[4] = (abs_pulses >> 8) & 0xFF;
  payload[5] = abs_pulses & 0xFF;

  bool ok = sendMksCommand(node, 0xFD, payload, 6);
  Serial.printf("[CAN TX] ID %u -> %s %ld 脈衝 (RPM=%u, ACC=%u) %s\n",
                node, (pulses >= 0) ? "正轉" : "反轉",
                (long)abs_pulses, rpm, acc, ok ? "已發送" : "【發送失敗! 請檢查CAN硬體】");
  return ok;
}

// 緊急停止 (0xF7)
void emergencyStop(uint8_t node) {
  sendMksCommand(node, 0xF7, nullptr, 0);
  Serial.printf("[CAN TX] ID %u -> 緊急停止 (0xF7)\n", node);
}

// 接收 CAN 回報
void processCanRx() {
  CAN_message_t rxMsg;
  while (can1.read(rxMsg)) {
    Serial.printf("[CAN RX] ID=0x%02X (%u) DLC=%u DATA: ", rxMsg.id, rxMsg.id, rxMsg.len);
    for (uint8_t i = 0; i < rxMsg.len; ++i) {
      Serial.printf("%02X ", rxMsg.buf[i]);
    }

    // 解析 MKS 常用回報
    if (rxMsg.len >= 2) {
      if (rxMsg.buf[0] == 0xFD) {
        if (rxMsg.buf[1] == 0x01) Serial.print(" -> 【開始移動】");
        else if (rxMsg.buf[1] == 0x02) Serial.print(" -> 【移動完成 到位!】");
        else if (rxMsg.buf[1] == 0x00) Serial.print(" -> 【移動失敗/異常】");
      } else if (rxMsg.buf[0] == 0xF3) {
        Serial.print(" -> 【使能回報 ACK】");
      } else if (rxMsg.buf[0] == 0x8C) {
        Serial.print(" -> 【0x8C 設定回報 ACK】");
      }
    }
    Serial.println();
  }
}

// 顯示提示選單
void printMenu() {
  Serial.println("\n================================================");
  Serial.printf(" ★ MKS 步進馬達測試工具 | 目前目標: ID %u | 鮑率: %lu\n", targetNodeId, CAN_BAUD);
  Serial.println("================================================");
  Serial.println(" [Enter] 或 [g] : 正轉 2000 脈衝");
  Serial.println(" [b]            : 反轉 2000 脈衝");
  Serial.println(" [s]            : 緊急停止 (Stop)");
  Serial.println(" [e]            : 重新使能 (Enable)");
  Serial.println(" [4] ~ [7]      : 切換目標馬達 ID (例如按 4 切到 ID 4)");
  Serial.println(" 輸入數字       : 移動自訂脈衝數 (例如 4200 或 -4200)");
  Serial.println("================================================\n");
}

void setup() {
  Serial.begin(115200);

  // 等待 Serial 連線（最多等 1.5 秒，若未開 Monitor 亦會直接執行）
  uint32_t startWait = millis();
  while (!Serial && millis() - startWait < 1500) {
    delay(10);
  }

  Serial.println("\n[INIT] 正在啟動 CAN1 匯流排...");
  can1.begin();
  can1.setBaudRate(CAN_BAUD);

  delay(200);

  // 1. 先使能馬達
  enableMotor(targetNodeId, true);
  delay(150);

  // 2. 立即執行使用者要求的：正轉 2000 脈衝！
  Serial.println("\n>>> [自動執行] 正在驅動步進 正轉 2000 脈衝... <<<");
  moveRelative(targetNodeId, TEST_PULSES, DEFAULT_RPM, DEFAULT_ACC);

  printMenu();
}

void loop() {
  // 1. 處理 CAN 接收與狀態顯示
  processCanRx();

  // 2. 處理 Serial 輸入指令
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.length() == 0 || input.equalsIgnoreCase("g") || input == "1") {
      // 預設動作：正轉 2000 脈衝
      moveRelative(targetNodeId, TEST_PULSES);
    } else if (input.equalsIgnoreCase("b") || input == "2") {
      // 反轉 2000 脈衝
      moveRelative(targetNodeId, -TEST_PULSES);
    } else if (input.equalsIgnoreCase("s")) {
      emergencyStop(targetNodeId);
    } else if (input.equalsIgnoreCase("e")) {
      enableMotor(targetNodeId, true);
    } else if (input == "4" || input == "5" || input == "6" || input == "7") {
      targetNodeId = input.toInt();
      Serial.printf(">> 目標馬達已切換為: ID %u <<\n", targetNodeId);
      enableMotor(targetNodeId, true);
    } else if (input.equalsIgnoreCase("?") || input.equalsIgnoreCase("h")) {
      printMenu();
    } else {
      // 嘗試解析為自訂脈衝數字
      long customPulses = input.toInt();
      if (customPulses != 0 || input == "0") {
        Serial.printf(">> 執行自訂位移: %ld 脈衝 <<\n", customPulses);
        moveRelative(targetNodeId, customPulses);
      } else {
        Serial.printf("未知指令: '%s'，輸入 '?' 查看選單。\n", input.c_str());
      }
    }
  }
}
