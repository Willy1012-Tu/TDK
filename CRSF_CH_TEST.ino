#include <Arduino.h>
#include <AlfredoCRSF.h>

// ============================================================================
// CRSF_CH_TEST.ino — Teensy 4.0 / 4.1 ELRS / CRSF 遙控通道 獨立即時數值測試程式
// ============================================================================
// 硬體接線：
//   Teensy 4.0 / 4.1 UART2：
//     Pin 7 (RX2)  <-- ELRS 接收器 TX
//     Pin 8 (TX2)  --> ELRS 接收器 RX (選接，若有回傳 Telemetry)
//     5V / 3.3V    <-- ELRS 供電 (請依接收器規格)
//     GND          <-- ELRS 接地
//
// 鮑率說明：
//   - USB Serial (電腦監控視窗)：115200 baud
//   - ELRS CRSF (Serial2)：420000 baud
//
// 特色功能：
//   1. 自動偵測 ELRS 連線狀態 (LinkUp / LinkDown)。
//   2. 顯示 CH1 ～ CH16 所有通道微秒數值 (us)，範圍約 988 ~ 2012 us (中立 1500 us)。
//   3. 專為 TDK 機器人標註對應功能（底盤前進/平移/旋轉、關卡切換 CH10~13、NEXT/PREV/CLEAR 等）。
//   4. 清楚判斷開關狀態：[LOW <1300] / [MID ~1500] / [HIGH >1700]。
//   5. 支援兩種輸出模式（可在 Serial Monitor 輸入切換）：
//        - 輸入 '1'：即時儀表板模式（多行整齊表格，附對應功能名稱，容易閱讀）
//        - 輸入 '2'：單行簡潔連續模式（方便複製或搭配 Serial Plotter 繪圖）
// ============================================================================

#define ELRS_SERIAL Serial2
constexpr uint32_t CRSF_BAUD = 420000;

AlfredoCRSF crsf;

// 刷新間隔 (ms)
constexpr uint32_t PRINT_INTERVAL_MS = 100; // 每 100ms 更新一次
uint32_t lastPrintTime = 0;
uint8_t displayMode = 1; // 1: 儀表板模式, 2: 單行模式

// TDK 機器人專案定義之常用通道名稱
const char* getChannelLabel(uint8_t ch) {
  switch (ch) {
    case 1:  return "底盤 旋轉 (ROTATE)";
    case 2:  return "底盤 前進/後退 (FORWARD)";
    case 3:  return "底盤 左右平移 (STRAFE)";
    case 4:  return "閒置 (CH4)";
    case 5:  return "動作 PREV (上一步)";
    case 6:  return "動作 NEXT (下一步)";
    case 9:  return "ODrive 清除錯誤 (CLEAR)";
    case 10: return "關卡 1 (Stage 1)";
    case 11: return "關卡 2 (Stage 2)";
    case 12: return "關卡 3 (Stage 3)";
    case 13: return "關卡 4 (Stage 4)";
    default: return "-";
  }
}

// 判斷按鍵/開關高低位
const char* getSwitchState(int value) {
  if (value > 1700) return "[ HIGH ]";
  if (value < 1300) return "[ LOW  ]";
  return "[ MID  ]";
}

void setup() {
  Serial.begin(115200);
  ELRS_SERIAL.begin(CRSF_BAUD);
  crsf.begin(ELRS_SERIAL);

  // 等待 USB Serial 啟動（最多 1 秒）
  uint32_t start = millis();
  while (!Serial && (millis() - start < 1000));

  Serial.println("\n=======================================================");
  Serial.println("  Teensy 4.0 / 4.1 ELRS CRSF 遙控通道 測試程式");
  Serial.println("=======================================================");
  Serial.println("接線說明：ELRS 接收器 TX 接 Teensy Pin 7 (RX2)");
  Serial.println("通訊鮑率：Serial2 = 420000 baud");
  Serial.println("提示：可在下方輸入 '1' 切換儀表板模式，輸入 '2' 切換單行模式");
  Serial.println("=======================================================\n");
}

void printDashboard(bool linkUp) {
  Serial.println("------------------- [ ELRS CRSF 狀態 ] -------------------");
  Serial.print("連線狀態: ");
  if (linkUp) {
    Serial.println(">>> CONNECTED (已連線) <<<");
  } else {
    Serial.println("... WAITING / DISCONNECTED (等待連線中) ...");
  }

  Serial.println("通道 | 數值 (us) | 狀態       | TDK 機器人預設功能對應");
  Serial.println("-----+-----------+------------+---------------------------");

  for (uint8_t ch = 1; ch <= 16; ++ch) {
    int val = crsf.getChannel(ch);
    char buf[80];
    snprintf(buf, sizeof(buf), "CH%-2d | %4d us   | %s   | %s",
             ch, val, getSwitchState(val), getChannelLabel(ch));
    Serial.println(buf);
  }
  Serial.println("----------------------------------------------------------\n");
}

void printCompact(bool linkUp) {
  if (!linkUp) {
    Serial.println("[ELRS DISCONNECTED]");
    return;
  }
  // 印出前 13 通道，方便即時觀察
  Serial.printf("CH1:%4d | CH2:%4d | CH3:%4d | CH5:%4d | CH6:%4d | CH9:%4d | S1(10):%4d | S2(11):%4d | S3(12):%4d | S4(13):%4d\n",
                crsf.getChannel(1), crsf.getChannel(2), crsf.getChannel(3),
                crsf.getChannel(5), crsf.getChannel(6), crsf.getChannel(9),
                crsf.getChannel(10), crsf.getChannel(11), crsf.getChannel(12), crsf.getChannel(13));
}

void loop() {
  crsf.update();

  // 接收鍵盤輸入切換模式
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '1') {
      displayMode = 1;
      Serial.println("\n[已切換為：完整儀表板模式]");
    } else if (c == '2') {
      displayMode = 2;
      Serial.println("\n[已切換為：單行連續模式]");
    }
  }

  uint32_t now = millis();
  if (now - lastPrintTime >= PRINT_INTERVAL_MS) {
    lastPrintTime = now;
    bool linkUp = crsf.isLinkUp();

    if (displayMode == 1) {
      printDashboard(linkUp);
    } else {
      printCompact(linkUp);
    }
  }
}
