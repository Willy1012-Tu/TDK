#include <Arduino.h>
#include <Wire.h>

// ============================================================================
// IMUTEST.ino — Teensy 4.0 / 4.1 BNO055 / WCMCU-055 九軸 IMU 獨立測試程式
// ============================================================================
// 硬體接線：
//   Teensy 4.0 / 4.1 預設 Wire (I2C1)：
//     - SDA: Pin 18
//     - SCL: Pin 19
//     - VCC: 3.3V (請接 3.3V，切勿接 5V 避免損壞晶片)
//     - GND: GND
//     - ADR/COM3: 空接或接 GND (預設位址 0x28)；接 3.3V 為 0x29
//
// 功能特色：
//   1. 自動掃描 I2C 匯流排，自動識別 0x28 或 0x29 位址。
//   2. 驗證 BNO055 晶片 ID (0xA0)，若接線異常會直接提供排查提示。
//   3. 自動切換為 NDOF 絕對姿態融合模式，即時輸出 Yaw (航向)、Pitch、Roll。
//   4. 顯示 4 項校準狀態 (System, Gyro, Accel, Mag; 0~3, 3=完全校準)。
//   5. 支援 Serial Monitor (115200) 互動指令：
//        - 輸入 'z' : 將目前航向設為相對 0 度 (Tare / 歸零)
//        - 輸入 'r' : 恢復絕對航向 (Reset Tare)
//        - 輸入 'c' : 顯示感測器校準教學指南
//        - 輸入 's' : 重新掃描 I2C 位址
// ============================================================================

// BNO055 常用 I2C 位址
constexpr uint8_t ADDR_PRIMARY  = 0x28;
constexpr uint8_t ADDR_FALLBACK = 0x29;

// BNO055 暫存器
constexpr uint8_t BNO055_CHIP_ID_ADDR  = 0x00;
constexpr uint8_t BNO055_PAGE_ID_ADDR  = 0x07;
constexpr uint8_t BNO055_OPR_MODE_ADDR = 0x3D;
constexpr uint8_t BNO055_PWR_MODE_ADDR = 0x3E;
constexpr uint8_t BNO055_UNIT_SEL_ADDR = 0x3B;
constexpr uint8_t BNO055_EULER_H_LSB   = 0x1A; // 歐拉角起點 (連續 6 bytes: Yaw, Roll, Pitch)
constexpr uint8_t BNO055_CALIB_STAT    = 0x35; // 校準狀態
constexpr uint8_t BNO055_SYS_STATUS    = 0x39; // 系統狀態
constexpr uint8_t BNO055_SYS_ERR       = 0x3A; // 系統錯誤

// 操作模式
constexpr uint8_t OPERATION_MODE_CONFIG = 0x00;
constexpr uint8_t OPERATION_MODE_NDOF   = 0x0C; // 九軸融合模式

// 全域變數
uint8_t imuAddress = 0;       // 實際偵測到的位址 (0x28 或 0x29)
bool imuReady = false;
float yawOffset = 0.0f;       // 相對零點偏移量

// 讀寫輔助函式
bool writeRegister(uint8_t addr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  return (Wire.endTransmission() == 0);
}

bool readRegisters(uint8_t addr, uint8_t reg, uint8_t *buffer, uint8_t length) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false; // 重複起始條件 (Repeated Start)
  
  uint8_t readCount = Wire.requestFrom(addr, length);
  if (readCount != length) return false;
  
  for (uint8_t i = 0; i < length; i++) {
    buffer[i] = Wire.read();
  }
  return true;
}

// 角度轉換 (16-bit signed / 16.0)
float parseAngle(uint8_t lsb, uint8_t msb) {
  int16_t raw = (int16_t)(lsb | (msb << 8));
  return (float)raw / 16.0f;
}

// 限制角度於 0 ~ 360 度
float normalizeAngle360(float angle) {
  while (angle < 0.0f) angle += 360.0f;
  while (angle >= 360.0f) angle -= 360.0f;
  return angle;
}

// I2C 匯流排掃描
uint8_t scanI2C() {
  Serial.println("[I2C] 正在掃描 I2C 匯流排 (Pin 18 SDA, Pin 19 SCL)...");
  uint8_t foundAddr = 0;
  for (uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  -> 發現裝置: 0x%02X (%d)", address, address);
      if (address == ADDR_PRIMARY) {
        Serial.print(" ★ [可能是 BNO055 預設位址 0x28]");
        if (!foundAddr) foundAddr = address;
      } else if (address == ADDR_FALLBACK) {
        Serial.print(" ★ [可能是 BNO055 備用位址 0x29]");
        if (!foundAddr) foundAddr = address;
      }
      Serial.println();
    }
  }
  return foundAddr;
}

// 初始化 BNO055
bool initBNO055() {
  imuReady = false;
  
  // 1. 偵測位址
  imuAddress = 0;
  Wire.beginTransmission(ADDR_PRIMARY);
  if (Wire.endTransmission() == 0) imuAddress = ADDR_PRIMARY;
  else {
    Wire.beginTransmission(ADDR_FALLBACK);
    if (Wire.endTransmission() == 0) imuAddress = ADDR_FALLBACK;
  }

  if (imuAddress == 0) {
    Serial.println("\n【錯誤】在 0x28 與 0x29 皆未偵測到 BNO055！");
    Serial.println("排查建議：");
    Serial.println("  1. 確認接線：SDA -> Teensy Pin 18，SCL -> Teensy Pin 19");
    Serial.println("  2. 確認供電：VCC 接 3.3V，GND 接地");
    Serial.println("  3. 若模組沒有內建提升電阻，SDA/SCL 需各接 4.7kΩ 電阻至 3.3V");
    return false;
  }

  Serial.printf("\n[IMU] 偵測到位址: 0x%02X，正在讀取 Chip ID...\n", imuAddress);
  delay(100);

  // 2. 切換到 Page 0
  writeRegister(imuAddress, BNO055_PAGE_ID_ADDR, 0x00);
  delay(10);

  // 3. 讀取 Chip ID
  uint8_t chipId = 0;
  if (!readRegisters(imuAddress, BNO055_CHIP_ID_ADDR, &chipId, 1) || chipId != 0xA0) {
    Serial.printf("【錯誤】讀取到的 Chip ID = 0x%02X (預期為 0xA0)！請檢查感測器晶片型號。\n", chipId);
    return false;
  }
  Serial.printf("[IMU] 晶片驗證成功！Chip ID = 0x%02X (Bosch BNO055)\n", chipId);

  // 4. 切換為 Config 模式進行設定
  writeRegister(imuAddress, BNO055_OPR_MODE_ADDR, OPERATION_MODE_CONFIG);
  delay(30);

  // 5. 設定 Normal Power Mode
  writeRegister(imuAddress, BNO055_PWR_MODE_ADDR, 0x00);
  delay(15);

  // 6. 設定單位為 Degrees、dps、m/s^2 (預設 0x00)
  writeRegister(imuAddress, BNO055_UNIT_SEL_ADDR, 0x00);
  delay(15);

  // 7. 切換為 NDOF 九軸融合模式
  writeRegister(imuAddress, BNO055_OPR_MODE_ADDR, OPERATION_MODE_NDOF);
  delay(30);

  Serial.println("[IMU] 已成功設定為 NDOF 絕對姿態融合模式！開始輸出數據...\n");
  imuReady = true;
  return true;
}

// 顯示校準提示教學
void printCalibrationHelp() {
  Serial.println("\n------------------------------------------------------------");
  Serial.println(" 【BNO055 九軸校準操作指南】 (數值 0=未校準, 3=完全校準)");
  Serial.println("------------------------------------------------------------");
  Serial.println(" 1. 陀螺儀 (Gyro) : 將感測器靜止平放於桌面上 2~3 秒即可達到 3。");
  Serial.println(" 2. 磁力計 (Mag)  : 將感測器在空中劃「8」字形晃動數次。");
  Serial.println(" 3. 加速度計(Acc) : 將感測器朝 6 個不同的正向面分別靜止放置數秒。");
  Serial.println(" 4. 系統融合(Sys) : 上述三者逐漸完成後，系統會自動升至 3。");
  Serial.println("------------------------------------------------------------\n");
}

void printHelpMenu() {
  Serial.println("\n============================================================");
  Serial.println(" ★ IMUTEST.ino 互動指令選單 (輸入後按 Enter 發送)");
  Serial.println("============================================================");
  Serial.println(" [z] : 將當前航向 (Yaw) 設為相對 0 度 (歸零 / Tare)");
  Serial.println(" [r] : 恢復絕對原始航向 (Reset Tare)");
  Serial.println(" [c] : 查看感測器校準說明指南");
  Serial.println(" [s] : 重新掃描 I2C 匯流排位址");
  Serial.println(" [h] : 顯示本說明選單");
  Serial.println("============================================================\n");
}

void setup() {
  Serial.begin(115200);

  uint32_t startWait = millis();
  while (!Serial && millis() - startWait < 1500) {
    delay(10);
  }

  Serial.println("\n============================================================");
  Serial.println("  Teensy 4.0 / 4.1 BNO055 (WCMCU-055) IMU 姿態感測器測試工具");
  Serial.println("============================================================");

  // 初始化 I2C，BNO055 標準使用 100kHz
  Wire.begin();
  Wire.setClock(100000);

  delay(650); // BNO055 開機至少需要 650ms 啟動

  // 初始化 BNO055
  if (!initBNO055()) {
    scanI2C();
  }

  printHelpMenu();
}

void loop() {
  // 1. 處理 Serial 輸入指令
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 'z' || cmd == 'Z') {
      // 讀取當前絕對角度作為 offset
      uint8_t buf[2];
      if (readRegisters(imuAddress, BNO055_EULER_H_LSB, buf, 2)) {
        yawOffset = parseAngle(buf[0], buf[1]);
        Serial.printf("\n>> [歸零完成] 目前角度 %.2f° 已設為 0.00° <<\n\n", yawOffset);
      }
    } else if (cmd == 'r' || cmd == 'R') {
      yawOffset = 0.0f;
      Serial.println("\n>> [重置] 已恢復為感測器絕對航向 <<\n");
    } else if (cmd == 'c' || cmd == 'C') {
      printCalibrationHelp();
    } else if (cmd == 's' || cmd == 'S') {
      scanI2C();
    } else if (cmd == 'h' || cmd == 'H') {
      printHelpMenu();
    }
  }

  // 2. 定期讀取並顯示 IMU 姿態資料 (每 100ms 更新一次)
  static uint32_t lastPrint = 0;
  if (imuReady && millis() - lastPrint >= 100) {
    lastPrint = millis();

    // 一次讀取 6 個 bytes 歐拉角 (Yaw, Roll, Pitch)
    uint8_t eulerBuf[6];
    bool okEuler = readRegisters(imuAddress, BNO055_EULER_H_LSB, eulerBuf, 6);

    // 讀取 1 個 byte 校準狀態
    uint8_t calibStat = 0;
    bool okCalib = readRegisters(imuAddress, BNO055_CALIB_STAT, &calibStat, 1);

    if (okEuler && okCalib) {
      float rawYaw   = parseAngle(eulerBuf[0], eulerBuf[1]);
      float rawRoll  = parseAngle(eulerBuf[2], eulerBuf[3]);
      float rawPitch = parseAngle(eulerBuf[4], eulerBuf[5]);

      // 計算相對歸零後的 Yaw
      float relYaw = normalizeAngle360(rawYaw - yawOffset);

      // 解析校準位元 (0..3)
      uint8_t calSys = (calibStat >> 6) & 0x03;
      uint8_t calGyr = (calibStat >> 4) & 0x03;
      uint8_t calAcc = (calibStat >> 2) & 0x03;
      uint8_t calMag = calibStat & 0x03;

      Serial.printf("[IMU] 航向(Yaw): %6.2f° (原:%6.2f°) | 俯仰(Pitch): %+6.2f° | 翻滾(Roll): %+6.2f° || 校準[S:%u G:%u A:%u M:%u]\n",
                    relYaw, rawYaw, rawPitch, rawRoll, calSys, calGyr, calAcc, calMag);
    } else {
      Serial.println("[IMU 警告] 讀取逾時或 NACK！正在嘗試重新連線...");
      delay(500);
      initBNO055();
    }
  }
}
