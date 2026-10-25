#include <Arduino.h>

/**
 * ============================================================================
 *   TDK 30th - 氣壓缸 / 電動缸 (XY-160D / 電磁閥) 獨立 Serial 控制測試程式
 * ============================================================================
 * 
 * 支援硬體：
 *   1. XY-160D 雙向馬達驅動板 (控制電動缸伸出/縮回，數位 HIGH/LOW 全速驅動)
 *   2. 雙動氣壓缸雙電磁閥 (IN1 伸出, IN2 縮回)
 *   3. 單動氣壓缸單電磁閥 (只接 IN1 與 GND)
 * 
 * 腳位設定 (使用 Pin 16 / 17 數位輸出)：
 *   IN1: Pin 17 (伸出 Extend，輸出 HIGH/LOW)
 *   IN2: Pin 16 (縮回 Retract，輸出 HIGH/LOW)
 *   LED: Pin 13 (板載狀態指示燈)
 * 
 * 鮑率 (Baud Rate): 115200 bps
 */

// ==================== 腳位設定 ====================
constexpr uint8_t IN1_PIN = 17; // 伸出訊號 (Extend) -> HIGH 伸出, LOW 停止
constexpr uint8_t IN2_PIN = 16; // 縮回訊號 (Retract) -> HIGH 縮回, LOW 停止
constexpr uint8_t LED_PIN = 13; // 狀態 LED

// ==================== 參數設定 ====================
constexpr uint32_t DEADTIME_MS = 50; // 換向死區時間 (防止 H 橋短路直通)

enum class State : uint8_t {
  STOPPED,
  EXTENDING,
  RETRACTING,
  DEADTIME_TO_EXTEND,
  DEADTIME_TO_RETRACT,
  CYCLE_EXTEND,
  CYCLE_WAIT1,
  CYCLE_RETRACT,
  CYCLE_WAIT2
};

State currentState = State::STOPPED;
uint32_t actionStartTime = 0;
uint32_t actionDurationMs = 0; // 0 代表持續運動直到手動停止
uint32_t cycleParamMs = 3000;  // 自動循環單趟時間 (ms)

void printMenu();
void stopAll();
void startExtend(uint32_t durationMs = 0);
void startRetract(uint32_t durationMs = 0);
void handleSerial();

void setup() {
  Serial.begin(115200);
  
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  stopAll();

  // 等待 Serial 初始化 (最多等 1.5 秒，沒接電腦也能獨立開機)
  uint32_t startWait = millis();
  while (!Serial && (millis() - startWait < 1500));

  delay(200);
  printMenu();
}

void loop() {
  handleSerial();

  uint32_t now = millis();

  // 狀態機處理計時與自動停止
  switch (currentState) {
    case State::STOPPED:
      digitalWrite(LED_PIN, LOW);
      break;

    case State::DEADTIME_TO_EXTEND:
      if (now - actionStartTime >= DEADTIME_MS) {
        currentState = State::EXTENDING;
        actionStartTime = now;
        digitalWrite(IN1_PIN, HIGH);
        digitalWrite(IN2_PIN, LOW);
        Serial.printf(">> [動作中] 氣缸伸出 (Pin %u=HIGH, Pin %u=LOW)%s\n",
                      unsigned(IN1_PIN), unsigned(IN2_PIN), actionDurationMs ? "" : " [持續運行]");
      }
      break;

    case State::DEADTIME_TO_RETRACT:
      if (now - actionStartTime >= DEADTIME_MS) {
        currentState = State::RETRACTING;
        actionStartTime = now;
        digitalWrite(IN1_PIN, LOW);
        digitalWrite(IN2_PIN, HIGH);
        Serial.printf(">> [動作中] 氣缸縮回 (Pin %u=LOW, Pin %u=HIGH)%s\n",
                      unsigned(IN1_PIN), unsigned(IN2_PIN), actionDurationMs ? "" : " [持續運行]");
      }
      break;

    case State::EXTENDING:
      digitalWrite(LED_PIN, (now / 200) % 2); // 伸出時快閃
      if (actionDurationMs > 0 && (now - actionStartTime >= actionDurationMs)) {
        stopAll();
        Serial.println(">> [完成] 定時伸出結束，已停止。");
      }
      break;

    case State::RETRACTING:
      digitalWrite(LED_PIN, (now / 500) % 2); // 縮回時慢閃
      if (actionDurationMs > 0 && (now - actionStartTime >= actionDurationMs)) {
        stopAll();
        Serial.println(">> [完成] 定時縮回結束，已停止。");
      }
      break;

    // 自動循環往返測試
    case State::CYCLE_EXTEND:
      digitalWrite(LED_PIN, (now / 200) % 2);
      if (now - actionStartTime >= cycleParamMs) {
        digitalWrite(IN1_PIN, LOW);
        digitalWrite(IN2_PIN, LOW);
        currentState = State::CYCLE_WAIT1;
        actionStartTime = now;
        Serial.println(">> [循環] 伸出完畢，停頓 1 秒...");
      }
      break;

    case State::CYCLE_WAIT1:
      if (now - actionStartTime >= 1000) {
        currentState = State::CYCLE_RETRACT;
        actionStartTime = now;
        digitalWrite(IN1_PIN, LOW);
        digitalWrite(IN2_PIN, HIGH);
        Serial.println(">> [循環] 開始縮回...");
      }
      break;

    case State::CYCLE_RETRACT:
      digitalWrite(LED_PIN, (now / 500) % 2);
      if (now - actionStartTime >= cycleParamMs) {
        digitalWrite(IN1_PIN, LOW);
        digitalWrite(IN2_PIN, LOW);
        currentState = State::CYCLE_WAIT2;
        actionStartTime = now;
        Serial.println(">> [循環] 縮回完畢，停頓 1 秒...");
      }
      break;

    case State::CYCLE_WAIT2:
      if (now - actionStartTime >= 1000) {
        currentState = State::CYCLE_EXTEND;
        actionStartTime = now;
        digitalWrite(IN1_PIN, HIGH);
        digitalWrite(IN2_PIN, LOW);
        Serial.println(">> [循環] 重新開始伸出...");
      }
      break;
  }
}

// ==================== 動作控制函式 ====================

void stopAll() {
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(LED_PIN, LOW);
  currentState = State::STOPPED;
  actionDurationMs = 0;
}

void startExtend(uint32_t durationMs) {
  stopAll();
  actionDurationMs = durationMs;
  actionStartTime = millis();
  currentState = State::DEADTIME_TO_EXTEND; // 先經死區再通電
}

void startRetract(uint32_t durationMs) {
  stopAll();
  actionDurationMs = durationMs;
  actionStartTime = millis();
  currentState = State::DEADTIME_TO_RETRACT; // 先經死區再通電
}

// ==================== 序列埠指令解析 ====================

void handleSerial() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  char cmd = line.charAt(0);
  cmd = tolower(cmd);

  // 解析第二參數 (毫秒數值)
  int spaceIdx = line.indexOf(' ');
  uint32_t param = 0;
  if (spaceIdx > 0) {
    param = line.substring(spaceIdx + 1).toInt();
  }

  if (cmd == '0' || cmd == 's') {
    stopAll();
    Serial.println(">> [手動停止] 氣缸煞車停止 (Pin 16=LOW, Pin 17=LOW)");
  }
  else if (cmd == '1' || cmd == 'e' || cmd == '+') {
    startExtend(param);
    if (param > 0) {
      Serial.printf(">> 收到指令：伸出 %u 毫秒\n", param);
    } else {
      Serial.println(">> 收到指令：持續伸出 (按 0 停止)");
    }
  }
  else if (cmd == '2' || cmd == 'r' || cmd == '-') {
    startRetract(param);
    if (param > 0) {
      Serial.printf(">> 收到指令：縮回 %u 毫秒\n", param);
    } else {
      Serial.println(">> 收到指令：持續縮回 (按 0 停止)");
    }
  }
  else if (cmd == 'c') { // 自動循環往返測試
    stopAll();
    cycleParamMs = (param > 0) ? param : 3000;
    currentState = State::CYCLE_EXTEND;
    actionStartTime = millis();
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
    Serial.printf(">> [啟動自動循環] 單趟 %u ms (按 0 或 s 隨時停止)\n", cycleParamMs);
  }
  else if (cmd == 'h' || cmd == '?') {
    printMenu();
  }
  else {
    Serial.printf(">> 未知指令 '%s'，輸入 'h' 查看說明選單。\n", line.c_str());
  }
}

void printMenu() {
  Serial.println("\n========================================================");
  Serial.println("     TDK 30th 氣壓缸 / 電動缸 (XY-160D) 測試程式");
  Serial.printf ("  目前腳位: IN1=Pin %d (伸出), IN2=Pin %d (縮回)\n", IN1_PIN, IN2_PIN);
  Serial.println("  控制模式: 數位 HIGH / LOW (全速開關控制)");
  Serial.println("========================================================");
  Serial.println("  【即時控制指令】(在下方輸入框輸入後按 Enter)");
  Serial.println("   1        : 持續伸出 (Extend)");
  Serial.println("   2        : 持續縮回 (Retract)");
  Serial.println("   0 或 s   : 立即停止 (Stop)");
  Serial.println("--------------------------------------------------------");
  Serial.println("  【定時控制指令】(時間到自動停止)");
  Serial.println("   1 3000   : 伸出 3000 ms (3秒) 後自動停止");
  Serial.println("   2 5000   : 縮回 5000 ms (5秒) 後自動停止");
  Serial.println("--------------------------------------------------------");
  Serial.println("  【進階功能】");
  Serial.println("   c        : 自動往返循環測試 (預設單趟 3 秒)");
  Serial.println("   c 5000   : 自動往返循環測試 (自訂單趟 5 秒)");
  Serial.println("   h 或 ?   : 重新顯示此選單");
  Serial.println("========================================================\n");
}
