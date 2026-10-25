底盤搖桿運動控制程式
====================

由提供的 Arduino 程式整理成獨立 PlatformIO 專案。
預設 Teensy 4.0，另提供 Teensy 4.1 編譯環境。

保留功能
--------
• ELRS / CRSF 接收搖桿。
• CH3 前進／後退、CH4 左右平移、CH1 原地旋轉。
• 四輪混控，保留原本馬達正反向與搖桿校正值。
• 輪速按比例縮放，任一輪不超過 MAX_SPEED。
• CAN1 控制前左 ID10、前右 ID11；CAN2 控制後左 ID12、後右 ID13。
• 搖桿持續置中 500 ms，四顆 ODrive 心跳正常且為 IDLE，才自動解鎖。
• 接收機失聯、通道無效、ODrive 異常／心跳逾時、退出閉迴路時鎖定。
• 解鎖期間移動搖桿或超過 3 秒，取消解鎖。

簡化內容
--------
移除原本的大段啟動提示、重複分隔註解與預設定期輸出。
運動／安全流程集中在 src/main.cpp；可調參數集中在 include/chassis_config.h。
補上 CAN 傳送失敗的鎖定處理，故障恢復後重新計算置中時間。
全部時間判斷使用 millis()，主迴圈沒有 delay() 或等待解鎖的阻塞迴圈。

檔案架構
--------
platformio.ini               開發板、套件、Monitor 設定
src/main.cpp                 搖桿讀取、CAN 通訊、解鎖／鎖定與速度輸出
include/chassis_config.h     馬達 ID、通道校正、速度、逾時設定
include/chassis_math.h       死區映射與四輪混控
lib/FlexCAN_T4/              專案內附 CAN 函式庫與原授權
test/                       主機端控制流程回歸測試
VERIFICATION.txt             驗證結果與範圍

透過 VS Code / PlatformIO IDE 操作
--------------------------------
1. 選「檔案 → 開啟資料夾」，開啟此「底盤搖桿運動控制程式」資料夾。
   請開啟這個包含 platformio.ini 的新資料夾。
2. 等待 PlatformIO 安裝 Teensy 平台及 AlfredoCRSF 2.4.0。
3. 按左側 PlatformIO 圖示，展開 PROJECT TASKS → teensy40 → General。
4. Build：編譯；Upload：上傳；Monitor：查看狀態。
5. 實體板為 Teensy 4.1 時，改用 teensy41 的任務。
   也可將 platformio.ini 的 default_envs 改成 teensy41。

預設 Monitor 115200，只在 ARMING、RUNNING、LOCKED 狀態切換時輸出。
程式由搖桿控制，Monitor 沒有 run / next / stop 等文字指令。
若要觀察搖桿通道和輪速，將 PRINT_STATUS 改為 true，重新 Build / Upload。
Wheel 顯示順序為前左、前右、後左、後右。

在 PlatformIO 終端機亦可使用：
pio run -e teensy40
pio run -e teensy40 -t upload
pio device monitor -b 115200

主要設定範例
------------
修改 include/chassis_config.h 後重新 Build / Upload：

constexpr float MAX_SPEED = 20.0f;       // 最大馬達速度 20 turns/s
constexpr int DEAD_ZONE = 30;           // 搖桿置中死區 ±30
constexpr bool PRINT_STATUS = true;     // 每 500 ms 輸出通道／輪速

Channel 格式：{通道編號, 最小值, 中心值, 最大值}
例如：constexpr Channel FORWARD = {3, 1000, 1500, 2000};

目前最高速度沿用原程式的 90 turns/s，即 5400 RPM，指馬達軸轉速。
實際輪速取決於減速比。要調整原有輪向，修改 chassis_math.h 的四輪混控公式。

接線與 ODrive 前置設定
---------------------
Teensy 4.0 / 4.1 預設腳位：
• ELRS TX → Teensy Serial2 RX 腳 7；ELRS RX → Serial2 TX 腳 8；共地。
• CAN1 TX 腳 22、RX 腳 23 → 第一組 CAN 收發器 → 前左／前右 ODrive。
• CAN2 TX 腳 1、RX 腳 0 → 第二組 CAN 收發器 → 後左／後右 ODrive。
• 兩組 CAN 使用 1000000 bit/s（1 Mbps），各自依實際總線配置終端電阻。
• ELRS 接收機需使用 CRSF UART 輸出，速率 420000。

Teensy CAN 腳位需要經 CAN 收發器，不能直接接 CANH / CANL。
邏輯介面需相容 Teensy 3.3 V。
ODrive 應事先完成馬達校正、CAN ID 與速率設定，並設為速度控制。
輸入模式須能處理 Set_Input_Vel，例如 PASSTHROUGH 或適當設定的 VEL_RAMP。
四顆 ODrive 必須主動送 Heartbeat，週期小於 500 ms，例如 100 ms。
此程式送速度與軸狀態，不設定 ODrive 的控制模式、校正、加減速或 watchdog。

鎖定會嘗試送零速度及 IDLE，並每 250 ms 重試。
CAN 完全斷線時無法保證停止命令送達；要涵蓋此情況，ODrive 端需另設 watchdog。
請使用對應 ODrive 韌體版本的設定方式。

啟動流程
--------
LOCKED → 搖桿置中 500 ms，ELRS 與四顆 ODrive 正常 → ARMING
ARMING → 四顆皆回報 CLOSED_LOOP → RUNNING
RUNNING → 搖桿輸入持續轉換成四輪速度，每 15 ms 發送一次
異常 → LOCKED → 恢復正常並重新置中 500 ms 後，再自動解鎖

搖桿放回中央時輸出零速度，但仍保持閉迴路。
此專案保留原程式的「置中自動解鎖」，沒有新增解鎖開關。

測試
----
Windows 安裝 Visual Studio Build Tools（含 C++）後，執行 test\run_tests.cmd。
測試用模擬 CAN／接收機介面，不會操作實體馬達。

函式庫來源與介面參考
-------------------
AlfredoCRSF（PlatformIO 固定版本 2.4.0）：
https://github.com/AlfredoSystems/AlfredoCRSF
FlexCAN_T4：由現有專案 lib/FlexCAN_T4 複製，保留原始碼與 LICENSE；
不需要引用上一層 Stage4 的程式碼或函式庫。
https://github.com/tonton81/FlexCAN_T4
Teensy UART 腳位：
https://www.pjrc.com/teensy/td_uart.html
ODrive CANSimple 及 watchdog（請選擇對應韌體版本）：
https://docs.odriverobotics.com/v/latest/manual/can-protocol.html
