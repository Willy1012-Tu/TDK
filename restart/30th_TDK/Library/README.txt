專案函數庫與來源（2026-10-09）

Stage4_Mechanism_Test 實際使用：
  FlexCAN_T4/    CAN1 驅動，完整保留標頭、模板、範例與 MIT LICENSE。
  TeensyThreads/ 執行緒、互斥鎖，版本 1.0.2，保留組合語言檔與 license.txt。
  Servo/        PJRC Teensy Servo，版本 1.1.2；檔頭保留 LGPL 2.1 或更新版聲明。
上述三份由本機 Teensy board package 1.62.0 的 libraries 原樣複製。
FlexCAN_T4 隨附版本沒有 library.properties，依原套件保留舊式 Arduino library 結構。
Arduino.h、IntervalTimer.h 屬於 Teensy core，由板型套件提供，不能用 test/stubs 替代。

來源：
  https://www.pjrc.com/teensy/td_download.html
  https://github.com/tonton81/FlexCAN_T4
  https://github.com/ftrias/TeensyThreads
  https://github.com/PaulStoffregen/Servo
Arduino IDE 選 Teensy 4.0 或 4.1 時可使用板型套件附帶的三份庫。
Stage4 現已改為 PlatformIO，將上述三份庫完整複製至 Stage4_Mechanism_Test/lib/。
Stage4 的實際函數庫版本以該 lib/ 為準；本資料夾保留供其他程式共用。
不要把 test/stubs 加入實機函數庫路徑。

原有共用函數：
  alg_pid.h/.cpp     PID 控制，依賴 drv_math.h。
  drv_math.h/.cpp    端序轉換、求和、限幅、sinc 等數學函數。
  trajectory.h/.cpp Teensy IntervalTimer 驅動的雙實例梯形軌跡規劃。
  servoxxd_can.h/.cpp 舊版多馬達 CAN API，位置指令使用 F5。
Stage4 控制器自行處理 FE 位置指令、回報與逾時，沒有使用這四組共用函數。
共用 .h/.cpp 留在原位置；未將舊 CAN 模組加入 Stage4，避免同時建立兩個 CAN1 控制器。

本次共用庫修正：
  trajectory.cpp 明確 include <IntervalTimer.h>。
  servoxxd_can 移除多餘 Print.h；支援 ID1..6，所有命令先檢查 ID，避免 ID0/7/255 索引越界。
  CAN 回報檢查標準資料幀、長度與加總校驗；位置回報須 8 bytes。
  PID 修正變速積分區間被覆蓋、A/B 判斷筆誤、speed_ratio 未初始化與 K_I=0 除法。
  drv_math 使用相同有號性迴圈及 sinf，保留 float 精度介面。
共用庫 smoke/regression 在 Stage4_Mechanism_Test/test/library_regression.cpp。
