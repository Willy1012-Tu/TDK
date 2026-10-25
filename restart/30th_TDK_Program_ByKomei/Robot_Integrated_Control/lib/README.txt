本專案的 PlatformIO 私有函數庫

FlexCAN_T4/、TeensyThreads/、Servo/ 完整複製自上層 Library 對應資料夾。
來源為 PJRC Teensy board package 1.62.0；授權、範例與組合語言檔均保留。
這三個資料夾由 PlatformIO Library Dependency Finder 偵測並編譯。
Arduino.h、IntervalTimer.h 由 framework-arduinoteensy 提供。
test/stubs/ 只供主機測試使用，不在實機 include 路徑內。

沒有加入上層 Library 的舊 CAN、PID、數學與軌跡模組；Stage4 控制器未使用它們。
若要更新正式函數庫，以本 lib/ 為此 PlatformIO 專案的實際使用版本。
