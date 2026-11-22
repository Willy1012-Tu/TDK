# TDK

第 30 屆 TDK 比賽的機器人控制程式，使用 Teensy 4.0 / 4.1，以 PlatformIO 開發。

這個倉庫只保留最終版本 `Robot_Integrated_Control`：整合各關卡、道具機構、XY-160D 電動缸、安全切關與即時 IMU 的控制程式。

## 目錄結構

```
TDK/
├── README.md
├── 30th_TDK_Match_Parameters.txt        各關 STEP 流程參數與燒入指令筆記
└── 30th_TDK/
    └── Robot_Integrated_Control/        整合控制程式（最終版本）
        ├── src/ include/                控制器、CAN、遙控、安全等原始碼
        ├── lib/                         專案內附函式庫（FlexCAN_T4、AlfredoCRSF、Servo）
        ├── stage*_steps.yaml            各關動作流程
        ├── stage1_*.yaml                第 1 關各道具動作流程
        ├── tools/                       編譯、燒錄、網頁 GUI 腳本
        ├── test/                        主機端回歸測試
        └── docs/                        各項功能與驗證的說明文件
```

## 開始使用

需要 VS Code 與 PlatformIO，支援 `teensy40`（預設）與 `teensy41`。在 `30th_TDK/Robot_Integrated_Control` 資料夾內，用 PowerShell 執行：

編譯（只編譯，不燒錄）：

```
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\compile.ps1 -Board teensy40
```

燒錄到 Teensy 4.0：

```
powershell -ExecutionPolicy Bypass -File .\tools\pio.ps1 run -e teensy40 -t upload
```

啟動網頁 GUI（網址 http://127.0.0.1:8765）：

```
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\start_web_gui.ps1
```

啟動 GUI 不會更新韌體，要使用最新版必須另外燒錄。

## 比賽前務必確認

預設開啟「檯面單關獨立測試」模式（`include/system_config.h` 的 `SystemConfig::BENCH_TEST_MODE=true`），這個模式放寬了部分比賽互鎖。上場比賽前，要把它改成 `false`，重新編譯並燒錄。

## 動作流程與參數

- 各關的動作流程寫在 `stage*_steps.yaml` 與 `stage1_*.yaml`。修改預設值後需要重新編譯並燒錄；用 GUI 在記憶體中熱調試則不需重新燒錄。
- `30th_TDK_Match_Parameters.txt` 記錄比賽用的各關 STEP 流程參數與燒入指令，裡面的資料夾路徑是原本那台電腦上的路徑，使用前請換成你自己的資料夾位置。

## 更多說明

詳細的啟動流程、致動器設定、診斷與協定，請看 [Robot_Integrated_Control/README.txt](30th_TDK/Robot_Integrated_Control/README.txt) 與 `docs/` 資料夾。主機端回歸測試放在 `test/`。

## 整理紀錄

2026-11-22 整理倉庫，只留下最終版本。整理前的內容（包含舊的 `Chassis_Control_Remote_Test`、`Stage4_Mechanism_Test`、共用 `Library` 與單獨測零件的程式）可以從提交 `5201bc0` 找回。
