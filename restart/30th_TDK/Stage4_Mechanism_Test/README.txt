TDK30th Stage4_Mechanism_Test — 雙軸尋原點／第二關ID7四段／第三關暫時待機
2026-10-12 / Asia/Taipei

開啟與執行
  用 VS Code／PlatformIO 開啟本專案；支援 teensy40 與 teensy41。
  powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\compile.ps1 -Board teensy40
  powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\compile.ps1 -Board teensy41
  powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\start_web_gui.ps1
  GUI：http://127.0.0.1:8765。compile.ps1 只編譯；使用最新版須另外 Upload，GUI 啟動不更新韌體。
  可用關卡 2、3、4；YAML 變更預設後 Build／Upload，RAM 熱調試不需重新燒錄。

開機尋原點與位置
  本版啟用 MKS ID6／ID7；ID4/5 未接線，Pin3 不 attach，也不接受動作指令。
  STARTUP_RETURN_ZERO=true：兩顆完成 0x8C 回報設定及 0xF3 啟用 ACK 後，同批發送 0x91 尋原點。
  開機雙軸尋原點，兩顆各自開始 01→完成 02 都收到後才 Ready；不自動執行第二關 STEP1。
  Pin2=1150、Pin5=2100、Pin4=2000、Pin6=1400、Pin9=1800 us，為本次指定開機待機值。
  position_known_mask 初始化0；bit2=ID6（4）、bit3=ID7（8），全部確認為12才完成開機尋原點。
  未確認的軸 GUI 顯示「尚未確認位置」；確認後顯示最近完成目標，仍不是編碼器實測值。
  HOME第四關只尋ID6；HOME 第二關只尋 ID7，並恢復 Pin4/6/9 待機2000/1400/1800。
  Debug 可選 ID6 或 ID7；GoHome 發91依既有HM實際移動，「移到位置0」則發FE絕對目標0。
  0x91 是尋原點，0x80 才是校正；本版不自動發80校正或92重設目前坐標。
  若自行改 STARTUP_RETURN_ZERO=false，開機僅設定／啟用，軟體0不重設驅動器FE坐標，也不確認原點。
  8C／F3 每500ms非阻塞重試，每顆每種設定指令各有固定10秒ACK上限；明確拒絕立即Fault。
  91／FE 運動不因缺回覆而自動重送；任一Fault或STOP鎖存，需檢查後重啟控制板。
  鮑率CAN1為1Mbps，HM方向／速度／限位與工作模式沿用驅動器既有設定。

USB 診斷（115200 baud）
  Monitor 或 GUI 擇一使用串口；只讀查詢一行：@1 status。
  本版 firmware_id="Stage4_Mechanism_Test"、firmware_revision="stage2-id67-home-v1"、startup_return_zero=true。
  motor_ids=[6,7]、servo_pins=[2,5,4,6,9]、stage3_enabled=false；新增 position7；保留 pulse4/pulse6/pulse9。
  舊 position4/5 與 pulse3 保留為 0 供協定相容，不能視為已接線或已完成動作。
  先看 phase=fault 是否已鎖存故障，再查 startup_phase、startup_node、startup_command、startup_attempts。
  startup_command 以十進位回報：140=8C、243=F3、145=91。
  設定等待時startup_node是目前等ACK的ID；尋原點期間需以mask與各ID通知確認兩軸，不只看最近發送node。
  詳細 ACK、Fault 與驅動器協定診斷見 docs/errors.txt；GUI 連線只查詢，不代替 Build／Upload。

模組與通訊
  main.cpp 初始化 CAN1 1Mbps（RX23/TX22）及 USB Serial 115200；test_controller.cpp 排程機構。
  test_config.h 集中硬體與計時；stage4_config.h 定義共享 Action、HOME 及三關生成表。
  TeensyThreads 狀態鎖保護 USB／控制器快照；運動等待與 duration 緩移使用非阻塞狀態機。
  此專案只測試機構，沒有 ODrive 底盤或 ELRS；整車請使用同層 Robot_Integrated_Control。
  Normal 提供 NEXT／RUN／HOME／STOP；本機構測試版本不提供 PREV。

第二關：stage2_steps.yaml，4段、repeat_from=2
  從其他關切入第二關，自動執行 STEP1 預備；同一有效第二關重複選入不重做。
  STEP1：ID7=-8000，320 RPM/ACC2；同時 Pin9=1800、Pin4=1300 us。
    等 ID7 開始→完成回報與同組伺服指令完成才接受下一鍵。
  第一次 NEXT（STEP2）：Pin9=1100、Pin4=2000 us 同時輸出。
  第二次 NEXT（STEP3）：ID7=-3000，320 RPM/ACC2；同時 Pin6=2200 us。
  第三次 NEXT（STEP4）：Pin6=1600、ID7=-8000（320 RPM/ACC2）、Pin9=1800、Pin4=1300同動。
  之後 NEXT 循環 STEP2→3→4；STEP1僅新切入或新套用RAM時自動準備，RUN則從STEP1全表跑一次。
  所有servo duration_ms省略，預設立即寫入；未列出的伺服保持最近輸出。
  套用第二關新RAM也自動STEP1；網頁等預備完成並Ready後才報成功，Fault／STOP／逾時不標記成功。
  HOME第二關以91尋ID7，並恢復Pin4=2000、Pin6=1400、Pin9=1800；HOME完成後NEXT由STEP1重新開始。

第三關：stage3_steps.yaml，STAGE3_ENABLED=false
  暫時拔除稻草卷機構，只保留repeat_from=1、一段delay0待機。
  HOME／NEXT／RUN不發馬達或舵機指令；離開第三關立即切換，不執行Park。
  第三關不能套用含致動器的RAM表；實際控制器選中第三關時，Debug致動器控制全部拒收。
  切到第二或第四關即可調試；僅載入第三關電腦YAML不改控制器關卡。

第四關：stage4_steps.yaml，repeat_from=1（擲筊流程維持原值）
  STEP1：ID6=-600，320 RPM/ACC0；Pin2=1500、Pin5=1400。
  STEP2：Pin2=1120，步進保持。
  STEP3：ID6=-8100，320 RPM/ACC5；Pin5 在 2000ms 緩移至 1550；等兩者完成。
  STEP4：ID6=-600，1500 RPM/ACC0；Pin5=1400。
    上升完成立即接續 ID6=-2600，320 RPM/ACC0；Pin5=1800，不需再按 NEXT。
  STEP5：Pin2=1500；Pin5 保持 1800。
  STEP6：ID6=0，320 RPM/ACC0；Pin2=1150、Pin5=2100。
  此 STEP6 是 FE 移至驅動器絕對坐標 0；手動 HOME 才是 91 尋原點。

GUI 與 RAM
  電腦編輯關卡選單只載入檔案；Normal「控制器切關」才改啟用關卡。
  「立即套用」驗證完整 RAM 表並切到該關，第二關會先自動STEP1預備；其餘關卡由NEXT／RUN開始。
  Debug 可選 ID6／ID7 與 Pin2/5/4/6/9 五顆伺服；各有獨立角度、duration_ms 與發送按鈕。
  角度 0..180 映射 500..2500 us；拖滑桿只改欄位，按發送才輸出。
  儲存 YAML、套用 RAM、切關是分開操作；重選同一第二關保留當前RAM；切往其他關恢復內建表，回第二關要重新套用；重啟亦恢復內建。
  每關最多254 STEP、2048 個展開後 Action；parallel 支援最多7個不同致動器，含等待屏障。
  STOP 在任一模式或 RAM 傳輸期間可發送；停止並停用 ID6/7、取消緩移、detach 五顆 PWM。
  機構 STOP 鎖存至重啟。畫面保留 STOP 前指令，USB 中斷連線不會自動發 STOP。

工具與驗證
  include/test_config.h：active ID6/7、五 PWM 腳位、startup=true、stage3=false；未擅改驅動器模式、HM 或鮑率。
  tools/generate_steps.py／platformio_steps.py：Build 前產生 三份 stageN_steps.generated.h。
  tools/web_gui.py／web/：同源、token、Host 驗證，無 Node/CDN runtime 依賴。
  tools/python_runtime.ps1：處理複製後失效 venv，沿用同層 tmp 可攜 Python／套件。
  powershell -NoProfile -ExecutionPolicy Bypass -File .\test\run_tests.ps1
  test/test_generate_steps.py、test/test_web_gui.py、test/frontend_regression.mjs 覆蓋新腳位與第二關 RAM。
  模擬串口／DOM 測試與編譯不代表實體機構到位、CAN 電氣品質或瀏覽器視覺驗證。
  詳細見 docs/yaml_steps.txt、web_gui.txt、errors.txt、verification.txt；.pio/logs/ 保存測試與兩板編譯。
