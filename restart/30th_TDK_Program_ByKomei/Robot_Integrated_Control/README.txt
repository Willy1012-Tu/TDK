TDK30th Robot_Integrated_Control — 五關／五道具機構／XY-160D電動缸／安全切關／即時IMU
2026-10-14 / Asia/Taipei

檯面單關獨立測試（目前預設開啟）
  SystemConfig::BENCH_TEST_MODE=true；修改include/system_config.h的預設值後Build／Upload生效。
  遙控器必須連線、搖桿通道有效且未全車STOP；機構選關／NEXT／PREV不要求ODrive在線或state8。
  底盤自身仍維持四輪健康、置中500ms及15ms排程；缺ODrive時保持LOCKED、輪速0，不等於機構鎖住。
  每顆MKS開機8C／83／F3／92無有效ACK達10秒，記錄Bench warning並標記offline／unreferenced後略過。
  CAN無法排入亦於500ms略過，避免整條匯流排無電時卡死；不因這些開機缺回報觸發全車STOP。
  僅在線且已建立零點的軸執行FE；離線／未參考軸的STEP、PREV、HOME及回程略過、不等ACK。
  .active只表示正在運動，與新增.online分開，閒置但在線的馬達不會被誤判為離線。
  ID6在線時仍開機91尋零；若10秒沒有開始回報，先盡力停止該軸再略過；已開始後無完成仍真故障STOP。
  ACK0、運動拒絕、已在線運動失去回報、PWM初始化失敗及操作者STOP保持原保護。
  status新增bench_test_mode、motor_online_mask及motor_offline_mask，bit0／1／2為ID4／5／6；零點用position_known_mask。
  online是開機通訊確認，不是MKS心跳或持續線路偵測；offline需重啟重新檢查，遲到ACK不使軸自動加入。
  第二關無MKS，GPIO電動缸與Pin4／6／9仍照原HOME2與六段流程；GPIO／PWM沒有接線自動偵測。
  回到比賽互鎖時把BENCH_TEST_MODE改false並重新Build／Upload；主機回歸另外驗證true與false兩路。
  單關測試接線、等待與判讀見docs/bench_test.txt。

開啟與執行
  用 VS Code／PlatformIO 開啟本專案；支援 teensy40 與 teensy41。
  powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\compile.ps1 -Board teensy40
  powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\compile.ps1 -Board teensy41
  powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\start_web_gui.ps1
  GUI：http://127.0.0.1:8765。compile.ps1 只編譯；使用最新版須另外 Upload，GUI 啟動不更新韌體。
  可用關卡 1～5，第一關另有五種道具；YAML 變更預設後 Build／Upload，RAM 熱調試不需重新燒錄。

開機坐標與致動器
  啟用三顆MKS ID4、5、6、九顆PWM Pin2、3、4、5、6、9、10、11、12及XY-160D電動缸Pin16／17；ID7已移除。
  待機脈寬依序1150、2250、2000、2100、1400、1100 us；Pin3為第三關稻草卷夾爪。
  每次開機依ID4→5→6，各軸執行8C→83工作電流ACK1→F3；ID4/5=3000mA，ID6=5200mA。
  83設工作電流（面板Ma），ACK1才確認成功並保存；在該軸8C ACK之後、F3之前執行。
  一般模式三軸全部設定ACK後才92原位零ID4/5與91尋零ID6；電動缸開機LOW/LOW，不自動執行STEP。
  ID4/5不執行91尋原點；各發92 Set Current Axis zero，在不轉動下真正建立驅動器FE原位零點。
  各92成功ACK後才把該軸標記已知位置，不是只把軟體position欄位改為0。
  STARTUP_RETURN_ZERO=true：僅ID6發91尋原點，沿用驅動器既有HM設定。
  ID6收到開始01→完成02才完成；全接線時三軸坐標確認後Ready；檯面模式容許未接軸略過，不等待ID7。
  position_known_mask：bit0=ID4、bit1=ID5、bit2=ID6；三軸確認後為7。
  開機0；4/5原位零點確認後3；ID6尋零完成後為7。
  未確認的軸GUI顯示「尚未確認位置」；已確認仍只顯示最近完成目標，不是編碼器實測位置。
  手動HOME第三關以FE將4/5回0、Pin3回2250；使用開機原位，禁止對4/5發91。
  手動HOME第一／四關以91尋ID6並設Pin2=1150、Pin5=2100；HOME2電動缸縮回5000ms並設4/6/9=2000/1400/1100。
  Debug ID4/5/6皆可FE點動；GoHome91只允許ID6。電動缸另提供伸出／縮回／停止及毫秒設定。
  8C/83/F3/92每500ms非阻塞重試；每軸每種設定從首次成功排入CAN起固定10秒ACK上限。
  重試不延長期限；83 ACK0仍Fault／全車STOP，ACK2不視為保存成功，只有ACK1接F3；Bench純缺回報可略過。
  91/FE運動不因缺回覆而重送；92回覆依開機zero_queue/zero_wait診斷。
  91是尋原點、92是在目前位置建立零點、80才是校正；本版不自動發80。
  CAN1仍1Mbps；保存ID4/5=3000mA、ID6=5200mA；模式、HM與限位沿用既有配置。
  此83不是HM_Ma或HoldMa；完整分檔責任、封包與流程見docs/startup_current.txt。

USB 診斷（115200 baud）
  Monitor 或 GUI 擇一使用串口；只讀查詢一行：@1 status。
  本版 firmware_id="Robot_Integrated_Control"、firmware_revision="bench-test-v7"、startup_return_zero=true。
  motor_ids=[4,5,6]、servo_pins=[2,3,4,5,6,9,10,11,12]、stage3_enabled=true；另回報cylinder，無position7或current7_ma。
  position4/5與pulse3供第三關使用；請搭配三軸known mask與完成通知判斷。
  先看 phase=fault 是否已鎖存故障，再查 startup_phase、startup_node、startup_command、startup_attempts。
  startup_command 以十進位回報：140=8C、131=83（工作電流）、243=F3、145=91、146=92（當前軸建立零點）。
  83階段startup_phase=current_queue/current_wait；startup_current_ma為該83目標，Fault時需搭配phase判讀。
  current4_ma/current5_ma/current6_ma初始0，83 ACK1後為3000/3000/5200；不是實測電流。
  current_configured_mask位元依ID−4，全接線三軸依序確認為0→1→3→7，與position_known_mask分開。
  startup_current_settings=[{"node":4,"milliamps":3000},{"node":5,"milliamps":3000},{"node":6,"milliamps":5200}]。
  設定等待時startup_node是目前等ACK的ID；startup_home_ids=[6]，startup_hold_ids=[4,5]。
  詳細 ACK、Fault 與驅動器協定診斷見 docs/errors.txt；GUI 連線只查詢，不代替 Build／Upload。

模組與底盤
  main.cpp 為非阻塞總入口；CAN、底盤、RC、機構、USB gateway、全車安全分別成檔。
  CAN1 1Mbps：ODrive前輪ID10/11、MKS ID4/5/6（RX23/TX22）；電動缸使用GPIO，不在CAN清單。
  CAN2 1Mbps：ODrive 後輪 ID12/13（RX0/TX1）。
  Serial2 RX7/TX8，420000 baud CRSF；Serial USB 115200 baud。
  底盤每 15ms 發速度，機構等待與 PWM 緩移採非阻塞狀態機；USB 收發也有每圈上限。
  CH2前後輸入、CH3平移輸入、CH1自轉，CH4空閒；搖桿混控等比例縮放至每輪±90turns/s。
  四顆 ODrive 健康且搖桿置中 500ms 才解鎖；失聯、心跳逾時或錯誤立即鎖定。
  CH7 HIGH為第一關道具模式；LOW為一般模式；MID保持前一模式。
  一般模式CH10／11／12／13分別選第二／三／四／五關；第一關HIGH模式CH10～14改為五道具。
  CH6 NEXT、CH5 PREV，30ms 防彈跳；動作中不排隊，需完成後放開再按。
  CH9 只清除四顆 ODrive 錯誤，不解除全車 STOP 或 MKS Fault；CH11在一般模式選第三關。
  本版不配置 RC STOP 通道；使用者已有實體急停，未指定 GPIO，韌體不宣稱讀取該開關。
  PREV 恢復上一段完成狀態；回退使用 320 RPM/ACC0 與 300ms 舵機緩移，不逆向高速彈射。
  HOME、RUN、換關或套用 RAM 會重建回退起點；沒有歷史時 PREV 拒收。

各關底盤方向對照
  先按各自通道校正／死區將CH2與CH3化為前後輸入F、平移輸入S，再轉為車身方向；不交換原PWM值。
  關卡    車身前後    車身平移    CH1車身旋轉
  第一關     -S           F           不變
  第二關     -F          -S           不變
  第三關      F           S           不變
  第四／五關  F           S           不變
  第一關前→右、右→後、左→前；第二關前→後、右→左、左→右；依上表轉換平移，CH1仍維持車身旋轉。
  底盤每15ms發送輪速時讀MechanismController的實際stage；不使用遙控器選關或pending_stage作方向來源。
  return_wait期間仍用舊關方向，舊關復位完成、實際stage換成新關後，下一輪速發送才用新對照。
  切關不另等搖桿歸中；握桿未動時方向會依新關對照改變，既有底盤解鎖與斷線互鎖保留。
  CH1 維持車身旋轉分量，與關卡平移方向對照獨立。

舊關復位後切關與操作互鎖
  CH10／11／12／13或Web選關先安全復位舊關；第二關另自動HOME2，其餘新關直接Ready／STEP0。
  第一／四關離關：FE回ID6=0、Pin2=1150、Pin5=2100；第二關離關：縮回5000ms、Pin4/6/9=2000/1400/1100。
  第三關離關Park0：FE到ID4=0／ID5=0，320RPM／ACC0、Pin3=2250；雙軸完成才切任一目的關，不發91。
  復位範圍包含目前有效profile全部用過的致動器及標準關卡群組；自訂跨關馬達也FE回0，舵機回各自配置初始值。
  JobSwitchReturn期間phase=return_wait，stage仍是舊關、pending_stage是目的；完成屏障後新關ready。
  忙碌也可請求選關，僅保留最新目標；有電動缸則立即Stop並取消未送缸動作，其餘純MKS／伺服STEP等完整當段。
  有待切請求時，RUN不繼續後續STEP；NEXT／PREV不排隊，待新關Ready後放開再按。
  同關Ready重選不動；復位尚未開始時選回舊關可取消待切，已開始則完成復位後留原關STEP0。
  新關captureEntry擷取實際完成位置與PWM命令作PREV起點，不把未回位的獨立軸假設為0。
  第一／四關共享ID6與Pin2/5，前關復位後以該待機狀態建立新關基線。
  第三關再次切入不發動作：正常基線為上次Park0；第一次STEP1後PREV回本次實際進入快照。
  首次開機進第三關基線為0；離關Park0與手動HOME3均FE回0／Pin3=2250，兩者都禁止91。
  手動HOME維持home_wait；91只允許ID6開機／HOME／Debug及CH7 LOW退出；第二關HOME2為GPIO縮回。
  RAM commit含同關新表也先完成舊關復位／Park，再交換表、更新版本並Ready；交易中不能改選關。
  Bridge等待ready、新版本、runtime_plan、目標stage＋prop及非uploading才確認成功；return_wait／home_wait均未完成。
  Bridge最多60秒等待復位／Park、交換與第二關入關HOME2；plan.abort保留舊表並繼續已開始復位。

第一關：五道具獨立流程
  CH7 HIGH進入第一關Ready／STEP0；MID保持先前模式，LOW退出尋零並固定返回第四關。
  HIGH模式中CH10／11／12／13／14觸發竹籃／花瓶／竹筒／雞／小鳥，直接執行STEP1。
  每份stage1_<道具>.yaml含stage:1和prop:1..5，各自編輯、生成、RAM保留，不互相覆寫。
  竹籃stage1_basket.yaml：3段；第一段ID6=8000 RPM1000 ACC230、Pin5=2056、Pin2=1650；NEXT第二段Pin2緩移1000ms至2100後ID6=3000 RPM500 ACC150；第三段Pin2緩移1000ms至1300。
  花瓶stage1_vase.yaml：3段；第一段ID6=8000 RPM1000 ACC230，Pin5=2056、Pin2=1833；
    NEXT第二段Pin2=2111；再NEXT第三段ID6=1000 RPM1000 ACC245、Pin2=1389。
  竹筒stage1_tube.yaml：2段；第一段ID6=8000 RPM1000 ACC230，Pin5=2056、Pin2=1667；
    NEXT第二段ID6=0 RPM1000 ACC255。
  雞stage1_chicken.yaml：4段；第一段ID6=8300 RPM2000 ACC0、Pin5=1833、Pin2=1611；第二段Pin2=900；第三段ID6=2000 RPM320 ACC5；第四段Pin2=1667（105度放爪）。
  小鳥stage1_bird.yaml：4段；第一段ID6=7900 RPM1000 ACC230、Pin5=1944、Pin2=1611；第二段Pin2=1056；第三段ID6=4000 RPM1500 ACC255；第四段Pin2=1667（105度放爪）。
  動作中道具按鍵不排隊，須Ready後放開再按；同時觸發多個道具會拒收。
  最後一段完成後NEXT停止；重新觸發道具可從STEP1再開始，PREV仍可恢復上一段狀態。
  LOW等待目前整段完成，再ID6用91尋原點、Pin2=1150／Pin5=2100，完成後第四關Ready／STEP0。
  退出尋零中一般選關可更新最新目標；MID不新增動作，已接受的動作／尋零仍正常完成。
  切換HIGH／LOW時保持按下的共享通道不會被當成新模式命令，先放開再按。
  開機MID不選動作；初始LOW可辨識唯一保持HIGH的一般選關，HIGH入關不自動夾任何道具。
  舊stage1_steps.yaml只保留花瓶三段相容表；開機實際道具來源為五份stage1_<道具>.yaml。

第二關：XY-160D 電動缸與 stage2_steps.yaml
  第二關已移除MKS ID7；XY-160D MOTOR1接電動缸動力線，邏輯IN1=Pin17、IN2=Pin16，模組與Teensy共地。
  Extend輸出HIGH/LOW；Retract輸出LOW/HIGH；Stop輸出LOW/LOW。程式不輸出HIGH/HIGH。
  開機只初始化LOW/LOW，不自行伸縮；切入第二關先HOME2縮回5000ms及Pin4/6/9=2000/1400/1100。
  phase=home_wait、STEP0，計時結束後LOW/LOW並Ready，首次NEXT才開始STEP1。
  STEP1：僅Pin9=1800、Pin4=1300平行，電動缸保持。
  STEP2：平行Pin9=1000、Pin4=2100；STEP3：縮回3000ms。
  STEP4：Pin6緩移1000ms至550us、等待1000ms、再緩移1000ms至1400us。
  STEP5：伸出3000ms；STEP6：Pin4=2000／Pin9=1100平行緩移1000ms。
  repeat_from=2：首次1→2→3→4，後續2→3→4循環；RUN跑全表一次。
  HOME2及離第二關復位均安全縮回5000ms並恢復待機舵機；不發ID7 CAN或91。
  每次開始伸縮前保持LOW/LOW 50ms，運行秒數從實際輸出HIGH開始計算，定時完成自動Stop。
  切關立即取消當前／尚未派發的舊電動缸動作，先LOW/LOW，再依安全回程重新縮回。
  已送出的其他MKS與舵機運動仍等待完成屏障；不影響15ms底盤排程。
  電動缸無外部限位回讀：計時完成不等於Teensy已感測到位，內置微動開關由電動缸本身斷路。
  PREV僅以先前完成的伸／縮計時姿態重新運轉預設6000ms；未知或中斷姿態只Stop，不假設脈衝座標。
  新RAM commit先舊關復位再交換表；Stage2另完成入關HOME2，Bridge確認Ready後才回報成功。

第三關：現行 stage3_steps.yaml 是動作來源
  目前磁碟為4段、repeat_from=2，控制ID4/5與Pin3；各STEP位置、rpm、acc與脈寬以該檔為準。
  現場STEP調參與固定離關Park0分開；不以舊版座標或Pin3脈寬覆蓋使用者YAML。
  首輪1→2→3→4，之後2→3→4循環；每段等待雙軸開始／完成回報與伺服指令屏障。
  切往任何關先FE ID4=0、ID5=0，320RPM／ACC0、Pin3=2250；雙軸完成後才切目的Ready／STEP0。
  普通busy等完整當前STEP完成再回程，只保留最新目的；新關不HOME或自動STEP。
  初次開機入關基線0；正常再次入關為已完成的Park0，PREV以本次實際進入快照為準。
  手動HOME3仍FE回0/0與Pin3=2250；ID4/5在開機、切關或手動HOME皆不發91。

第四關：保留現場 stage4_steps.yaml 調整，6段、repeat_from=1
  本次未修改第四關YAML；以下為目前檔案快照，後續以YAML為準。
  STEP1：ID6=-700，320RPM/ACC0；Pin2=1500、Pin5=1400。
  STEP2：Pin2=1120；STEP3：ID6=8099，320RPM/ACC5，Pin5在2000ms緩移至1550。
  STEP4：ID6=-604，1500RPM/ACC0、Pin5=1400；完成後立即接續ID6=2600，320RPM/ACC0、Pin5=1800。
  STEP5：Pin2=1500；STEP6：ID6=0，320RPM/ACC0、Pin2=1150、Pin5=2100。
  STEP6與切出第一／四關均使用FE絕對0；進入第一／三／四／五關不尋零，第二關先縮回；HOME1/4仍91。

GUI 與 RAM
  電腦編輯關卡選單只載入檔案；Normal「控制器切關」才改啟用關卡。
  「立即套用」先舊關復位／Park，再交換新表；Stage2完成入關HOME2後Ready／STEP0。
  Debug可選ID4/5/6、九顆PWM及電動缸；Bench離線軸拒收，未參考軸僅在線ID6可GoHome建立零點。
  角度 0..180 映射 500..2500 us；拖滑桿只改欄位，按發送才輸出。
  儲存 YAML、套用 RAM、切關是分開操作；切關保留該關 RAM 設定，重啟恢復內建表。
  每關最多254 STEP、512 個展開後 Action；parallel 支援最多10個不同致動器，含等待屏障。
  全車STOP在任一模式或RAM傳輸均可發送；立即Pin16/17 LOW，停用ID4/5/6、取消緩移、detach六顆PWM。
  整合版同時將四顆 ODrive 歸零／IDLE；全車 STOP 鎖存至重啟。畫面保留 STOP 前指令，USB 中斷連線不會自動發 STOP。

BNO055 與航向補償已移除（2026-10-15）
  韌體不再初始化或讀取 BNO055，不再占用 SDA18/SCL19 進行 IMU 通訊。
  底盤只使用遙控器、關卡方向、ICR 與加減速設定；不會自動鎖定或補正航向。
  正常搖桿輪速上限保留 90 turns/s，防護上限保留 115 turns/s。
  15ms CAN 速度排程、心跳鎖定、斷線保護、全車 STOP、各關機構與 PWM 功能保留。
  status 不再提供 imu、yaw、pitch、roll 或 chassis.yaw_compensation；GUI 姿態卡已移除。
  請同步燒錄新韌體並重新啟動 Web GUI、重新整理網頁。

工具與驗證
  include/system_config.h：ID4/5/6、六PWM、Cylinder GPIO／計時、startup=true、stage3=true與三軸工作電流。
  src/mechanism_controller.cpp：CurrentQueue/CurrentWait與ACK屏障；src/serial_gateway.cpp：版本／電流診斷JSON。
  未修改驅動器模式、HM或鮑率；開機電流不放在關卡YAML，詳見docs/startup_current.txt。
  tools/generate_steps.py／platformio_steps.py：Build前產生九個profile與舊Stage1相容共十份標頭。
  tools/web_gui.py／web/：同源、token、Host 驗證，無 Node/CDN runtime 依賴。
  tools/python_runtime.ps1：處理複製後失效 venv，沿用同層 tmp 可攜 Python／套件。
  powershell -NoProfile -ExecutionPolicy Bypass -File .\test\run_tests.ps1
  test/test_generate_steps.py、test/test_web_gui.py、test/frontend_regression.mjs 覆蓋新腳位與第二關 RAM。
  模擬串口／DOM 測試與編譯不代表實體機構到位、CAN 電氣品質或瀏覽器視覺驗證。
  詳細見 docs/yaw_compensation.txt、startup_current.txt、yaml_steps.txt、web_gui.txt、errors.txt、verification.txt；.pio/logs/ 保存測試與兩板編譯。
  startup_current.txt記錄現行三軸83電流與92／91流程；cylinder_controller獨立處理GPIO與非阻塞計時。

第五關：stage5_steps.yaml
  備用流程預設delay0，不驅動任何機構；可用GUI編輯並套用，機構員新增STEP後Build固化。
  第一關五道具及第二～五關共九個獨立RAM設定，另有一個交易暫存bank，STOP互鎖照常。

電動缸通訊與安全
  @req debug.cylinder <0Stop|1Extend|2Retract> <duration_ms>；需Debug模式，Stop可中斷目前電動缸Debug運動。
  status.cylinder含action、active、duration_ms、remaining_ms、IN腳位／電平、reference_action／reference_known。
  reference只記錄已完成的定時計畫，沒有行程或微動開關感測，不是實測位置。
  USB／Web STOP與軟體Fault會立即LOW/LOW並鎖存全車STOP。既有實體急停未接MCU訊號，仍以外部斷電停機。
  模組控制真值表沿用本次指定接線；Teensy輸出停止不表示可量測H橋煞車力或內部限位。

Pin14呼吸PWM與方向修正（2026-10-15）
  電動缸邏輯IN1=Pin17、IN2=Pin16，Pin14專用於0→55→0的8-bit呼吸PWM，每5ms變動一階。
  PwmController以begin／poll非阻塞排程，與15ms底盤CAN及舵機／電動缸計時互不等待；STOP只停機構，呼吸仍繼續。
  這次僅交換程式邏輯IN1／IN2定義，保留現有16／17接線；Pin14不可接電動缸，本配置不啟用Wire1，IMU仍用18／19。
  詳見docs/pwm_breathing.txt；主機回歸不能代替實車速度與電壓量測。

電動缸方向語意（2026-10-15）
  僅交換程式腳位定義，不修改任何YAML action；邏輯IN1=17／IN2=16。保持目前實體接線，不再交換线材。
  Extend：Pin17 HIGH、Pin16 LOW；Retract：Pin17 LOW、Pin16 HIGH；Stop：兩腳LOW。
  HOME2依舊Retract5000ms；目前第二關STEP3 Retract3000ms、STEP5 Extend3000ms，均由原檔執行。
  Pin14亮度0→55→0，每5ms一階；8-bit解析度保持，理想週期550ms。詳見docs/cylinder_direction.txt。

Pin14呼吸燈GUI（2026-10-15）
  Normal／Debug共用面板可設定最高亮度0..255（0關閉）與每階間隔1..1000ms（越大越慢）。
  按套用即改RAM，恢復預設／重啟回55／5ms；初次需燒錄新版，後續調參免燒錄。
  動作中／開機中／STOP仍可調獨立燈；USB未連接、狀態過期或舊韌體缺pwm能力時停用。
  狀態回報pwm物件，命令為@req pwm.set <最高亮度> <每階ms>；參數驗證與USB STOP優先規則保留。
  YAML／電動缸17/16／六舵機／步進與底盤90+25≤115及15ms排程均保持。詳見docs/pwm_gui.txt。

切關即時方向與復位（2026-10-15）
  底盤優先pendingStage，在下一個15ms週期換方向；機構activeStage仍等待原有MKS安全屏障。
  CH7一般模式選關採最近穩定HIGH優先、釋放回退／唯一HIGH與暫時拒收重試，不重播已接受請求。
  自動復位僅在電缸已完成縮回且LOWLOW／inactive／參考有效時省略重複5000ms；手動HOME仍完整縮回。
  返回中新目標不重發MKS或重開缸計時；整體期限與原ACK逾時使故障明確STOP，不能偽造已到位。
  電動缸17/16維持digitalWrite，Pin14獨立PWM；YAML、輪速預算與互鎖保護保持。詳見docs/stage_switch.txt。

2026-10-15 底盤ICR／線性加減速
  加速90、減速180 turns/s²；輪距260×320mm；ICR預設(0,0)，X向前／Y向右。
  GUI新增RAM調校面板，停止且置中後可套用；永久值修改system_config.h。
  IMU補償共用ICR並維持25turns/s預算，實際輪速依漸變送出；操作及參數見docs/chassis_motion.txt。

左右轉 ICR 更新（2026-10-15）
  左轉 Y=-500mm、右轉 Y=+500mm、共用 X=0mm。GUI 提供兩側 Y 欄位，停止後套用 RAM。
  韌體與 GUI 需同步更新；完整參數／指令見 docs/chassis_motion.txt。

CH14 繞境獨立底盤設定（2026-10-15）
  CH7 LOW時，CH14選擇繞境參數：加速90／減速180、輪距260／320、X120、左Y+180／右Y-180。
  方向與CH10第二關相同(-F,-S)，不切機構關卡，也不自動發出機構回位／尋零指令。
  與CH10..13共用最近HIGH優先／釋放回退；CH7 HIGH時CH14仍是第一關夾小鳥。
  GUI標示目前參數庫，兩份RAM設定各自保留；寫入附模式編號，切換模式會清除未套用草稿。
  重新燒錄、重啟GUI並重新整理網頁後使用。完整參數／指令見docs/chassis_motion.txt。

新增 Debug 伺服 Pin10／11／12（2026-10-15）
  三顆新增舵機各自開機1500us；原六顆待機值1150/2250/2000/2100/1400/1100保留。
  重燒韌體、重啟Web GUI並重新整理，USB連線後等Ready，切換Debug即可分別調角度與duration_ms。
  角度0..180°對應500..2500us。滑桿編輯不送出，按各自發送鈕才執行。
  新腳位可選入YAML／RAM動作，但本次沒有加入任何關卡的自動動作。
  原CH14繞境、底盤加減速、電缸17/16與Pin14呼吸控制保留。

CH4 第一關 Pin2 即時微調（2026-10-15）
  CH4旋鈕1000..2000us映射±150us；1500us中心、1470..1530us死區。
  僅在第一關實際啟用且CH7第一關情境時生效，STEP執行／完成保持都可調。
  基準YAML／緩移／PREV快照不含偏移；最終輸出為基準+偏移並限在500..2500us。
  切關／CH7LOW退出／HOME／斷線／無效搖桿／STOP時撤除；HOME後等下一個第一關動作才恢復。
  Bench下不要求ODrive連線；無效CH4只歸零微調，不使底盤搖桿三軸失效。
  GUI「第一關 Pin2 微調」唯讀面板顯示CH4、要求偏移、實際偏移、基準與輸出指令值。
  include/system_config.h中SystemConfig::Rc::STAGE1_TRIM_*可調通道、中心、死區、最大偏移與輸出限制。
  重新燒錄，將CH4旋鈕置中，重啟GUI、重新整理頁面後使用。
  詳細規格與操作見docs/stage1_trim.txt。

電動缸上電保持STOP（2026-10-15）
  確認硬體仍為XY-160D電動缸；初始化本身只要求16／17兩路LOW，沒有自動伸出指令。
  Teensy4.x的startup_middle_hook在USB啟動等待前即preparePins()，setup最前面再確認LOW/LOW，
  機構begin仍保留相同初始化，避免單獨測試模組時遺漏。
  先清除實際GPIO輸出暫存器，再設OUTPUT，降低暖重啟保留舊HIGH造成短脈衝的可能。
  方向定義、HOME2縮回、deadtime、Pin14PWM、YAML及CH4微調不變。
  這是提前軟體接管；Teensy未供電／Bootloader／韌體尚未執行前不能由程式保證低電位，
  若仍上電瞬動，應確認控制線16與17各有硬體下拉且與驅動模組共地。
  詳細排查及來源見docs/cylinder_startup.txt。
