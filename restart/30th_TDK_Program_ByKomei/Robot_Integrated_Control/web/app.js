"use strict";
const $=id=>document.getElementById(id);
let token="",documentModel={steps:[]},savedHash="",savedSnapshot="",appliedSnapshot="",lastLog=0,state={},busy=false,view="control",pollRunning=false,profile=4;
let pwmDirty=false,pwmPending=null;
let motionDirty=false,motionPending=null,motionBank=null,motionBankVersion=0,motionBankNotice="";
const propNames={1:"竹籃",2:"花瓶",3:"竹筒",4:"雞",5:"小鳥"};
function profileIdentity(key){key=String(key);if(key==="1")key="1_2";return key.startsWith("1_")?{stage:1,prop:Number(key.slice(2))}:{stage:Number(key),prop:0};}
function editorStage(){return documentModel.stage??profileIdentity(profile).stage;}
function profileLabel(stage,prop=0){return stage===1?`第 1 關 · ${propNames[prop||2]??"道具"}`:`第 ${stage} 關`;}
function firmwareProp(f){return f?.stage===1?(f.prop||2):0;}
const stageChoices=[["1_1","第一關 · 夾竹籃 (CH10)"],["1_2","第一關 · 夾花瓶 (CH11)"],["1_3","第一關 · 夾竹筒 (CH12)"],["1_4","第一關 · 夾雞 (CH13)"],["1_5","第一關 · 夾小鳥 (CH14)"],["2","第二關 · 文蛤分級 (CH10)"],["3","第三關 · 稻草卷堆放 (CH11)"],["4","第四關 · 擲筊 (CH12)"],["5","第五關 · 備用關卡（Web／Serial）"],["6","第六關 · 取水果 (CH14)"]];
function stageAvailable(stage,f=state.firmware){
if(stage===3&&f?.stage3_enabled===false)return false;
return Array.isArray(f?.stage_ids)?f.stage_ids.includes(stage):stage!==3||f?.stage3_enabled===true;
}
function syncStageChoices(f){
const choices=stageChoices.filter(([key])=>{const item=profileIdentity(key);return stageAvailable(item.stage,f)&&(item.stage!==1||!Array.isArray(f?.stage1_prop_ids)||f.stage1_prop_ids.includes(item.prop));});
const key=choices.map(([value])=>value).join(",");
for(const id of ["profile","activeStage"]){const select=$(id);if(select.dataset.stageChoicesKey===key)continue;const old=select.value;select.replaceChildren(...choices.map(([value,label])=>new Option(label,value)));select.value=choices.some(([value])=>value===old)?old:choices.some(([value])=>value==="4")?"4":choices[0]?.[0]??"";select.dataset.stageChoicesKey=key;}
}
const motorChoices=[[6,"ID6 · 花瓶／擲筊升降"],[4,"ID4 · 共用左升降"],[5,"ID5 · 共用右升降"]];
const servoChoices=[[2,"Pin2 · 花瓶／筊杯夾持"],[5,"Pin5 · 花瓶／筊杯傾角"],[4,"Pin4 · 第二關成對伺服"],[6,"Pin6 · 第二關獨立伺服"],[9,"Pin9 · 第二關成對伺服"],[3,"Pin3 · 第二關／共用伺服"],[10,"Pin10 · 獨立調試伺服"],[11,"Pin11 · 獨立調試伺服"],[12,"Pin12 · 獨立調試伺服"]];
const servoPins=servoChoices.map(([pin])=>pin);
function servoSupported(f,pin){return Array.isArray(f?.servo_pins)?f.servo_pins.includes(pin):f?.["pin"+pin]===pin;}
const defaults={stepper:()=>({node:editorStage()===3?4:6,position:0,rpm:320,acc:5}),servo:()=>({pin:editorStage()===3?3:editorStage()===2?4:2,pulse_us:1500,duration_ms:1000}),delay:()=>({ms:1000}),cylinder:()=>({action:"extend",duration_ms:6000}),parallel:()=>editorStage()===3?[{stepper:{node:4,position:0,rpm:320,acc:0}},{stepper:{node:5,position:0,rpm:320,acc:0}},{servo:{pin:3,pulse_us:2250}}]:editorStage()===2?[{cylinder:{action:"extend",duration_ms:6000}},{servo:{pin:9,pulse_us:1800}},{servo:{pin:4,pulse_us:1300}}]:[{servo:{pin:2,pulse_us:1500}},{servo:{pin:5,pulse_us:1400}}]};
const clone=value=>JSON.parse(JSON.stringify(value));
function notice(text,error=false){$("message").textContent=text;$("message").className="message"+(error?" error":"");}
async function api(path,payload){const response=await fetch(path,{method:payload===undefined?"GET":"POST",headers:payload===undefined?{}:{"Content-Type":"application/json","X-Stage4-Token":token},body:payload===undefined?undefined:JSON.stringify(payload)});const data=await response.json();if(!response.ok)throw Error(data.error||"請求失敗");return data;}
async function perform(fn){if(busy)return;busy=true;renderState();try{await fn();}catch(error){notice(error.message,true);}finally{busy=false;await poll();renderState();}}
function integer(id){const element=$(id);if(!element.reportValidity())throw Error("請檢查輸入欄位");return Number(element.value);}
function calibration(){return {min:state.firmware?.servo_min_us??500,max:state.firmware?.servo_max_us??2500};}
function angleToPulse(angle){const c=calibration();return Math.round(c.min+angle*(c.max-c.min)/180);}
function pulseToAngle(pulse){const c=calibration();return Math.round((pulse-c.min)*1800/(c.max-c.min))/10;}
async function send(command,extra={}){if(command==="stage"&&!stageAvailable(extra.stage))throw Error("此關卡已停用，請選擇其他關卡。");await api("/api/command",{command,...extra});notice(command==="stage"?"選關請求已接受；更換關卡時先復位目前關卡，完成後切入目標關 Ready（STEP0）。":command==="stop"?"全車急停已接受；底盤與機構停用，需重啟 Teensy 才能繼續。":"命令已接受；請查看控制器狀態與紀錄。");}
function appendLogs(logs){const box=$("logs");if(logs.length&&box.querySelector(".log-empty"))box.replaceChildren();for(const entry of logs){lastLog=Math.max(lastLog,entry.id);const line=document.createElement("div");line.className="log-line"+(entry.direction==="tx"?" log-tx":"");const time=document.createElement("span");time.className="log-time";time.textContent=entry.time;const text=document.createElement("span");text.textContent=(entry.direction==="tx"?"→ ":"")+entry.text;line.append(time,text);box.append(line);}while(box.children.length>400)box.firstChild.remove();if(logs.length)box.scrollTop=box.scrollHeight;}
// Online masks confirm this boot's startup; they are not live MKS heartbeats.
function motorAvailability(f,node){
const bit=[4,5,6].includes(node)?1<<(node-4):0;
const recovery=Number.isInteger(f?.motor_recovery_mask)&&!!(f.motor_recovery_mask&bit);
const reported=Number.isInteger(f?.motor_online_mask)&&Number.isInteger(f?.motor_offline_mask);
const online=reported&&!!(f.motor_online_mask&bit),offline=reported&&!!(f.motor_offline_mask&bit),known=Number.isInteger(f?.position_known_mask)&&!!(f.position_known_mask&bit);
return {reported,online,offline,known,recovery,blocked:f?.bench_test_mode===true&&reported&&!online,unreferenced:f?.bench_test_mode===true&&reported&&online&&Number.isInteger(f?.position_known_mask)&&!known};
}
function renderBench(f,fresh){
const bench=f?.bench_test_mode;
$("benchState").textContent=!fresh?"尚未取得即時狀態":bench===true?"BENCH / 檯面獨立測試":bench===false?"COMPETITION / 比賽互鎖":"模式未回報";
$("benchState").className="badge"+(fresh&&bench===true?" online":"");
$("benchHint").textContent=!fresh?"連線後顯示韌體測試模式與開機馬達連線狀態。":bench===true?"底盤可保持 LOCKED；遙控器連線且未急停時仍可操作機構。開機離線步進略過；已連線機構的動作故障仍會急停。":bench===false?"機構遙控操作需底盤 RUNNING、四輪正常且閉迴路；Web 操作維持原有機構互鎖。":"韌體未回報測試模式；保留既有操作相容性。";
$("motorAvailability").textContent=!fresh?"MKS 開機狀態尚未取得":(f.motor_ids??[4,5,6]).map(node=>{const m=motorAvailability(f,node);return `ID${node}：${!m.reported?"連線狀態未回報":m.recovery?"局部隔離／需手動 HOME":m.offline?"離線／未建立零點":!m.online?"尚未完成開機通訊":m.known?"開機已確認／零點已建立":"開機已確認／尚未建立零點"}`;}).join(" · ")+"（開機確認，非即時心跳）";
}
function id6PositionMax(f=state.firmware){return Number.isFinite(f?.id6_position_max)?f.id6_position_max:10500;}
function fruitPositionLimits(f){
return {min:Number.isFinite(f?.fruit_position_min)?f.fruit_position_min:-1400,max:id6PositionMax(f)};
}
function stepperPositionLimits(node,stage,f=state.firmware){
return {min:node===6&&stage===6?fruitPositionLimits(f).min:-8388608,max:node===6?id6PositionMax(f):8388607};
}
function renderMechanismRecovery(f,fresh){
const mask=Number.isInteger(f?.motor_recovery_mask)?f.motor_recovery_mask:0;
const nodes=[4,5,6].filter(node=>mask&(1<<(node-4))),warning=typeof f?.mechanism_warning==="string"?f.mechanism_warning:"";
const visible=!!fresh&&(nodes.length>0||!!warning),stopped=!!f?.stopped||f?.phase==="fault";
$("mechanismRecovery").classList.toggle("hidden",!visible);
$("mechanismRecoveryState").textContent=!visible?"尚無局部隔離警告":stopped?"全域 STOP／無法以 HOME 恢復":nodes.length?nodes.map(node=>"ID"+node).join("、")+" 已局部隔離":"機構警告";
$("mechanismRecoveryHint").textContent=!visible?"":stopped?"全域 STOP 仍鎖定全部機構；請處理急停原因後重啟 Teensy，切關與 HOME 都不能解除。":
(warning?warning+"。":"")+(nodes.includes(6)?(stageAvailable(3,f)?"可切換第二／第三關繼續；":"可切換第二關繼續；")+"第一／第四／第六關共用 ID6，NEXT／RUN／PREV 暫停。先確認卡點已排除，再在共用 ID6 關卡按 HOME；收到尋零開始與完成回報後才解除隔離。STOP 未成功送出前不接受新運動。":"隔離軸恢復前禁止新運動；可切換不使用該軸的關卡。確認機構後使用手動 HOME 恢復。");
}
function renderState(){const f=state.firmware;syncStageChoices(f);const connected=!!state.connected,fresh=connected&&f&&state.status_age!==null&&state.status_age<4,stopped=!!f?.stopped||f?.phase==="fault",recovery=!!f?.stage_recovery_required,ready=fresh&&!stopped&&f.phase==="ready"&&!f.uploading;
$("connectionBadge").textContent=connected?state.port:"未連線";$("connectionBadge").className="badge"+(connected?" online":"");$("connectionDot").className="dot"+(connected?" connected":"");$("sidebarConnection").textContent=connected?`${state.port} 已連線`:"USB 尚未連接";
$("connect").disabled=busy||connected||!$("port").value;$("disconnect").disabled=busy||!connected;$("scan").disabled=busy;$("globalStop").disabled=!connected;
const phases={ready:"Ready / 閒置",return_wait:"目前關卡復位中",home_wait:"HOME / 歸零待機",busy:"執行中",booting:"初始化中",fault:"已停止 / Fault"};$("phase").textContent=f?phases[f.phase]||f.phase:"—";$("phaseDetail").textContent=state.error||(!connected?"選擇串口後開始連線":!fresh?"狀態未更新；確認韌體與 USB 連線":f.phase==="fault"?"請確認機構後重啟 Teensy":f.phase==="return_wait"?`${profileLabel(f.stage??4,firmwareProp(f))}復位中${f.pending_stage?" → "+profileLabel(f.pending_stage,f.pending_prop):""}`:f.uploading?"正在接收設定，完成後才切換":f.phase==="booting"?"開機電缸安全縮回與 MKS 初始化並行，完成後進入 Ready":f.phase==="home_wait"?(f.stage===2?"第二關電缸回縮待機，等待計時完成":`第 ${f.stage??4} 關歸零待機，等待 Ready`):`模式：${f.mode.toUpperCase()}`);
$("currentStep").textContent=f?.phase==="return_wait"?"STEP0 / 復位":f?.phase==="home_wait"?"STEP0 / HOME":f?.step?`STEP${f.step}${f.branch?"-"+f.branch:""}`:f?.mode==="debug"?"DEBUG":f?"STEP0":"—";$("stepDetail").textContent=f?`控制器${profileLabel(f.stage??4,firmwareProp(f))} · 已完成 ${f.completed} / ${f.steps} 段`:"STEP 自動編號";$("planSource").textContent=f?.runtime_plan?`RAM · v${f.plan_version}`:"韌體內建";$("planDetail").textContent=f?.phase==="return_wait"?`${profileLabel(f.stage??4,firmwareProp(f))}復位中${f.pending_stage?" → "+profileLabel(f.pending_stage,f.pending_prop):"，等待 Ready"}`:f?.pending_stage?`${f.phase==="home_wait"?(f.stage===1?"第一關尋零完成後切關":"手動HOME完成後再復位切關"):"本段完成後再復位切關"}，目標${profileLabel(f.pending_stage,f.pending_prop)}`:f?.runtime_plan?"重新啟動後恢復韌體預設":"切關先復位目前關卡，目標關待機 STEP0";
const debug=f?.mode==="debug";$("normalMode").classList.toggle("selected",!debug);$("debugMode").classList.toggle("selected",debug);$("normalMode").disabled=busy||!ready||!debug;$("debugMode").disabled=busy||!ready||debug;$("normalPanel").classList.toggle("hidden",debug);$("debugPanel").classList.toggle("hidden",!debug);$("modeHint").textContent=debug?"Debug：三顆步進 FE 點動、九顆伺服與電缸；ID6 可尋原點。":`Normal：使用控制器第 ${f?.stage??4} 關的有效 STEP 設定。`;
document.querySelectorAll("[data-command]").forEach(button=>button.disabled=busy||!ready||debug||(recovery&&button.dataset.command!=="home")||((button.dataset.command==="prev"&&!f?.previous_available)));
const selected=profileIdentity($("activeStage").value||4);
$("propControls").classList.toggle("hidden",f?.stage!==1);$("propHint").classList.toggle("hidden",f?.stage!==1);
$("triggerProp").disabled=busy||!ready||debug||recovery||f?.stage!==1||selected.stage!==1||!(f?.stage1_prop_ids??[2]).includes(selected.prop);
$("exitStage1").disabled=busy||!fresh||stopped||debug||f?.stage!==1||f?.uploading||!["ready","busy","return_wait","home_wait"].includes(f?.phase);
$("triggerProp").textContent=selected.stage===1?`觸發${propNames[selected.prop]} / STEP1`:"先選第一關道具";
$("propHint").textContent=f?.prop_complete?"道具流程已完成；按 NEXT 回 HOME，歸位後再按 NEXT 從 STEP1 開始。":"選關只到 STEP0；觸發執行 STEP1，NEXT 接續，末段後 NEXT 歸位，再按開始新一輪。";
$("selectStage").disabled=busy||!fresh||stopped||debug||f?.uploading||!["ready","busy","return_wait","home_wait"].includes(f?.phase);
if($("chassisState"))renderChassis(f,fresh);
renderCylinder(f,fresh);renderBench(f,fresh);renderMechanismRecovery(f,fresh);renderPwm(f,fresh);renderMotion(f,fresh);renderStage1Trim(f,fresh);renderStage2Trim(f,fresh);renderStage4Trim(f,fresh);
const actuatorBlocked=busy||!fresh||stopped||!debug||f?.phase==="fault"||f?.phase==="booting"||f?.phase==="home_wait"||f?.phase==="return_wait"||f?.uploading;
$("cylinderDuration").disabled=actuatorBlocked;const cylinderReady=ready&&debug&&!!f?.cylinder?.available;
$("extendCylinder").disabled=busy||!cylinderReady;$("retractCylinder").disabled=busy||!cylinderReady;
$("stopCylinder").disabled=!fresh||stopped||!debug||!f?.cylinder?.available||f?.phase==="fault";
$("cylinderSeconds").textContent=`${$("cylinderDuration").value} ms = ${Number($("cylinderDuration").value)/1000} 秒`;
for(const id of ["motorNode","motorPosition","motorRpm","motorAcc","sendMotor","motorZero","motorGoHome"])$(id).disabled=actuatorBlocked;
for(const pin of servoPins){const supported=servoSupported(f,pin);for(const prefix of ["servoSlider","servoAngle","servoDuration","sendServo"]){const control=$(prefix+pin);if(control)control.disabled=actuatorBlocked||!supported;}const v=$("servoValue"+pin),pulse=f?.["pulse"+pin];if(v)v.textContent=fresh&&!supported?"韌體未支援 Pin "+pin+"；請重新燒錄新版韌體":pulse!==undefined?((f.stopped||f.phase==="fault")?"STOP 前指令":"目前輸出")+" "+pulse+" µs / 約 "+pulseToAngle(pulse)+"°（指令值）":"沒有角度回授";}
const node=Number($("motorNode").value||6),motor=motorAvailability(f,node);
for(const id of ["motorPosition","motorRpm","motorAcc","sendMotor","motorZero"])$(id).disabled=actuatorBlocked||motor.blocked||motor.unreferenced||motor.recovery;
$("motorGoHome").disabled=actuatorBlocked||node!==6||motor.blocked;
const fruitLimits=fruitPositionLimits(f),motorLimits=stepperPositionLimits(node,f?.stage,f);$("motorPosition").min=String(motorLimits.min);$("motorPosition").max=String(motorLimits.max);
$("motorHint").textContent=motor.recovery?`ID${node} 已隔離；先排除卡點，再按 GoHome 尋原點。開始與完成回報都確認後，才恢復 FE 運動。可切到未使用此軸的關卡。`:motor.blocked?(motor.offline?`ID${node} 在本次開機離線，Bench 已略過此軸；接線後重啟以重新初始化。`:`ID${node} 尚未完成開機通訊，請等待初始化完成。`):motor.unreferenced?`ID${node} 已連線但尚未建立零點，FE 點動停用。${node===6?"可按 GoHome 重新尋原點。":"請確認開機原位零點設定後重啟。"}`:[4,5].includes(node)?"ID4/5 在開機原位建立 FE 零點；只用絕對位置，GoHome 91 停用。":"ID6 可用 GoHome 91，沿用驅動器既有尋原點設定。";
if(node===6)$("motorHint").textContent+=" ID6 各關共用絕對位置上限 "+id6PositionMax(f)+" 脈衝，適用正式 STEP、CH4、Debug 與 RAM。";
if(f?.stage===6&&node===6)$("motorHint").textContent+=" 水果 ID6 位置限於 "+fruitLimits.min+"～"+fruitLimits.max+" 脈衝；正式 STEP、Debug 與 RAM 目標同樣限制。";
const position=f?.["position"+node];$("motorId").textContent=node;$("motorValue").textContent=motor.reported&&motor.offline?`ID${node} 離線／未建立零點（本次開機略過）`:motor.reported&&!motor.online?`ID${node} 開機通訊尚未確認`:position!==undefined&&motor.known?`最近完成目標 ${position} 脈衝（非位置感測值）`:"尚未確認位置";$("profile").disabled=busy;$("activeStage").disabled=busy;$("repeatFrom").disabled=busy||documentModel.stage===1;$("applyConfig").disabled=busy||!ready||debug||recovery||!stageAvailable(editorStage(),f);$("saveConfig").disabled=busy;$("reloadConfig").disabled=busy;$("addStep").disabled=busy;$("editState").textContent=JSON.stringify(documentModel)===savedSnapshot?"與電腦 YAML 一致":"有未儲存的修改";$("appliedState").textContent=appliedSnapshot===JSON.stringify(documentModel)&&state.applied_hash?"此份編輯已套用至 RAM":"編輯內容未確認套用；儲存 YAML 不會自動套用";
}
async function poll(){if(pollRunning)return;pollRunning=true;try{if(state.connected&&!busy)try{await api("/api/refresh",{});}catch(error){notice(error.message,true);}const incoming=await api(`/api/state?after=${lastLog}`);state=incoming;appendLogs(incoming.logs);renderState();}catch(error){notice("閘道服務未回應："+error.message,true);state={};renderState();}finally{pollRunning=false;}}
async function scan(){const data=await api("/api/ports");const old=$("port").value;$("port").replaceChildren(new Option("選擇 COM Port",""));for(const p of data.ports)$("port").add(new Option(`${p.device} · ${p.description}`,p.device));if(data.ports.some(p=>p.device===old))$("port").value=old;else if(data.ports.length===1)$("port").value=data.ports[0].device;renderState();}
function servoCards(){
  for(const [pin,title] of servoChoices){
    const card=document.createElement("form");card.className="panel debug-card servo-card";
    card.innerHTML=`<div class="section-heading"><div><span class="eyebrow">PWM / PIN ${pin}</span><h2>${title}</h2></div></div><label>獨立角度滑桿<input id="servoSlider${pin}" type="range" min="0" max="180" step="0.1" value="90" aria-label="Pin ${pin} 目標角度"></label><div class="field-grid"><label>角度 <span>0–180°</span><input id="servoAngle${pin}" type="number" min="0" max="180" step="0.1" value="90" required></label><label>緩移時間 <span>duration_ms</span><input id="servoDuration${pin}" type="number" min="0" max="60000" value="1000" required></label></div><button class="primary wide" id="sendServo${pin}" type="submit">發送 Pin ${pin}</button><p id="servoValue${pin}" class="servo-value">沒有角度回授</p>`;
    card.addEventListener("submit",event=>{event.preventDefault();if(!servoSupported(state.firmware,pin)){notice("韌體未支援 Pin "+pin+"；請重新燒錄新版韌體",true);return;}perform(()=>send("debug.servo",{pin,angle:integer("servoAngle"+pin),duration_ms:integer("servoDuration"+pin)}));});
    $("servoCards").append(card);
    $("servoSlider"+pin).addEventListener("input",()=>$("servoAngle"+pin).value=$("servoSlider"+pin).value);
    $("servoAngle"+pin).addEventListener("input",()=>$("servoSlider"+pin).value=$("servoAngle"+pin).value);
  }
}
const motionFields=[["ramp_enabled","motionEnabled",0,1],["accel","motionAccel",1,10000],["decel","motionDecel",1,10000],["wheelbase_mm","motionWheelbase",1,5000],["track_mm","motionTrack",1,5000],["icr_x_mm","motionX",-5000,5000],["icr_left_y_mm","motionLeftY",-5000,5000],["icr_right_y_mm","motionRightY",-5000,5000]];
function motionConfiguration(f=state.firmware){
  const m=f?.chassis?.motion;
  return m?.available===true&&m.icr_mode==="directional"&&m.profiles===true&&Number.isInteger(m.profile_id)&&[0,1].includes(m.profile_id)&&typeof m.ramp_enabled==="boolean"&&motionFields.slice(1).every(([key,,low,high])=>Number.isFinite(m[key])&&m[key]>=low&&m[key]<=high)?m:null;
}
function motionProfileLabel(id){return id===1?"舊版繞境":"一般底盤";}
function motionEditable(){return !busy&&!!state.connected&&Number.isFinite(state.status_age)&&state.status_age<4&&motionConfiguration()?.editable===true&&!state.firmware?.stopped&&!state.firmware?.chassis?.emergency;}
function motionDraft(){return Object.fromEntries(motionFields.map(([key,id])=>[key,$(id).value.trim()===""?NaN:Number($(id).value)]));}
function renderMotion(f,fresh){
  const m=motionConfiguration(f),available=!!fresh&&!!m,blocked=!motionEditable();
  if(available&&motionBank!==m.profile_id){
    motionBankNotice=motionBank!==null&&(motionDirty||motionPending)?"已切換底盤模式，上一模式未確認的編輯已清除。":"";
    motionBank=m.profile_id;++motionBankVersion;motionDirty=false;motionPending=null;
  }
  if(available&&motionPending?.profile_id===m.profile_id&&motionFields.every(([key])=>Number(m[key])===motionPending[key])){motionPending=null;motionDirty=false;}
  if(available&&!motionDirty&&!motionPending)for(const [key,id] of motionFields)$(id).value=String(key==="ramp_enabled"?Number(m[key]):m[key]);
  for(const [,id] of motionFields)$(id).disabled=busy||!available;
  $("motionApply").disabled=blocked;$("motionCenter").disabled=busy||!available;
  $("motionProfile").textContent=available?motionProfileLabel(m.profile_id):"模式尚未取得";
  $("motionApply").textContent=available?"套用"+motionProfileLabel(m.profile_id)+" / RAM":"套用底盤設定 / RAM";
  $("motionState").textContent=!available?"尚未取得支援的即時狀態":m.editable?"可套用":"等待底盤停止";
  $("motionCurrent").textContent=available?motionProfileLabel(m.profile_id)+"：加速 "+m.accel+"、減速 "+m.decel+" turns/s² · 輪距 "+m.wheelbase_mm+" × "+m.track_mm+" mm · 左轉 ICR ("+m.icr_x_mm+", "+m.icr_left_y_mm+")／右轉 ("+m.icr_x_mm+", "+m.icr_right_y_mm+") mm · "+(m.ramp_enabled?"漸變啟用":"直接輪速"):"尚未取得底盤設定";
  $("motionHint").textContent=!available?"請連接 USB 並燒錄支援底盤調校的韌體。":motionPending?"指令已接受，等待回報確認。":motionDirty?"有未套用的修改；先置中並等輪速降至 0，再按套用。":motionBankNotice||"套用只更新目前模式的 RAM；一般與繞境設定獨立，重啟恢復 system_config.h 預設。";
}
async function applyMotion(){
  if(!motionEditable())return;
  const profileId=motionConfiguration().profile_id,bankVersion=motionBankVersion;
  await perform(async()=>{
    if(motionConfiguration()?.profile_id!==profileId||motionBankVersion!==bankVersion)throw Error("底盤模式已切換，請重新確認設定");
    const settings={...motionDraft(),profile_id:profileId};
    if(!motionFields.every(([key,,low,high])=>Number.isInteger(settings[key])&&settings[key]>=low&&settings[key]<=high))throw Error("底盤設定需填入範圍內整數");
    motionDirty=true;motionPending=null;motionBankNotice="";
    const response=await api("/api/command",{command:"chassis.motion",...settings});
    if(motionConfiguration()?.profile_id!==profileId||motionBankVersion!==bankVersion){
      notice("已送出"+motionProfileLabel(profileId)+"設定；目前模式已變更，請查看目前模式回報。");return;
    }
    motionPending=settings;
    if(response.result?.chassis?.motion?.profile_id===profileId&&state.firmware?.chassis)state.firmware.chassis.motion=response.result.chassis.motion;
    notice(motionProfileLabel(profileId)+"加減速與 ICR 已套用至 RAM。");
  });
}
function pwmConfiguration(f=state.firmware){
const p=f?.pwm,integerRange=(value,low,high)=>Number.isInteger(value)&&value>=low&&value<=high;
return p?.available===true&&p.pin===14&&integerRange(p.value,0,255)&&integerRange(p.max_value,0,255)&&integerRange(p.step_interval_ms,1,1000)&&integerRange(p.default_max_value,0,255)&&integerRange(p.default_step_interval_ms,1,1000)&&p.min_step_interval_ms===1&&p.max_step_interval_ms===1000?p:null;
}
function pwmEditable(){return !busy&&!!state.connected&&state.status_age!==null&&Number.isFinite(state.status_age)&&state.status_age<4&&!!pwmConfiguration();}
function pwmDraft(){const number=id=>$(id).value.trim()===""?NaN:Number($(id).value);return {max_value:number("pwmMaxValue"),step_interval_ms:number("pwmStepInterval")};}
function setPwmInputs(maximum,interval){$("pwmMaxSlider").value=$("pwmMaxValue").value=String(maximum);$("pwmIntervalSlider").value=$("pwmStepInterval").value=String(interval);}
function renderPwm(f,fresh){
const p=pwmConfiguration(f),available=!!fresh&&!!p,blocked=busy||!available;
if(available&&pwmPending&&p.max_value===pwmPending.max_value&&p.step_interval_ms===pwmPending.step_interval_ms){pwmPending=null;pwmDirty=false;}
if(available&&!pwmDirty&&!pwmPending)setPwmInputs(p.max_value,p.step_interval_ms);
for(const id of ["pwmMaxSlider","pwmMaxValue","pwmIntervalSlider","pwmStepInterval","pwmApply","pwmReset"])$(id).disabled=blocked;
const strobe=available&&p.strobe_enabled===true,rcControlled=available&&p.rc_controlled===true;
const strobeInterval=Number.isInteger(p?.strobe_interval_ms)&&p.strobe_interval_ms>0?p.strobe_interval_ms:50;
const strobeValue=Number.isInteger(p?.strobe_value)&&p.strobe_value>=0&&p.strobe_value<=255?p.strobe_value:255;
const rcChannel=Number.isInteger(p?.rc_channel)?p.rc_channel:13;
$("pwmState").textContent=!fresh?"尚未取得即時狀態":!p?"需更新韌體":strobe?`CH${rcChannel} / 爆閃中`:rcControlled?`CH${rcChannel} / 關燈`:p.max_value===0?"關燈":"PWM / 呼吸中";
$("pwmApply").textContent=strobe?"保存呼吸設定 / RAM":"套用燈光設定 / RAM";
$("pwmState").className="badge"+(available?" online":"");
$("pwmCurrentValue").textContent=available?`${p.value} / 255`:"--";
$("pwmCurrentConfig").textContent=!available?"尚未取得目前設定":strobe?`CH${rcChannel} 爆閃：0 ↔ ${strobeValue} · 每 ${strobeInterval} ms 切換 · 呼吸備用：峰值 ${p.max_value} / 255、每階 ${p.step_interval_ms} ms`:rcControlled?`CH${rcChannel} 關燈 · 呼吸備用：峰值 ${p.max_value} / 255、每階 ${p.step_interval_ms} ms`:`目前峰值 ${p.max_value} / 255 · 每階 ${p.step_interval_ms} ms · 呼吸預設 ${p.default_max_value} / ${p.default_step_interval_ms} ms`;
const draft=pwmDraft(),valid=Number.isInteger(draft.max_value)&&draft.max_value>=0&&draft.max_value<=255&&Number.isInteger(draft.step_interval_ms)&&draft.step_interval_ms>=1&&draft.step_interval_ms<=1000;
$("pwmCycle").textContent=strobe?`爆閃週期：${(2*strobeInterval/1000).toFixed(3)} 秒（亮 ${strobeInterval} ms／滅 ${strobeInterval} ms）`:!valid?"請輸入整數：亮度 0～255、每階間隔 1～1000 ms。":draft.max_value===0?"峰值為 0：套用後關燈。":`預估完整呼吸週期：${(2*draft.max_value*draft.step_interval_ms/1000).toFixed(3)} 秒（升亮＋漸暗）`;
$("pwmHint").textContent=!state.connected?"請連接 USB；第一次使用需先燒錄支援 PWM 調校的韌體。":!fresh?"狀態未更新，燈光調整暫停；請確認 USB 連線。":!p?"韌體尚未支援完整的 Pin14 PWM 調校；請先燒錄新版韌體。":strobe?`CH${rcChannel} 爆閃優先；套用只保存手動呼吸參數，不改變爆閃。CH${rcChannel} 關閉、失聯或切入第一關模式時立即關燈。`+(pwmPending?" 等待呼吸設定回報確認。":pwmDirty?" 有未套用的呼吸修改。":""):pwmPending?"設定指令已接受，等待控制器確認；輸入值暫時保留。":pwmDirty?"有未套用的燈光修改；滑桿只編輯，按「套用」才更新 RAM。":rcControlled?`CH${rcChannel} 已關燈；按套用可切為手動呼吸。下次 CH${rcChannel} 撥動、失聯或離開一般模式時遙控重新接管；STOP 後皆可調整。`:"燈光獨立於機構，Normal／Debug、動作中或 STOP 後皆可調整；開機先關燈，CH13 在一般模式控制爆閃。";
}
function editPwm(source,target){$(target).value=$(source).value;pwmDirty=true;pwmPending=null;renderState();}
async function applyPwm(reset=false){
if(!pwmEditable())return;
await perform(async()=>{
const p=pwmConfiguration(),settings=reset?{max_value:p.default_max_value,step_interval_ms:p.default_step_interval_ms}:pwmDraft();
if(!reset&&( !$("pwmMaxValue").reportValidity()||!$("pwmStepInterval").reportValidity()))throw Error("請檢查燈光調整欄位");
if(!Number.isInteger(settings.max_value)||settings.max_value<0||settings.max_value>255||!Number.isInteger(settings.step_interval_ms)||settings.step_interval_ms<1||settings.step_interval_ms>1000)throw Error("亮度與每階間隔必須為範圍內整數");
if(reset)setPwmInputs(settings.max_value,settings.step_interval_ms);
pwmDirty=true;pwmPending=null;
const response=await api("/api/command",{command:"pwm.set",...settings});
pwmPending=settings;
if(response.result?.pwm&&state.firmware)state.firmware.pwm=response.result.pwm;
renderState();
notice(pwmConfiguration()?.strobe_enabled===true?"呼吸參數已保存至 RAM；CH13 爆閃持續，亮度與閃速維持遙控設定。":reset?"已套用呼吸預設至 RAM；重啟先關燈。":"燈光設定已套用至 RAM；重啟先關燈，CH13 仍可接管爆閃。");
});
}
function renderCylinder(f,fresh){
const c=f?.cylinder,available=fresh&&c?.available;
$("cylinderState").textContent=!available?"尚未取得電缸狀態":({0:"停止",1:"伸出",2:"縮回"}[c.action]??"未知");
const output=value=>value===true?"HIGH":value===false?"LOW":"--";
$("cylinderDetail").textContent=available?`IN1 / GPIO${c.in1_pin}: ${output(c.in1_high)} · IN2 / GPIO${c.in2_pin}: ${output(c.in2_high)}`:"GPIO 狀態尚未取得";
const remaining=typeof c?.remaining_ms==="number"&&Number.isFinite(c.remaining_ms)?(c.remaining_ms/1000).toFixed(3)+" 秒":"--";
const fullHome=c?.fully_retracted===true?"完整縮回計時已完成":c?.fully_retracted===false?"完整縮回計時尚未完成":"韌體未回報完整縮回計時";
const reference=c?.reference_known?({1:"估計伸出參考位",2:"估計縮回參考位"}[c.reference_action]??"參考狀態未知"):"參考位未確認";
$("cylinderTiming").textContent=available?`計時剩餘 ${remaining} · ${fullHome} · ${reference}；時間估計，無到位回授。`:"時間只作行程估計，沒有到位微動開關回授。";
}
// Read-only telemetry: the applied delta may be smaller than the knob request at a PWM limit.
function renderStage1Trim(f,fresh){
  const supported=typeof f?.stage1_trim_allowed==="boolean"&&
    [f?.pulse2_base,f?.pulse2,f?.stage1_trim_us,f?.rc?.stage1_trim_us].every(Number.isFinite);
  const available=!!fresh&&supported,stopped=!!f?.stopped||f?.phase==="fault";
  const enabled=available&&!stopped&&f.stage1_trim_allowed,raw=f?.rc?.channels?.[3];
  const signed=value=>(value>0?"+":"")+value+" µs";
  $("stage1TrimRaw").textContent=available&&Number.isFinite(raw)?raw+" µs":"--";
  $("stage1TrimRequested").textContent=available?signed(f.rc.stage1_trim_us):"--";
  $("stage1TrimApplied").textContent=available?signed(f.stage1_trim_us):"--";
  $("stage1TrimState").textContent=!fresh?"尚未取得即時狀態":!supported?"韌體未提供微調狀態":stopped?"STOP / 微調停用":enabled?"第一關微調啟用":"微調停用";
  $("stage1TrimState").className="badge"+(enabled?" online":"");
  $("stage1TrimCommand").textContent=!available?"Pin2 基準／輸出指令尚未取得":
    stopped?"Pin2 基準 "+f.pulse2_base+" µs · STOP 前最後輸出指令 "+f.pulse2+" µs":
    "Pin2 基準 "+f.pulse2_base+" µs + 實際微調 "+signed(f.stage1_trim_us)+" → 輸出指令 "+f.pulse2+" µs";
  $("stage1TrimHint").textContent=!fresh?"請連接 USB 並等待最新狀態。":!supported?"目前韌體未提供完整 CH4 微調資料；請燒錄支援此功能的版本。":
    stopped?"微調已停用；最後輸出指令不代表急停後仍有 PWM 輸出，也沒有位置感測回授。":
    !enabled?"微調目前停用；僅第一關遙控模式可微調，退出／HOME、其他關卡、遙控斷線時歸零。":
    "轉動 CH4 可即時微調 Pin2，步驟運行與完成保持時皆可用；實際疊加偏移已含安全極限截限。數值是程式指令，沒有位置感測回授。";
}

// Read-only paired trim: one bounded offset is applied with opposite signs to ID4/5.
function renderStage2Trim(f,fresh){
  const supported=typeof f?.stage2_trim_allowed==="boolean"&&typeof f?.stage2_trim_moving==="boolean"&&
    [f?.position4_base,f?.position5_base,f?.position4,f?.position5,f?.stage2_trim_pulses,
     f?.stage2_trim_target4,f?.stage2_trim_target5,f?.rc?.stage2_trim_pulses].every(Number.isFinite);
  const available=!!fresh&&supported,stopped=!!f?.stopped||f?.phase==="fault";
  const axes=[4,5].map(node=>motorAvailability(f,node));
  const pairReady=axes.every(m=>m.reported&&m.online&&m.known&&!m.recovery);
  const context=f?.stage===2&&f?.mode==="normal"&&f?.rc?.connected===true&&f?.rc?.stage1_mode!==true&&!f?.pending_stage&&!f?.uploading;
  const enabled=available&&!stopped&&pairReady&&context&&f.stage2_trim_allowed;
  const moving=available&&!stopped&&f.stage2_trim_moving,ready=enabled&&f.phase==="ready";
  const raw=f?.rc?.channels?.[3],signed=value=>(value>0?"+":"")+value+" 脈衝";
  $("stage2TrimRaw").textContent=available&&Number.isFinite(raw)?raw+" µs":"--";
  $("stage2TrimRequested").textContent=available?signed(f.rc.stage2_trim_pulses):"--";
  $("stage2TrimApplied").textContent=available?signed(f.stage2_trim_pulses):"--";
  $("stage2TrimState").textContent=!fresh?"尚未取得即時狀態":!supported?"韌體未提供雙步進微調狀態":
    stopped?"STOP / 微調停用":moving?"雙步進微調中 / 等待雙 ACK":
    f.stage===2&&!pairReady?"雙軸未就緒 / 微調停用":ready?"文蛤高度微調啟用":
    enabled?"等待 STEP 完成":f.stage===2&&f.phase==="home_wait"?"HOME / 微調停用":"微調停用";
  $("stage2TrimState").className="badge"+(ready||moving?" online":"");
  $("stage2TrimCommand").textContent=!available?"ID4／ID5 基準／已完成位置／目標尚未取得":
    "ID4 基準 "+f.position4_base+" · "+(stopped?"STOP 前最後完成位置 ":"最後完成位置 ")+f.position4+
    " · "+(stopped?"STOP 前最後目標 ":"已送目標 ")+f.stage2_trim_target4+
    " 脈衝；ID5 基準 "+f.position5_base+" · "+(stopped?"STOP 前最後完成位置 ":"最後完成位置 ")+f.position5+
    " · "+(stopped?"STOP 前最後目標 ":"已送目標 ")+f.stage2_trim_target5+" 脈衝";
  $("stage2TrimHint").textContent=!fresh?"請連接 USB 並等待最新狀態。":
    !supported?"目前韌體未提供完整第二關雙步進微調資料；請燒錄支援此功能的版本。":
    stopped?"微調已停用；顯示急停前最後指令與完成回報，不能代表急停後仍在運轉。":
    moving?"兩顆步進以相反方向調整，須等待 ID4 與 ID5 都回報完成才接受 NEXT；停用後不再追隨旋鈕。":
    f.stage===2&&!pairReady?"第二關高度微調需要 ID4 與 ID5 都已初始化、坐標已知且未隔離；不會單獨驅動一顆。":
    ready?"一般模式第二關在 STEP0 待機即可使用 CH4，完成 STEP 後亦可調整。預設 ±1600 脈衝，100 RPM、ACC 5，每 100 ms 最多更新一次；已完成偏移在雙 ACK 後更新。":
    enabled?"正式 STEP 執行中先等待原動作完成，再接受高度微調。":
    "僅一般模式第二關可微調；失聯、HOME、Debug、切關會停用。HOME 後按 NEXT／PREV 執行正式 STEP 才重新啟用。";
  if(available&&!stopped)$("stage2TrimHint").textContent+=" ID4 = 基準 + 共同偏移且不小於 0；ID5 = 基準 - 共同偏移且不大於 0。遇界共同截斷，維持反向等量，不累加偏移、不改 STEP 歷史。位置是已完成指令，並非即時編碼器回授。";
}

// Read-only MKS telemetry: position6 is the last completed command, not live encoder feedback.
function renderStage4Trim(f,fresh){
  const supported=typeof f?.stage4_trim_allowed==="boolean"&&typeof f?.stage4_trim_moving==="boolean"&&
    [f?.position6_base,f?.position6,f?.stage4_trim_pulses,f?.stage4_trim_target,f?.rc?.stage4_trim_pulses].every(Number.isFinite);
  const available=!!fresh&&supported,stopped=!!f?.stopped||f?.phase==="fault";
  const recovery=motorAvailability(f,6).recovery;
  const enabled=available&&!stopped&&!recovery&&f.stage4_trim_allowed,moving=available&&!stopped&&!recovery&&f.stage4_trim_moving;
  const ready=enabled&&f.phase==="ready",raw=f?.rc?.channels?.[3],fruit=f?.stage===6;
  const stageName=fruit?"第六關水果":"第四關擲筊";
  const signed=value=>(value>0?"+":"")+value+" 脈衝";
  $("stage4TrimRaw").textContent=available&&Number.isFinite(raw)?raw+" µs":"--";
  $("stage4TrimRequested").textContent=available?signed(f.rc.stage4_trim_pulses):"--";
  $("stage4TrimApplied").textContent=available?signed(f.stage4_trim_pulses):"--";
  $("stage4TrimState").textContent=!fresh?"尚未取得即時狀態":!supported?"韌體未提供高度微調狀態":stopped?"STOP / 微調停用":
    recovery?"ID6 隔離 / 等待手動 HOME":moving?"高度微調中 / 等待 ACK":ready?(fruit?"水果高度微調啟用":"擲筊高度微調啟用"):enabled?"等待 STEP 完成":
    fruit&&f.phase==="home_wait"?"水果回 HOME / 微調停用":
    [4,6].includes(f?.stage)&&f?.mode!=="debug"?"微調停用 / 等待 NEXT":"微調停用";
  $("stage4TrimState").className="badge"+(ready||moving?" online":"");
  $("stage4TrimCommand").textContent=!available?"ID6 基準／已完成位置／目標尚未取得":
    "ID6 基準 "+f.position6_base+" 脈衝 · "+(stopped?"STOP 前最後完成位置 ":"最後完成位置 ")+f.position6+
    " 脈衝 · "+(stopped?"STOP 前最後目標 ":"已送目標 ")+f.stage4_trim_target+" 脈衝";
  $("stage4TrimHint").textContent=!fresh?"請連接 USB 並等待最新狀態。":!supported?"目前韌體未提供完整擲筊／水果高度微調資料；請燒錄支援此功能的版本。":
    stopped?"微調已停用；顯示急停前最後指令與完成回報，不能代表急停後仍在運轉。":
    recovery?"ID6 已局部隔離，不再追隨 CH4。"+(stageAvailable(3,f)?"可切換第二／第三關；":"可切換第二關；")+"先排除卡點，再手動 HOME，確認尋零開始與完成後恢復。":
    moving?"ID6 正以低速移至基準 + CH4 偏移；收到完成回報後才接受 NEXT。停用後只等待既有指令完成，不追旋鈕。":
    enabled&&!ready?(fruit?"水果正式 STEP 尚在執行；步進回報與伺服緩移都完成後，才進行高度微調。第5段完成後直接自動 HOME，停用微調。":"正式 STEP 尚在執行；原擲筊動作完成後才進行高度微調，STEP4 的彈射與回降會完整執行。"):
    ready?stageName+" STEP 完成待機時，CH4 可調整 ID6 高度，預設 ±"+(fruit?2000:200)+" 脈衝；置中為 0。已完成偏移只在收到 ACK 後更新。":
    "僅一般模式第四關或第六關可微調。HOME、切關、遙控斷線與 Debug 會停用；下一次 NEXT 正式 STEP 完成後恢復。水果第5段完成後自動 HOME，旋鈕未置中也保持停用，下一輪正式 STEP 完成後才恢復。完成位置是驅動器完成回報對應的目標，並非即時編碼器讀值。";
  if(available&&[4,6].includes(f?.stage)&&!stopped&&!recovery)$("stage4TrimHint").textContent+=" ID6 各關共用絕對位置上限 "+id6PositionMax(f)+" 脈衝；微調遇界截斷。";
  if(available&&fruit&&!stopped&&!recovery){const limits=fruitPositionLimits(f);$("stage4TrimHint").textContent+=" 水果絕對位置限制 "+limits.min+"～"+limits.max+" 脈衝；±2000 的旋鈕要求遇上下限即截斷，因此要求與已完成偏移可能不同。";}
}

function renderChassis(f,fresh){
  const chassis=f?.chassis,rc=f?.rc;
  const reportedDirection=chassis?.direction_stage;
  const directionStage=Number.isInteger(reportedDirection)&&reportedDirection>=1&&reportedDirection<=6?reportedDirection:(f?.pending_stage||f?.stage||4);
  const names={locked:"LOCKED / 鎖定",arming:"ARMING / 置中解鎖中",running:"RUNNING / 可操縱"};
  $("chassisState").textContent=chassis?(chassis.emergency?"STOP / 全車急停":names[chassis.state]||chassis.state):"尚無底盤資料";
  $("chassisState").className="badge"+(fresh&&chassis?.state==="running"?" online":"");
  $("chassisReason").textContent=!fresh?"狀態未更新":chassis?.reason||"四顆 ODrive 正常且搖桿置中 500 ms 才能解鎖。";
  const rcAge=rc?.age_ms>=0xFFFFFFFF?"尚未連線":`${rc?.connected?"最近確認連線":"斷線經過"} ${rc?.age_ms??"—"} ms`;
  $("rcState").textContent=rc?`遙控器：${rc.connected?"已連線":"斷線／逾時"} · ${rcAge} · 選擇第 ${rc.stage??"—"} 關${rc.stage1_mode===undefined?"":rc.stage1_mode?" · 第一關模式":" · 一般選關模式"}`:"遙控器：尚無資料";
  $("rcState").textContent+=` · 底盤有效方向：第 ${directionStage} 關`;
  if(rc?.procession_mode)$("rcState").textContent+=" · 舊版繞境模式";
  const wheels=chassis?.wheels??[10,11,12,13].map(node=>({node,bus:node<12?1:2})),host=$("wheelStatus");
  host.replaceChildren();
  for(const wheel of wheels){
    const row=document.createElement("tr"),name={10:"前左",11:"前右",12:"後左",13:"後右"}[wheel.node]||"輪位";
    const age=wheel.heartbeat_age_ms,error=wheel.error;
    for(const value of [`${name} / ${wheel.node}`,`CAN${wheel.bus}`,age===undefined||age===null||age>=0xFFFFFFFF?"無心跳":`${age} ms`,error===undefined?"—":`0x${Number(error).toString(16).toUpperCase()}`,wheel.axis_state??"—",wheel.velocity===undefined?"—":Number(wheel.velocity).toFixed(2)]){
      const cell=document.createElement("td");cell.textContent=value;row.append(cell);
    }
    host.append(row);
  }
}
function button(label,fn){const b=document.createElement("button");b.type="button";b.textContent=label;b.addEventListener("click",fn);return b;}
function changed(){$("repeatFrom").max=documentModel.steps.length;updateYaml();renderState();}
function field(container,label,value,low,high,update,step=1){const wrap=document.createElement("label");wrap.textContent=label;const input=document.createElement("input");input.type="number";input.value=value;input.min=low;input.max=high;input.step=step;input.required=true;input.addEventListener("input",()=>{update(Number(input.value));changed();});wrap.append(input);container.append(wrap);return input;}
function selectField(container,label,value,options,update,numeric=true){const wrap=document.createElement("label");wrap.textContent=label;const select=document.createElement("select");for(const [v,title] of options)select.add(new Option(title,v));select.value=String(value);select.addEventListener("change",()=>{update(numeric?Number(select.value):select.value);changed();});wrap.append(select);container.append(wrap);}
function actionRow(item,label,list,index,child=false){const kind=Object.keys(item)[0],params=item[kind];const row=document.createElement("div");row.className="action-row";const top=document.createElement("div");top.className="action-top";const id=document.createElement("span");id.className="action-label";id.textContent=label;const select=document.createElement("select");select.setAttribute("aria-label",label+" 動作類型");for(const [key,title] of [["stepper","步進馬達"],["servo","單顆伺服"],["cylinder","電缸"],["delay","非阻塞延遲"],["parallel","平行動作群組"]])if(!child||["stepper","servo","cylinder"].includes(key))select.add(new Option(title,key));select.value=kind;select.addEventListener("change",()=>{list[index]={[select.value]:defaults[select.value]()};renderEditor();changed();});const buttons=document.createElement("div");buttons.className="small-buttons";buttons.append(button("↑",()=>{if(index>0){[list[index-1],list[index]]=[list[index],list[index-1]];renderEditor();changed();}}),button("↓",()=>{if(index<list.length-1){[list[index+1],list[index]]=[list[index],list[index+1]];renderEditor();changed();}}),button("刪除",()=>{list.splice(index,1);renderEditor();changed();}));top.append(id,select,buttons);row.append(top);const fields=document.createElement("div");fields.className="action-fields";row.append(fields);
if(kind==="stepper"){
selectField(fields,"步進馬達",params.node??6,motorChoices,v=>{params.node=v;renderEditor();});
const limits=stepperPositionLimits(params.node??6,editorStage());
field(fields,"絕對位置 / 脈衝",params.position,limits.min,limits.max,v=>params.position=v);
field(fields,"速度 / RPM",params.rpm,1,3000,v=>params.rpm=v);
field(fields,"加減速 / ACC",params.acc??0,0,255,v=>params.acc=v);
}else if(kind==="servo"){
selectField(fields,"伺服",params.pin,servoChoices,v=>params.pin=v);
let angleInput,pulseInput;
angleInput=field(fields,"目標角度 / 度",pulseToAngle(params.pulse_us),0,180,v=>{
  params.pulse_us=angleToPulse(v); pulseInput.value=params.pulse_us;
},0.1);
field(fields,"緩移 / duration_ms",params.duration_ms??0,0,60000,v=>params.duration_ms=v);
pulseInput=field(fields,"目標脈寬 / µs",params.pulse_us,500,2500,v=>{
  params.pulse_us=v; angleInput.value=pulseToAngle(v);
});
}else if(kind==="cylinder"){
selectField(fields,"電缸方向",params.action,[["extend","伸出"],["retract","縮回"],["stop","停止 / LOW LOW"]],v=>{
params.action=v;if(v==="stop")params.duration_ms=0;else if(!(params.duration_ms>0))params.duration_ms=6000;renderEditor();changed();
},false);
let msInput,secondsInput;const minimum=params.action==="stop"?0:1,ms=params.duration_ms??(params.action==="stop"?0:6000);
msInput=field(fields,"運行時間 / ms",ms,minimum,60000,v=>{params.duration_ms=v;secondsInput.value=v/1000;});
secondsInput=field(fields,"運行時間 / 秒",ms/1000,minimum/1000,60,v=>{params.duration_ms=Math.round(v*1000);msInput.value=params.duration_ms;},0.001);
}else if(kind==="delay"){
field(fields,"等待 / ms",params.ms,0,60000,v=>params.ms=v);
}else if(kind==="parallel"){
const children=document.createElement("div");children.className="parallel-children";
params.forEach((a,i)=>children.append(actionRow(a,`致動器 ${i+1}`,params,i,true)));
const add=button("＋ 加入致動器",()=>{params.push({servo:defaults.servo()});renderEditor();changed();});
add.className="secondary add-action";add.disabled=params.length>=10;children.append(add);row.append(children);
}return row;}
function renderEditor(){const host=$("steps");host.replaceChildren();documentModel.steps.forEach((entry,index)=>{const card=document.createElement("section");card.className="panel step-card";const title=document.createElement("div");title.className="step-title";const strong=document.createElement("strong");strong.textContent=`STEP${index+1}`;const controls=document.createElement("div");controls.className="small-buttons";controls.append(button("↑",()=>{if(index){[documentModel.steps[index-1],documentModel.steps[index]]=[entry,documentModel.steps[index-1]];renderEditor();changed();}}),button("↓",()=>{if(index<documentModel.steps.length-1){[documentModel.steps[index+1],documentModel.steps[index]]=[entry,documentModel.steps[index+1]];renderEditor();changed();}}),button("刪除此段",()=>{documentModel.steps.splice(index,1);renderEditor();changed();}));title.append(strong,controls);card.append(title);entry.actions.forEach((item,i)=>card.append(actionRow(item,`STEP${index+1}-${i+1}`,entry.actions,i)));const add=button("＋ 新增 Action",()=>{entry.actions.push({servo:defaults.servo()});renderEditor();changed();});add.className="secondary add-action";card.append(add);host.append(card);});updateYaml();}
function yamlAction(action,indent){const [kind,params]=Object.entries(action)[0];if(kind==="parallel")return `${indent}- parallel:\n`+params.map(child=>yamlAction(child,indent+"    ")).join("\n");return `${indent}- ${kind}: {${Object.entries(params).map(([key,value])=>`${key}: ${value}`).join(", ")}}`;}
function updateYaml(){$("yamlPreview").textContent=`stage: ${editorStage()}\n${editorStage()===1?"prop: "+(documentModel.prop||2)+"\n":""}repeat_from: ${documentModel.repeat_from??1}\nsteps:\n`+documentModel.steps.map(step=>"  - actions:\n"+step.actions.map(a=>yamlAction(a,"      ")).join("\n")).join("\n");}
function editorValid(){if(!$("repeatFrom").reportValidity())throw Error("循環起始段必須在有效 STEP 範圍內");for(const input of $("editorView").querySelectorAll("input"))if(!input.reportValidity())throw Error("請修正 Action 數值");if(!documentModel.steps.length||documentModel.steps.some(s=>!s.actions.length))throw Error("至少需要一段 STEP，每段至少一個 Action");return clone(documentModel);}
function loadConfig(config){documentModel=clone(config.document);profile=String(config.profile??(documentModel.stage===1?"1_"+(documentModel.prop||2):documentModel.stage??4));if(profile==="1")profile="1_2";$("profile").value=String(profile);$("repeatFrom").value=documentModel.repeat_from??1;$("repeatFrom").max=documentModel.steps.length;savedHash=config.hash;savedSnapshot=JSON.stringify(documentModel);appliedSnapshot="";renderEditor();renderState();}
document.querySelectorAll("[data-view]").forEach(b=>b.addEventListener("click",()=>{view=b.dataset.view;document.querySelectorAll("[data-view]").forEach(v=>v.classList.toggle("active",v===b));$("controlView").classList.toggle("hidden",view!=="control");$("editorView").classList.toggle("hidden",view!=="editor");}));
$("scan").addEventListener("click",()=>perform(scan));$("port").addEventListener("change",renderState);$("connect").addEventListener("click",()=>perform(async()=>{const result=await api("/api/connect",{port:$("port").value});state.connected=true;state.firmware=result.result;notice("已連接控制器。連線本身不會發送動作指令。");}));$("disconnect").addEventListener("click",()=>perform(async()=>{await api("/api/disconnect",{});state={};notice("USB 已中斷連線。此操作不會停止正在執行的機構。");}));
$("globalStop").addEventListener("click",async()=>{try{await send("stop");}catch(error){notice(error.message,true);}await poll();});$("normalMode").addEventListener("click",()=>perform(()=>send("mode",{mode:"normal"})));$("debugMode").addEventListener("click",()=>perform(()=>send("mode",{mode:"debug"})));document.querySelectorAll("[data-command]").forEach(b=>b.addEventListener("click",()=>perform(()=>send(b.dataset.command))));
$("pwmMaxSlider").addEventListener("input",()=>editPwm("pwmMaxSlider","pwmMaxValue"));
$("pwmMaxValue").addEventListener("input",()=>editPwm("pwmMaxValue","pwmMaxSlider"));
$("pwmIntervalSlider").addEventListener("input",()=>editPwm("pwmIntervalSlider","pwmStepInterval"));
$("pwmStepInterval").addEventListener("input",()=>editPwm("pwmStepInterval","pwmIntervalSlider"));
for(const [,id] of motionFields)$(id).addEventListener("input",()=>{motionDirty=true;motionPending=null;motionBankNotice="";renderState();});
$("motionApply").addEventListener("click",()=>applyMotion());
$("motionCenter").addEventListener("click",()=>{$("motionX").value="0";$("motionLeftY").value="0";$("motionRightY").value="0";motionDirty=true;motionPending=null;motionBankNotice="";renderState();});
$("pwmApply").addEventListener("click",()=>applyPwm());
$("pwmReset").addEventListener("click",()=>applyPwm(true));
$("motorNode").addEventListener("change",renderState);
$("cylinderDuration").addEventListener("input",renderState);
$("extendCylinder").addEventListener("click",()=>perform(()=>send("debug.cylinder",{action:"extend",duration_ms:integer("cylinderDuration")})));
$("retractCylinder").addEventListener("click",()=>perform(()=>send("debug.cylinder",{action:"retract",duration_ms:integer("cylinderDuration")})));
$("stopCylinder").addEventListener("click",async()=>{try{await send("debug.cylinder",{action:"stop",duration_ms:0});}catch(error){notice(error.message,true);}await poll();});
$("selectStage").addEventListener("click",()=>perform(()=>send("stage",profileIdentity($("activeStage").value))));
$("activeStage").addEventListener("change",renderState);
$("triggerProp").addEventListener("click",()=>perform(()=>send("prop",{prop:profileIdentity($("activeStage").value).prop})));
$("exitStage1").addEventListener("click",()=>perform(()=>send("stage1.exit",{stage:4})));
$("repeatFrom").addEventListener("input",()=>{documentModel.repeat_from=Number($("repeatFrom").value);changed();});
$("stepperForm").addEventListener("submit",e=>{e.preventDefault();perform(()=>send("debug.move",{node:Number($("motorNode").value),position:integer("motorPosition"),rpm:integer("motorRpm"),acc:integer("motorAcc")}));});$("motorZero").addEventListener("click",()=>perform(()=>send("debug.move",{node:Number($("motorNode").value),position:0,rpm:integer("motorRpm"),acc:integer("motorAcc")})));$("motorGoHome").addEventListener("click",()=>perform(()=>send("debug.gohome",{node:Number($("motorNode").value)})));
$("profile").addEventListener("change",()=>perform(async()=>{const requested=$("profile").value;try{const result=await api(`/api/init?profile=${requested}`);loadConfig(result.config);notice(`已載入${profileLabel(documentModel.stage,documentModel.prop)}電腦 YAML；控制器流程尚未切換。`);}catch(error){$("profile").value=String(profile);throw error;}}));
$("addStep").addEventListener("click",()=>{documentModel.steps.push({actions:[{delay:{ms:1000}}]});renderEditor();changed();});$("reloadConfig").addEventListener("click",()=>perform(async()=>{const result=await api(`/api/init?profile=${profile}`);loadConfig(result.config);notice("已重新載入電腦 YAML；尚未套用至韌體。");}));$("saveConfig").addEventListener("click",()=>perform(async()=>{const result=await api("/api/config",{document:editorValid(),expected_hash:savedHash,profile});loadConfig(result.result);notice("YAML 已儲存到電腦。可立即套用，或下次 Build／Upload 作為預設。");}));$("applyConfig").addEventListener("click",()=>perform(async()=>{const document=editorValid();if(!stageAvailable(document.stage??4))throw Error("此關卡已停用，不能套用至 RAM。");notice("正在套用 RAM；先復位目前關卡與交換動作表，第二關再等待電缸回縮待機。");const result=await api("/api/apply",{document});if(result.result.phase!=="ready"||result.result.step!==0||result.result.completed!==0||result.result.stage!==(document.stage??4)||(result.result.prop??((document.stage??4)===1?2:0))!==((document.stage??4)===1?(document.prop||2):0))throw Error("尚未確認原關卡復位與RAM交換完成；請查看控制器狀態。");appliedSnapshot=JSON.stringify(document);notice(`原關卡復位與${profileLabel(document.stage??4,document.prop)} RAM 交換已完成；Ready（STEP0），${document.stage===1?"按道具觸發開始 STEP1":"NEXT 從 STEP1 開始"}。重啟後恢復韌體內建設定。`);}));$("clearLog").addEventListener("click",()=>$("logs").replaceChildren());
async function init(){servoCards();renderState();try{const result=await api("/api/init");token=result.token;if(result.simulation){document.querySelector(".eyebrow").textContent="SIMULATION PREVIEW / 不連接實體 USB";notice("此預覽使用模擬韌體與 YAML 副本，不會控制實體機構。");}loadConfig(result.config);await scan();await poll();}catch(error){notice(error.message,true);}setInterval(poll,1200);}
init();
