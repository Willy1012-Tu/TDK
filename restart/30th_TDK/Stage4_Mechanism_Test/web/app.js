"use strict";
const $=id=>document.getElementById(id);
let token="",documentModel={steps:[]},savedHash="",savedSnapshot="",appliedSnapshot="",lastLog=0,state={},busy=false,view="control",pollRunning=false,profile=4;
const motorChoices=[[6,"ID6 · 花瓶／擲筊升降"],[7,"ID7 · 第二關升降"]];
const servoChoices=[[2,"Pin2 · 花瓶／筊杯夾持"],[5,"Pin5 · 花瓶／筊杯傾角"],[4,"Pin4 · 第二關成對伺服"],[6,"Pin6 · 第二關獨立伺服"],[9,"Pin9 · 第二關成對伺服"]];
const servoPins=servoChoices.map(([pin])=>pin);
const defaults={stepper:()=>({node:profile===2?7:6,position:0,rpm:320,acc:5}),servo:()=>({pin:profile===2?4:2,pulse_us:1500,duration_ms:1000}),delay:()=>({ms:1000}),parallel:()=>[{servo:{pin:2,pulse_us:1500}},{servo:{pin:5,pulse_us:1400}}]};
const clone=value=>JSON.parse(JSON.stringify(value));
function notice(text,error=false){$("message").textContent=text;$("message").className="message"+(error?" error":"");}
async function api(path,payload){const response=await fetch(path,{method:payload===undefined?"GET":"POST",headers:payload===undefined?{}:{"Content-Type":"application/json","X-Stage4-Token":token},body:payload===undefined?undefined:JSON.stringify(payload)});const data=await response.json();if(!response.ok)throw Error(data.error||"請求失敗");return data;}
async function perform(fn){if(busy)return;busy=true;renderState();try{await fn();}catch(error){notice(error.message,true);}finally{busy=false;await poll();renderState();}}
function integer(id){const element=$(id);if(!element.reportValidity())throw Error("請檢查輸入欄位");return Number(element.value);}
function calibration(){return {min:state.firmware?.servo_min_us??500,max:state.firmware?.servo_max_us??2500};}
function angleToPulse(angle){const c=calibration();return Math.round(c.min+angle*(c.max-c.min)/180);}
function pulseToAngle(pulse){const c=calibration();return Math.round((pulse-c.min)*1800/(c.max-c.min))/10;}
async function send(command,extra={}){await api("/api/command",{command,...extra});notice(command==="stage"&&extra.stage===2?"第二關已選入；首次切入會自動執行 STEP1 預備，等待 Ready 後 NEXT 接續後續段落。":command==="stop"?"機構急停已接受；所有啟用馬達已停用，需重啟 Teensy 才能繼續。":"命令已接受；請查看控制器狀態與紀錄。");}
function appendLogs(logs){const box=$("logs");if(logs.length&&box.querySelector(".log-empty"))box.replaceChildren();for(const entry of logs){lastLog=Math.max(lastLog,entry.id);const line=document.createElement("div");line.className="log-line"+(entry.direction==="tx"?" log-tx":"");const time=document.createElement("span");time.className="log-time";time.textContent=entry.time;const text=document.createElement("span");text.textContent=(entry.direction==="tx"?"→ ":"")+entry.text;line.append(time,text);box.append(line);}while(box.children.length>400)box.firstChild.remove();if(logs.length)box.scrollTop=box.scrollHeight;}
function renderState(){const f=state.firmware,connected=!!state.connected,fresh=connected&&f&&state.status_age!==null&&state.status_age<4,ready=fresh&&f.phase==="ready"&&!f.uploading;
$("connectionBadge").textContent=connected?state.port:"未連線";$("connectionBadge").className="badge"+(connected?" online":"");$("connectionDot").className="dot"+(connected?" connected":"");$("sidebarConnection").textContent=connected?`${state.port} 已連線`:"USB 尚未連接";
$("connect").disabled=busy||connected||!$("port").value;$("disconnect").disabled=busy||!connected;$("scan").disabled=busy;$("globalStop").disabled=!connected;
const phases={ready:"Ready / 閒置",busy:"執行中",booting:"初始化中",fault:"已停止 / Fault"};$("phase").textContent=f?phases[f.phase]||f.phase:"—";$("phaseDetail").textContent=state.error||(!connected?"選擇串口後開始連線":!fresh?"狀態未更新；確認韌體與 USB 連線":f.phase==="fault"?"請確認機構後重啟 Teensy":f.uploading?"正在接收設定，完成後才切換":`模式：${f.mode.toUpperCase()}`);
$("currentStep").textContent=f?.step?`STEP${f.step}${f.branch?"-"+f.branch:""}`:f?.mode==="debug"?"DEBUG":"—";$("stepDetail").textContent=f?`控制器第 ${f.stage??4} 關 · 已完成 ${f.completed} / ${f.steps} 段`:"STEP 自動編號";$("planSource").textContent=f?.runtime_plan?`RAM · v${f.plan_version}`:"韌體內建";$("planDetail").textContent=f?.pending_stage?`設定切換中，將切至第 ${f.pending_stage} 關`:f?.runtime_plan?"重新啟動後恢復韌體預設":"STEP 進度見「目前執行」；第三關為待機／未接線";
const debug=f?.mode==="debug";$("normalMode").classList.toggle("selected",!debug);$("debugMode").classList.toggle("selected",debug);$("normalMode").disabled=busy||!ready||!debug;$("debugMode").disabled=busy||!ready||debug;$("normalPanel").classList.toggle("hidden",debug);$("debugPanel").classList.toggle("hidden",!debug);$("modeHint").textContent=debug?(f?.stage===3&&f?.stage3_enabled===false?"第三關待機／未接線；請切到第二或第四關調試。":"Debug：ID6／ID7 點動；五顆伺服可獨立發送。"):`Normal：使用控制器第 ${f?.stage??4} 關的有效 STEP 設定。`;
document.querySelectorAll("[data-command]").forEach(button=>button.disabled=busy||!ready||debug||(button.dataset.command==="prev"&&!f?.previous_available));
$("selectStage").disabled=busy||!fresh||debug||f?.uploading||f?.phase==="fault"||f?.phase==="booting"||(f?.phase==="busy"&&f?.stage!==3);
if($("chassisState"))renderChassis(f,fresh);
const standby=f?.stage===3&&f?.stage3_enabled===false;
const actuatorBlocked=busy||!fresh||!debug||f?.phase==="fault"||f?.phase==="booting"||f?.uploading||standby;
for(const id of ["motorNode","motorPosition","motorRpm","motorAcc","sendMotor","motorZero","motorGoHome"])$(id).disabled=actuatorBlocked;
for(const pin of servoPins){for(const prefix of ["servoSlider","servoAngle","servoDuration","sendServo"]){const control=$(prefix+pin);if(control)control.disabled=actuatorBlocked;}const v=$("servoValue"+pin),pulse=f?.["pulse"+pin];if(v)v.textContent=pulse!==undefined?`${(f.stopped || f.phase === "fault") ? "STOP 前指令" : "目前輸出"} ${pulse} µs / 約 ${pulseToAngle(pulse)}°（指令值）`:"沒有角度回授";}
const node=Number($("motorNode").value||6),position=f?.["position"+node];$("motorId").textContent=node;$("motorValue").textContent=position!==undefined&&((f?.position_known_mask??0)&(1<<(node-4)))?`最近完成目標 ${position} 脈衝（非位置感測值）`:"尚未確認位置";$("profile").disabled=busy;$("activeStage").disabled=busy;$("repeatFrom").disabled=busy||profile===3;$("applyConfig").disabled=busy||!ready||debug;$("saveConfig").disabled=busy;$("reloadConfig").disabled=busy;$("addStep").disabled=busy||profile===3;$("editState").textContent=JSON.stringify(documentModel)===savedSnapshot?"與電腦 YAML 一致":"有未儲存的修改";$("appliedState").textContent=appliedSnapshot===JSON.stringify(documentModel)&&state.applied_hash?"此份編輯已套用至 RAM":"編輯內容未確認套用；儲存 YAML 不會自動套用";
}
async function poll(){if(pollRunning)return;pollRunning=true;try{if(state.connected&&!busy)try{await api("/api/refresh",{});}catch(error){notice(error.message,true);}const incoming=await api(`/api/state?after=${lastLog}`);state=incoming;appendLogs(incoming.logs);renderState();}catch(error){notice("閘道服務未回應："+error.message,true);state={};renderState();}finally{pollRunning=false;}}
async function scan(){const data=await api("/api/ports");const old=$("port").value;$("port").replaceChildren(new Option("選擇 COM Port",""));for(const p of data.ports)$("port").add(new Option(`${p.device} · ${p.description}`,p.device));if(data.ports.some(p=>p.device===old))$("port").value=old;else if(data.ports.length===1)$("port").value=data.ports[0].device;renderState();}
function servoCards(){
  for(const [pin,title] of servoChoices){
    const card=document.createElement("form");card.className="panel debug-card servo-card";
    card.innerHTML=`<div class="section-heading"><div><span class="eyebrow">PWM / PIN ${pin}</span><h2>${title}</h2></div></div><label>獨立角度滑桿<input id="servoSlider${pin}" type="range" min="0" max="180" step="0.1" value="90" aria-label="Pin ${pin} 目標角度"></label><div class="field-grid"><label>角度 <span>0–180°</span><input id="servoAngle${pin}" type="number" min="0" max="180" step="0.1" value="90" required></label><label>緩移時間 <span>duration_ms</span><input id="servoDuration${pin}" type="number" min="0" max="60000" value="1000" required></label></div><button class="primary wide" id="sendServo${pin}" type="submit">發送 Pin ${pin}</button><p id="servoValue${pin}" class="servo-value">沒有角度回授</p>`;
    card.addEventListener("submit",event=>{event.preventDefault();perform(()=>send("debug.servo",{pin,angle:integer("servoAngle"+pin),duration_ms:integer("servoDuration"+pin)}));});
    $("servoCards").append(card);
    $("servoSlider"+pin).addEventListener("input",()=>$("servoAngle"+pin).value=$("servoSlider"+pin).value);
    $("servoAngle"+pin).addEventListener("input",()=>$("servoSlider"+pin).value=$("servoAngle"+pin).value);
  }
}
function renderChassis(f,fresh){
  const chassis=f?.chassis,rc=f?.rc;
  const names={locked:"LOCKED / 鎖定",arming:"ARMING / 置中解鎖中",running:"RUNNING / 可操縱"};
  $("chassisState").textContent=chassis?(chassis.emergency?"STOP / 全車急停":names[chassis.state]||chassis.state):"尚無底盤資料";
  $("chassisState").className="badge"+(fresh&&chassis?.state==="running"?" online":"");
  $("chassisReason").textContent=!fresh?"狀態未更新":chassis?.reason||"四顆 ODrive 正常且搖桿置中 500 ms 才能解鎖。";
  const rcAge=rc?.age_ms>=0xFFFFFFFF?"尚未連線":`${rc?.connected?"最近確認連線":"斷線經過"} ${rc?.age_ms??"—"} ms`;
  $("rcState").textContent=rc?`遙控器：${rc.connected?"已連線":"斷線／逾時"} · ${rcAge} · 選擇第 ${rc.stage??"—"} 關`:"遙控器：尚無資料";
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
function selectField(container,label,value,options,update){const wrap=document.createElement("label");wrap.textContent=label;const select=document.createElement("select");for(const [v,title] of options)select.add(new Option(title,v));select.value=String(value);select.addEventListener("change",()=>{update(Number(select.value));changed();});wrap.append(select);container.append(wrap);}
function actionRow(item,label,list,index,child=false){const kind=Object.keys(item)[0],params=item[kind];const row=document.createElement("div");row.className="action-row";const top=document.createElement("div");top.className="action-top";const id=document.createElement("span");id.className="action-label";id.textContent=label;const select=document.createElement("select");select.setAttribute("aria-label",label+" 動作類型");for(const [key,title] of [["stepper","步進馬達"],["servo","單顆伺服"],["delay","非阻塞延遲"],["parallel","平行動作群組"]])if(!child||["stepper","servo"].includes(key))select.add(new Option(title,key));select.value=kind;select.addEventListener("change",()=>{list[index]={[select.value]:defaults[select.value]()};renderEditor();changed();});const buttons=document.createElement("div");buttons.className="small-buttons";buttons.append(button("↑",()=>{if(index>0){[list[index-1],list[index]]=[list[index],list[index-1]];renderEditor();changed();}}),button("↓",()=>{if(index<list.length-1){[list[index+1],list[index]]=[list[index],list[index+1]];renderEditor();changed();}}),button("刪除",()=>{list.splice(index,1);renderEditor();changed();}));top.append(id,select,buttons);row.append(top);const fields=document.createElement("div");fields.className="action-fields";row.append(fields);
if(kind==="stepper"){
selectField(fields,"步進馬達",params.node??6,motorChoices,v=>params.node=v);
field(fields,"絕對位置 / 脈衝",params.position,-8388607,8388607,v=>params.position=v);
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
}else if(kind==="delay"){
field(fields,"等待 / ms",params.ms,0,60000,v=>params.ms=v);
}else if(kind==="parallel"){
const children=document.createElement("div");children.className="parallel-children";
params.forEach((a,i)=>children.append(actionRow(a,`致動器 ${i+1}`,params,i,true)));
const add=button("＋ 加入致動器",()=>{params.push({servo:defaults.servo()});renderEditor();changed();});
add.className="secondary add-action";add.disabled=params.length>=7;children.append(add);row.append(children);
}return row;}
function renderEditor(){const host=$("steps");host.replaceChildren();if(profile===3){const card=document.createElement("section");card.className="panel step-card";card.textContent="STEP1 · 第三關待機／未接線（delay 0）；切出不執行停放。";host.append(card);updateYaml();return;}documentModel.steps.forEach((entry,index)=>{const card=document.createElement("section");card.className="panel step-card";const title=document.createElement("div");title.className="step-title";const strong=document.createElement("strong");strong.textContent=`STEP${index+1}`;const controls=document.createElement("div");controls.className="small-buttons";controls.append(button("↑",()=>{if(index){[documentModel.steps[index-1],documentModel.steps[index]]=[entry,documentModel.steps[index-1]];renderEditor();changed();}}),button("↓",()=>{if(index<documentModel.steps.length-1){[documentModel.steps[index+1],documentModel.steps[index]]=[entry,documentModel.steps[index+1]];renderEditor();changed();}}),button("刪除此段",()=>{documentModel.steps.splice(index,1);renderEditor();changed();}));title.append(strong,controls);card.append(title);entry.actions.forEach((item,i)=>card.append(actionRow(item,`STEP${index+1}-${i+1}`,entry.actions,i)));const add=button("＋ 新增 Action",()=>{entry.actions.push({servo:defaults.servo()});renderEditor();changed();});add.className="secondary add-action";card.append(add);host.append(card);});updateYaml();}
function yamlAction(action,indent){const [kind,params]=Object.entries(action)[0];if(kind==="parallel")return `${indent}- parallel:\n`+params.map(child=>yamlAction(child,indent+"    ")).join("\n");return `${indent}- ${kind}: {${Object.entries(params).map(([key,value])=>`${key}: ${value}`).join(", ")}}`;}
function updateYaml(){$("yamlPreview").textContent=`stage: ${documentModel.stage??profile}\nrepeat_from: ${documentModel.repeat_from??1}\nsteps:\n`+documentModel.steps.map(step=>"  - actions:\n"+step.actions.map(a=>yamlAction(a,"      ")).join("\n")).join("\n");}
function editorValid(){if(profile===3)return {stage:3,repeat_from:1,steps:[{actions:[{delay:{ms:0}}]}]};if(!$("repeatFrom").reportValidity())throw Error("循環起始段必須在有效 STEP 範圍內");for(const input of $("editorView").querySelectorAll("input"))if(!input.reportValidity())throw Error("請修正 Action 數值");if(!documentModel.steps.length||documentModel.steps.some(s=>!s.actions.length))throw Error("至少需要一段 STEP，每段至少一個 Action");return clone(documentModel);}
function loadConfig(config){documentModel=clone(config.document);profile=config.profile??documentModel.stage??4;$("profile").value=String(profile);$("repeatFrom").value=documentModel.repeat_from??1;$("repeatFrom").max=documentModel.steps.length;savedHash=config.hash;savedSnapshot=JSON.stringify(documentModel);appliedSnapshot="";renderEditor();renderState();}
document.querySelectorAll("[data-view]").forEach(b=>b.addEventListener("click",()=>{view=b.dataset.view;document.querySelectorAll("[data-view]").forEach(v=>v.classList.toggle("active",v===b));$("controlView").classList.toggle("hidden",view!=="control");$("editorView").classList.toggle("hidden",view!=="editor");}));
$("scan").addEventListener("click",()=>perform(scan));$("port").addEventListener("change",renderState);$("connect").addEventListener("click",()=>perform(async()=>{const result=await api("/api/connect",{port:$("port").value});state.connected=true;state.firmware=result.result;notice("已連接控制器。連線本身不會發送動作指令。");}));$("disconnect").addEventListener("click",()=>perform(async()=>{await api("/api/disconnect",{});state={};notice("USB 已中斷連線。此操作不會停止正在執行的機構。");}));
$("globalStop").addEventListener("click",async()=>{try{await send("stop");}catch(error){notice(error.message,true);}await poll();});$("normalMode").addEventListener("click",()=>perform(()=>send("mode",{mode:"normal"})));$("debugMode").addEventListener("click",()=>perform(()=>send("mode",{mode:"debug"})));document.querySelectorAll("[data-command]").forEach(b=>b.addEventListener("click",()=>perform(()=>send(b.dataset.command))));
$("motorNode").addEventListener("change",renderState);
$("selectStage").addEventListener("click",()=>perform(()=>send("stage",{stage:Number($("activeStage").value)})));
$("repeatFrom").addEventListener("input",()=>{documentModel.repeat_from=Number($("repeatFrom").value);changed();});
$("stepperForm").addEventListener("submit",e=>{e.preventDefault();perform(()=>send("debug.move",{node:Number($("motorNode").value),position:integer("motorPosition"),rpm:integer("motorRpm"),acc:integer("motorAcc")}));});$("motorZero").addEventListener("click",()=>perform(()=>send("debug.move",{node:Number($("motorNode").value),position:0,rpm:integer("motorRpm"),acc:integer("motorAcc")})));$("motorGoHome").addEventListener("click",()=>perform(()=>send("debug.gohome",{node:Number($("motorNode").value)})));
$("profile").addEventListener("change",()=>perform(async()=>{const requested=Number($("profile").value);try{const result=await api(`/api/init?profile=${requested}`);loadConfig(result.config);notice(`已載入第 ${profile} 關電腦 YAML；控制器流程尚未切換。`);}catch(error){$("profile").value=String(profile);throw error;}}));
$("addStep").addEventListener("click",()=>{documentModel.steps.push({actions:[{delay:{ms:1000}}]});renderEditor();changed();});$("reloadConfig").addEventListener("click",()=>perform(async()=>{const result=await api(`/api/init?profile=${profile}`);loadConfig(result.config);notice("已重新載入電腦 YAML；尚未套用至韌體。");}));$("saveConfig").addEventListener("click",()=>perform(async()=>{const result=await api("/api/config",{document:editorValid(),expected_hash:savedHash,profile});loadConfig(result.result);notice("YAML 已儲存到電腦。可立即套用，或下次 Build／Upload 作為預設。");}));$("applyConfig").addEventListener("click",()=>perform(async()=>{const document=editorValid();notice(document.stage===2?"正在套用第二關 RAM，並等待 STEP1 預備完成。":"正在驗證並套用 RAM 動作設定。");await api("/api/apply",{document});appliedSnapshot=JSON.stringify(document);notice(document.stage===2?`第二關 RAM 已套用、STEP1 預備已完成；NEXT 從 STEP${document.steps.length>1?2:document.repeat_from??1} 開始。`:"Action 已套用；按 NEXT／RUN 執行流程。重啟後恢復韌體內建設定。");}));$("clearLog").addEventListener("click",()=>$("logs").replaceChildren());
async function init(){servoCards();renderState();try{const result=await api("/api/init");token=result.token;if(result.simulation){document.querySelector(".eyebrow").textContent="SIMULATION PREVIEW / 不連接實體 USB";notice("此預覽使用模擬韌體與 YAML 副本，不會控制實體機構。");}loadConfig(result.config);await scan();await poll();}catch(error){notice(error.message,true);}setInterval(poll,1200);}
init();
