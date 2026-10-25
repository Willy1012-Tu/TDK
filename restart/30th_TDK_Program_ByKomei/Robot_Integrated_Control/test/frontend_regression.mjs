// DOM model checks plus request payloads; no hardware or external browser dependencies.
import fs from 'node:fs/promises';
import vm from 'node:vm';
import assert from 'node:assert/strict';
const html=await fs.readFile(new URL('../web/index.html',import.meta.url),'utf8');
const source=await fs.readFile(new URL('../web/app.js',import.meta.url),'utf8');
const elements=new Map();
class Element {
  constructor(tag='div'){this.tagName=tag;this.children=[];this.listeners={};this.value='';this.textContent='';this.disabled=false;this.dataset={};this.className='';this.classList={toggle(){}};}
  set innerHTML(value){for(const match of value.matchAll(/<(\w+)[^>]*\bid="([^"]+)"[^>]*>/g)){const e=new Element(match[1]);const v=match[0].match(/value="([^"]*)"/);if(v)e.value=v[1];elements.set(match[2],e);}}
  get firstChild(){return this.children[0];}
  append(...items){for(const item of items){item.parent=this;this.children.push(item);}}
  add(option){this.append(option);if(this.children.length===1)this.value=option.value;}
  replaceChildren(...items){this.children=[];this.append(...items);}
  remove(){if(this.parent)this.parent.children=this.parent.children.filter(c=>c!==this);}
  addEventListener(type,callback){this.listeners[type]=callback;}
  setAttribute(){}
  reportValidity(){if(this.required&&this.value==='')return false;const v=Number(this.value);return Number.isFinite(v)&&(this.min===undefined||v>=Number(this.min))&&(this.max===undefined||v<=Number(this.max));}
  querySelector(){return null;}
  querySelectorAll(selector){const all=[];const collect=e=>{for(const child of e.children){if(selector==='input'&&child.tagName==='input')all.push(child);collect(child);}};collect(this);return all;}
}
for(const match of html.matchAll(/<(\w+)[^>]*\bid="([^"]+)"[^>]*>/g)){const e=new Element(match[1]);const v=match[0].match(/value="([^"]*)"/);if(v)e.value=v[1];elements.set(match[2],e);}
elements.get('editorView').append(elements.get('steps'));
elements.get('motorNode').value='6';
const navs=['control','editor'].map(view=>{const e=new Element('button');e.dataset.view=view;return e;});
const normals=[...html.matchAll(/data-command="([^"]+)"/g)].map(([,command])=>{const e=new Element('button');e.dataset.command=command;return e;});
const normal=name=>normals.find(button=>button.dataset.command===name);
const stages=elements.has('chassisState')?[1,2,3,4,5,6]:[2,3,4,5,6];
const eyebrow=new Element();
const model={stage:4,repeat_from:1,steps:[{actions:[{servo:{pin:2,pulse_us:1500,duration_ms:1000}}]}]};
const thirdModel={"stage":3,"repeat_from":2,"steps":[{"actions":[{"parallel":[{"stepper":{"node":4,"position":7100,"rpm":3000,"acc":254}},{"stepper":{"node":5,"position":-7100,"rpm":3000,"acc":254}},{"servo":{"pin":3,"pulse_us":1630}}]}]},{"actions":[{"parallel":[{"stepper":{"node":4,"position":0,"rpm":50,"acc":1}},{"stepper":{"node":5,"position":0,"rpm":50,"acc":1}},{"servo":{"pin":3,"pulse_us":1700}}]}]},{"actions":[{"parallel":[{"stepper":{"node":4,"position":4000,"rpm":320,"acc":2}},{"stepper":{"node":5,"position":-4000,"rpm":320,"acc":2}},{"servo":{"pin":3,"pulse_us":1620}}]}]},{"actions":[{"parallel":[{"stepper":{"node":4,"position":7100,"rpm":320,"acc":2}},{"stepper":{"node":5,"position":-7100,"rpm":320,"acc":2}},{"servo":{"pin":3,"pulse_us":1620}}]}]}]};
const firstModel={stage:1,repeat_from:1,steps:[{actions:[{stepper:{node:6,position:8000,rpm:1000,acc:230}}]}]};
// Synthetic four-step editor fixture; current Stage2 YAML is verified by the Python file/RAM roundtrips.
const secondModel={"stage":2,"repeat_from":2,"steps":[{"actions":[{"parallel":[{"cylinder":{"action":"extend","duration_ms":6000}},{"servo":{"pin":9,"pulse_us":1800}},{"servo":{"pin":4,"pulse_us":1300}}]}]},{"actions":[{"parallel":[{"servo":{"pin":9,"pulse_us":1000}},{"servo":{"pin":4,"pulse_us":2100}}]}]},{"actions":[{"cylinder":{"action":"retract","duration_ms":6000}}]},{"actions":[{"servo":{"pin":6,"pulse_us":600,"duration_ms":1000}},{"delay":{"ms":5000}}]}]};
// Editor protocol fixtures are independent of the operator's tuned on-disk YAML defaults.
const profileModels={"1_1":{"stage":1,"prop":1,"repeat_from":1,"steps":[{"actions":[{"parallel":[{"stepper":{"node":6,"position":8000,"rpm":1000,"acc":230}},{"servo":{"pin":5,"pulse_us":2056}},{"servo":{"pin":2,"pulse_us":1611}}]}]},{"actions":[{"stepper":{"node":6,"position":6000,"rpm":500,"acc":150}}]}]},"1_2":{"stage":1,"prop":2,"repeat_from":1,"steps":[{"actions":[{"parallel":[{"stepper":{"node":6,"position":8000,"rpm":1000,"acc":230}},{"servo":{"pin":5,"pulse_us":2056}},{"servo":{"pin":2,"pulse_us":1833}}]}]},{"actions":[{"servo":{"pin":2,"pulse_us":2111}}]},{"actions":[{"parallel":[{"stepper":{"node":6,"position":5000,"rpm":1000,"acc":245}},{"servo":{"pin":2,"pulse_us":1389}}]}]}]},"1_3":{"stage":1,"prop":3,"repeat_from":1,"steps":[{"actions":[{"parallel":[{"stepper":{"node":6,"position":8000,"rpm":1000,"acc":230}},{"servo":{"pin":5,"pulse_us":2056}},{"servo":{"pin":2,"pulse_us":1667}}]}]},{"actions":[{"stepper":{"node":6,"position":0,"rpm":1000,"acc":255}}]}]},"1_4":{"stage":1,"prop":4,"repeat_from":1,"steps":[{"actions":[{"parallel":[{"stepper":{"node":6,"position":7900,"rpm":1000,"acc":230}},{"servo":{"pin":5,"pulse_us":1833}},{"servo":{"pin":2,"pulse_us":1611}}]}]},{"actions":[{"servo":{"pin":2,"pulse_us":1111}}]},{"actions":[{"stepper":{"node":6,"position":4000,"rpm":1500,"acc":255}}]},{"actions":[{"servo":{"pin":2,"pulse_us":1667}}]}]},"1_5":{"stage":1,"prop":5,"repeat_from":1,"steps":[{"actions":[{"parallel":[{"stepper":{"node":6,"position":7900,"rpm":1000,"acc":230}},{"servo":{"pin":5,"pulse_us":1944}},{"servo":{"pin":2,"pulse_us":1611}}]}]},{"actions":[{"servo":{"pin":2,"pulse_us":1056}}]},{"actions":[{"stepper":{"node":6,"position":4000,"rpm":1500,"acc":255}}]},{"actions":[{"servo":{"pin":2,"pulse_us":1667}}]}]},"2":{"stage":2,"repeat_from":2,"steps":[{"actions":[{"parallel":[{"cylinder":{"action":"extend","duration_ms":6000}},{"servo":{"pin":9,"pulse_us":1800}},{"servo":{"pin":4,"pulse_us":1300}}]}]},{"actions":[{"parallel":[{"servo":{"pin":9,"pulse_us":1000}},{"servo":{"pin":4,"pulse_us":2100}}]}]},{"actions":[{"cylinder":{"action":"retract","duration_ms":6000}}]},{"actions":[{"servo":{"pin":6,"pulse_us":600,"duration_ms":1000}},{"delay":{"ms":5000}}]}]},"3":{"stage":3,"repeat_from":2,"steps":[{"actions":[{"parallel":[{"stepper":{"node":4,"position":7100,"rpm":3000,"acc":254}},{"stepper":{"node":5,"position":-7100,"rpm":3000,"acc":254}},{"servo":{"pin":3,"pulse_us":1630}}]}]},{"actions":[{"parallel":[{"stepper":{"node":4,"position":0,"rpm":50,"acc":1}},{"stepper":{"node":5,"position":0,"rpm":50,"acc":1}},{"servo":{"pin":3,"pulse_us":1700}}]}]},{"actions":[{"parallel":[{"stepper":{"node":4,"position":4000,"rpm":320,"acc":2}},{"stepper":{"node":5,"position":-4000,"rpm":320,"acc":2}},{"servo":{"pin":3,"pulse_us":1620}}]}]},{"actions":[{"parallel":[{"stepper":{"node":4,"position":7100,"rpm":320,"acc":2}},{"stepper":{"node":5,"position":-7100,"rpm":320,"acc":2}},{"servo":{"pin":3,"pulse_us":1620}}]}]}]},"4":{"stage":4,"repeat_from":1,"steps":[{"actions":[{"parallel":[{"stepper":{"node":6,"position":-700,"rpm":320,"acc":0}},{"servo":{"pin":2,"pulse_us":1500}},{"servo":{"pin":5,"pulse_us":1400}}]}]},{"actions":[{"servo":{"pin":2,"pulse_us":1120}}]},{"actions":[{"parallel":[{"stepper":{"node":6,"position":8099,"rpm":320,"acc":5}},{"servo":{"pin":5,"pulse_us":1550,"duration_ms":2000}}]}]},{"actions":[{"parallel":[{"stepper":{"node":6,"position":-604,"rpm":1500,"acc":0}},{"servo":{"pin":5,"pulse_us":1400}}]},{"parallel":[{"stepper":{"node":6,"position":2600,"rpm":320,"acc":0}},{"servo":{"pin":5,"pulse_us":1800}}]}]},{"actions":[{"servo":{"pin":2,"pulse_us":1500}}]},{"actions":[{"parallel":[{"stepper":{"node":6,"position":0,"rpm":320,"acc":0}},{"servo":{"pin":2,"pulse_us":1150}},{"servo":{"pin":5,"pulse_us":2100}}]}]}]},"5":{"stage":5,"repeat_from":1,"steps":[{"actions":[{"delay":{"ms":0}}]}]}};
const stage4FirstPosition=-700;
profileModels['6']={stage:6,repeat_from:1,steps:[{actions:[{servo:{pin:2,pulse_us:1611,duration_ms:1000}}]}]};
profileModels['1']=profileModels['1_2'];profileModels['4']=model;
const fixture={connected:false,firmware:null,status_age:null,logs:[]};
const requests=[];
const context=vm.createContext({fixture,document:{getElementById:id=>elements.get(id),createElement:tag=>new Element(tag),querySelector:()=>eyebrow,querySelectorAll:selector=>selector==='[data-view]'?navs:selector==='[data-command]'?normals:[]},
  Option:class extends Element {constructor(text,value){super('option');this.textContent=text;this.value=value;}},
  fetch:async(path,options)=>{const body=options?.body?JSON.parse(options.body):undefined;requests.push({path,body});const requested=path.match(/profile=([^&]+)/)?.[1]??"4";if(path==='/api/apply'){const phase=fixture.apply_phase??'ready';Object.assign(fixture.firmware,{stage:phase==='ready'?body.document.stage:fixture.firmware.stage,prop:phase==='ready'?(fixture.apply_prop??(body.document.stage===1?(body.document.prop||2):0)):(fixture.firmware.prop||0),phase,step:0,branch:0,completed:0,pending_stage:phase==='ready'?0:body.document.stage,previous_available:false});}if(path==='/api/command'&&body.command==='stage'&&['busy','return_wait','home_wait'].includes(fixture.firmware.phase))fixture.firmware.pending_stage=body.stage;let pwmResult=null,motionResult=null;if(path==='/api/command'&&body.command==='chassis.motion'){assert.equal(body.profile_id,fixture.firmware.chassis.motion.profile_id);for(const key of ['ramp_enabled','accel','decel','wheelbase_mm','track_mm','icr_x_mm','icr_left_y_mm','icr_right_y_mm'])fixture.firmware.chassis.motion[key]=key==='ramp_enabled'?!!body[key]:body[key];motionResult=JSON.parse(JSON.stringify(fixture.firmware));if(fixture.motionReplyWait)await fixture.motionReplyWait();}if(path==='/api/command'&&body.command==='pwm.set'){Object.assign(fixture.firmware.pwm,{max_value:body.max_value,step_interval_ms:body.step_interval_ms});if(!fixture.firmware.pwm.strobe_enabled)Object.assign(fixture.firmware.pwm,{value:Math.min(fixture.firmware.pwm.value,body.max_value),rc_controlled:false});pwmResult=JSON.parse(JSON.stringify(fixture.firmware));if(fixture.pwmReplyWait)await fixture.pwmReplyWait();}return {ok:true,json:async()=>path.startsWith('/api/init')?{token:'test',config:{document:profileModels[requested],hash:'hash'+requested,profile:requested}}:path==='/api/ports'?{ports:[]}:path==='/api/apply'?{result:{stage:fixture.firmware.stage,prop:fixture.firmware.prop,phase:fixture.apply_phase??'ready',step:0,completed:0}}:path==='/api/command'?{result:body.command==='pwm.set'?pwmResult:body.command==='chassis.motion'?motionResult:{}}:fixture};},setInterval(){},console});
new vm.Script(source).runInContext(context);
await new Promise(resolve=>setTimeout(resolve,20));
assert.equal(elements.get('steps').children.length,1);
assert.equal(elements.get('applyConfig').disabled,true);
assert.equal(elements.get('globalStop').disabled,true);
assert.equal(elements.get('pwmApply').disabled,true);assert.equal(elements.get('pwmReset').disabled,true);assert.equal(elements.get('pwmCurrentValue').textContent,'--');assert.ok(html.indexOf('id="pwmPanel"')<html.indexOf('id="controlView"'));
assert.equal(elements.get('servoCards').children.length,9);
assert.equal(elements.has('sendServo3'),true);
assert.match(html,/<option value="6"[^>]*>ID\s*6/);
assert.doesNotMatch(html,/<option value="7"[^>]*>ID\s*7/);
assert.match(html,/切關先復位目前關卡/);assert.doesNotMatch(html,/各關切入先 HOME|再啟動目標關 HOME|自動執行 STEP1 預備/);
assert.match(html,/<option value="4"[^>]*>ID\s*4/);
assert.match(html,/<option value="5"[^>]*>ID\s*5/);
for(const pin of [4,6,9,10,11,12]){elements.get('servoSlider'+pin).value='123.4';elements.get('servoSlider'+pin).listeners.input();assert.equal(elements.get('servoAngle'+pin).value,'123.4');}
elements.get('servoAngle5').value='45.5';elements.get('servoAngle5').listeners.input();assert.equal(elements.get('servoSlider5').value,'45.5');
assert.equal(requests.filter(r=>r.path==='/api/command').length,0);
const fields=elements.get('steps').querySelectorAll('input');
fields[0].value='180';fields[0].listeners.input();assert.equal(fields[2].value,2500);assert.match(elements.get('yamlPreview').textContent,/pulse_us: 2500/);
fields[2].value='500';fields[2].listeners.input();assert.equal(fields[0].value,0);
Object.assign(fixture,{connected:true,status_age:0,applied_hash:null,firmware:{firmware_revision:"bench-test-v7",bench_test_mode:true,motor_online_mask:7,motor_offline_mask:0,prop:0,pending_prop:0,prop_complete:false,stage_ids:[1,2,3,4,5,6],stage1_prop_ids:[1,2,3,4,5],stage:2,stage3_enabled:true,mode:'normal',phase:'ready',previous_available:true,uploading:false,steps:4,step:2,branch:1,completed:1,pulse2:1150,pulse3:2250,pulse5:2100,pulse4:1300,pulse6:1400,pulse9:1800,pulse10:1500,pulse11:1500,pulse12:1500,servo_pins:[2,3,4,5,6,9,10,11,12],position4:0,position5:0,position6:0,position4_base:0,position5_base:0,stage2_trim_pulses:0,stage2_trim_target4:0,stage2_trim_target5:0,stage2_trim_allowed:false,stage2_trim_moving:false,position_known_mask:0,pwm:{available:true,pin:14,value:0,max_value:55,step_interval_ms:5,default_max_value:55,default_step_interval_ms:5,min_step_interval_ms:1,max_step_interval_ms:1000,strobe_enabled:false,rc_controlled:false,strobe_interval_ms:50,strobe_value:255,rc_channel:13},cylinder:{available:true,action:0,active:false,duration_ms:0,remaining_ms:0,in1_pin:17,in2_pin:16,in1_high:false,in2_high:false,reference_action:2,reference_known:true,fully_retracted:true},chassis:{state:'running',reason:'centered',emergency:false,wheels:[10,11,12,13].map(node=>({node,bus:node<12?1:2,heartbeat_age_ms:25,error:0,axis_state:8,velocity:1.5}))},rc:{connected:true,age_ms:10,stage:2,channels:Array(16).fill(1500),stage1_trim_us:0,stage2_trim_pulses:0}}});
vm.runInContext('state=fixture;renderState()',context);
assert.equal(elements.get('applyConfig').disabled,false);
assert.equal(elements.get('currentStep').textContent,'STEP2-1');
assert.match(elements.get('stepDetail').textContent,/第 2 關/);
assert.match(elements.get('servoValue4').textContent,/1300/);
assert.match(elements.get('motorValue').textContent,/尚未確認位置/);
fixture.firmware.position_known_mask=4;fixture.firmware.position6=stage4FirstPosition;vm.runInContext('renderState()',context);assert.ok(elements.get('motorValue').textContent.includes(String(stage4FirstPosition)));
assert.equal(elements.get('motorId').textContent,6);assert.ok(elements.get('motorValue').textContent.includes(String(stage4FirstPosition)));
if(normal('prev')){assert.equal(normal('prev').disabled,false);fixture.firmware.previous_available=false;vm.runInContext('renderState()',context);assert.equal(normal('prev').disabled,true);}
assert.equal(normal('next').disabled,false);
if(elements.has('chassisState')){
  assert.match(elements.get('chassisState').textContent,/RUNNING/);assert.equal(elements.get('wheelStatus').children.length,4);assert.equal(elements.get('wheelStatus').children[2].children[1].textContent,'CAN2');assert.match(elements.get('rcState').textContent,/最近確認連線 10 ms/);
  fixture.firmware.rc.connected=false;fixture.firmware.rc.age_ms=500;vm.runInContext('renderState()',context);assert.match(elements.get('rcState').textContent,/斷線經過 500 ms/);
}
// The mechanical active stage may remain old while accepted pending selection controls steering.
Object.assign(fixture.firmware,{stage:2,pending_stage:1,phase:'return_wait'});
fixture.firmware.chassis.direction_stage=1;
vm.runInContext('renderState()',context);
assert.match(elements.get('rcState').textContent,/底盤有效方向：第 1 關/);
assert.match(elements.get('phaseDetail').textContent,/第 2 關.*復位中/);
fixture.firmware.chassis.direction_stage=4;
vm.runInContext('renderState()',context);
assert.match(elements.get('rcState').textContent,/底盤有效方向：第 4 關/);
delete fixture.firmware.chassis.direction_stage;
vm.runInContext('renderState()',context);
assert.match(elements.get('rcState').textContent,/底盤有效方向：第 1 關/);
fixture.firmware.pending_stage=0;
vm.runInContext('renderState()',context);
assert.match(elements.get('rcState').textContent,/底盤有效方向：第 2 關/);
fixture.firmware.chassis.direction_stage=0;
vm.runInContext('renderState()',context);
assert.match(elements.get('rcState').textContent,/底盤有效方向：第 2 關/);
delete fixture.firmware.chassis.direction_stage;
Object.assign(fixture.firmware,{phase:'ready',pending_stage:0});
fixture.firmware.phase='busy';vm.runInContext('renderState()',context);assert.equal(normal('next').disabled,true);assert.equal(elements.get('selectStage').disabled,false);
for(const stage of [1,2,3,4]){
  Object.assign(fixture.firmware,{stage,phase:'home_wait',step:0,branch:0,completed:0,previous_available:false,pending_stage:0});
  vm.runInContext('renderState()',context);
  assert.equal(elements.get('currentStep').textContent,'STEP0 / HOME');
  assert.match(elements.get('phase').textContent,/歸零待機/);
  for(const cmd of ['next','prev','run','home'])assert.equal(normal(cmd).disabled,true);
  assert.equal(elements.get('selectStage').disabled,false);
  assert.equal(elements.get('applyConfig').disabled,true);
  assert.equal(elements.get('debugMode').disabled,true);
  assert.equal(elements.get('globalStop').disabled,false);
}
fixture.firmware.pending_stage=2;vm.runInContext('renderState()',context);assert.match(elements.get('planDetail').textContent,/手動HOME完成後再復位切關.*第 2 關/);
fixture.firmware.mode='debug';vm.runInContext('renderState()',context);for(const pin of [2,3,4,5,6,9])assert.equal(elements.get('sendServo'+pin).disabled,true);assert.equal(elements.get('sendMotor').disabled,true);
fixture.firmware.mode='normal';
for(const stage of [1,2,3,4]){
  const target=stage%4+1;
  Object.assign(fixture.firmware,{stage,phase:'return_wait',step:0,branch:0,completed:0,pending_stage:target,previous_available:false,uploading:false});
  vm.runInContext('renderState()',context);
  assert.equal(elements.get('currentStep').textContent,'STEP0 / 復位');
  assert.match(elements.get('phase').textContent,/目前關卡復位中/);
  assert.ok(elements.get('phaseDetail').textContent.includes('第 '+stage+' 關')&&elements.get('phaseDetail').textContent.includes('復位中'));
  assert.ok(elements.get('planDetail').textContent.includes('第 '+stage+' 關')&&elements.get('planDetail').textContent.includes('復位中 → 第 '+target+' 關'));
  for(const cmd of ['next','prev','run','home'])assert.equal(normal(cmd).disabled,true);
  assert.equal(elements.get('selectStage').disabled,false);
  assert.equal(elements.get('applyConfig').disabled,true);
  assert.equal(elements.get('debugMode').disabled,true);
  assert.equal(elements.get('globalStop').disabled,false);
  assert.equal(fixture.firmware.stage,stage);
}
fixture.firmware.mode='debug';vm.runInContext('renderState()',context);for(const pin of [2,3,4,5,6,9])assert.equal(elements.get('sendServo'+pin).disabled,true);assert.equal(elements.get('sendMotor').disabled,true);
Object.assign(fixture.firmware,{stage:2,phase:'ready',mode:'normal',pending_stage:0});vm.runInContext('renderState()',context);assert.equal(elements.get('currentStep').textContent,'STEP0');assert.equal(normal('next').disabled,false);assert.equal(normal('prev').disabled,true);
fixture.firmware.phase='ready';fixture.firmware.mode='debug';vm.runInContext('renderState()',context);assert.equal(elements.get('applyConfig').disabled,true);
for(const pin of [2,5,4,6,9])assert.equal(elements.get('sendServo'+pin).disabled,false);
fixture.firmware.stage=3;vm.runInContext('renderState()',context);
assert.match(elements.get('modeHint').textContent,/三顆步進.*九顆伺服/);
for(const pin of [2,3,4,5,6,9])assert.equal(elements.get('sendServo'+pin).disabled,false);
fixture.firmware.stage=2;vm.runInContext('renderState()',context);
// Loading the Stage3 file edits only the PC copy and keeps active Stage2 Debug.
elements.get('profile').value='3';await elements.get('profile').listeners.change();
assert.equal(elements.get('steps').children.length,4);assert.match(elements.get('yamlPreview').textContent,/stage: 3\nrepeat_from: 2/);for(const node of [4,5]){const a=thirdModel.steps[0].actions[0].parallel.find(a=>a.stepper?.node===node).stepper;assert.ok(elements.get('yamlPreview').textContent.includes('node: '+node+', position: '+a.position));}assert.ok(elements.get('yamlPreview').textContent.includes('pin: 3, pulse_us: '+thirdModel.steps[0].actions[0].parallel.find(a=>a.servo?.pin===3).servo.pulse_us));assert.equal(elements.get('addStep').disabled,false);assert.equal(elements.get('repeatFrom').disabled,false);assert.equal(elements.get('sendServo3').disabled,false);
assert.equal(vm.runInContext('editorValid().repeat_from',context),2);assert.equal(vm.runInContext('defaults.stepper().node',context),4);
assert.deepEqual(JSON.parse(vm.runInContext('JSON.stringify(defaults.parallel())',context)),[{stepper:{node:4,position:0,rpm:320,acc:0}},{stepper:{node:5,position:0,rpm:320,acc:0}},{servo:{pin:3,pulse_us:2250}}]);
// ID4/5 allow FE while explicitly disabling GoHome91, with per-axis known bits.
fixture.firmware.position_known_mask=7;fixture.firmware.position4=-4200;fixture.firmware.position5=4200;
for(const [node,position] of [[4,-4200],[5,4200]]){elements.get('motorNode').value=String(node);elements.get('motorNode').listeners.change();assert.equal(elements.get('sendMotor').disabled,false);assert.equal(elements.get('motorGoHome').disabled,true);assert.match(elements.get('motorHint').textContent,/原位建立 FE 零點/);assert.ok(elements.get('motorValue').textContent.includes(String(position)));}
elements.get('motorNode').value='6';elements.get('motorNode').listeners.change();assert.equal(elements.get('motorGoHome').disabled,false);
// Bench UI keeps mechanism control independent of a locked/unpowered chassis.
Object.assign(fixture.firmware,{mode:'normal',phase:'ready',bench_test_mode:true,motor_online_mask:4,motor_offline_mask:3,position_known_mask:4});
fixture.firmware.chassis.state='locked';fixture.firmware.chassis.healthy=false;fixture.firmware.chassis.reason='ODrive error or heartbeat timeout';
vm.runInContext('renderState()',context);
assert.match(elements.get('benchState').textContent,/BENCH.*檯面獨立測試/);
assert.match(elements.get('benchHint').textContent,/底盤可保持 LOCKED.*仍可操作機構/);
assert.match(elements.get('motorAvailability').textContent,/ID4：離線.*ID5：離線.*ID6：開機已確認.*非即時心跳/);
assert.match(elements.get('chassisState').textContent,/LOCKED/);
assert.equal(normal('next').disabled,false);assert.equal(elements.get('selectStage').disabled,false);
fixture.firmware.mode='debug';elements.get('motorNode').value='4';vm.runInContext('renderState()',context);
assert.equal(elements.get('motorNode').disabled,false);
for(const id of ['motorPosition','motorRpm','motorAcc','sendMotor','motorZero','motorGoHome'])assert.equal(elements.get(id).disabled,true);
assert.match(elements.get('motorValue').textContent,/ID4 離線.*本次開機略過/);
assert.match(elements.get('motorHint').textContent,/接線後重啟/);
assert.equal(elements.get('sendServo3').disabled,false);assert.equal(elements.get('extendCylinder').disabled,false);
elements.get('motorNode').value='6';fixture.firmware.position_known_mask=0;vm.runInContext('renderState()',context);
assert.equal(elements.get('sendMotor').disabled,true);assert.equal(elements.get('motorZero').disabled,true);
assert.equal(elements.get('motorGoHome').disabled,false);assert.match(elements.get('motorHint').textContent,/尚未建立零點.*GoHome/);
fixture.firmware.motor_online_mask=0;fixture.firmware.motor_offline_mask=7;vm.runInContext('renderState()',context);
assert.equal(elements.get('motorGoHome').disabled,true);
// Legacy firmware and competition mode do not inherit optional Bench guards.
fixture.firmware.bench_test_mode=false;vm.runInContext('renderState()',context);
assert.match(elements.get('benchState').textContent,/COMPETITION/);assert.match(elements.get('benchHint').textContent,/RUNNING.*四輪正常/);
assert.equal(elements.get('sendMotor').disabled,false);
fixture.firmware.bench_test_mode=true;delete fixture.firmware.motor_online_mask;delete fixture.firmware.motor_offline_mask;
vm.runInContext('renderState()',context);assert.equal(elements.get('sendMotor').disabled,false);
assert.match(elements.get('motorAvailability').textContent,/連線狀態未回報/);
Object.assign(fixture.firmware,{motor_online_mask:0,motor_offline_mask:0,position_known_mask:0});
vm.runInContext('renderState()',context);assert.match(elements.get('motorAvailability').textContent,/尚未完成開機通訊/);assert.doesNotMatch(elements.get('motorAvailability').textContent,/離線/);assert.equal(elements.get('sendMotor').disabled,true);
Object.assign(fixture.firmware,{motor_online_mask:7,motor_offline_mask:0,position_known_mask:7});
fixture.status_age=5;vm.runInContext('renderState()',context);assert.match(elements.get('benchState').textContent,/尚未取得/);
assert.equal(elements.get('sendMotor').disabled,true);assert.equal(elements.get('globalStop').disabled,false);
fixture.status_age=0;fixture.firmware.chassis.state='running';vm.runInContext('renderState()',context);
fixture.status_age=5;vm.runInContext('renderState()',context);assert.equal(elements.get('sendServo4').disabled,true);assert.equal(elements.get('globalStop').disabled,false);
fixture.firmware.phase='fault';vm.runInContext('renderState()',context);assert.match(elements.get('servoValue4').textContent,/STOP 前指令/);fixture.firmware.phase='ready';
fixture.status_age=0;fixture.firmware.mode='normal';vm.runInContext('renderState()',context);
for(const stage of stages){elements.get('profile').value=String(stage);await elements.get('profile').listeners.change();assert.equal(vm.runInContext('documentModel.stage',context),stage);assert.equal(vm.runInContext('defaults.stepper().node',context),stage===3?4:6);}
assert.equal(requests.filter(r=>r.path==='/api/apply'||r.path==='/api/command').length,0);
elements.get('profile').value='2';await elements.get('profile').listeners.change();
assert.equal(elements.get('steps').children.length,4);assert.match(elements.get('yamlPreview').textContent,/repeat_from: 2/);assert.equal(vm.runInContext('defaults.cylinder().action',context),'extend');assert.ok(elements.get('yamlPreview').textContent.includes('cylinder: {action: extend, duration_ms: 6000}'));const ramp=secondModel.steps[3].actions[0].servo;assert.ok(elements.get('yamlPreview').textContent.includes('pin: 6, pulse_us: '+ramp.pulse_us+', duration_ms: '+ramp.duration_ms));
const cylinderFields=elements.get('steps').children[0].querySelectorAll('input');
cylinderFields[0].value='9000';cylinderFields[0].listeners.input();assert.equal(Number(cylinderFields[1].value),9);
cylinderFields[1].value='1.5';cylinderFields[1].listeners.input();assert.equal(Number(cylinderFields[0].value),1500);
cylinderFields[0].value='6000';cylinderFields[0].listeners.input();
await elements.get('applyConfig').listeners.click();
assert.deepEqual(requests.find(r=>r.path==='/api/apply').body.document,secondModel);
assert.match(elements.get('message').textContent,/原關卡復位.*RAM 交換已完成.*Ready（STEP0）.*NEXT 從 STEP1/);
assert.equal(normal('next').disabled,false);assert.equal(elements.get('currentStep').textContent,'STEP0');
assert.equal(elements.get('selectStage').disabled,false);
// A premature old-version return_wait response cannot mark a different editor snapshot applied.
const acceptedSnapshot=vm.runInContext('appliedSnapshot',context);
vm.runInContext('documentModel.steps[0].actions[0].parallel[0].cylinder.duration_ms+=1;changed()',context);
fixture.apply_phase='return_wait';
await elements.get('applyConfig').listeners.click();
assert.match(elements.get('message').textContent,/尚未確認原關卡復位與RAM交換完成/);
assert.equal(vm.runInContext('appliedSnapshot',context),acceptedSnapshot);
assert.equal(normal('next').disabled,true);assert.equal(elements.get('currentStep').textContent,'STEP0 / 復位');
fixture.apply_phase='ready';fixture.firmware.phase='ready';fixture.firmware.pending_stage=0;
elements.get('profile').value='3';await elements.get('profile').listeners.change();await elements.get('applyConfig').listeners.click();
assert.deepEqual(requests.filter(r=>r.path==='/api/apply').at(-1).body.document,thirdModel);
assert.match(elements.get('message').textContent,/RAM 交換已完成.*Ready（STEP0）.*NEXT 從 STEP1/);
assert.equal(normal('next').disabled,false);
elements.get('profile').value='2';await elements.get('profile').listeners.change();
// Nine independent editor identities and first-stage finite workflows.
for(const key of Object.keys(profileModels).filter(k=>k!=='1')){
  elements.get('profile').value=key;await elements.get('profile').listeners.change();
  assert.equal(vm.runInContext('documentModel.stage',context),profileModels[key].stage);
  assert.equal(vm.runInContext('documentModel.prop??0',context),profileModels[key].prop??0);
  assert.equal(elements.get('steps').children.length,profileModels[key].steps.length);
  assert.deepEqual(JSON.parse(vm.runInContext('JSON.stringify(editorValid())',context)),profileModels[key]);
  if(key.startsWith('1_'))assert.match(elements.get('yamlPreview').textContent,new RegExp('prop: '+key.slice(2)));
}
// Lift and release retain separate NEXT boundaries in the editor and RAM payload.
for(const key of ['1_1','1_4','1_5']){
  elements.get('profile').value=key;await elements.get('profile').listeners.change();
  const liftIndex=key==='1_1'?1:2;
  const expectedLift=key==='1_1'?{node:6,position:6000,rpm:500,acc:150}:{node:6,position:4000,rpm:1500,acc:255};
  const edited=JSON.parse(vm.runInContext('JSON.stringify(editorValid())',context));
  assert.deepEqual(edited.steps[liftIndex].actions,[{stepper:expectedLift}]);
  assert.ok(elements.get('yamlPreview').textContent.includes('position: '+expectedLift.position+', rpm: '+expectedLift.rpm+', acc: '+expectedLift.acc));
  if(key!=='1_1'){
    assert.equal(elements.get('steps').children.length,4);
    assert.deepEqual(edited.steps[3].actions,[{servo:{pin:2,pulse_us:1667}}]);
    assert.ok(elements.get('yamlPreview').textContent.includes('pin: 2, pulse_us: 1667'));
  }
  await elements.get('applyConfig').listeners.click();
  assert.deepEqual(requests.filter(r=>r.path==='/api/apply').at(-1).body.document,profileModels[key]);
}
for(const prop of [1,2,3,4,5]){
  Object.assign(fixture.firmware,{stage:1,prop,phase:'ready',mode:'normal',steps:[2,3,2,4,4][prop-1],step:0,completed:0,prop_complete:false});
  elements.get('activeStage').value='1_'+prop;elements.get('activeStage').listeners.change();
  assert.equal(elements.get('triggerProp').disabled,false);await elements.get('triggerProp').listeners.click();
  assert.ok(requests.some(r=>r.path==='/api/command'&&r.body.command==='prop'&&r.body.prop===prop));
  Object.assign(fixture.firmware,{step:fixture.firmware.steps,completed:fixture.firmware.steps,prop_complete:true});vm.runInContext('renderState()',context);
  assert.equal(normal('next').disabled,false);assert.equal(elements.get('triggerProp').disabled,false);
  assert.match(elements.get('propHint').textContent,/流程已完成/);
}
fixture.firmware.phase='busy';vm.runInContext('renderState()',context);assert.equal(elements.get('triggerProp').disabled,true);assert.equal(elements.get('globalStop').disabled,false);
fixture.firmware.phase='ready';elements.get('profile').value='1_1';await elements.get('profile').listeners.change();await elements.get('applyConfig').listeners.click();
assert.equal(requests.filter(r=>r.path==='/api/apply').at(-1).body.document.prop,1);
const propSnapshot=vm.runInContext('appliedSnapshot',context);fixture.apply_prop=2;
vm.runInContext('documentModel.steps[0].actions[0].parallel[0].stepper.position+=1;changed()',context);await elements.get('applyConfig').listeners.click();
assert.match(elements.get('message').textContent,/尚未確認/);assert.equal(vm.runInContext('appliedSnapshot',context),propSnapshot);
delete fixture.apply_prop;Object.assign(fixture.firmware,{stage:2,prop:0,phase:'ready',mode:'normal',prop_complete:false});
elements.get('profile').value='2';await elements.get('profile').listeners.change();
// Independent controls retain their own pin, angle and interpolation duration.
fixture.firmware.mode='debug';vm.runInContext('state=fixture;renderState()',context);
for(const pin of [4,6,9]){elements.get('servoAngle'+pin).value='117';elements.get('servoDuration'+pin).value='700';const card=elements.get('servoCards').children[[2,3,4][[4,6,9].indexOf(pin)]];card.listeners.submit({preventDefault(){}});await new Promise(resolve=>setTimeout(resolve,5));assert.ok(requests.some(r=>r.path==='/api/command'&&r.body.command==='debug.servo'&&r.body.pin===pin&&r.body.angle===117&&r.body.duration_ms===700));}
// New debug pins keep independent sliders, angle requests, duration and live pulse reports.
for(const [pin,angle,duration,pulse] of [[10,0,0,500],[11,90.1,1750,1501],[12,180,60000,2500]]){
  elements.get('servoAngle'+pin).value=String(angle);elements.get('servoAngle'+pin).listeners.input();
  elements.get('servoDuration'+pin).value=String(duration);
  const index=JSON.parse(vm.runInContext('JSON.stringify(servoPins)',context)).indexOf(pin);
  elements.get('servoCards').children[index].listeners.submit({preventDefault(){}});
  await new Promise(resolve=>setTimeout(resolve,5));
  assert.ok(requests.some(r=>r.body?.command==='debug.servo'&&r.body.pin===pin&&r.body.angle===angle&&r.body.duration_ms===duration));
  fixture.firmware['pulse'+pin]=pulse;vm.runInContext('renderState()',context);
  assert.match(elements.get('servoValue'+pin).textContent,new RegExp(pulse+' µs'));
}
assert.equal(elements.get('servoAngle10').value,'0');assert.equal(elements.get('servoAngle11').value,'90.1');assert.equal(elements.get('servoAngle12').value,'180');
// Six-pin firmware keeps the original controls but rejects unsupported new pins before HTTP.
fixture.firmware.servo_pins=[2,3,4,5,6,9];vm.runInContext('renderState()',context);
assert.equal(elements.get('sendServo5').disabled,false);
for(const pin of [10,11,12]){
  for(const prefix of ['servoSlider','servoAngle','servoDuration','sendServo'])assert.equal(elements.get(prefix+pin).disabled,true);
  assert.match(elements.get('servoValue'+pin).textContent,/韌體未支援/);
  const before=requests.filter(r=>r.body?.command==='debug.servo').length;
  const index=JSON.parse(vm.runInContext('JSON.stringify(servoPins)',context)).indexOf(pin);
  elements.get('servoCards').children[index].listeners.submit({preventDefault(){}});
  assert.equal(requests.filter(r=>r.body?.command==='debug.servo').length,before);
}
// Explicit pin fields are the compatibility fallback when servo_pins is absent.
delete fixture.firmware.servo_pins;
for(const pin of [2,3,4,5,6,9,10,11,12])fixture.firmware['pin'+pin]=pin;
vm.runInContext('renderState()',context);
for(const pin of [10,11,12])assert.equal(elements.get('sendServo'+pin).disabled,false);
delete fixture.firmware.pin12;vm.runInContext('renderState()',context);assert.equal(elements.get('sendServo12').disabled,true);
fixture.firmware.servo_pins=[2,3,4,5,6,9,10,11,12];vm.runInContext('renderState()',context);
// Cylinder controls use timed GPIO commands; local Debug STOP bypasses perform busy.
fixture.firmware.mode='debug';fixture.firmware.phase='ready';vm.runInContext('state=fixture;renderState()',context);
elements.get('cylinderDuration').value='7250';elements.get('cylinderDuration').listeners.input();assert.match(elements.get('cylinderSeconds').textContent,/7.25 秒/);
await elements.get('extendCylinder').listeners.click();assert.ok(requests.some(r=>r.body?.command==='debug.cylinder'&&r.body.action==='extend'&&r.body.duration_ms===7250));
Object.assign(fixture.firmware.cylinder,{action:1,active:true,remaining_ms:3250,in1_high:true,in2_high:false});fixture.firmware.phase='busy';vm.runInContext('busy=true;renderState()',context);
assert.equal(elements.get('stopCylinder').disabled,false);assert.equal(elements.get('extendCylinder').disabled,true);assert.match(elements.get('cylinderState').textContent,/伸出/);assert.match(elements.get('cylinderDetail').textContent,/GPIO17: HIGH.*GPIO16: LOW/);assert.match(elements.get('cylinderTiming').textContent,/3.250 秒.*無到位回授/);
await elements.get('stopCylinder').listeners.click();assert.ok(requests.some(r=>r.body?.command==='debug.cylinder'&&r.body.action==='stop'&&r.body.duration_ms===0));vm.runInContext('busy=false',context);
Object.assign(fixture.firmware.cylinder,{action:0,active:false,remaining_ms:0,in1_high:false,in2_high:false});fixture.firmware.phase='ready';fixture.firmware.mode='normal';
// Boot home is initialization, not a fault. STOP stays available; NEXT waits for Ready.
const bootCommandCount=requests.filter(r=>r.path==='/api/command').length;
Object.assign(fixture.firmware,{stage:4,phase:'booting',mode:'normal',stopped:false});
Object.assign(fixture.firmware.cylinder,{action:2,active:true,duration_ms:5000,remaining_ms:4500,in1_high:false,in2_high:true,fully_retracted:false});
vm.runInContext('renderState()',context);
assert.equal(normal('next').disabled,true);assert.equal(elements.get('globalStop').disabled,false);
assert.match(elements.get('phaseDetail').textContent,/開機.*縮回.*MKS/);
assert.match(elements.get('cylinderTiming').textContent,/完整縮回計時尚未完成.*無到位回授/);
// A completed partial retract is still not a full-home confirmation.
Object.assign(fixture.firmware.cylinder,{action:0,active:false,duration_ms:1800,remaining_ms:0,in1_high:false,in2_high:false,reference_action:2,reference_known:true,fully_retracted:false});
fixture.firmware.phase='ready';vm.runInContext('renderState()',context);
assert.match(elements.get('cylinderTiming').textContent,/完整縮回計時尚未完成/);
fixture.firmware.cylinder.fully_retracted=true;vm.runInContext('renderState()',context);
assert.match(elements.get('cylinderTiming').textContent,/完整縮回計時已完成/);
delete fixture.firmware.cylinder.fully_retracted;vm.runInContext('renderState()',context);
assert.match(elements.get('cylinderTiming').textContent,/韌體未回報完整縮回計時/);
fixture.firmware.cylinder.fully_retracted=true;
assert.equal(normal('next').disabled,false);
// Rendering a latched STOP never sends a new retract or other motion command.
Object.assign(fixture.firmware,{phase:'fault',stopped:true});
Object.assign(fixture.firmware.cylinder,{action:0,active:false,fully_retracted:false,in1_high:false,in2_high:false});
vm.runInContext('renderState();renderState()',context);
assert.equal(normal('next').disabled,true);
assert.equal(requests.filter(r=>r.path==='/api/command').length,bootCommandCount);
Object.assign(fixture.firmware,{phase:'ready',stopped:false});
// All stages accept selection while a segment is busy; old stage remains active.
for(const stage of [1,2,3,4]){
  Object.assign(fixture.firmware,{stage,mode:'normal',phase:'busy',step:2,pending_stage:0});
  vm.runInContext('renderState()',context);
  assert.equal(elements.get('selectStage').disabled,false);assert.equal(normal('next').disabled,true);
  const target=stage%4+1;
  elements.get('activeStage').value=String(target);await elements.get('selectStage').listeners.click();
  assert.equal(requests.filter(r=>r.path==='/api/command').at(-1).body.stage,target);
  assert.equal(fixture.firmware.stage,stage);assert.equal(fixture.firmware.pending_stage,target);
  assert.match(elements.get('planDetail').textContent,/本段完成後再復位切關/);
}
// Latest return-wait request updates only the destination until the old stage is reset.
Object.assign(fixture.firmware,{stage:2,phase:'return_wait',pending_stage:3,step:0});
for(const target of [1,4]){
  elements.get('activeStage').value=String(target);await elements.get('selectStage').listeners.click();
  assert.equal(fixture.firmware.stage,2);assert.equal(fixture.firmware.pending_stage,target);
  assert.ok(elements.get('planDetail').textContent.includes('第 2 關復位中 → 第 '+target+' 關'));
}
fixture.firmware.phase='ready';fixture.firmware.pending_stage=0;
// Third-stage Park0 reports both completed FE targets at zero without an incoming HOME.
Object.assign(fixture.firmware,{stage:3,phase:'return_wait',pending_stage:4,position4:0,position5:0,pulse3:2250,position_known_mask:7});
vm.runInContext('renderState()',context);assert.equal(normal('next').disabled,true);
Object.assign(fixture.firmware,{stage:4,phase:'ready',pending_stage:0});vm.runInContext('renderState()',context);
for(const [node,position] of [[4,0],[5,0]]){
  elements.get('motorNode').value=String(node);elements.get('motorNode').listeners.change();
  assert.ok(elements.get('motorValue').textContent.includes(String(position)));
}
assert.match(elements.get('servoValue3').textContent,/2250/);
fixture.firmware.pulse9=1100;vm.runInContext('renderState()',context);assert.match(elements.get('servoValue9').textContent,/1100 µs/);
assert.equal(requests.some(r=>r.path==='/api/command'&&r.body.command==='home'),false);
elements.get('motorNode').value='6';elements.get('motorNode').listeners.change();
// NEXT remains blocked during Park0 and is available after 3 -> 1/2, then reentry to 3.
for(const target of [1,2]){
  Object.assign(fixture.firmware,{stage:3,mode:'normal',phase:'return_wait',pending_stage:target,step:0,completed:0,previous_available:false,position4:0,position5:0,pulse3:2250});
  vm.runInContext('renderState()',context);assert.equal(normal('next').disabled,true);
  Object.assign(fixture.firmware,{stage:target,phase:'ready',pending_stage:0});vm.runInContext('renderState()',context);
  assert.equal(normal('next').disabled,false);const before=requests.filter(r=>r.path==='/api/command'&&r.body.command==='next').length;
  await normal('next').listeners.click();assert.equal(requests.filter(r=>r.path==='/api/command'&&r.body.command==='next').length,before+1);
  Object.assign(fixture.firmware,{stage:3,phase:'ready',step:0,completed:0});vm.runInContext('renderState()',context);
  assert.equal(elements.get('currentStep').textContent,'STEP0');assert.equal(normal('next').disabled,false);
  await normal('next').listeners.click();assert.equal(requests.filter(r=>r.path==='/api/command'&&r.body.command==='next').length,before+2);
}
// Invalid editor steps remain rejected instead of silently reaching RAM.
fixture.firmware.mode='normal';elements.get('addStep').listeners.click();assert.equal(elements.get('steps').children.length,5);vm.runInContext('documentModel.steps[4].actions=[]',context);assert.throws(()=>vm.runInContext('editorValid()',context),/至少一個 Action/);
// Pin14 lighting is independent of Normal/Debug and mechanism motion or latched STOP.
const pwmCommands=()=>requests.filter(r=>r.path==='/api/command'&&r.body.command==='pwm.set');
const pwmControls=['pwmMaxSlider','pwmMaxValue','pwmIntervalSlider','pwmStepInterval','pwmApply','pwmReset'];
const pwmDefaults={available:true,pin:14,value:12,max_value:55,step_interval_ms:5,default_max_value:55,default_step_interval_ms:5,min_step_interval_ms:1,max_step_interval_ms:1000,strobe_enabled:false,rc_controlled:false,strobe_interval_ms:50,strobe_value:255,rc_channel:13};
fixture.connected=true;fixture.status_age=0;fixture.firmware.pwm={...pwmDefaults};
vm.runInContext('busy=false;pwmDirty=false;pwmPending=null;state=fixture;renderState()',context);
assert.equal(elements.get('pwmMaxValue').value,'55');assert.equal(elements.get('pwmStepInterval').value,'5');
assert.equal(elements.get('pwmCurrentValue').textContent,'12 / 255');assert.match(elements.get('pwmCycle').textContent,/0.550 秒/);
for(const mode of ['normal','debug'])for(const phase of ['ready','busy','home_wait','return_wait','booting','fault']){
  Object.assign(fixture.firmware,{mode,phase,stopped:phase==='fault'});vm.runInContext('renderState()',context);
  for(const id of pwmControls)assert.equal(elements.get(id).disabled,false);
}
assert.match(elements.get('pwmHint').textContent,/STOP 後皆可調整/);
// Local slider/number edits never send commands, and polling updates telemetry only.
const beforeEdits=pwmCommands().length;
elements.get('pwmMaxSlider').value='200';elements.get('pwmMaxSlider').listeners.input();
elements.get('pwmStepInterval').value='10';elements.get('pwmStepInterval').listeners.input();
assert.equal(elements.get('pwmMaxValue').value,'200');assert.equal(elements.get('pwmIntervalSlider').value,'10');
Object.assign(fixture.firmware.pwm,{value:20,max_value:40,step_interval_ms:7});vm.runInContext('renderState()',context);
assert.equal(elements.get('pwmMaxValue').value,'200');assert.equal(elements.get('pwmStepInterval').value,'10');
assert.equal(elements.get('pwmCurrentValue').textContent,'20 / 255');assert.match(elements.get('pwmCurrentConfig').textContent,/峰值 40.*每階 7 ms/);
assert.match(elements.get('pwmCycle').textContent,/4.000 秒/);assert.match(elements.get('pwmHint').textContent,/未套用.*滑桿只編輯/);assert.equal(pwmCommands().length,beforeEdits);
// Browser operations still block competing lighting requests without changing the draft.
vm.runInContext('busy=true;renderState()',context);for(const id of pwmControls)assert.equal(elements.get(id).disabled,true);
await elements.get('pwmApply').listeners.click();assert.equal(pwmCommands().length,beforeEdits);
vm.runInContext('busy=false;renderState()',context);
await elements.get('pwmApply').listeners.click();
assert.deepEqual(pwmCommands().at(-1).body,{command:'pwm.set',max_value:200,step_interval_ms:10});
assert.equal(vm.runInContext('pwmDirty',context),false);assert.equal(elements.get('pwmMaxValue').value,'200');assert.match(elements.get('message').textContent,/RAM.*重啟/);
// Reset uses the reported firmware defaults, then the real 55/5 defaults.
Object.assign(fixture.firmware.pwm,{default_max_value:42,default_step_interval_ms:9});
await elements.get('pwmReset').listeners.click();assert.deepEqual(pwmCommands().at(-1).body,{command:'pwm.set',max_value:42,step_interval_ms:9});
Object.assign(fixture.firmware.pwm,{default_max_value:55,default_step_interval_ms:5});
await elements.get('pwmReset').listeners.click();assert.deepEqual(pwmCommands().at(-1).body,{command:'pwm.set',max_value:55,step_interval_ms:5});
assert.equal(elements.get('pwmMaxSlider').value,'55');assert.equal(elements.get('pwmStepInterval').value,'5');
// Both allowed boundaries reach RAM; zero peak turns the light off.
for(const [peak,interval] of [[0,1000],[255,1]]){
  elements.get('pwmMaxValue').value=String(peak);elements.get('pwmMaxValue').listeners.input();
  elements.get('pwmIntervalSlider').value=String(interval);elements.get('pwmIntervalSlider').listeners.input();
  await elements.get('pwmApply').listeners.click();assert.deepEqual(pwmCommands().at(-1).body,{command:'pwm.set',max_value:peak,step_interval_ms:interval});
  if(peak===0){assert.equal(elements.get('pwmState').textContent,'關燈');assert.match(elements.get('pwmCycle').textContent,/套用後關燈/);assert.equal(elements.get('pwmCurrentValue').textContent,'0 / 255');}
}
assert.match(elements.get('pwmCycle').textContent,/0.510 秒/);
// Invalid, stale, missing-capability and disconnected requests cannot be transmitted.
for(const [peak,interval] of [['256','5'],['-1','5'],['1.5','5'],['55','0'],['55','1001'],['55','1.5'],['','5'],['55','']]){
  elements.get('pwmMaxValue').value=peak;elements.get('pwmMaxValue').listeners.input();
  elements.get('pwmStepInterval').value=interval;elements.get('pwmStepInterval').listeners.input();
  const before=pwmCommands().length;await elements.get('pwmApply').listeners.click();assert.equal(pwmCommands().length,before);assert.match(elements.get('message').textContent,/整數|欄位/);
}
const beforeUnsupported=pwmCommands().length;
delete fixture.firmware.pwm;vm.runInContext('renderState()',context);
for(const id of pwmControls)assert.equal(elements.get(id).disabled,true);assert.match(elements.get('pwmHint').textContent,/先燒錄新版韌體/);
await elements.get('pwmApply').listeners.click();assert.equal(pwmCommands().length,beforeUnsupported);
fixture.firmware.pwm={...pwmDefaults,available:false};vm.runInContext('renderState()',context);assert.equal(elements.get('pwmApply').disabled,true);
fixture.firmware.pwm={...pwmDefaults};fixture.status_age=5;vm.runInContext('renderState()',context);
assert.equal(elements.get('pwmApply').disabled,true);assert.match(elements.get('pwmHint').textContent,/狀態未更新/);
await elements.get('pwmReset').listeners.click();assert.equal(pwmCommands().length,beforeUnsupported);
fixture.connected=false;fixture.status_age=0;vm.runInContext('renderState()',context);assert.equal(elements.get('pwmApply').disabled,true);assert.equal(elements.get('pwmCurrentValue').textContent,'--');
assert.match(elements.get('pwmHint').textContent,/連接 USB/);await elements.get('pwmReset').listeners.click();assert.equal(pwmCommands().length,beforeUnsupported);
// A fresh boot refreshes clean fields from compiled defaults without sending settings.
fixture.connected=true;fixture.firmware.pwm={...pwmDefaults,value:0};vm.runInContext('pwmDirty=false;pwmPending=null;renderState()',context);
assert.equal(elements.get('pwmMaxValue').value,'55');assert.equal(elements.get('pwmStepInterval').value,'5');assert.equal(pwmCommands().length,beforeUnsupported);
// A late accepted PWM response must survive status loss and await fresh confirmation.
const delayedFirmware=fixture.firmware;let releasePwmReply;
fixture.pwmReplyWait=()=>new Promise(resolve=>{releasePwmReply=resolve;});
elements.get('pwmMaxValue').value='80';elements.get('pwmMaxValue').listeners.input();
elements.get('pwmStepInterval').value='11';elements.get('pwmStepInterval').listeners.input();
const delayedApply=elements.get('pwmApply').listeners.click();
await new Promise(resolve=>setTimeout(resolve,0));assert.equal(typeof releasePwmReply,'function');
fixture.connected=false;fixture.status_age=null;fixture.firmware=null;vm.runInContext('state={};renderState()',context);
releasePwmReply();await delayedApply;delete fixture.pwmReplyWait;
assert.equal(vm.runInContext('state.firmware',context),null);
assert.deepEqual(JSON.parse(vm.runInContext('JSON.stringify(pwmPending)',context)),{max_value:80,step_interval_ms:11});
assert.equal(vm.runInContext('pwmDirty',context),true);assert.equal(elements.get('pwmApply').disabled,true);
assert.doesNotMatch(elements.get('message').textContent,/Cannot set|undefined/);
fixture.connected=true;fixture.status_age=0;fixture.firmware=delayedFirmware;vm.runInContext('state=fixture;renderState()',context);
assert.equal(vm.runInContext('pwmPending',context),null);assert.equal(vm.runInContext('pwmDirty',context),false);
assert.equal(elements.get('pwmMaxValue').value,'80');assert.equal(elements.get('pwmStepInterval').value,'11');
// CH13 owns a distinct 0/255 strobe. Editing saved breathing settings cannot reduce its output.
const beforeStrobe=pwmCommands().length;
fixture.firmware.pwm={...pwmDefaults,value:255,strobe_enabled:true,rc_controlled:true};
Object.assign(fixture.firmware,{mode:'normal',phase:'ready',stopped:false});
vm.runInContext('pwmDirty=false;pwmPending=null;renderState()',context);
assert.equal(elements.get('pwmState').textContent,'CH13 / 爆閃中');
assert.equal(elements.get('pwmCurrentValue').textContent,'255 / 255');
assert.match(elements.get('pwmCurrentConfig').textContent,/0 ↔ 255.*50 ms.*呼吸備用/);
assert.match(elements.get('pwmCycle').textContent,/0.100 秒.*亮 50 ms.*滅 50 ms/);
assert.match(elements.get('pwmHint').textContent,/爆閃優先.*只保存.*不改變爆閃/);
assert.equal(elements.get('pwmApply').textContent,'保存呼吸設定 / RAM');
assert.equal(pwmCommands().length,beforeStrobe);
elements.get('pwmMaxValue').value='0';elements.get('pwmMaxValue').listeners.input();
elements.get('pwmStepInterval').value='100';elements.get('pwmStepInterval').listeners.input();
await elements.get('pwmApply').listeners.click();
assert.deepEqual(pwmCommands().at(-1).body,{command:'pwm.set',max_value:0,step_interval_ms:100});
assert.equal(fixture.firmware.pwm.strobe_enabled,true);
assert.equal(elements.get('pwmCurrentValue').textContent,'255 / 255');
assert.equal(elements.get('pwmMaxValue').value,'0');
assert.match(elements.get('message').textContent,/呼吸參數.*RAM.*CH13 爆閃持續/);
// A low strobe phase remains a strobe; latched motor STOP does not disable this independent light.
Object.assign(fixture.firmware.pwm,{value:0});
Object.assign(fixture.firmware,{phase:'fault',stopped:true});vm.runInContext('renderState()',context);
assert.equal(elements.get('pwmState').textContent,'CH13 / 爆閃中');
assert.equal(elements.get('pwmApply').disabled,false);
assert.match(elements.get('pwmCycle').textContent,/爆閃週期/);
// RC OFF is distinct from breathing. A new manual apply may take over until the next RC event.
Object.assign(fixture.firmware.pwm,{strobe_enabled:false,rc_controlled:true,value:0,max_value:55});
vm.runInContext('renderState()',context);
assert.equal(elements.get('pwmState').textContent,'CH13 / 關燈');
assert.match(elements.get('pwmHint').textContent,/CH13 已關燈.*手動呼吸.*重新接管/);
assert.equal(elements.get('pwmApply').textContent,'套用燈光設定 / RAM');
await elements.get('pwmApply').listeners.click();
assert.equal(fixture.firmware.pwm.rc_controlled,false);
assert.equal(elements.get('pwmState').textContent,'PWM / 呼吸中');
fixture.status_age=5;Object.assign(fixture.firmware.pwm,{strobe_enabled:true,rc_controlled:true,value:255});vm.runInContext('renderState()',context);
assert.equal(elements.get('pwmState').textContent,'尚未取得即時狀態');
assert.equal(elements.get('pwmApply').disabled,true);
fixture.status_age=0;fixture.firmware.pwm={...pwmDefaults};vm.runInContext('pwmDirty=false;pwmPending=null;renderState()',context);
// CH13 remains a Stage1 chicken control; Stage5 is still available through Web/Serial without RC label.
assert.match(html,/第一關 · 夾雞 \(CH13\)/);
assert.doesNotMatch(html,/第五關[^<]*CH13/);
assert.match(html,/第五關 · 備用關卡（Web／Serial）/);
assert.match(source,/第五關 · 備用關卡（Web／Serial）/);
console.log(`PASS frontend model: Pin14 runtime brightness/interval RAM, dirty-poll protection, reported defaults, boundaries, independent STOP/motion and capability/stale guards / Bench locked-chassis independent controls / optional offline and reference masks / online ID6 explicit-home recovery / 9 profiles, nine servo controls, Stage2 timed-cylinder eleven-field RAM payload / outgoing-stage return before activation / all-stage busy and latest pending selection / Ready-only RAM / manual HOME guards, ID6 position-known mask, paired Stage3 / selective GoHome / file-only profile load, mode/stale/previous guards`);

// ICR/ramp GUI: edit without motion, preserve drafts, require stationary fresh firmware.
fixture.connected=true;fixture.status_age=0;fixture.firmware.stopped=false;fixture.firmware.chassis.emergency=false;
fixture.firmware.chassis.motion={available:true,editable:true,profiles:true,profile_id:0,ramp_enabled:true,accel:90,decel:180,wheelbase_mm:260,track_mm:320,icr_mode:"directional",icr_x_mm:0,icr_left_y_mm:-500,icr_right_y_mm:500,target:[0,0,0,0]};
vm.runInContext('state=fixture;motionDirty=false;motionPending=null;renderState()',context);
assert.equal(elements.get('motionAccel').value,'90');assert.equal(elements.get('motionWheelbase').value,'260');
const motionCommands=()=>requests.filter(r=>r.path==='/api/command'&&r.body.command==='chassis.motion');
elements.get('motionX').value='130';elements.get('motionX').listeners.input();
elements.get('motionLeftY').value='-350';elements.get('motionLeftY').listeners.input();
elements.get('motionRightY').value='700';elements.get('motionRightY').listeners.input();
vm.runInContext('renderState()',context);assert.equal(elements.get('motionX').value,'130');assert.equal(motionCommands().length,0);
await elements.get('motionApply').listeners.click();
assert.deepEqual(motionCommands().at(-1).body,{command:'chassis.motion',ramp_enabled:1,accel:90,decel:180,wheelbase_mm:260,track_mm:320,icr_x_mm:130,icr_left_y_mm:-350,icr_right_y_mm:700,profile_id:0});
assert.equal(vm.runInContext('motionDirty',context),false);
fixture.firmware.chassis.motion.editable=false;vm.runInContext('renderState()',context);
assert.equal(elements.get('motionApply').disabled,true);const beforeMotion=motionCommands().length;
await elements.get('motionApply').listeners.click();assert.equal(motionCommands().length,beforeMotion);
fixture.firmware.chassis.motion.editable=true;fixture.status_age=5;vm.runInContext('renderState()',context);
assert.equal(elements.get('motionApply').disabled,true);
fixture.status_age=0;vm.runInContext('renderState()',context);
elements.get('motionX').value='5001';elements.get('motionX').listeners.input();await elements.get('motionApply').listeners.click();
assert.equal(motionCommands().length,beforeMotion);
elements.get('motionCenter').listeners.click();assert.equal(elements.get('motionX').value,'0');assert.equal(elements.get('motionLeftY').value,'0');assert.equal(elements.get('motionRightY').value,'0');
await elements.get('motionApply').listeners.click();assert.equal(motionCommands().at(-1).body.icr_x_mm,0);
// A bank switch discards the other bank's unsent draft and loads only its own report.
const normalMotion=JSON.parse(JSON.stringify(fixture.firmware.chassis.motion));
elements.get('motionX').value='999';elements.get('motionX').listeners.input();
fixture.firmware.chassis.motion={...normalMotion,profile_id:1,icr_x_mm:120,icr_left_y_mm:180,icr_right_y_mm:-180};
fixture.firmware.rc.procession_mode=true;fixture.firmware.chassis.direction_stage=2;vm.runInContext('renderState()',context);
assert.equal(elements.get('motionProfile').textContent,'舊版繞境');
assert.equal(elements.get('motionApply').textContent,'套用舊版繞境 / RAM');
assert.equal(elements.get('motionX').value,'120');assert.equal(elements.get('motionLeftY').value,'180');assert.equal(elements.get('motionRightY').value,'-180');
assert.equal(vm.runInContext('motionDirty',context),false);assert.match(elements.get('motionHint').textContent,/編輯已清除/);
assert.match(elements.get('rcState').textContent,/舊版繞境/);
elements.get('motionAccel').value='95';elements.get('motionAccel').listeners.input();await elements.get('motionApply').listeners.click();
assert.deepEqual(motionCommands().at(-1).body,{command:'chassis.motion',ramp_enabled:1,accel:95,decel:180,wheelbase_mm:260,track_mm:320,icr_x_mm:120,icr_left_y_mm:180,icr_right_y_mm:-180,profile_id:1});
// A delayed normal-bank acknowledgement cannot overwrite a newer procession bank.
fixture.firmware.chassis.motion={...normalMotion};fixture.firmware.rc.procession_mode=false;vm.runInContext('renderState()',context);
assert.equal(elements.get('motionProfile').textContent,'一般底盤');
let releaseMotionReply;fixture.motionReplyWait=()=>new Promise(resolve=>{releaseMotionReply=resolve;});
elements.get('motionX').value='321';elements.get('motionX').listeners.input();
const delayedMotionApply=elements.get('motionApply').listeners.click();
await new Promise(resolve=>setTimeout(resolve,0));assert.equal(typeof releaseMotionReply,'function');
fixture.firmware.chassis.motion={...normalMotion,profile_id:1,icr_x_mm:120,icr_left_y_mm:180,icr_right_y_mm:-180};
fixture.firmware.rc.procession_mode=true;vm.runInContext('renderState()',context);
releaseMotionReply();await delayedMotionApply;delete fixture.motionReplyWait;
assert.equal(fixture.firmware.chassis.motion.profile_id,1);assert.equal(fixture.firmware.chassis.motion.icr_x_mm,120);
assert.equal(elements.get('motionX').value,'120');assert.equal(vm.runInContext('motionPending',context),null);
assert.equal(vm.runInContext('motionDirty',context),false);
// An old bank's pending acknowledgement cannot confirm or clear this bank's draft.
elements.get('motionX').value='122';elements.get('motionX').listeners.input();
vm.runInContext('motionPending={...motionDraft(),profile_id:0};renderState()',context);
assert.equal(vm.runInContext('motionDirty',context),true);assert.equal(elements.get('motionX').value,'122');
vm.runInContext('motionPending=null',context);
delete fixture.firmware.chassis.motion.profiles;vm.runInContext('renderState()',context);assert.equal(elements.get('motionApply').disabled,true);
fixture.firmware.chassis.motion.profiles=true;fixture.firmware.chassis.motion.profile_id=2;vm.runInContext('renderState()',context);assert.equal(elements.get('motionApply').disabled,true);
fixture.firmware.chassis.motion.profile_id=1;
delete fixture.firmware.chassis.motion.icr_mode;vm.runInContext('renderState()',context);assert.equal(elements.get('motionApply').disabled,true);
delete fixture.firmware.chassis.motion;vm.runInContext('renderState()',context);assert.equal(elements.get('motionApply').disabled,true);
// Action editor offers the three added pins and preserves each pulse/duration independently.
vm.runInContext('documentModel={stage:4,repeat_from:1,steps:[{actions:[{servo:{pin:10,pulse_us:500,duration_ms:0}},{servo:{pin:11,pulse_us:1500,duration_ms:1750}},{servo:{pin:12,pulse_us:2500,duration_ms:60000}}]}]};renderEditor()',context);
const descendants=(parent,tag)=>parent.children.flatMap(c=>[...(c.tagName===tag?[c]:[]),...descendants(c,tag)]);
const pinMenus=descendants(elements.get('steps'),'select').filter(e=>e.children.some(o=>String(o.value)==='12'));
assert.equal(pinMenus.length,3);
for(const pin of [10,11,12])assert.ok(pinMenus.every(e=>e.children.some(o=>Number(o.value)===pin)));
for(const pin of [10,11,12])assert.match(elements.get('yamlPreview').textContent,new RegExp('pin: '+pin+', pulse_us:'));
pinMenus[0].value='12';pinMenus[0].listeners.change();
assert.equal(vm.runInContext('documentModel.steps[0].actions[0].servo.pin',context),12);
assert.equal(vm.runInContext('documentModel.steps[0].actions[0].servo.pulse_us',context),500);
assert.equal(vm.runInContext('documentModel.steps[0].actions[0].servo.duration_ms',context),0);
console.log('PASS nine-servo GUI: Pin10/11/12 independent controls, ranges, duration payloads, live pulses, declared-pin capability guards and RAM editor choices');
console.log('PASS motion GUI: independent normal/CH14 banks, exact signed procession parameters, bank-bound drafts and delayed acknowledgement, ICR XY, geometry, ramp rates, stationary/fresh/capability checks and exact nine-field RAM payload');


// CH4 trim is read-only, reports command/base/effective delta, and never invents old-firmware zeros.
assert.ok(html.indexOf('id="stage1TrimPanel"')<html.indexOf('id="controlView"'));
assert.match(elements.get('stage1TrimState').textContent,/韌體未提供/);
assert.equal(elements.get('stage1TrimApplied').textContent,'--');
const beforeTrimCommands=requests.filter(r=>r.path==='/api/command').length;
fixture.connected=true;fixture.status_age=0;
Object.assign(fixture.firmware,{stage:1,mode:'normal',phase:'busy',stopped:false,pulse2_base:1611,pulse2:1761,stage1_trim_us:150,stage1_trim_allowed:true});
Object.assign(fixture.firmware.rc,{connected:true,stage1_mode:true,channels:Array(16).fill(1500),stage1_trim_us:150});
fixture.firmware.rc.channels[3]=2000;
vm.runInContext('state=fixture;renderState()',context);
assert.equal(elements.get('stage1TrimRaw').textContent,'2000 µs');
assert.equal(elements.get('stage1TrimRequested').textContent,'+150 µs');
assert.equal(elements.get('stage1TrimApplied').textContent,'+150 µs');
assert.equal(elements.get('stage1TrimState').textContent,'第一關微調啟用');
assert.match(elements.get('stage1TrimCommand').textContent,/基準 1611 µs.*\+150 µs.*1761 µs/);
assert.match(elements.get('stage1TrimHint').textContent,/步驟運行與完成保持/);
fixture.firmware.phase='ready';fixture.firmware.rc.channels[3]=1000;
Object.assign(fixture.firmware,{pulse2:1461,stage1_trim_us:-150});
fixture.firmware.rc.stage1_trim_us=-150;
vm.runInContext('renderState()',context);
assert.equal(elements.get('stage1TrimApplied').textContent,'-150 µs');
assert.equal(elements.get('stage1TrimState').textContent,'第一關微調啟用');
// Clamp reports the effective delta, separately from the full knob request.
Object.assign(fixture.firmware,{pulse2_base:2450,pulse2:2500,stage1_trim_us:50});
fixture.firmware.rc.stage1_trim_us=150;
vm.runInContext('renderState()',context);
assert.equal(elements.get('stage1TrimRequested').textContent,'+150 µs');
assert.equal(elements.get('stage1TrimApplied').textContent,'+50 µs');
assert.match(elements.get('stage1TrimCommand').textContent,/2450 µs.*\+50 µs.*2500 µs/);
for(const stage of [1,2,3,4,5]){
  Object.assign(fixture.firmware,{stage,stage1_trim_allowed:false,stage1_trim_us:0,pulse2_base:1150,pulse2:1150});
  fixture.firmware.rc.stage1_trim_us=0;
  vm.runInContext('renderState()',context);
  assert.equal(elements.get('stage1TrimState').textContent,'微調停用');
  assert.equal(elements.get('stage1TrimApplied').textContent,'0 µs');
  assert.match(elements.get('stage1TrimHint').textContent,/退出／HOME.*遙控斷線/);
}
// STOP displays the historical command without presenting it as current hardware output.
Object.assign(fixture.firmware,{stage:1,stopped:true,phase:'fault',pulse2_base:1611,pulse2:1761,stage1_trim_allowed:false});
vm.runInContext('renderState()',context);
assert.equal(elements.get('stage1TrimState').textContent,'STOP / 微調停用');
assert.match(elements.get('stage1TrimCommand').textContent,/STOP 前最後輸出指令 1761 µs/);
assert.match(elements.get('stage1TrimHint').textContent,/不代表急停後仍有 PWM 輸出/);
fixture.firmware.stopped=false;fixture.firmware.phase='ready';
for(const [name,value] of [['pulse2_base',NaN],['pulse2',Infinity],['stage1_trim_us',null],['stage1_trim_allowed',undefined]]){
  const original=fixture.firmware[name];fixture.firmware[name]=value;
  vm.runInContext('renderState()',context);
  assert.match(elements.get('stage1TrimState').textContent,/韌體未提供/);
  assert.equal(elements.get('stage1TrimApplied').textContent,'--');
  assert.doesNotMatch(elements.get('stage1TrimCommand').textContent,/NaN|Infinity|null|undefined/);
  fixture.firmware[name]=original;
}
fixture.status_age=5;vm.runInContext('renderState()',context);
assert.equal(elements.get('stage1TrimRaw').textContent,'--');
assert.equal(elements.get('stage1TrimRequested').textContent,'--');
assert.equal(elements.get('stage1TrimApplied').textContent,'--');
assert.match(elements.get('stage1TrimState').textContent,/尚未取得/);
fixture.status_age=0;delete fixture.firmware.stage1_trim_us;
vm.runInContext('renderState()',context);
assert.match(elements.get('stage1TrimState').textContent,/韌體未提供/);
assert.equal(elements.get('stage1TrimApplied').textContent,'--');
assert.equal(requests.filter(r=>r.path==='/api/command').length,beforeTrimCommands);
console.log('PASS CH4 Stage1 trim GUI: readonly raw/request/applied/base/output telemetry, running/hold, clamp, disabled/STOP last-command labels, stale/unsupported/nonfinite guards and no commands');

// Stage4 ID6 height trim is independently reported and waits for MKS completion.
assert.ok(html.indexOf('id="stage4TrimPanel"')<html.indexOf('id="controlView"'));
assert.match(elements.get('stage4TrimState').textContent,/韌體未提供/);
assert.equal(elements.get('stage4TrimApplied').textContent,'--');
const beforeStage4TrimCommands=requests.filter(r=>r.path==='/api/command').length;
fixture.connected=true;fixture.status_age=0;
Object.assign(fixture.firmware,{stage:4,mode:'normal',phase:'ready',stopped:false,pending_stage:0,
  position6_base:2600,position6:2800,stage4_trim_pulses:200,stage4_trim_target:2800,stage4_trim_allowed:true,stage4_trim_moving:false});
Object.assign(fixture.firmware.rc,{connected:true,stage1_mode:false,stage4_trim_pulses:200});
fixture.firmware.rc.channels[3]=2000;
vm.runInContext('state=fixture;renderState()',context);
assert.equal(elements.get('stage4TrimRaw').textContent,'2000 µs');
assert.equal(elements.get('stage4TrimRequested').textContent,'+200 脈衝');
assert.equal(elements.get('stage4TrimApplied').textContent,'+200 脈衝');
assert.equal(elements.get('stage4TrimState').textContent,'擲筊高度微調啟用');
assert.match(elements.get('stage4TrimCommand').textContent,/基準 2600 脈衝.*最後完成位置 2800 脈衝.*已送目標 2800 脈衝/);
assert.match(elements.get('stage4TrimHint').textContent,/±200.*收到 ACK 後更新/);
Object.assign(fixture.firmware,{position6:2400,stage4_trim_pulses:-200,stage4_trim_target:2400});
fixture.firmware.rc.stage4_trim_pulses=-200;fixture.firmware.rc.channels[3]=1000;
vm.runInContext('renderState()',context);
assert.equal(elements.get('stage4TrimRaw').textContent,'1000 µs');
assert.equal(elements.get('stage4TrimRequested').textContent,'-200 脈衝');
assert.equal(elements.get('stage4TrimApplied').textContent,'-200 脈衝');
// The commanded target may change before ACK; keep the last completed position and delta.
Object.assign(fixture.firmware,{phase:'busy',stage4_trim_moving:true,stage4_trim_target:2800});
fixture.firmware.rc.stage4_trim_pulses=200;
vm.runInContext('renderState()',context);
assert.equal(elements.get('stage4TrimState').textContent,'高度微調中 / 等待 ACK');
assert.equal(elements.get('stage4TrimRequested').textContent,'+200 脈衝');
assert.equal(elements.get('stage4TrimApplied').textContent,'-200 脈衝');
assert.match(elements.get('stage4TrimCommand').textContent,/最後完成位置 2400 脈衝.*已送目標 2800 脈衝/);
assert.match(elements.get('stage4TrimHint').textContent,/收到完成回報後才接受 NEXT/);
// Losing eligibility stops new targets while an already-sent trim still awaits ACK.
fixture.firmware.stage4_trim_allowed=false;fixture.firmware.rc.connected=false;
fixture.firmware.rc.stage4_trim_pulses=0;
vm.runInContext('renderState()',context);
assert.equal(elements.get('stage4TrimState').textContent,'高度微調中 / 等待 ACK');
assert.equal(elements.get('stage4TrimApplied').textContent,'-200 脈衝');
assert.match(elements.get('stage4TrimHint').textContent,/停用後只等待既有指令完成，不追旋鈕/);
Object.assign(fixture.firmware,{stage4_trim_allowed:true,stage4_trim_moving:false});
fixture.firmware.rc.connected=true;
vm.runInContext('renderState()',context);
assert.equal(elements.get('stage4TrimState').textContent,'等待 STEP 完成');
assert.match(elements.get('stage4TrimHint').textContent,/STEP4 的彈射與回降會完整執行/);
Object.assign(fixture.firmware,{phase:'ready',stage4_trim_allowed:false});
vm.runInContext('renderState()',context);
assert.match(elements.get('stage4TrimState').textContent,/停用.*等待 NEXT/);
assert.match(elements.get('stage4TrimHint').textContent,/HOME.*切關.*遙控斷線.*Debug.*NEXT/);
Object.assign(fixture.firmware,{phase:'fault',stopped:true,stage4_trim_moving:true});
vm.runInContext('renderState()',context);
assert.equal(elements.get('stage4TrimState').textContent,'STOP / 微調停用');
assert.match(elements.get('stage4TrimCommand').textContent,/STOP 前最後完成位置 2400.*STOP 前最後目標 2800/);
assert.match(elements.get('stage4TrimHint').textContent,/不能代表急停後仍在運轉/);
Object.assign(fixture.firmware,{phase:'ready',stopped:false,stage4_trim_moving:false});
for(const [name,value] of [['position6_base',NaN],['position6',Infinity],['stage4_trim_pulses',null],
  ['stage4_trim_target',undefined],['stage4_trim_allowed',undefined],['stage4_trim_moving',undefined]]){
  const original=fixture.firmware[name];fixture.firmware[name]=value;
  vm.runInContext('renderState()',context);
  assert.match(elements.get('stage4TrimState').textContent,/韌體未提供/);
  for(const suffix of ['Raw','Requested','Applied'])assert.equal(elements.get('stage4Trim'+suffix).textContent,'--');
  assert.doesNotMatch(elements.get('stage4TrimCommand').textContent,/NaN|Infinity|null|undefined|2600|2400|2800/);
  fixture.firmware[name]=original;
}
const savedStage4Knob=fixture.firmware.rc.stage4_trim_pulses;
delete fixture.firmware.rc.stage4_trim_pulses;
vm.runInContext('renderState()',context);
assert.match(elements.get('stage4TrimState').textContent,/韌體未提供/);
assert.equal(elements.get('stage4TrimApplied').textContent,'--');
fixture.firmware.rc.stage4_trim_pulses=savedStage4Knob;
fixture.status_age=5;vm.runInContext('renderState()',context);
for(const suffix of ['Raw','Requested','Applied'])assert.equal(elements.get('stage4Trim'+suffix).textContent,'--');
assert.match(elements.get('stage4TrimState').textContent,/尚未取得/);
assert.doesNotMatch(elements.get('stage4TrimCommand').textContent,/2600|2400|2800/);
fixture.status_age=0;fixture.connected=false;vm.runInContext('renderState()',context);
assert.equal(elements.get('stage4TrimApplied').textContent,'--');
assert.equal(requests.filter(r=>r.path==='/api/command').length,beforeStage4TrimCommands);
console.log('PASS CH4 Stage4 height trim GUI: ±200 pulse request/completed delta, base/ACK position/commanded target, busy/ineligible pending ACK, STEP wait, STOP history, stale/legacy/nonfinite guards and no commands');

// Fruit shares the read-only ID6 panel, but final STEP auto-HOME must keep trim disabled.
assert.match(html,/CH4 \/ STAGE4 \+ STAGE6 · ID6/);
assert.match(html,/擲筊／水果高度旋鈕微調/);
const beforeFruitTrimCommands=requests.filter(r=>r.path==='/api/command').length;
fixture.connected=true;fixture.status_age=0;
Object.assign(fixture.firmware,{stage:6,mode:'normal',phase:'ready',stopped:false,pending_stage:0,step:3,
  position6_base:5500,position6:7500,stage4_trim_pulses:2000,stage4_trim_target:7500,
  stage4_trim_allowed:true,stage4_trim_moving:false,stage1_trim_allowed:false,stage1_trim_us:0});
Object.assign(fixture.firmware.rc,{connected:true,stage1_mode:false,stage4_trim_pulses:2000,stage1_trim_us:0});
fixture.firmware.rc.channels[3]=2000;
vm.runInContext('renderState()',context);
assert.equal(elements.get('stage4TrimState').textContent,'水果高度微調啟用');
assert.equal(elements.get('stage4TrimRaw').textContent,'2000 µs');
assert.equal(elements.get('stage4TrimRequested').textContent,'+2000 脈衝');
assert.equal(elements.get('stage4TrimApplied').textContent,'+2000 脈衝');
assert.match(elements.get('stage4TrimCommand').textContent,/基準 5500 脈衝.*最後完成位置 7500 脈衝.*已送目標 7500 脈衝/);
assert.match(elements.get('stage4TrimHint').textContent,/第六關水果.*±2000.*收到 ACK 後更新/);
assert.equal(elements.get('stage1TrimState').textContent,'微調停用');
Object.assign(fixture.firmware,{phase:'busy',stage4_trim_moving:true,stage4_trim_target:3500});
fixture.firmware.rc.stage4_trim_pulses=-2000;
vm.runInContext('renderState()',context);
assert.equal(elements.get('stage4TrimState').textContent,'高度微調中 / 等待 ACK');
assert.equal(elements.get('stage4TrimRequested').textContent,'-2000 脈衝');
assert.equal(elements.get('stage4TrimApplied').textContent,'+2000 脈衝');
assert.match(elements.get('stage4TrimCommand').textContent,/最後完成位置 7500 脈衝.*已送目標 3500 脈衝/);
assert.equal(normal('next').disabled,true);
assert.match(elements.get('stage4TrimHint').textContent,/收到完成回報後才接受 NEXT/);
Object.assign(fixture.firmware,{stage4_trim_moving:false});
vm.runInContext('renderState()',context);
assert.equal(elements.get('stage4TrimState').textContent,'等待 STEP 完成');
assert.match(elements.get('stage4TrimHint').textContent,/水果正式 STEP.*第5段.*自動 HOME/);
// The firmware clears both offsets during auto-HOME even while CH4 remains at its high end.
Object.assign(fixture.firmware,{phase:'home_wait',step:0,stage4_trim_allowed:false,stage4_trim_pulses:0,
  position6_base:0,stage4_trim_target:0});
fixture.firmware.rc.stage4_trim_pulses=0;
vm.runInContext('renderState()',context);
assert.equal(elements.get('stage4TrimState').textContent,'水果回 HOME / 微調停用');
assert.equal(elements.get('stage4TrimRaw').textContent,'2000 µs');
assert.equal(elements.get('stage4TrimRequested').textContent,'0 脈衝');
assert.equal(elements.get('stage4TrimApplied').textContent,'0 脈衝');
assert.match(elements.get('stage4TrimHint').textContent,/第5段完成後自動 HOME.*旋鈕未置中.*下一輪正式 STEP 完成/);
assert.equal(normal('next').disabled,true);
Object.assign(fixture.firmware,{phase:'ready',position6:0});
vm.runInContext('renderState()',context);
assert.equal(elements.get('stage4TrimState').textContent,'微調停用 / 等待 NEXT');
assert.equal(elements.get('stage4TrimRequested').textContent,'0 脈衝');
assert.equal(elements.get('stage4TrimApplied').textContent,'0 脈衝');
assert.equal(normal('next').disabled,false);
assert.equal(requests.filter(r=>r.path==='/api/command').length,beforeFruitTrimCommands);
console.log('PASS CH4 fruit Stage6 height trim GUI: shared ±2000 pulse panel, ACK interlock, STEP wait, final auto-HOME disabling despite held knob, Stage1 separation and no commands');

assert.equal(elements.has("imuYaw"),false);
assert.equal(elements.has("imuState"),false);

assert.match(html,/value="6">第六關/);
Object.assign(fixture.firmware,{stage:6,phase:'ready',mode:'normal',pending_stage:0,uploading:false});
fixture.status_age=0;fixture.connected=true;vm.runInContext('renderState()',context);
elements.get('activeStage').value='6';
await elements.get('selectStage').listeners.click();
assert.ok(requests.some(r=>r.path==='/api/command'&&r.body.command==='stage'&&r.body.stage===6));
console.log('PASS fruit Stage6 GUI load, selection and editor identity');

// Fruit FE hard stops constrain actual motion, while CH4 still reports the full requested offset.
Object.assign(fixture.firmware,{stage:6,mode:'normal',phase:'ready',stopped:false,uploading:false,
  motor_online_mask:7,motor_offline_mask:0,position_known_mask:7,motor_recovery_mask:0,
  stage_recovery_required:false,mechanism_warning:'',id6_position_max:10500,fruit_position_min:-1400,fruit_position_max:10500,
  previous_available:true,stage4_trim_allowed:true,stage4_trim_moving:false,
  position6_base:9000,position6:10500,stage4_trim_pulses:1500,stage4_trim_target:10500});
Object.assign(fixture.firmware.rc,{stage4_trim_pulses:2000});
vm.runInContext('state=fixture;renderState()',context);
assert.equal(elements.get('stage4TrimRequested').textContent,'+2000 脈衝');
assert.equal(elements.get('stage4TrimApplied').textContent,'+1500 脈衝');
assert.match(elements.get('stage4TrimHint').textContent,/-1400～10500.*遇上下限即截斷.*要求與已完成偏移可能不同/);
assert.match(elements.get('stage4TrimCommand').textContent,/基準 9000.*最後完成位置 10500.*已送目標 10500/);
Object.assign(fixture.firmware,{position6_base:-1000,position6:-1400,stage4_trim_pulses:-400,stage4_trim_target:-1400});
fixture.firmware.rc.stage4_trim_pulses=-2000;vm.runInContext('renderState()',context);
assert.equal(elements.get('stage4TrimRequested').textContent,'-2000 脈衝');
assert.equal(elements.get('stage4TrimApplied').textContent,'-400 脈衝');
assert.match(elements.get('stage4TrimCommand').textContent,/最後完成位置 -1400.*已送目標 -1400/);
fixture.firmware.mode='debug';elements.get('motorNode').value='6';vm.runInContext('renderState()',context);
assert.equal(elements.get('motorPosition').min,'-1400');assert.equal(elements.get('motorPosition').max,'10500');
assert.match(elements.get('motorHint').textContent,/正式 STEP、Debug 與 RAM 目標同樣限制/);
const beforeInvalidLimit=requests.filter(r=>r.body?.command==='debug.move').length;
elements.get('motorPosition').value='10501';elements.get('motorRpm').value='100';elements.get('motorAcc').value='5';
await elements.get('stepperForm').listeners.submit({preventDefault(){}});
await new Promise(resolve=>setTimeout(resolve,5));
assert.equal(requests.filter(r=>r.body?.command==='debug.move').length,beforeInvalidLimit);
fixture.firmware.stage=4;vm.runInContext('renderState()',context);
assert.equal(elements.get('motorPosition').min,'-8388608');assert.equal(elements.get('motorPosition').max,'10500');
assert.match(elements.get('stage4TrimHint').textContent,/±200 脈衝/);assert.doesNotMatch(elements.get('stage4TrimHint').textContent,/-1400～10500/);
assert.match(elements.get('stage4TrimHint').textContent,/ID6 各關共用絕對位置上限 10500/);
// Healthy Stage1 ID6 follows its own chicken/bird YAML, outside the fruit-only bounds.
fixture.firmware.stage=1;vm.runInContext('renderState()',context);
assert.equal(elements.get('motorPosition').max,'10500');
elements.get('motorPosition').value='9200';assert.equal(elements.get('motorPosition').reportValidity(),true);
assert.equal(elements.get('sendMotor').disabled,false);

// An isolated ID6 blocks only its shared profiles; normal stage selection and explicit HOME remain usable.
Object.assign(fixture.firmware,{stage:6,mode:'normal',phase:'ready',stopped:false,previous_available:true,
  motor_recovery_mask:4,position_known_mask:3,stage_recovery_required:true,
  mechanism_warning:'Fruit ID6 completion timeout',stage4_trim_allowed:false,stage4_trim_moving:false});
vm.runInContext('renderState()',context);
assert.match(elements.get('mechanismRecoveryState').textContent,/ID6.*局部隔離/);
assert.match(elements.get('mechanismRecoveryHint').textContent,/completion timeout.*第二／第三關.*第一／第四／第六關.*尋零開始與完成/);
assert.match(elements.get('motorAvailability').textContent,/ID6：局部隔離／需手動 HOME/);
assert.equal(elements.get('stage4TrimState').textContent,'ID6 隔離 / 等待手動 HOME');
for(const command of ['next','prev','run'])assert.equal(normal(command).disabled,true);
assert.equal(normal('home').disabled,false);assert.equal(elements.get('selectStage').disabled,false);
assert.equal(elements.get('applyConfig').disabled,true);assert.equal(elements.get('globalStop').disabled,false);
elements.get('activeStage').value='2';await elements.get('selectStage').listeners.click();
assert.equal(requests.filter(r=>r.body?.command==='stage').at(-1).body.stage,2);
for(const stage of [2,3]){
  Object.assign(fixture.firmware,{stage,stage_recovery_required:false});vm.runInContext('renderState()',context);
  assert.equal(normal('next').disabled,false);assert.equal(normal('run').disabled,false);
  assert.match(elements.get('mechanismRecoveryState').textContent,/ID6.*局部隔離/);
}
for(const stage of [1,4,6]){
  Object.assign(fixture.firmware,{stage,stage_recovery_required:true,phase:'ready',mode:'normal'});
  elements.get('activeStage').value=stage===1?'1_2':String(stage);vm.runInContext('renderState()',context);
  for(const command of ['next','prev','run'])assert.equal(normal(command).disabled,true);
  assert.equal(normal('home').disabled,false);assert.equal(elements.get('selectStage').disabled,false);
  if(stage===1)assert.equal(elements.get('triggerProp').disabled,true);
}
const beforeRecoveryHome=requests.filter(r=>r.body?.command==='home').length;
await normal('home').listeners.click();
assert.equal(requests.filter(r=>r.body?.command==='home').length,beforeRecoveryHome+1);
// A failed recovery retains its mask; a different axis remains available in Debug.
fixture.firmware.mode='debug';elements.get('motorNode').value='6';vm.runInContext('renderState()',context);
assert.equal(elements.get('sendMotor').disabled,true);assert.equal(elements.get('motorZero').disabled,true);
assert.equal(elements.get('motorGoHome').disabled,false);assert.match(elements.get('motorHint').textContent,/排除卡點.*GoHome.*開始與完成/);
elements.get('motorNode').value='4';vm.runInContext('renderState()',context);assert.equal(elements.get('sendMotor').disabled,false);
// Latched global STOP always wins, even if a malformed/old response still labels its phase ready.
Object.assign(fixture.firmware,{stage:6,mode:'normal',phase:'ready',stopped:true});vm.runInContext('renderState()',context);
for(const command of ['next','prev','run','home'])assert.equal(normal(command).disabled,true);
assert.equal(elements.get('selectStage').disabled,true);assert.equal(elements.get('exitStage1').disabled,true);
assert.match(elements.get('mechanismRecoveryState').textContent,/全域 STOP／無法以 HOME 恢復/);
assert.match(elements.get('mechanismRecoveryHint').textContent,/重啟 Teensy.*切關與 HOME 都不能解除/);
fixture.firmware.mode='debug';elements.get('motorNode').value='6';vm.runInContext('renderState()',context);
for(const id of ['sendMotor','motorZero','motorGoHome','sendServo2','extendCylinder'])assert.equal(elements.get(id).disabled,true);
Object.assign(fixture.firmware,{mode:'normal',stopped:false,motor_recovery_mask:0,stage_recovery_required:false,
  mechanism_warning:'',position_known_mask:7});vm.runInContext('renderState()',context);
assert.equal(normal('next').disabled,false);assert.equal(normal('home').disabled,false);
assert.equal(elements.get('mechanismRecoveryState').textContent,'尚無局部隔離警告');
console.log('PASS fruit FE limit and local recovery GUI: -1400..10500 fruit bounds and shared ID6 upper bound, unequal requested/applied offsets, unchanged Stage4 range, ID6 quarantine without blocking Stage2/3, explicit HOME and stage controls, shared-profile interlocks and global STOP priority');

// All stages use ID6's shared upper bound; ID4/5 retain full signed 24-bit limits.
Object.assign(fixture.firmware,{stage:1,mode:'debug',phase:'ready',stopped:false,id6_position_max:10500,
  motor_online_mask:7,motor_offline_mask:0,position_known_mask:7,motor_recovery_mask:0,stage_recovery_required:false});
for(const stage of [1,2,3,4,5,6]){
  fixture.firmware.stage=stage;elements.get('motorNode').value='6';vm.runInContext('renderState()',context);
  assert.equal(elements.get('motorPosition').max,'10500');
  assert.equal(elements.get('motorPosition').min,stage===6?'-1400':'-8388608');
}
for(const node of [4,5]){
  elements.get('motorNode').value=String(node);vm.runInContext('renderState()',context);
  assert.equal(elements.get('motorPosition').max,'8388607');assert.equal(elements.get('motorPosition').min,'-8388608');
}
delete fixture.firmware.id6_position_max;elements.get('motorNode').value='6';vm.runInContext('renderState()',context);
assert.equal(elements.get('motorPosition').max,'10500');
fixture.firmware.id6_position_max=10500;
// The RAM Action editor updates its position boundaries immediately when the selected node changes.
vm.runInContext('documentModel={stage:6,repeat_from:1,steps:[{actions:[{stepper:{node:6,position:9300,rpm:320,acc:200}}]}]};renderEditor()',context);
elements.get('repeatFrom').value='1';
let editorPosition=elements.get('steps').querySelectorAll('input')[0];
assert.equal(editorPosition.min,-1400);assert.equal(editorPosition.max,10500);
assert.equal(editorPosition.reportValidity(),true);
let editorNode=descendants(elements.get('steps'),'select').find(e=>e.children.some(o=>o.value===4));
assert.ok(editorNode);editorNode.value='4';editorNode.listeners.change();
editorPosition=elements.get('steps').querySelectorAll('input')[0];
assert.equal(editorPosition.min,-8388608);assert.equal(editorPosition.max,8388607);
editorNode=descendants(elements.get('steps'),'select').find(e=>e.children.some(o=>o.value===4));
editorNode.value='6';editorNode.listeners.change();
editorPosition=elements.get('steps').querySelectorAll('input')[0];
assert.equal(editorPosition.min,-1400);assert.equal(editorPosition.max,10500);
editorPosition.value='10501';editorPosition.listeners.input();assert.throws(()=>vm.runInContext('editorValid()',context),/Action 數值/);
editorPosition.value='10500';editorPosition.listeners.input();assert.equal(vm.runInContext('editorValid().steps[0].actions[0].stepper.position',context),10500);
vm.runInContext('documentModel.stage=1;renderEditor()',context);editorPosition=elements.get('steps').querySelectorAll('input')[0];
assert.equal(editorPosition.min,-8388608);assert.equal(editorPosition.max,10500);
console.log('PASS ID6 shared upper 10500 GUI: all-stage Debug cap, fruit-only lower -1400, ID4/5 24-bit limits, 9200 Stage1 YAML support and live Action-editor node-bound changes');

console.log("PASS Stage2 startup home GUI: boot wait keeps NEXT blocked and STOP available, full-home vs partial reference telemetry, stopped rendering never restarts motion");

// Stage2 pairs equal/opposite offsets from STEP0 and reports both ACK positions.
assert.ok(html.indexOf('id="stage2TrimPanel"')<html.indexOf('id="controlView"'));
assert.match(html,/CH4 \/ STAGE2 · ID4 \+ ID5/);
const beforeStage2TrimCommands=requests.filter(r=>r.path==='/api/command').length;
fixture.connected=true;fixture.status_age=0;
Object.assign(fixture.firmware,{stage:2,mode:'normal',phase:'ready',step:0,stopped:false,pending_stage:0,
  uploading:false,motor_online_mask:7,motor_offline_mask:0,position_known_mask:7,motor_recovery_mask:0,
  stage_recovery_required:false,position4_base:1000,position5_base:-1000,position4:1800,position5:-1800,
  stage2_trim_pulses:800,stage2_trim_target4:1800,stage2_trim_target5:-1800,
  stage2_trim_allowed:true,stage2_trim_moving:false,stage1_trim_allowed:false,stage4_trim_allowed:false});
Object.assign(fixture.firmware.rc,{connected:true,stage1_mode:false,stage2_trim_pulses:800});
fixture.firmware.rc.channels[3]=2000;
vm.runInContext('state=fixture;renderState()',context);
assert.equal(elements.get('currentStep').textContent,'STEP0');
assert.equal(elements.get('stage2TrimState').textContent,'文蛤高度微調啟用');
assert.equal(elements.get('stage2TrimRaw').textContent,'2000 µs');
assert.equal(elements.get('stage2TrimRequested').textContent,'+800 脈衝');
assert.equal(elements.get('stage2TrimApplied').textContent,'+800 脈衝');
assert.match(elements.get('stage2TrimCommand').textContent,/ID4 基準 1000.*最後完成位置 1800.*已送目標 1800.*ID5 基準 -1000.*最後完成位置 -1800.*已送目標 -1800/);
assert.match(elements.get('stage2TrimHint').textContent,/STEP0.*±1600.*100 RPM.*100 ms.*雙 ACK.*ID4 = 基準 \+.*ID5 = 基準 -.*共同截斷.*不累加.*不改 STEP/);
Object.assign(fixture.firmware,{phase:'busy',stage2_trim_moving:true,stage2_trim_target4:200,stage2_trim_target5:-200});
fixture.firmware.rc.stage2_trim_pulses=-800;
vm.runInContext('renderState()',context);
assert.equal(elements.get('stage2TrimState').textContent,'雙步進微調中 / 等待雙 ACK');
assert.equal(elements.get('stage2TrimRequested').textContent,'-800 脈衝');
assert.equal(elements.get('stage2TrimApplied').textContent,'+800 脈衝');
assert.match(elements.get('stage2TrimCommand').textContent,/最後完成位置 1800.*已送目標 200.*最後完成位置 -1800.*已送目標 -200/);
assert.match(elements.get('stage2TrimHint').textContent,/ID4 與 ID5 都回報完成才接受 NEXT/);
assert.equal(normal('next').disabled,true);
fixture.firmware.stage2_trim_moving=false;
vm.runInContext('renderState()',context);
assert.equal(elements.get('stage2TrimState').textContent,'等待 STEP 完成');
Object.assign(fixture.firmware,{phase:'ready',position4_base:0,position5_base:0,position4:0,position5:0,
  stage2_trim_pulses:0,stage2_trim_target4:0,stage2_trim_target5:0});
vm.runInContext('renderState()',context);
assert.equal(elements.get('stage2TrimRequested').textContent,'-800 脈衝');
assert.equal(elements.get('stage2TrimApplied').textContent,'0 脈衝');
assert.match(elements.get('stage2TrimHint').textContent,/不小於 0.*不大於 0.*共同截斷/);
for(const change of [{motor_online_mask:6},{position_known_mask:6},{motor_recovery_mask:1}]){
  Object.assign(fixture.firmware,{motor_online_mask:7,position_known_mask:7,motor_recovery_mask:0},change);
  vm.runInContext('renderState()',context);
  assert.equal(elements.get('stage2TrimState').textContent,'雙軸未就緒 / 微調停用');
  assert.match(elements.get('stage2TrimHint').textContent,/不會單獨驅動一顆/);
}
Object.assign(fixture.firmware,{motor_online_mask:7,position_known_mask:7,motor_recovery_mask:0});
for(const change of [{stage:4},{stage:6},{mode:'debug'},{pending_stage:3},{uploading:true}]){
  Object.assign(fixture.firmware,{stage:2,mode:'normal',phase:'ready',pending_stage:0,uploading:false},change);
  vm.runInContext('renderState()',context);
  assert.equal(elements.get('stage2TrimState').textContent,'微調停用');
}
Object.assign(fixture.firmware,{stage:2,mode:'normal',pending_stage:0,uploading:false});
for(const change of [{connected:false},{stage1_mode:true}]){
  Object.assign(fixture.firmware.rc,{connected:true,stage1_mode:false},change);
  vm.runInContext('renderState()',context);
  assert.equal(elements.get('stage2TrimState').textContent,'微調停用');
}
Object.assign(fixture.firmware.rc,{connected:true,stage1_mode:false});
Object.assign(fixture.firmware,{phase:'home_wait',stage2_trim_allowed:false});
vm.runInContext('renderState()',context);
assert.equal(elements.get('stage2TrimState').textContent,'HOME / 微調停用');
fixture.firmware.phase='ready';vm.runInContext('renderState()',context);
assert.equal(elements.get('stage2TrimState').textContent,'微調停用');
assert.match(elements.get('stage2TrimHint').textContent,/HOME 後按 NEXT／PREV.*重新啟用/);
fixture.firmware.stopped=true;vm.runInContext('renderState()',context);
assert.equal(elements.get('stage2TrimState').textContent,'STOP / 微調停用');
assert.match(elements.get('stage2TrimCommand').textContent,/STOP 前最後完成位置.*STOP 前最後目標/);
assert.match(elements.get('stage2TrimHint').textContent,/不能代表急停後仍在運轉/);
fixture.firmware.stopped=false;
for(const [name,value] of [['position4_base',NaN],['position5',Infinity],['stage2_trim_target4',null],
  ['stage2_trim_pulses',undefined],['stage2_trim_allowed',undefined],['stage2_trim_moving',undefined]]){
  const saved=fixture.firmware[name];fixture.firmware[name]=value;
  vm.runInContext('renderState()',context);
  assert.match(elements.get('stage2TrimState').textContent,/韌體未提供/);
  assert.equal(elements.get('stage2TrimApplied').textContent,'--');
  assert.doesNotMatch(elements.get('stage2TrimCommand').textContent,/NaN|Infinity|null|undefined/);
  fixture.firmware[name]=saved;
}
const savedStage2Request=fixture.firmware.rc.stage2_trim_pulses;
delete fixture.firmware.rc.stage2_trim_pulses;vm.runInContext('renderState()',context);
assert.match(elements.get('stage2TrimState').textContent,/韌體未提供/);
fixture.firmware.rc.stage2_trim_pulses=savedStage2Request;
fixture.status_age=5;vm.runInContext('renderState()',context);
assert.match(elements.get('stage2TrimState').textContent,/尚未取得/);
for(const id of ['stage2TrimRaw','stage2TrimRequested','stage2TrimApplied'])assert.equal(elements.get(id).textContent,'--');
assert.doesNotMatch(elements.get('stage2TrimCommand').textContent,/基準 0/);
assert.equal(requests.filter(r=>r.path==='/api/command').length,beforeStage2TrimCommands);
console.log('PASS Stage2 paired CH4 trim GUI: STEP0 ±1600 request/completed offset, both-axis base/ACK/targets, common zero clamp, dual-ACK interlock, online/known/context guards, HOME/STOP/stale/legacy downgrade and no commands');

// Disabling Stage3 removes only its selectable workflow, never its shared hardware.
Object.assign(fixture,{connected:true,status_age:0});
Object.assign(fixture.firmware,{stage:2,stage3_enabled:false,stage_ids:[1,2,3,4,5,6],mode:'debug',phase:'ready',stopped:false,uploading:false,motor_online_mask:7,motor_offline_mask:0,position_known_mask:7,motor_recovery_mask:0});
vm.runInContext('renderState()',context);
for(const id of ['profile','activeStage'])assert.equal(elements.get(id).children.some(option=>option.value==='3'),false);
for(const node of [4,5]){elements.get('motorNode').value=String(node);vm.runInContext('renderState()',context);assert.equal(elements.get('sendMotor').disabled,false);}
assert.equal(elements.get('sendServo3').disabled,false);
fixture.firmware.stage=3;vm.runInContext('renderState()',context);
assert.equal(elements.get('sendMotor').disabled,false);assert.equal(elements.get('sendServo3').disabled,false);
fixture.firmware.mode='normal';fixture.firmware.stage=2;
context.disabledStage3Model=thirdModel;vm.runInContext('documentModel=clone(disabledStage3Model)',context);
vm.runInContext('renderState()',context);
assert.equal(elements.get('applyConfig').disabled,true);
const disabledStage3Requests=requests.filter(request=>request.path==='/api/apply').length;
await elements.get('applyConfig').listeners.click();
assert.equal(requests.filter(request=>request.path==='/api/apply').length,disabledStage3Requests);
assert.match(elements.get('message').textContent,/停用/);
// Even inconsistent older telemetry cannot re-enable it when the explicit flag is false.
fixture.firmware.stage_ids=[1,2,4,5,6];vm.runInContext('renderState()',context);
assert.equal(elements.get('activeStage').children.some(option=>option.value==='3'),false);
fixture.firmware.stage3_enabled=true;fixture.firmware.stage_ids=[1,2,3,4,5,6];vm.runInContext('renderState()',context);
assert.equal(elements.get('activeStage').children.some(option=>option.value==='3'),true);
console.log('PASS disabled Stage3 GUI: both dropdowns/capability filtering, RAM refusal and shared ID4/5 + Pin3 Debug preserved');
