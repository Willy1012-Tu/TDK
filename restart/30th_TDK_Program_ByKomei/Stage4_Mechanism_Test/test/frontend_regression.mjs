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
const stages=elements.has('chassisState')?[1,2,3,4]:[2,3,4];
const eyebrow=new Element();
const model={stage:4,repeat_from:1,steps:[{actions:[{servo:{pin:2,pulse_us:1500,duration_ms:1000}}]}]};
const thirdModel={stage:3,repeat_from:1,steps:[{actions:[{delay:{ms:0}}]}]};
const firstModel={stage:1,repeat_from:1,steps:[{actions:[{stepper:{node:6,position:8000,rpm:1000,acc:230}}]}]};
const secondModel={stage:2,repeat_from:2,steps:[{actions:[{parallel:[{stepper:{node:7,position:-8000,rpm:320,acc:2}},{servo:{pin:9,pulse_us:1800}},{servo:{pin:4,pulse_us:1300}}]}]},{actions:[{parallel:[{servo:{pin:9,pulse_us:1100}},{servo:{pin:4,pulse_us:2000}}]}]},{actions:[{parallel:[{stepper:{node:7,position:-3000,rpm:320,acc:2}},{servo:{pin:6,pulse_us:2200}}]}]},{actions:[{parallel:[{servo:{pin:6,pulse_us:1600}},{stepper:{node:7,position:-8000,rpm:320,acc:2}},{servo:{pin:9,pulse_us:1800}},{servo:{pin:4,pulse_us:1300}}]}]}]};
const profileModels={1:firstModel,2:secondModel,3:thirdModel,4:model};
const fixture={connected:false,firmware:null,status_age:null,logs:[]};
const requests=[];
const context=vm.createContext({fixture,document:{getElementById:id=>elements.get(id),createElement:tag=>new Element(tag),querySelector:()=>eyebrow,querySelectorAll:selector=>selector==='[data-view]'?navs:selector==='[data-command]'?normals:[]},
  Option:class extends Element {constructor(text,value){super('option');this.textContent=text;this.value=value;}},
  fetch:async(path,options)=>{const body=options?.body?JSON.parse(options.body):undefined;requests.push({path,body});const requested=Number(path.match(/profile=(\d+)/)?.[1]??4);return {ok:true,json:async()=>path.startsWith('/api/init')?{token:'test',config:{document:profileModels[requested],hash:'hash'+requested,profile:requested}}:path==='/api/ports'?{ports:[]}:path==='/api/apply'?{result:{stage:body.document.stage}}:path==='/api/command'?{result:{}}:fixture};},setInterval(){},console});
new vm.Script(source).runInContext(context);
await new Promise(resolve=>setTimeout(resolve,20));
assert.equal(elements.get('steps').children.length,1);
assert.equal(elements.get('applyConfig').disabled,true);
assert.equal(elements.get('globalStop').disabled,true);
assert.equal(elements.get('servoCards').children.length,5);
assert.equal(elements.has('sendServo3'),false);
assert.match(html,/<option value="6"[^>]*>ID\s*6/);
assert.match(html,/<option value="7"[^>]*>ID\s*7/);
assert.match(html,/第二關.*STEP1/);
assert.doesNotMatch(html,/<option value="[45]"[^>]*>ID\s*[45]/);
for(const pin of [4,6,9]){elements.get('servoSlider'+pin).value='123.4';elements.get('servoSlider'+pin).listeners.input();assert.equal(elements.get('servoAngle'+pin).value,'123.4');}
elements.get('servoAngle5').value='45.5';elements.get('servoAngle5').listeners.input();assert.equal(elements.get('servoSlider5').value,'45.5');
assert.equal(requests.filter(r=>r.path==='/api/command').length,0);
const fields=elements.get('steps').querySelectorAll('input');
fields[0].value='180';fields[0].listeners.input();assert.equal(fields[2].value,2500);assert.match(elements.get('yamlPreview').textContent,/pulse_us: 2500/);
fields[2].value='500';fields[2].listeners.input();assert.equal(fields[0].value,0);
Object.assign(fixture,{connected:true,status_age:0,applied_hash:null,firmware:{stage:2,stage3_enabled:false,mode:'normal',phase:'ready',previous_available:true,uploading:false,steps:4,step:2,branch:1,completed:1,pulse2:1150,pulse5:2100,pulse4:1300,pulse6:1400,pulse9:1800,position6:0,position7:0,position_known_mask:0,chassis:{state:'running',reason:'centered',emergency:false,wheels:[10,11,12,13].map(node=>({node,bus:node<12?1:2,heartbeat_age_ms:25,error:0,axis_state:8,velocity:1.5}))},rc:{connected:true,age_ms:10,stage:2}}});
vm.runInContext('state=fixture;renderState()',context);
assert.equal(elements.get('applyConfig').disabled,false);
assert.equal(elements.get('currentStep').textContent,'STEP2-1');
assert.match(elements.get('stepDetail').textContent,/第 2 關/);
assert.match(elements.get('servoValue4').textContent,/1300/);
assert.match(elements.get('motorValue').textContent,/尚未確認位置/);
fixture.firmware.position_known_mask=4;fixture.firmware.position6=-600;vm.runInContext('renderState()',context);assert.match(elements.get('motorValue').textContent,/-600/);
elements.get('motorNode').value='7';elements.get('motorNode').listeners.change();assert.match(elements.get('motorValue').textContent,/尚未確認位置/);fixture.firmware.position_known_mask=12;fixture.firmware.position7=-8000;vm.runInContext('renderState()',context);assert.equal(elements.get('motorId').textContent,7);assert.match(elements.get('motorValue').textContent,/-8000/);elements.get('motorNode').value='6';elements.get('motorNode').listeners.change();assert.match(elements.get('motorValue').textContent,/-600/);
if(normal('prev')){assert.equal(normal('prev').disabled,false);fixture.firmware.previous_available=false;vm.runInContext('renderState()',context);assert.equal(normal('prev').disabled,true);}
assert.equal(normal('next').disabled,false);
if(elements.has('chassisState')){
  assert.match(elements.get('chassisState').textContent,/RUNNING/);assert.equal(elements.get('wheelStatus').children.length,4);assert.equal(elements.get('wheelStatus').children[2].children[1].textContent,'CAN2');assert.match(elements.get('rcState').textContent,/最近確認連線 10 ms/);
  fixture.firmware.rc.connected=false;fixture.firmware.rc.age_ms=500;vm.runInContext('renderState()',context);assert.match(elements.get('rcState').textContent,/斷線經過 500 ms/);
}
fixture.firmware.phase='busy';vm.runInContext('renderState()',context);assert.equal(normal('next').disabled,true);assert.equal(elements.get('selectStage').disabled,true);
fixture.firmware.phase='ready';fixture.firmware.mode='debug';vm.runInContext('renderState()',context);assert.equal(elements.get('applyConfig').disabled,true);
for(const pin of [2,5,4,6,9])assert.equal(elements.get('sendServo'+pin).disabled,false);
fixture.firmware.stage=3;vm.runInContext('renderState()',context);
assert.match(elements.get('modeHint').textContent,/第三關待機/);
for(const id of ['motorNode','motorPosition','motorRpm','motorAcc','sendMotor','motorZero','motorGoHome'])assert.equal(elements.get(id).disabled,true);
for(const pin of [2,5,4,6,9])for(const prefix of ['servoSlider','servoAngle','servoDuration','sendServo'])assert.equal(elements.get(prefix+pin).disabled,true);
fixture.firmware.stage=2;vm.runInContext('renderState()',context);
// Loading a standby file only edits the PC copy and must not lock active Stage2 Debug.
elements.get('profile').value='3';await elements.get('profile').listeners.change();
assert.equal(elements.get('steps').children.length,1);assert.match(elements.get('yamlPreview').textContent,/stage: 3\nrepeat_from: 1/);assert.doesNotMatch(elements.get('yamlPreview').textContent,/stepper:|servo:/);assert.equal(elements.get('addStep').disabled,true);assert.equal(elements.get('repeatFrom').disabled,true);assert.equal(elements.get('sendServo4').disabled,false);
assert.equal(vm.runInContext('editorValid().repeat_from',context),1);assert.equal(vm.runInContext('defaults.stepper().node',context),6);
fixture.status_age=5;vm.runInContext('renderState()',context);assert.equal(elements.get('sendServo4').disabled,true);assert.equal(elements.get('globalStop').disabled,false);
fixture.firmware.phase='fault';vm.runInContext('renderState()',context);assert.match(elements.get('servoValue4').textContent,/STOP 前指令/);fixture.firmware.phase='ready';
fixture.status_age=0;fixture.firmware.mode='normal';vm.runInContext('renderState()',context);
for(const stage of stages){elements.get('profile').value=String(stage);await elements.get('profile').listeners.change();assert.equal(vm.runInContext('documentModel.stage',context),stage);assert.equal(vm.runInContext('defaults.stepper().node',context),stage===2?7:6);}
assert.equal(requests.filter(r=>r.path==='/api/apply'||r.path==='/api/command').length,0);
elements.get('profile').value='2';await elements.get('profile').listeners.change();
assert.equal(elements.get('steps').children.length,4);assert.match(elements.get('yamlPreview').textContent,/repeat_from: 2/);assert.match(elements.get('yamlPreview').textContent,/node: 7, position: -8000/);assert.match(elements.get('yamlPreview').textContent,/pin: 9, pulse_us: 1100/);assert.match(elements.get('yamlPreview').textContent,/pin: 4, pulse_us: 2000/);assert.match(elements.get('yamlPreview').textContent,/pin: 6, pulse_us: 2200/);assert.match(elements.get('yamlPreview').textContent,/pin: 6, pulse_us: 1600/);
await elements.get('applyConfig').listeners.click();
assert.deepEqual(requests.find(r=>r.path==='/api/apply').body.document,secondModel);
assert.match(elements.get('message').textContent,/STEP1 預備已完成.*NEXT 從 STEP2/);
// Independent controls retain their own pin, angle and interpolation duration.
fixture.firmware.mode='debug';vm.runInContext('state=fixture;renderState()',context);
for(const pin of [4,6,9]){elements.get('servoAngle'+pin).value='117';elements.get('servoDuration'+pin).value='700';const card=elements.get('servoCards').children[[2,3,4][[4,6,9].indexOf(pin)]];card.listeners.submit({preventDefault(){}});await new Promise(resolve=>setTimeout(resolve,5));assert.ok(requests.some(r=>r.path==='/api/command'&&r.body.command==='debug.servo'&&r.body.pin===pin&&r.body.angle===117&&r.body.duration_ms===700));}
// ID7 Debug uses the selected node; selecting a PC profile alone never sends a motor command.
elements.get('motorNode').value='7';elements.get('motorPosition').value='-3000';elements.get('motorRpm').value='320';elements.get('motorAcc').value='2';elements.get('stepperForm').listeners.submit({preventDefault(){}});await new Promise(resolve=>setTimeout(resolve,5));assert.ok(requests.some(r=>r.path==='/api/command'&&r.body.command==='debug.move'&&r.body.node===7&&r.body.position===-3000));
// Invalid editor steps remain rejected instead of silently reaching RAM.
fixture.firmware.mode='normal';elements.get('addStep').listeners.click();assert.equal(elements.get('steps').children.length,5);vm.runInContext('documentModel.steps[4].actions=[]',context);assert.throws(()=>vm.runInContext('editorValid()',context),/至少一個 Action/);
console.log(`PASS frontend model: ${stages.length} profiles, five servo controls, Stage2 four-step ID7 RAM payload / prepare notice, ID6/7 position-known masks, disconnected Stage3 guards, file-only profile load, mode/stale/previous guards`);
