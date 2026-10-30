const assert=require('node:assert/strict'),fs=require('node:fs'),vm=require('node:vm'),path=require('node:path');
const html=fs.readFileSync(path.join(__dirname,'../host/motor_tuner.html'),'utf8');
const elements=new Map(),calls=[],listeners={},intervals=[];
const element=id=>{if(!elements.has(id))elements.set(id,{value:'4',textContent:'',disabled:false});return elements.get(id)};
const ctx=vm.createContext({document:{getElementById:element,addEventListener(){}},window:{addEventListener(name,fn){listeners[name]=fn}},setInterval(fn,ms){intervals.push({fn,ms})},setTimeout(){},Date,console,
 fetch:async(url,opts)=>{if(opts)calls.push(JSON.parse(opts.body));return {ok:true,json:async()=>({})}}});
vm.runInContext(html.match(/<script>([\s\S]*)<\/script>/)[1].replace(/poll\(\);\s*$/,''),ctx);
const run=s=>vm.runInContext(s,ctx),evt={preventDefault(){},button:0};
(async()=>{
 assert.match(run("connectionSummary({gateway:{online:true,data:{serial_error:'Permission denied'}},control:{connected:false}})"),/权限/);
 assert.match(run("connectionSummary({gateway:{online:true,data:{}},control:{connected:true,motor_fresh:false}})"),/等待FPGA/);
 assert.match(run("connectionSummary({gateway:{online:false,error:'timeout'}})"),/自动重连/);
 assert.match(run("connectionSummary({gateway:{online:true},control:{connected:true,motor_fresh:true}})"),/遥测正常/);

 await element('forward').onclick(evt);assert.equal(calls.length,0,'No automatic claim/ARM');
 run("groundSupported=true;lease='lease';active=true;config={flags:3}");
 await element('forward').onclick(evt);
 assert.deepEqual(calls[0],{action:'heartbeat',lease:'lease',sequence:1,held:true,left:-4,right:4});
 await element('backward').onclick(evt);assert.equal(calls.length,1,'No opposite command while running');
 assert.equal(element('forward').onpointerup,undefined,'Mouse release does not cancel click-to-run');
 for(let i=0;i<30;i++)await intervals.find(x=>x.ms===50).fn();
 assert.equal(run('groundHeld'),true);assert.equal(calls.at(-1).held,true);assert.equal(calls.at(-1).left,-4);
 element('stop').onclick();await run('queue');assert.equal(calls.at(-1).action,'stop');assert.equal(run('lease'),null);
 run("lease='lease2';active=true;running=false;config={flags:3}");
 await element('backward').onclick(evt);assert.equal(calls.at(-1).left,4);assert.equal(calls.at(-1).right,-4);
 listeners.blur();await run('queue');assert.equal(calls.at(-1).action,'release');assert.equal(run('groundHeld'),false);
 run("lease='lease3';active=true;config={flags:1}");
 const n=calls.filter(c=>c.action==='heartbeat'&&c.held).length;
 await element('forward').onclick(evt);await run('queue');assert.equal(calls.filter(c=>c.action==='heartbeat'&&c.held).length,n);
 assert.equal(run('lease'),null);console.log('PASS ground UI: explicit ARM, continuous heartbeat, no overlapping direction, STOP/blur release, polarity guard');
})().catch(e=>{console.error(e);process.exitCode=1});
