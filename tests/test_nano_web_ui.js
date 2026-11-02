const assert=require('node:assert/strict'),fs=require('node:fs'),vm=require('node:vm');
const html=fs.readFileSync('host/index.html','utf8'),elements=new Map(),requests=[];
const noop=()=>{},canvas={clearRect:noop,beginPath:noop,arc:noop,stroke:noop,fillText:noop,fillRect:noop,drawImage:noop,save:noop,restore:noop,moveTo:noop,lineTo:noop};
const element=id=>{if(!elements.has(id))elements.set(id,{textContent:'',style:{},removeAttribute:noop,getContext:()=>canvas});return elements.get(id)};
const operators=[element('operatorLink'),element('stopMotor')];
const buttons=[{dataset:{api:'/api/shutdown'}},{dataset:{api:'/api/record/start'}}];
const state={read_only:true,host:'nano',mode:'live',pc:{online:false,fps:0},nano:{result_fresh:false,data:{}},
 imu:{online:false,healthy:false,attitude:{valid:false},frame:null,statistics:{}},tracking:{fresh:false},
 gateway:{online:true},control:{phase:'offline'},lidar:{online:false},recording:{active:false},flow:null};
const ctx=vm.createContext({document:{getElementById:element,querySelectorAll(selector){
 if(selector==='[data-api]')return buttons;
 if(selector==='[data-operator], [data-api]')return [...operators,...buttons];
 return [];
}},setTimeout:noop,requestAnimationFrame:noop,console,Math,Date,
 fetch:async(url,options)=>{requests.push({url,options});return {ok:true,json:async()=>state}}});
for(const source of html.matchAll(/<script>([\s\S]*?)<\/script>/g)){
 if(source[1].trim()==='requestAnimationFrame(animateFlow);update();')continue;
 vm.runInContext(source[1],ctx);
}
(async()=>{
 await vm.runInContext('update()',ctx);
 assert.ok([...operators,...buttons].every(e=>e.hidden));
 assert.equal(element('motorTitle').textContent,'底盘状态');
 assert.equal(element('videoTitle').textContent,'Nano接收画面');
 assert.match(element('mode').textContent,/Nano共享观测/);
 for(const button of buttons)await button.onclick();
 await assert.rejects(vm.runInContext("controlRequest({action:'claim'})",ctx),/仅供查看/);
 vm.runInContext('stopMotor();offline()',ctx);
 await Promise.resolve();
 assert.ok(requests.every(r=>!r.options),'Read-only page must never send a POST');
 vm.runInContext('configureViewer({read_only:false})',ctx);
 assert.ok([...operators,...buttons].every(e=>!e.hidden),'PC controls remain available in the PC console');
 console.log('PASS Nano UI: whole page, shared media URLs, hidden controls, no mutation requests, PC mode preserved');
})().catch(error=>{console.error(error);process.exitCode=1});
