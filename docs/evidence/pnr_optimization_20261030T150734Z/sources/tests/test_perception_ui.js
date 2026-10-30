const assert=require('node:assert/strict'),fs=require('node:fs'),vm=require('node:vm');
const html=fs.readFileSync('host/index.html','utf8'),elements=new Map(),calls=[];
const canvas={clearRect(){},beginPath(){},moveTo(){},lineTo(){},stroke(){},arc(){},fillRect(){}};
const element=id=>{if(!elements.has(id))elements.set(id,{textContent:'',style:{},removeAttribute(){},getContext(){return canvas}});return elements.get(id)};
const ctx=vm.createContext({document:{getElementById:element,querySelectorAll(){return []}},setTimeout(){},Date,Math,console,
 fetch:async(url,opts)=>{if(opts)calls.push(url);return {ok:false,status:503}}});
for(const source of html.matchAll(/<script>([\s\S]*?)<\/script>/g))if(source[1].trim()!=='update();')vm.runInContext(source[1],ctx);
vm.runInContext('renderFlow(null)',ctx);
assert.match(element('flowDetails').textContent,/unknown/);
assert.match(element('flowDetails').textContent,/建议停车/);
assert.match(element('flowDetails').textContent,/0\/255/);
vm.runInContext("renderFlow({control:{fresh:true,state:'left',stop_request:false,speed_limit:96,frame_id:1,valid_counts:[4,5,6],risk_counts:[0,2,2]},vectors:[{valid:true,x:8,y:8,u:-.5,v:0}]})",ctx);
assert.match(element('flowDetails').textContent,/96\/255/);
assert.match(element('flowDetails').textContent,/自动控制关闭/);
vm.runInContext('offline()',ctx);
assert.match(element('flowDetails').textContent,/unknown/);
assert.match(element('flowDetails').textContent,/建议停车/);
assert.equal(calls.length,0,'Reading flow observations never sends motor commands');
console.log('PASS perception UI: mixed-state rendering, stale clearing and no motion side effects');
