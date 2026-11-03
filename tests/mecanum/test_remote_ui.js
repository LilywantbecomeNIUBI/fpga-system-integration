// Browser event races against the actual production remote.js; no hardware IO.
const assert=require('node:assert/strict'),fs=require('node:fs'),vm=require('node:vm');
const noop=()=>{},elements=new Map(),requests=[],timers=[],listeners={},windowListeners={};
const canvas=new Proxy({}, {get:()=>noop,set:()=>true});
const element=id=>{
 if(!elements.has(id))elements.set(id,{value:'2',hidden:false,checked:false,textContent:'',options:[],
  dataset:{},classList:{toggle:noop},getContext:()=>canvas,setAttribute:noop,replaceChildren:noop,appendChild:noop,
  addEventListener(event,fn){this[event]=fn;},setPointerCapture:noop});
 return elements.get(id);
};
const button=element('forward');button.dataset.jog='1,0,0';
const ctx=vm.createContext({console,Math,Date,Number,JSON,Set,Array,Error,Blob,URL,
 AbortSignal:{timeout:()=>undefined},token:'test',client:'a'.repeat(32),owned:true,controlEpoch:'epoch',
 state:{mode:'task2'},$:element,fmt:n=>String(n),setInterval:fn=>timers.push(fn),setTimeout:noop,
 act:async()=>({ok:true}),poll:async()=>{},
 document:{hidden:false,querySelectorAll:()=>[button],addEventListener:(event,fn)=>listeners[event]=fn},
 window:{addEventListener:(event,fn)=>windowListeners[event]=fn,confirm:()=>true},
 fetch:async(url,options)=>{requests.push(JSON.parse(options.body));return {ok:true,json:async()=>({ok:true})};}
});
vm.runInContext('let halted=0; function halt(){halted++;clearHeldInput();owned=false;}',ctx);
vm.runInContext(fs.readFileSync('host/mecanum/remote.js','utf8'),ctx);
const run=code=>vm.runInContext(code,ctx);
(async()=>{
 assert.equal(requests.length,0,'Page load must not send motor or control requests');
 await run("startHeld([1,0,0],'keyboard')");
 await timers[0]();await run('releaseHeld()');
 assert.deepEqual(requests.map(r=>r.action),['jog_begin','jog_pulse','jog_end']);
 assert.equal(run('owned'),true,'Normal release must preserve browser ownership');
 await run("startHeld([0,1,0],'keyboard')");
 assert.ok(requests[3].gesture>requests[0].gesture);
 windowListeners.blur();assert.equal(run('halted'),1);assert.equal(run('heldInput'),null);
 run('owned=true');requests.length=0;
 listeners.keydown({key:'w',target:{tagName:'INPUT'},preventDefault:noop});
 assert.equal(requests.length,0,'Typing in route inputs cannot start motion');
 listeners.keydown({key:'w',target:{tagName:'BODY'},preventDefault:noop});
 await new Promise(resolve=>setImmediate(resolve));
 listeners.focusin({target:{tagName:'TEXTAREA'}});assert.equal(run('halted'),2);
 run('owned=true');requests.length=0;
 let finishBegin;
 ctx.fetch=async(url,options)=>{
  const r=JSON.parse(options.body);requests.push(r);
  if(r.action==='jog_begin')await new Promise(resolve=>finishBegin=resolve);
  return {ok:true,json:async()=>({ok:true})};
 };
 const delayed=run("startHeld([1,0,0],'keyboard')");
 await run('releaseHeld()');
 assert.equal(requests[1].action,'jog_end','Release cannot wait for a delayed begin response');
 finishBegin();await delayed;
 assert.equal(run('heldInput'),null);
 assert.deepEqual(requests.map(r=>r.action),['jog_begin','jog_end','jog_end']);
 assert.equal(requests[0].gesture,requests[1].gesture);
 console.log('PASS remote UI: no IO on load, hold/pulse/release, retained ownership, focus stop, input exclusion, delayed-begin race');
})().catch(error=>{console.error(error);process.exitCode=1});
