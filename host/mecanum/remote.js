'use strict';
// No motion on load. A fresh pointer/key press is required for each gesture.
let heldInput=null, gestureCounter=Date.now()*1000, pulseBusy=false, savedRouteKey='';
const remoteKeys=new Set();
const jogButtons=Array.from(document.querySelectorAll('[data-jog]'));

async function remoteCommand(action,values={}){
 const response=await fetch('/api/action',{method:'POST',headers:{'Content-Type':'application/json','X-Console-Token':token},
  body:JSON.stringify({action,client,...values}),signal:AbortSignal.timeout(400),keepalive:action==='jog_end'});
 const data=await response.json();if(!response.ok)throw Error(data.error||'遥控请求失败');return data;
}
function paintHeld(direction){for(const button of jogButtons)button.classList.toggle('held',button.dataset.jog===direction?.join(','));}
function clearHeldInput(){heldInput=null;remoteKeys.clear();paintHeld(null);}
async function releaseHeld(preserveKeys=false){
 const previous=heldInput;heldInput=null;paintHeld(null);if(!preserveKeys)remoteKeys.clear();if(!previous)return;
 try{await remoteCommand('jog_end',{gesture:previous.gesture,control_epoch:previous.epoch});}
 catch(e){halt();$('message').textContent='停车确认失败，已请求全局STOP：'+e.message;}
}
function leaveRemote(){if(heldInput)halt();clearHeldInput();}
async function startHeld(direction,source){
 if(!owned||document.hidden||$('teachPage').hidden||state?.mode!=='task2')return;
 if(heldInput?.direction.join(',')===direction.join(','))return;
 const input={gesture:++gestureCounter,direction,epoch:controlEpoch,source,ready:false};heldInput=input;paintHeld(direction);
 try{
  await remoteCommand('jog_begin',{gesture:input.gesture,control_epoch:input.epoch,direction,limit:Number($('remoteLimit').value)});
  if(heldInput!==input){await remoteCommand('jog_end',{gesture:input.gesture,control_epoch:input.epoch});return;}
  input.ready=true;
 }catch(e){if(heldInput===input){halt();$('message').textContent=e.message;}}
}
setInterval(async()=>{
 const input=heldInput;if(!input?.ready||pulseBusy)return;pulseBusy=true;
 try{await remoteCommand('jog_pulse',{gesture:input.gesture,control_epoch:input.epoch});}
 catch(e){if(heldInput===input){halt();$('message').textContent='遥控已停车：'+e.message;}}
 finally{pulseBusy=false;}
},80);
for(const button of jogButtons){
 button.addEventListener('pointerdown',e=>{
  if(e.button!==0||heldInput?.source?.startsWith('pointer'))return;e.preventDefault();
  button.setPointerCapture(e.pointerId);startHeld(button.dataset.jog.split(',').map(Number),'pointer:'+e.pointerId);
 });
 for(const event of ['pointerup','pointercancel','lostpointercapture'])button.addEventListener(event,e=>{
  if(heldInput?.source==='pointer:'+e.pointerId)releaseHeld();
 });
 button.addEventListener('contextmenu',e=>e.preventDefault());
}
const directionKeys=new Set(['arrowup','arrowdown','arrowleft','arrowright','w','a','s','d','q','e']);
function keyboardDirection(){
 const has=(...keys)=>keys.some(k=>remoteKeys.has(k));
 const x=Number(has('w','arrowup'))-Number(has('s','arrowdown'));
 const y=Number(has('a','arrowleft'))-Number(has('d','arrowright'));
 const z=Number(has('q'))-Number(has('e'));
 return z&&(x||y)?[0,0,0]:[x,y,z];
}
document.addEventListener('keydown',e=>{
 if($('teachPage').hidden||/^(INPUT|TEXTAREA|SELECT)$/.test(e.target.tagName)||e.ctrlKey||e.metaKey||e.altKey)return;
 const key=e.key.toLowerCase();if(key===' '){e.preventDefault();halt();return;}
 if(!directionKeys.has(key)||e.repeat||!owned||heldInput?.source?.startsWith('pointer'))return;
 e.preventDefault();remoteKeys.add(key);const d=keyboardDirection();
 if(d.some(Boolean))startHeld(d,'keyboard');else releaseHeld(true);
});
document.addEventListener('keyup',e=>{
 const key=e.key.toLowerCase();if(!remoteKeys.has(key))return;e.preventDefault();remoteKeys.delete(key);
 const d=keyboardDirection();if(d.some(Boolean))startHeld(d,'keyboard');else releaseHeld(true);
});
window.addEventListener('blur',()=>{if(heldInput)halt();});
document.addEventListener('focusin',e=>{if(heldInput&&/^(INPUT|TEXTAREA|SELECT)$/.test(e.target.tagName))halt();});
document.addEventListener('visibilitychange',()=>{if(document.hidden)clearHeldInput();});
$('remoteRelease').onclick=()=>releaseHeld();
$('teachTab').onclick=()=>{
 $('teachPage').hidden=false;for(const id of ['chassisPage','nanoPage','navigationPage'])$(id).hidden=true;
 for(const id of ['chassisTab','nanoTab','navigationTab'])$(id).setAttribute('aria-selected',false);
 $('teachTab').setAttribute('aria-selected',true);
};
$('teachConnect').onclick=()=>act('connect');$('teachAcquire').onclick=()=>act('acquire');
$('remoteMode').onclick=()=>act('mode',{mode:'task2'});
const mission=()=>Number($('teachMissionId').value);
async function teachAction(action,values={}){
 const result=await act(action,values);if(result)await poll();return result;
}
$('teachOrigin').onclick=()=>{
 if(state?.teaching?.points?.length&&!window.confirm('新建示教会清空当前未保存的路径点；已保存路线不变。继续吗？'))return;
 teachAction('teach_origin',{confirmed:$('fixedOrigin').checked});
};
$('teachReplayOrigin').onclick=()=>teachAction('teach_replay_origin',{confirmed:$('fixedOrigin').checked});
$('teachRecord').onclick=()=>teachAction('teach_record',{speed:Number($('teachSpeed').value)});
$('teachUndo').onclick=()=>teachAction('teach_undo');
$('teachSave').onclick=async()=>{const data=await teachAction('teach_save',{name:$('routeName').value});if(data)$('message').textContent='路线已保存到电脑，可重新载入或导出。';};
$('teachLoad').onclick=()=>teachAction('teach_load',{route_id:$('savedRoutes').value});
$('teachUpload').onclick=()=>teachAction('teach_upload',{mission_id:mission()});
$('teachStart').onclick=()=>teachAction('teach_start',{mission_id:mission()});
async function importJson(input,action,key){
 try{const file=input.files[0];if(!file)return;if(file.size>16384)throw Error('文件不能超过16KB');
  const value=JSON.parse(await file.text());await teachAction(action,{[key]:value});}
 catch(e){$('message').textContent=e.message;}finally{input.value='';}
}
$('teachCalibration').onchange=e=>importJson(e.target,'teach_calibration','calibration');
$('teachImport').onchange=e=>importJson(e.target,'teach_import','route');
$('teachExport').onclick=async()=>{
 const data=await teachAction('teach_export');if(!data)return;
 const url=URL.createObjectURL(new Blob([JSON.stringify(data.route,null,2)],{type:'application/json'}));
 const a=document.createElement('a');a.href=url;a.download='mecanum-route.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);
};
function renderTeaching(s){
 const t=s.teaching;if(!t)return;
 if(s.simulation){document.querySelector('header h1').textContent='模拟验证 · 无小车连接';document.title='模拟验证 · 遥控示教';}
 const fresh=s.connected&&s.motor_age_ms!==null&&s.motor_age_ms<300&&s.pose_age_ms!==null&&s.pose_age_ms<300;
 const own=s.operator&&!document.hidden,idle=own&&fresh&&!s.running;
 if(own)controlEpoch=s.control_epoch;
 else if(heldInput){clearHeldInput();}
 const jogAllowed=own&&fresh&&s.mode==='task2'&&(!s.running||s.remote_active)&&!s.navigation?.active;
 for(const button of jogButtons)button.disabled=!jogAllowed;
 $('remoteStatus').textContent=!fresh?'等待新鲜底盘与IMU遥测':!own?'请先接管控制':s.mode!=='task2'?'请切回遥控模式':s.remote_active?'按住移动中 · 松开即停车':'已接管 · 可连续按住遥控，无需每次解锁';
 $('teachOwnership').textContent=own?'已接管 · 普通松手停车保留控制权':s.occupied?'其他网页正在操作':'监测模式 · 接管一次后可连续操作';
 $('teachConnect').disabled=s.connected;$('teachAcquire').disabled=!fresh||s.occupied;
 $('remoteMode').disabled=!idle;$('remoteLimit').disabled=s.remote_active;
 if(s.remote_active)$('fixedOrigin').checked=false;
 const stopped=idle&&s.motor&&!s.motor.armed&&s.motor.velocity.every(v=>v===0)&&s.motor.pwm.every(v=>v===0)&&t.stationary_ms>=300&&!s.navigation?.active;
 for(const id of ['teachOrigin','teachReplayOrigin','teachUndo'])$(id).disabled=!stopped;
 $('teachRecord').disabled=!stopped||!t.valid||t.points.length>=16;
 for(const id of ['teachSave','teachExport'])$(id).disabled=!own||!t.points.length;
 $('teachImport').disabled=$('teachCalibration').disabled=!idle;
 $('teachUpload').disabled=!stopped||!t.measured||!t.replay_ready||!t.points.length;
 $('teachStart').disabled=!stopped||!t.measured||!t.replay_ready||s.navigation?.status!==2||s.navigation?.mission_id!==mission();
 $('teachCalibrationStatus').textContent=t.measured?'已加载实测标定；示教与FPGA使用一致的Q16运动比例。':'尚无实测标定：可用标称轮径记录路线草稿，自主回放禁止启动。';
 $('teachPose').textContent=`相对起点 X ${fmt(t.x_mm)} mm / Y ${fmt(t.y_mm)} mm · 朝向 ${fmt(t.heading_deg)}°`;
 $('teachNotice').textContent=t.error||(t.replay_ready?'已确认回放起点，上传校验后可开始任务。':t.valid?'示教坐标有效：移动后松开遥控，静止300ms再记录。':'请设置起点');
 $('teachPoints').replaceChildren();t.points.forEach((point,i)=>{const tr=document.createElement('tr');for(const v of [i+1,...point]){const td=document.createElement('td');td.textContent=String(v);tr.appendChild(td);}$('teachPoints').appendChild(tr);});
 const key=JSON.stringify(s.saved_routes||[]);if(key!==savedRouteKey){savedRouteKey=key;const old=$('savedRoutes').value;$('savedRoutes').replaceChildren();
  for(const route of s.saved_routes||[]){const option=document.createElement('option');option.value=route.id;option.textContent=`${route.name} · ${route.count}点${route.measured?'':' · 未标定草稿'}`;$('savedRoutes').appendChild(option);}
  if(!s.saved_routes?.length){const option=document.createElement('option');option.value='';option.textContent='暂无保存路线';$('savedRoutes').appendChild(option);}
  else if(Array.from($('savedRoutes').options).some(o=>o.value===old))$('savedRoutes').value=old;
 }
 $('teachLoad').disabled=!idle||!$('savedRoutes').value;
 const canvas=$('teachCanvas'),c=canvas.getContext('2d'),w=canvas.width,h=canvas.height;
 c.clearRect(0,0,w,h);const extent=Math.max(500,...t.points.flatMap(p=>[Math.abs(p[0]),Math.abs(p[1])]),Math.abs(t.x_mm),Math.abs(t.y_mm));
 const scale=(h/2-28)/extent,xy=(x,y)=>[w/2-y*scale,h/2-x*scale];
 c.strokeStyle='#dce4ef';c.beginPath();c.moveTo(w/2,0);c.lineTo(w/2,h);c.moveTo(0,h/2);c.lineTo(w,h/2);c.stroke();
 c.strokeStyle='#2864df';c.beginPath();c.moveTo(...xy(0,0));for(const [x,y]of t.points)c.lineTo(...xy(x,y));c.stroke();
 t.points.forEach(([x,y],i)=>{const [a,b]=xy(x,y);c.fillStyle='#2864df';c.fillRect(a-3,b-3,6,6);c.fillText(String(i+1),a+6,b);});
 const [x,y]=xy(t.x_mm,t.y_mm);c.fillStyle=t.valid?'#159c76':'#9aa7b9';c.fillRect(x-5,y-5,10,10);c.fillStyle='#667991';c.fillText('↑ 前向X / ← 左向Y · 蓝线为已记录路线',15,20);
}
