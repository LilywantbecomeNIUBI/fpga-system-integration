// Offline canvas checks for both served pages; no browser or device access.
const fs=require('fs'),vm=require('vm'),assert=require('assert');
for(const page of ['host/index.html','host/task4_console.html']){
 const html=fs.readFileSync(page,'utf8');
 for(const match of html.matchAll(/<script[^>]*>([\s\S]*?)<\/script>/g))new vm.Script(match[1],{filename:page});
 const body=html.match(/function drawLidar\([\s\S]*?\n\}/)[0],context=vm.createContext({});
 vm.runInContext(body,context);
 const noop=()=>{},ctx={clearRect:noop,beginPath:noop,arc:noop,stroke:noop,fillText:noop,fillRect:noop};
 const canvas={width:400,height:400,getContext:()=>ctx};
 const points=Array.from({length:720},(_,i)=>[i*.5+.01,i===719?6400:1000,50]);
 points.push([1,0,50],[2,1000,0]);
 const lidar={online:true,scan:{points}},original=JSON.stringify(points);
 for(const [mode,expected] of [['all',720],['720',720],['360',360]]){
  const result=context.drawLidar(canvas,lidar,'auto',mode);
  assert.equal(result.visible,expected);assert.equal(result.received,720);assert.equal(result.total,722);
  assert.equal(result.radius,7);assert.equal(result.hidden,0);assert.equal(result.thinned,720-expected);
 }
 const fixed=context.drawLidar(canvas,lidar,'3','all');assert.equal(fixed.hidden,1);assert.equal(fixed.visible,719);
 assert.equal(context.drawLidar(canvas,{online:false,scan:{points}},'auto').visible,0);
 assert.equal(JSON.stringify(points),original);
 console.log('PASS '+page+': syntax, all/720/360, auto/fixed range, offline and unchanged source data');
}
