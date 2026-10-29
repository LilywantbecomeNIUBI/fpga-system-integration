// Page-local zeroing must never emit a control command or erase FPGA totals.
const assert=require('node:assert/strict'),fs=require('node:fs'),vm=require('node:vm'),path=require('node:path');
const html=fs.readFileSync(path.join(__dirname,'../host/motor_tuner.html'),'utf8');
const elements=new Map(),calls=[];
const element=id=>{if(!elements.has(id))elements.set(id,{textContent:'',disabled:false,scrollHeight:0});return elements.get(id)};
const motor={left_position:100,right_position:-200,left_invalid:5681,right_invalid:0,left_applied:0,right_applied:0,armed:false,faults:1};
let state={gateway:{online:true},control:{motor_fresh:true,timed_test_supported:true,phase:'stopped',owned:false,motor}};
const context=vm.createContext({document:{getElementById:element,addEventListener(){}},window:{addEventListener(){}},setInterval(){},setTimeout(){},Date,console,
    fetch:async(url)=>{calls.push(url);return {ok:true,json:async()=>JSON.parse(JSON.stringify(state))}}});
const script=html.match(/<script>([\s\S]*)<\/script>/)[1].replace(/poll\(\);\s*$/, '');
vm.runInContext(script,context);
const evaluate=code=>vm.runInContext(code,context);
(async()=>{
    await element('zero').onclick();
    assert.equal(evaluate('counterView(zeroBase).left_invalid'),0);
    assert.equal(evaluate('counterView({...zeroBase,left_position:120,left_invalid:5683}).left_position'),20);
    assert.equal(evaluate('counterView({...zeroBase,left_invalid:5683}).left_invalid'),2);
    assert.equal(motor.left_invalid,5681);assert.equal(motor.faults,1);
    const baseline=evaluate('zeroBase');state.control.motor.armed=true;
    await element('zero').onclick();assert.equal(evaluate('zeroBase'),baseline);
    state.control.motor.armed=false;state.control.motor_fresh=false;
    await element('zero').onclick();assert.equal(evaluate('zeroBase'),baseline);
    state.control.motor_fresh=true;state.control.owned=true;
    await element('zero').onclick();assert.equal(evaluate('zeroBase'),baseline);
    evaluate('zeroBase={left_position:2147483647,right_position:0,left_invalid:4294967295,right_invalid:0}');
    assert.equal(evaluate('counterView({left_position:-2147483648,left_invalid:0,right_position:0,right_invalid:0}).left_position'),1);
    assert.equal(evaluate('counterView({left_position:-2147483648,left_invalid:0,right_position:0,right_invalid:0}).left_invalid'),1);
    evaluate("resetZero('test restart')");assert.equal(evaluate('zeroBase'),null);
    assert.ok(calls.every(url=>url==='/api/state'),'Zero must only read status');
    console.log('PASS tuner zero: stopped/fresh/ownership guards, relative counters, wrap, retained faults and raw totals, no control commands');
})().catch(error=>{console.error(error);process.exitCode=1});
