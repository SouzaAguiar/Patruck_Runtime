const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');

test('lateral lock zeros real outgoing commands and requires a new gesture on toggle', () => {
  const elements = new Map();
  function element(selector) {
    if (!elements.has(selector)) elements.set(selector, {
      style:{}, attrs:{}, classList:{toggle(){},remove(){},add(){}},
      setAttribute(k,v){this.attrs[k]=v;}, setPointerCapture(){},
      getBoundingClientRect:()=>({left:0,top:0,width:210,height:210}),
      querySelector:child=>element(selector+' '+child),
    });
    return elements.get(selector);
  }
  let socket;
  class WebSocket {
    static OPEN=1;
    constructor(){socket=this;this.readyState=1;this.sent=[];}
    send(data){this.sent.push(JSON.parse(data));}
  }
  vm.runInNewContext(fs.readFileSync(path.join(__dirname,
    '../mini_bdx_runtime/mini_bdx_runtime/web/control.js'),'utf8'),{
    URLSearchParams,WebSocket,location:{search:'?token=test',protocol:'http:',host:'localhost'},
    document:{querySelector:element,querySelectorAll:()=>[],addEventListener(){}},
    addEventListener(){},setInterval(){},clearInterval(){},setTimeout(){},
    fetch:()=>Promise.resolve({json:()=>Promise.resolve({})}),
  });
  const stick=element('#leftStick'), lock=element('#lateralLock');
  const gesture={pointerId:1,clientX:119,clientY:40};
  const last=()=>socket.sent.at(-1);
  stick.onpointerdown(gesture);
  assert.ok(last().left_x>0 && last().left_y>0);
  assert.equal(last().lateral_locked,false);
  assert.equal(last().client_version,'web-controlled-tests-v3');
  assert.equal(last().longitudinal_limit,.03);
  assert.equal(last().hand_support,false);
  element('#handSupport').onclick();
  assert.equal(last().hand_support,true);
  assert.ok(last().left_x>0 && last().left_y>0);
  element('#longitudinalLimit').onchange({target:{value:'0.05'}});
  assert.equal(last().longitudinal_limit,.05);
  assert.equal(last().left_x,0); assert.equal(last().left_y,0);
  stick.onpointermove(gesture);
  assert.equal(last().left_y,0);
  stick.onpointerdown(gesture);
  lock.onclick();
  assert.equal(lock.attrs['aria-pressed'],'true');
  assert.equal(last().lateral_locked,true);
  assert.equal(last().left_x,0); assert.equal(last().left_y,0);
  stick.onpointermove(gesture);
  assert.equal(last().left_y,0);
  stick.onpointerdown(gesture);
  assert.equal(last().left_x,0); assert.ok(last().left_y>0);
  assert.match(element('#leftStick .knob').style.transform,/translate\(0px,/);
  element('#rightStick').onpointerdown({pointerId:2,clientX:145,clientY:105});
  assert.ok(last().right_x>0); // Rotation remains independent.
  stick.onpointercancel(gesture);
  assert.equal(last().left_x,0); assert.equal(last().left_y,0);
  element('#stop').onclick();
  assert.equal(lock.attrs['aria-pressed'],'true');
  stick.onpointerdown(gesture);
  assert.equal(last().left_x,0);
  lock.onclick();
  assert.equal(lock.attrs['aria-pressed'],'false');
  assert.equal(last().lateral_locked,false);
  assert.equal(last().left_x,0); assert.equal(last().left_y,0);
  stick.onpointerdown(gesture);
  assert.ok(last().left_x>0);
  stick.onlostpointercapture(gesture);
  assert.equal(last().left_x,0); assert.equal(last().left_y,0);
  lock.onclick();
  element('#headMode').onclick({currentTarget:element('#headMode')});
  stick.onpointerdown(gesture);
  assert.equal(last().mode,'head'); assert.ok(last().left_x>0);
});
