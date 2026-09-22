const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');

test('IMU fault requires acknowledgement and reflects server pause state', () => {
  const elements = new Map();
  function element(selector) {
    if (!elements.has(selector)) elements.set(selector, {
      textContent:'', disabled:false, style:{},
      classList:{toggle(){}, remove(){}, add(){}},
      querySelector: child => element(selector+' '+child),
    });
    return elements.get(selector);
  }
  let socket;
  class WebSocket {
    static OPEN = 1;
    constructor(){socket=this;this.readyState=1;this.sent=[];}
    send(data){this.sent.push(JSON.parse(data));}
  }
  const context = {
    URLSearchParams, WebSocket, location:{search:'?token=test',protocol:'http:',host:'localhost'},
    document:{querySelector:element,querySelectorAll:()=>[],addEventListener(){}},
    addEventListener(){}, setInterval(){}, clearInterval(){}, setTimeout(){},
    fetch:()=>Promise.resolve({json:()=>Promise.resolve({})}),
  };
  vm.runInNewContext(fs.readFileSync(path.join(__dirname,'../mini_bdx_runtime/mini_bdx_runtime/web/control.js'),'utf8'),context);
  const receive = status => socket.onmessage({data:JSON.stringify({type:'runtime_status',...status})});
  receive({paused:false,imu_fault:null});
  assert.equal(element('#pause').textContent,'PAUSAR');
  receive({paused:true,imu_fault:'stale',fault_acknowledged:false});
  assert.equal(element('#pause').disabled,true);
  assert.equal(element('#pause').textContent,'INICIAR');
  assert.match(element('#imuFault').textContent,/PARAR/);
  element('#stop').onclick();
  assert.equal(socket.sent.at(-1).paused,true);
  receive({paused:true,imu_fault:'stale',fault_acknowledged:true});
  assert.equal(element('#pause').disabled,false);
  element('#pause').onclick();
  assert.equal(socket.sent.at(-1).paused,false);
  receive({paused:false,imu_fault:null});
  assert.equal(element('#pause').textContent,'PAUSAR');
  assert.equal(element('#imuFault').textContent,'');
  receive({paused:true,imu_fault:null,runtime_fault:'Limite de 30 s atingido',fault_acknowledged:false});
  assert.equal(element('#pause').disabled,true);
  assert.match(element('#imuFault').textContent,/30 s/);
  assert.match(element('#imuFault').textContent,/PARAR/);
  receive({paused:true,runtime_fault:'Limite de 30 s atingido',fault_acknowledged:true});
  assert.equal(element('#pause').disabled,false);
});
