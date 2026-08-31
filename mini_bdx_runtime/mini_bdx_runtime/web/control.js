(() => {
  const token = new URLSearchParams(location.search).get('token') || '';
  const state = {left_x:0,left_y:0,right_x:0,right_y:0,mode:'walk',buttons:{},left_trigger:0,right_trigger:0};
  let paused = true;
  const status = document.querySelector('#status');
  let socket, timer, reconnectDelay = 500;

  function setStatus(online, label) {
    status.classList.toggle('online', online);
    status.querySelector('span').textContent = label;
  }
  function zeroMotion() {
    state.left_x=state.left_y=state.right_x=state.right_y=0;
    state.buttons={}; state.left_trigger=state.right_trigger=0;
    document.querySelectorAll('.knob').forEach(k => k.style.transform='translate(0,0)');
    document.querySelectorAll('.active').forEach(b => b.classList.remove('active'));
    send();
  }
  function send() {
    if (socket?.readyState === WebSocket.OPEN) socket.send(JSON.stringify({type:'command',...state}));
  }
  function connect() {
    if (!token) { setStatus(false,'Token ausente'); return; }
    const protocol = location.protocol === 'https:' ? 'wss:' : 'ws:';
    socket = new WebSocket(`${protocol}//${location.host}/ws?token=${encodeURIComponent(token)}`);
    socket.onopen=()=>{reconnectDelay=500;setStatus(true,'Conectado');timer=setInterval(send,50);send()};
    socket.onclose=()=>{clearInterval(timer);zeroMotion();setStatus(false,'Reconectando…');setTimeout(connect,reconnectDelay);reconnectDelay=Math.min(5000,reconnectDelay*1.6)};
    socket.onerror=()=>socket.close();
  }

  function joystick(element, prefix, horizontalOnly=false) {
    const knob=element.querySelector('.knob'); let pointer=null;
    function update(event) {
      const r=element.getBoundingClientRect(), max=r.width*.31;
      let dx=event.clientX-(r.left+r.width/2), dy=event.clientY-(r.top+r.height/2);
      if(horizontalOnly) dy=0;
      const length=Math.hypot(dx,dy); if(length>max){dx*=max/length;dy*=max/length}
      state[`${prefix}_x`]=Math.abs(dx/max)<.08?0:dx/max;
      state[`${prefix}_y`]=Math.abs(dy/max)<.08?0:-dy/max;
      knob.style.transform=`translate(${dx}px,${dy}px)`; send();
    }
    function end(event){if(pointer!==event.pointerId)return;pointer=null;state[`${prefix}_x`]=state[`${prefix}_y`]=0;knob.style.transform='translate(0,0)';send()}
    element.onpointerdown=e=>{pointer=e.pointerId;element.setPointerCapture(pointer);update(e)};
    element.onpointermove=e=>{if(e.pointerId===pointer)update(e)};
    element.onpointerup=end;element.onpointercancel=end;
  }
  joystick(document.querySelector('#leftStick'),'left');
  joystick(document.querySelector('#rightStick'),'right',true);

  function bindMomentary(button, name) {
    const down=e=>{e.preventDefault();state.buttons[name]=true;button.classList.add('active');send()};
    const up=e=>{e.preventDefault();state.buttons[name]=false;button.classList.remove('active');send()};
    button.onpointerdown=down;button.onpointerup=up;button.onpointercancel=up;button.onpointerleave=e=>{if(e.buttons)up(e)};
  }
  document.querySelectorAll('[data-button]').forEach(b=>bindMomentary(b,b.dataset.button));
  bindMomentary(document.querySelector('#sprint'),'LB');
  const pauseButton=document.querySelector('#pause');
  pauseButton.onclick=()=>{paused=!paused;state.paused=paused;pauseButton.textContent=paused?'INICIAR':'PAUSAR';pauseButton.classList.toggle('active',!paused);if(paused)zeroMotion();send();setTimeout(()=>{delete state.paused},120)};
  document.querySelector('#stop').onclick=()=>{paused=true;state.paused=true;pauseButton.textContent='INICIAR';pauseButton.classList.remove('active');zeroMotion();send();setTimeout(()=>{delete state.paused},120)};
  document.querySelector('#headMode').onclick=e=>{state.mode=state.mode==='head'?'walk':'head';e.currentTarget.classList.toggle('active',state.mode==='head');zeroMotion()};
  document.querySelector('#fullscreen').onclick=()=>document.querySelector('#videoPanel').requestFullscreen?.();
  addEventListener('blur',zeroMotion);document.addEventListener('visibilitychange',()=>{if(document.hidden)zeroMotion()});
  addEventListener('contextmenu',e=>e.preventDefault());

  fetch('/api/config').then(r=>r.json()).then(config=>{
    if(config.camera){const p=document.querySelector('#videoPanel');p.classList.remove('hidden');document.querySelector('#video').src=`/stream.mjpg?token=${encodeURIComponent(token)}`}
    if(config.head_control)document.querySelector('#headMode').classList.remove('hidden');
  });
  connect();
})();
