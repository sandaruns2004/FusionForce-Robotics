#pragma once
#include <Arduino.h>

// =============================================================================
// COMPETITION DASHBOARD — Web Page (served at /comp)
// Real-time competition state machine control and monitoring dashboard.
//
// Features:
//   - Start/Stop competition with checkpoint selection
//   - Live progress tracker showing current subtask and state
//   - Task timeline visualization with pass/fail indicators
//   - Real-time sensor readings (ToF, color, line) during run
//   - Elapsed time display
//   - Event log
//   - Arm control shortcuts
//   - Ball color override for restart scenarios
// =============================================================================
const char COMP_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html lang="en"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Competition · RUNNER-4</title>
<style>
@import url('https://fonts.googleapis.com/css2?family=Inter:wght@300;400;600;700;800&family=Outfit:wght@300;400;700;800&display=swap');
:root{
  --bg:#080d18;--card:#0e1624;--in:#0b1020;--line:#1e2d47;
  --mut:#6b82a0;--ac:#06b6d4;--ac2:#8b5cf6;--ac3:#f59e0b;
  --red:#ef4444;--grn:#22c55e;--blue:#3b82f6;--pink:#ec4899;
}
*{box-sizing:border-box;margin:0;padding:0}
body{background:var(--bg);color:#e2eaf8;font-family:'Inter',system-ui,sans-serif;min-height:100vh}
.wrap{max-width:780px;margin:0 auto;padding:20px 16px}

/* ── Header ── */
.hdr{display:flex;align-items:center;gap:14px;margin-bottom:6px}
.hdr-icon{width:50px;height:50px;border-radius:16px;
  background:linear-gradient(135deg,#f59e0b,#ef4444);
  display:flex;align-items:center;justify-content:center;font-size:26px;flex-shrink:0;
  box-shadow:0 4px 20px rgba(245,158,11,.3)}
.hdr h1{font-family:'Outfit',sans-serif;font-size:26px;font-weight:800;
  background:linear-gradient(135deg,#f59e0b,#ef4444);-webkit-background-clip:text;-webkit-text-fill-color:transparent}
.hdr h1 small{color:var(--mut);font-size:13px;font-weight:400;margin-left:8px;
  -webkit-text-fill-color:var(--mut)}

/* ── Navigation ── */
.nav{display:flex;gap:8px;margin-bottom:20px;flex-wrap:wrap;align-items:center}
.nav a{font-size:12px;padding:5px 10px;border-radius:8px;text-decoration:none;font-weight:600;
  transition:all .15s;background:#1e2d47;color:#94a3b8}
.nav a:hover{background:#2d3f5e;color:#e2eaf8}
.nav a.active{background:linear-gradient(135deg,#f59e0b,#ef4444);color:#fff}
.nav .sep{color:var(--line);font-size:10px}

/* ── Pill badges ── */
.pill{display:inline-flex;align-items:center;gap:5px;padding:4px 11px;border-radius:99px;
  font-size:12px;font-weight:600;background:#1e2d47;transition:all .3s}
.pill.ok{background:#064e3b;color:#6ee7b7}
.pill.bad{background:#4c0519;color:#fda4af}
.pill.warn{background:#713f12;color:#fde68a}
.pill.run{background:#172554;color:#93c5fd;animation:pulse 2s infinite}
@keyframes pulse{0%,100%{box-shadow:0 0 0 0 rgba(59,130,246,.4)}50%{box-shadow:0 0 0 8px rgba(59,130,246,0)}}
@keyframes glow{0%,100%{box-shadow:0 0 8px rgba(245,158,11,.3)}50%{box-shadow:0 0 20px rgba(245,158,11,.5)}}

/* ── Cards ── */
.card{background:var(--card);border:1px solid var(--line);border-radius:18px;padding:20px;margin-bottom:16px;
  position:relative;overflow:hidden}
.card::before{content:'';position:absolute;top:0;left:0;right:0;height:1px;
  background:linear-gradient(90deg,transparent,rgba(6,182,212,.3),transparent)}
.card h2{margin:0 0 14px;font-size:15px;font-weight:700;display:flex;align-items:center;gap:8px}
.card h2 .ico{font-size:18px}

/* ── Big status display ── */
.status-hero{text-align:center;padding:30px 20px}
.status-label{font-size:12px;font-weight:600;color:var(--mut);text-transform:uppercase;letter-spacing:.12em;margin-bottom:8px}
.status-val{font-family:'Outfit',sans-serif;font-size:42px;font-weight:800;line-height:1.1;margin-bottom:6px}
.status-val.idle{color:#475569}
.status-val.running{background:linear-gradient(135deg,var(--ac),var(--ac2));
  -webkit-background-clip:text;-webkit-text-fill-color:transparent}
.status-val.done{color:var(--grn)}
.status-val.err{color:var(--red)}
.timer{font-family:'Outfit',sans-serif;font-size:56px;font-weight:800;color:#e2eaf8;
  letter-spacing:2px;margin:10px 0}
.timer-sub{font-size:12px;color:var(--mut)}

/* ── Task timeline ── */
.timeline{display:grid;grid-template-columns:repeat(4,1fr);gap:10px;margin:16px 0}
.task-block{background:var(--in);border:2px solid var(--line);border-radius:14px;
  padding:14px 10px;text-align:center;transition:all .4s;position:relative;overflow:hidden}
.task-block.active{border-color:var(--ac);box-shadow:0 0 20px rgba(6,182,212,.15);animation:glow 2s infinite}
.task-block.pass{border-color:var(--grn);background:#052e16}
.task-block.fail{border-color:var(--red);background:#450a0a}
.task-block.skip{opacity:.4}
.task-num{font-size:11px;font-weight:700;color:var(--mut);text-transform:uppercase;letter-spacing:.08em}
.task-icon{font-size:28px;margin:6px 0}
.task-name{font-size:12px;font-weight:600;color:#c9d1e0;margin-bottom:4px}
.task-status{font-size:11px;font-weight:700}
.task-status.waiting{color:#475569}
.task-status.active{color:var(--ac)}
.task-status.pass{color:var(--grn)}
.task-status.fail{color:var(--red)}
.task-progress{height:3px;background:#1e293b;border-radius:2px;margin-top:8px;overflow:hidden}
.task-progress .fill{height:100%;border-radius:2px;width:0%;
  background:linear-gradient(90deg,var(--ac),var(--ac2));transition:width .5s}

/* ── Start controls ── */
.ctrl-grid{display:grid;grid-template-columns:1fr auto;gap:12px;align-items:start}
@media(max-width:550px){.ctrl-grid{grid-template-columns:1fr}}
.start-btn{width:100%;padding:16px 20px;border:0;border-radius:14px;cursor:pointer;
  font-family:'Outfit',sans-serif;font-size:18px;font-weight:800;letter-spacing:.05em;
  transition:all .2s;position:relative;overflow:hidden}
.start-btn.go{background:linear-gradient(135deg,#16a34a,#22c55e);color:#fff;
  box-shadow:0 4px 20px rgba(34,197,94,.3)}
.start-btn.go:hover{transform:translateY(-2px);box-shadow:0 8px 30px rgba(34,197,94,.4)}
.start-btn.stop{background:linear-gradient(135deg,#dc2626,#ef4444);color:#fff;
  animation:pulse 1.5s infinite;box-shadow:0 4px 20px rgba(239,68,68,.3)}
.start-btn.stop:hover{transform:translateY(-2px)}
.start-btn:disabled{opacity:.5;cursor:not-allowed;transform:none;box-shadow:none;animation:none}
.ctrl-side{display:flex;flex-direction:column;gap:8px}
.cp-label{font-size:11px;font-weight:600;color:var(--mut);text-transform:uppercase;letter-spacing:.06em;margin-bottom:2px}
select,input[type=number]{background:var(--in);border:1px solid var(--line);border-radius:10px;
  color:#e2eaf8;padding:10px 12px;font-size:13px;font-weight:600;outline:none;width:100%;
  font-family:'Inter',sans-serif}
select:focus,input:focus{border-color:var(--ac)}

/* ── Sensor strip ── */
.sensor-strip{display:grid;grid-template-columns:repeat(3,1fr);gap:10px}
.sensor-box{background:var(--in);border:1px solid var(--line);border-radius:12px;padding:12px;text-align:center}
.sensor-box .s-icon{font-size:22px;margin-bottom:4px}
.sensor-box .s-label{font-size:10px;font-weight:600;color:var(--mut);text-transform:uppercase;letter-spacing:.06em}
.sensor-box .s-val{font-family:'Outfit',sans-serif;font-size:26px;font-weight:700;margin-top:2px}
.sensor-box .s-unit{font-size:11px;color:var(--mut)}

/* ── Ball color indicator ── */
.ball-display{display:flex;align-items:center;gap:14px;padding:12px 16px;background:var(--in);
  border-radius:14px;border:1px solid var(--line)}
.ball-circle{width:44px;height:44px;border-radius:50%;border:3px solid var(--line);
  transition:all .4s;flex-shrink:0}
.ball-circle.red{background:radial-gradient(circle at 35% 35%,#fca5a5,#ef4444,#991b1b);
  border-color:#ef4444;box-shadow:0 0 16px rgba(239,68,68,.4)}
.ball-circle.green{background:radial-gradient(circle at 35% 35%,#86efac,#22c55e,#166534);
  border-color:#22c55e;box-shadow:0 0 16px rgba(34,197,94,.4)}
.ball-circle.blue{background:radial-gradient(circle at 35% 35%,#93c5fd,#3b82f6,#1e3a8a);
  border-color:#3b82f6;box-shadow:0 0 16px rgba(59,130,246,.4)}
.ball-circle.none{background:#1e293b}
.ball-info{flex:1}
.ball-info .bi-label{font-size:11px;color:var(--mut);font-weight:600;text-transform:uppercase}
.ball-info .bi-val{font-size:18px;font-weight:700;font-family:'Outfit',sans-serif}

/* ── Arm controls ── */
.arm-grid{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}
.arm-btn{padding:10px 8px;border:0;border-radius:10px;cursor:pointer;font-size:12px;
  font-weight:700;transition:all .15s;font-family:'Inter',sans-serif}
.arm-btn:hover{transform:translateY(-1px)}
.arm-btn.pick{background:linear-gradient(135deg,#7c3aed,#a78bfa);color:#fff}
.arm-btn.place{background:linear-gradient(135deg,#0891b2,#22d3ee);color:#fff}
.arm-btn.up{background:#1e293b;color:#94a3b8}.arm-btn.down{background:#1e293b;color:#94a3b8}
.arm-btn.open{background:#1e293b;color:#94a3b8}.arm-btn.close{background:#1e293b;color:#94a3b8}
.arm-btn:hover{opacity:.9}

/* ── Event log ── */
.log{max-height:200px;overflow-y:auto;font-family:'JetBrains Mono',monospace;font-size:11px;
  color:var(--mut);background:var(--in);border-radius:12px;padding:10px;border:1px solid var(--line)}
.log div{padding:2px 0;border-bottom:1px solid rgba(30,45,71,.5)}
.log .t{color:#475569;margin-right:6px}
.log .info{color:var(--ac)}.log .warn{color:var(--ac3)}.log .err{color:var(--red)}.log .ok{color:var(--grn)}

/* ── Misc ── */
.row{display:flex;gap:12px;align-items:stretch;margin-bottom:16px}
.row>.card{flex:1;margin-bottom:0}
@media(max-width:550px){.row{flex-direction:column}}
.btn-s{padding:6px 12px;border:0;border-radius:8px;cursor:pointer;font-size:11px;font-weight:700;
  background:#1e2d47;color:#94a3b8;transition:all .15s;font-family:'Inter',sans-serif}
.btn-s:hover{background:#2d3f5e;color:#e2eaf8}
</style></head><body>
<div class="wrap">

<!-- ══ HEADER ══ -->
<div class="hdr">
  <div class="hdr-icon">🏆</div>
  <h1>Competition Control <small>RUNNER-4 · FSM v7.0</small></h1>
</div>
<nav class="nav">
  <a href="/">🤖 Robot</a>
  <a href="/line">📏 Line</a>
  <a href="/linefollow">🏁 Follow</a>
  <a href="/color">🎨 Colour</a>
  <a href="/tof">📡 ToF</a>
  <span class="sep">│</span>
  <a class="active" href="/comp">🏆 Competition</a>
  <span class="sep">│</span>
  <span class="pill" id="connPill">⏳ Connecting…</span>
</nav>

<!-- ══ STATUS HERO ══ -->
<div class="card status-hero" id="heroCard">
  <div class="status-label">Competition Status</div>
  <div class="status-val idle" id="stateText">IDLE</div>
  <div class="timer" id="timerText">00:00.0</div>
  <div class="timer-sub">Elapsed Time · 15:00 limit</div>
</div>

<!-- ══ TASK TIMELINE ══ -->
<div class="timeline" id="timeline">
  <div class="task-block" id="tb1">
    <div class="task-num">Task 1</div>
    <div class="task-icon">🔍</div>
    <div class="task-name">Grid Search</div>
    <div class="task-status waiting" id="ts1">Waiting</div>
    <div class="task-progress"><div class="fill" id="tp1"></div></div>
  </div>
  <div class="task-block" id="tb2">
    <div class="task-num">Task 2</div>
    <div class="task-icon">🌀</div>
    <div class="task-name">Corridor</div>
    <div class="task-status waiting" id="ts2">Waiting</div>
    <div class="task-progress"><div class="fill" id="tp2"></div></div>
  </div>
  <div class="task-block" id="tb3">
    <div class="task-num">Task 3</div>
    <div class="task-icon">📦</div>
    <div class="task-name">Push Box</div>
    <div class="task-status waiting" id="ts3">Waiting</div>
    <div class="task-progress"><div class="fill" id="tp3"></div></div>
  </div>
  <div class="task-block" id="tb4">
    <div class="task-num">Task 4</div>
    <div class="task-icon">🎨</div>
    <div class="task-name">Color Sort</div>
    <div class="task-status waiting" id="ts4">Waiting</div>
    <div class="task-progress"><div class="fill" id="tp4"></div></div>
  </div>
</div>

<!-- ══ CONTROLS ══ -->
<div class="card">
  <h2><span class="ico">🎮</span> Controls</h2>
  <div class="ctrl-grid">
    <div>
      <button class="start-btn go" id="startBtn" onclick="toggleComp()">▶ START COMPETITION</button>
    </div>
    <div class="ctrl-side">
      <div>
        <div class="cp-label">Start From</div>
        <select id="fromSel">
          <option value="1" selected>Task 1 — Grid Search</option>
          <option value="2">Task 2 — Corridor</option>
          <option value="3">Task 3 — Push Box</option>
          <option value="4">Task 4 — Color Sort</option>
        </select>
      </div>
      <div>
        <div class="cp-label">Ball Color Override</div>
        <select id="colorSel">
          <option value="0" selected>Auto (from Task 1)</option>
          <option value="1">🔴 Red</option>
          <option value="2">🟢 Green</option>
          <option value="3">🔵 Blue</option>
        </select>
      </div>
    </div>
  </div>
</div>

<!-- ══ LIVE SENSORS + BALL COLOR ══ -->
<div class="row">
  <div class="card">
    <h2><span class="ico">📡</span> Live Sensors</h2>
    <div class="sensor-strip">
      <div class="sensor-box">
        <div class="s-icon">📡</div>
        <div class="s-label">Front ToF</div>
        <div class="s-val" id="svTof">—</div>
        <div class="s-unit">mm</div>
      </div>
      <div class="sensor-box">
        <div class="s-icon">📏</div>
        <div class="s-label">Line Error</div>
        <div class="s-val" id="svLine">—</div>
        <div class="s-unit">mm</div>
      </div>
      <div class="sensor-box">
        <div class="s-icon">🔗</div>
        <div class="s-label">Junction</div>
        <div class="s-val" id="svJunc" style="font-size:16px">—</div>
        <div class="s-unit">&nbsp;</div>
      </div>
    </div>
  </div>
  <div class="card">
    <h2><span class="ico">🎱</span> Ball</h2>
    <div class="ball-display">
      <div class="ball-circle none" id="ballCircle"></div>
      <div class="ball-info">
        <div class="bi-label">Detected Color</div>
        <div class="bi-val" id="ballName">None</div>
      </div>
    </div>
    <div style="margin-top:12px;font-size:11px;color:var(--mut)">
      <span>Picked: </span><span class="pill" id="pickedPill" style="font-size:11px">—</span>
    </div>
  </div>
</div>

<!-- ══ ARM CONTROLS ══ -->
<div class="card">
  <h2><span class="ico">🦾</span> Arm Controls</h2>
  <div class="arm-grid">
    <button class="arm-btn pick" onclick="armCmd('pick')">🎯 PICK</button>
    <button class="arm-btn place" onclick="armCmd('place')">📥 PLACE</button>
    <button class="arm-btn up" onclick="armCmd('up')">⬆️ Up</button>
    <button class="arm-btn down" onclick="armCmd('down')">⬇️ Down</button>
    <button class="arm-btn open" onclick="armCmd('open')">✋ Open</button>
    <button class="arm-btn close" onclick="armCmd('close')">✊ Close</button>
  </div>
  <!-- Servo angle tuning sliders — GPIO 47 (Lift) and GPIO 44 (Grip) -->
  <details style="margin-top:14px">
    <summary style="cursor:pointer;font-size:12px;color:var(--mut);user-select:none">⚙️ Angle Calibration (expand)</summary>
    <div style="margin-top:10px;display:grid;gap:10px">
      <div>
        <label style="font-size:12px;color:var(--mut)">Lift Servo (GPIO 47) — <span id="liftVal">30</span>°</label>
        <input type="range" min="0" max="180" value="30" id="liftSlider"
               style="width:100%;accent-color:#7c3aed"
               oninput="$('liftVal').textContent=this.value"
               onchange="armSet('lift',this.value)">
        <div style="display:flex;gap:6px;margin-top:4px">
          <button class="btn-s" onclick="armSet('lift',30);$('liftSlider').value=30;$('liftVal').textContent=30">UP (30°)</button>
          <button class="btn-s" onclick="armSet('lift',150);$('liftSlider').value=150;$('liftVal').textContent=150">DOWN (150°)</button>
        </div>
      </div>
      <div>
        <label style="font-size:12px;color:var(--mut)">Grip Servo (GPIO 44) — <span id="gripVal">140</span>°</label>
        <input type="range" min="0" max="180" value="140" id="gripSlider"
               style="width:100%;accent-color:#0891b2"
               oninput="$('gripVal').textContent=this.value"
               onchange="armSet('grip',this.value)">
        <div style="display:flex;gap:6px;margin-top:4px">
          <button class="btn-s" onclick="armSet('grip',30);$('gripSlider').value=30;$('gripVal').textContent=30">OPEN (30°)</button>
          <button class="btn-s" onclick="armSet('grip',140);$('gripSlider').value=140;$('gripVal').textContent=140">CLOSE (140°)</button>
        </div>
      </div>
      <div style="font-size:11px;color:var(--mut)">
        💡 Drag sliders to find ideal angles, then update <code>ARM_LIFT_UP/DOWN_DEG</code>
        and <code>ARM_GRIP_OPEN/CLOSE_DEG</code> in <code>arm_interface.h</code>
      </div>
    </div>
  </details>
  <div id="armMsg" style="margin-top:8px;font-size:11px;color:var(--mut)"></div>
</div>

<!-- ══ EVENT LOG ══ -->
<div class="card">
  <h2 style="margin-bottom:4px"><span class="ico">📝</span> Event Log
    <button class="btn-s" onclick="$('logbox').innerHTML=''" style="margin-left:auto">Clear</button>
  </h2>
  <div class="log" id="logbox"></div>
</div>

</div>

<script>
const $=id=>document.getElementById(id);

// ── State constants ──
const STATES=['IDLE','STARTING','TASK1_GRID','TASK2_CORRIDOR','TASK3_OBSTACLE','TASK4_COLOR','FINISHED','ABORTED','ERROR'];
const TASK_NAMES=['Grid Search','Corridor','Push Box','Color Sort'];
const COLOR_MAP={0:'none',1:'red',2:'green',3:'blue'};
const COLOR_NAMES={0:'None',1:'Red',2:'Green',3:'Blue'};

let isRunning=false;
let lastState='';
let pollFails=0;
let localTimer=null;
let localStart=0;

// ── Logging ──
function addLog(msg,cls='info'){
  const d=document.createElement('div');
  const t=document.createElement('span');
  t.className='t';t.textContent=new Date().toLocaleTimeString();
  const m=document.createElement('span');
  m.className=cls;m.textContent=' '+msg;
  d.appendChild(t);d.appendChild(m);
  $('logbox').appendChild(d);
  $('logbox').scrollTop=$('logbox').scrollHeight;
}

// ── Start / Stop ──
async function toggleComp(){
  const btn=$('startBtn');
  if(isRunning){
    // STOP
    try{
      const r=await(await fetch('/comp/stop')).json();
      addLog('ABORT requested','warn');
      btn.disabled=true;
      setTimeout(()=>{btn.disabled=false},2000);
    }catch(e){addLog('Stop failed: '+e.message,'err')}
  }else{
    // START
    const from=$('fromSel').value;
    const color=$('colorSel').value;
    let url='/comp/start?from='+from;
    if(color!='0')url+='&color='+color;
    try{
      const r=await(await fetch(url)).json();
      if(r.ok){
        addLog('STARTED from Task '+from,'ok');
        localStart=Date.now();
        localTimer=setInterval(updateLocalTimer,100);
      }else{
        addLog('Start failed: '+r.msg,'err');
      }
    }catch(e){addLog('Start failed: '+e.message,'err')}
  }
}

function updateLocalTimer(){
  if(!isRunning&&localTimer){clearInterval(localTimer);localTimer=null;return}
  const ms=Date.now()-localStart;
  $('timerText').textContent=formatTime(ms);
}

function formatTime(ms){
  const min=Math.floor(ms/60000);
  const sec=Math.floor((ms/1000)%60);
  const dec=Math.floor((ms/100)%10);
  return String(min).padStart(2,'0')+':'+String(sec).padStart(2,'0')+'.'+dec;
}

// ── Arm commands ──
async function armCmd(cmd){
  $('armMsg').textContent='Sending: '+cmd+'...';
  try{
    const r=await fetch('/comp/arm?cmd='+cmd);
    const t=await r.text();
    $('armMsg').textContent='Response: '+t;
    addLog('Arm: '+cmd+' → '+t);
  }catch(e){
    $('armMsg').textContent='Error: '+e.message;
    addLog('Arm error: '+e.message,'err');
  }
}

// ── Arm angle calibration (direct servo write via /arm/set) ──
async function armSet(servo, deg){
  try{
    const r=await fetch('/arm/set?'+servo+'='+deg);
    const t=await r.text();
    $('armMsg').textContent=servo+'='+deg+'° → '+t;
  }catch(e){
    $('armMsg').textContent='armSet error: '+e.message;
  }
}

// ── Update UI from status data ──
function updateUI(d){
  const stId=d.stateId||0;
  const state=d.state||'IDLE';
  const running=stId>=1&&stId<=5;
  const finished=stId===6;
  const aborted=stId===7;
  const errored=stId===8;

  // Detect state transitions
  if(state!==lastState){
    if(lastState)addLog('State: '+lastState+' → '+state, running?'info':finished?'ok':'warn');
    lastState=state;
  }

  isRunning=running;

  // Connection pill
  $('connPill').textContent='✅ Connected';
  $('connPill').className='pill ok';
  pollFails=0;

  // Hero status
  const sv=$('stateText');
  sv.textContent=state;
  sv.className='status-val '+(running?'running':finished?'done':(aborted||errored)?'err':'idle');

  // Timer
  if(d.elapsed>0)$('timerText').textContent=formatTime(d.elapsed);

  // Start button
  const btn=$('startBtn');
  if(running){
    btn.className='start-btn stop';
    btn.textContent='■ EMERGENCY STOP';
    btn.disabled=false;
  }else{
    btn.className='start-btn go';
    btn.textContent='▶ START COMPETITION';
    btn.disabled=false;
    if(localTimer){clearInterval(localTimer);localTimer=null}
  }

  // Disable selectors while running
  $('fromSel').disabled=running;
  $('colorSel').disabled=running;

  // Task timeline
  const from=d.from||1;
  for(let i=1;i<=4;i++){
    const tb=$('tb'+i);
    const ts=$('ts'+i);
    const tp=$('tp'+i);
    const taskOk=d['task'+i];
    const isActive=stId===i+1; // stateId 2=TASK1, 3=TASK2, 4=TASK3, 5=TASK4
    const isPast=stId>i+1||finished;
    const isSkipped=i<from;

    // Reset classes
    tb.className='task-block';
    ts.className='task-status';

    if(isSkipped&&!isPast){
      tb.classList.add('skip');
      ts.textContent='Skipped';
      ts.classList.add('waiting');
      tp.style.width='0%';
    }else if(isActive){
      tb.classList.add('active');
      ts.textContent='RUNNING';
      ts.classList.add('active');
      tp.style.width='50%';
    }else if(isPast){
      tb.classList.add(taskOk?'pass':'fail');
      ts.textContent=taskOk?'PASS ✓':'FAIL ✗';
      ts.classList.add(taskOk?'pass':'fail');
      tp.style.width='100%';
    }else{
      ts.textContent='Waiting';
      ts.classList.add('waiting');
      tp.style.width='0%';
    }
  }

  // Ball color
  const colorId=d.ballColor||0;
  const cn=COLOR_MAP[colorId]||'none';
  $('ballCircle').className='ball-circle '+cn;
  $('ballName').textContent=d.colorName||COLOR_NAMES[colorId]||'None';
  $('ballName').style.color=colorId===1?'#ef4444':colorId===2?'#22c55e':colorId===3?'#3b82f6':'#94a3b8';
}

// ── Update sensor readings (fetched from existing endpoints) ──
async function updateSensors(){
  try{
    const [tof,line]=await Promise.all([
      fetch('/tof/data').then(r=>r.json()).catch(()=>null),
      fetch('/line/data').then(r=>r.json()).catch(()=>null)
    ]);
    if(tof&&tof.ok){
      $('svTof').textContent=tof.front.toFixed(0);
      $('svTof').style.color=tof.front<150?'var(--red)':tof.front<300?'var(--ac3)':'var(--grn)';
    }
    if(line){
      const err=line.err;
      $('svLine').textContent=typeof err==='number'?err.toFixed(1):'—';
      const junc=line.junction;
      const jtype=line.jtype||'—';
      $('svJunc').textContent=junc?jtype:'—';
      $('svJunc').style.color=junc?'var(--ac3)':'#475569';
    }
  }catch(e){}
}

// ── Polling ──
async function poll(){
  if(document.hidden){setTimeout(poll,2000);return}
  try{
    const r=await fetch('/comp/status',{cache:'no-store'});
    if(!r.ok)throw new Error('HTTP '+r.status);
    const d=await r.json();
    updateUI(d);
    updateSensors();
  }catch(e){
    pollFails++;
    if(pollFails>=3){
      $('connPill').textContent='❌ Disconnected';
      $('connPill').className='pill bad';
    }
  }
  setTimeout(poll,isRunning?600:1200);
}

addLog('Dashboard loaded — connecting to RUNNER-4...');
poll();
</script></body></html>
)rawliteral";
