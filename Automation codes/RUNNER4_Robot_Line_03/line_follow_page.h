#pragma once
#include <Arduino.h>

// =============================================================================
// LINE FOLLOW LAB — Web Page (served at /linefollow)
// Real-time tuning & testing dashboard for proportional line following
// =============================================================================
const char LF_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Line Follow Lab · RUNNER-4</title>
<style>
:root{--bg:#0b0f1a;--card:#111827;--in:#0d1424;--line:#1e293b;--mut:#8da0bd;--ac:#38bdf8;--ok:#22c55e;--er:#ef4444;--warn:#f59e0b;--pur:#a78bfa}
*{box-sizing:border-box}body{margin:0;font:14px 'Segoe UI',system-ui,sans-serif;background:var(--bg);color:#e8eefc}
.wrap{max-width:1100px;margin:auto;padding:14px}
.hdr{display:flex;align-items:center;gap:14px;flex-wrap:wrap;margin-bottom:10px}
h1{margin:0;font-size:22px;font-weight:800;background:linear-gradient(135deg,#38bdf8,#a78bfa);-webkit-background-clip:text;-webkit-text-fill-color:transparent}
h1 small{font-size:12px;color:var(--mut);-webkit-text-fill-color:var(--mut)}
.nav{display:flex;gap:6px;margin-left:auto;flex-wrap:wrap}
.nav a{font-size:12px;padding:5px 10px;border-radius:8px;text-decoration:none;font-weight:600;transition:all .15s}
.nav a.r{background:linear-gradient(135deg,#1f6feb,#58a6ff);color:#fff}
.nav a.l{background:linear-gradient(135deg,#0891b2,#22d3ee);color:#fff}
.nav a.c{background:linear-gradient(135deg,#7c3aed,#ec4899);color:#fff}
.pill{display:inline-block;padding:3px 9px;border-radius:99px;font-size:11px;font-weight:600;background:#334155;margin:0 4px 4px 0}
.ok{background:#14532d;color:#bbf7d0}.bad{background:#7f1d1d;color:#fecaca}.warn{background:#713f12;color:#fde68a}
.card{background:var(--card);border:1px solid var(--line);border-radius:14px;padding:16px;margin-top:12px}
.card h2{margin:0 0 12px;font-size:15px;font-weight:700;display:flex;align-items:center;gap:8px}
.card h2 .ico{font-size:18px}

/* Hero dashboard */
.dash{display:grid;grid-template-columns:1fr 1fr 1fr auto;gap:16px;align-items:center}
@media(max-width:700px){.dash{grid-template-columns:1fr 1fr;gap:10px}}
.big{font-size:36px;font-weight:800;font-family:monospace;line-height:1}.mut{color:var(--mut);font-size:12px}
.track{position:relative;height:20px;background:linear-gradient(90deg,#1e40af,#1e293b 48%,#334155 50%,#1e293b 52%,#b91c1c);border-radius:10px;margin-top:10px;border:1px solid var(--line)}
.track::after{content:'';position:absolute;left:50%;top:0;bottom:0;width:2px;background:#475569}
#mk{position:absolute;top:-4px;width:28px;height:28px;margin-left:-14px;border-radius:50%;background:var(--ac);box-shadow:0 0 16px var(--ac);left:50%;transition:left .12s}

/* Hip delta bars */
.hip-bars{display:flex;gap:12px;margin-top:10px;align-items:center}
.hip-bar{flex:1;display:flex;align-items:center;gap:8px}
.hip-bar .label{font-size:11px;color:var(--mut);min-width:42px;font-weight:700}
.hip-bar .bar-bg{flex:1;height:14px;background:var(--in);border-radius:7px;overflow:hidden;border:1px solid var(--line)}
.hip-bar .bar-fill{height:100%;border-radius:7px;transition:width .15s}
.hip-bar .bar-fill.left{background:linear-gradient(90deg,#3b82f6,#60a5fa)}
.hip-bar .bar-fill.right{background:linear-gradient(90deg,#22c55e,#4ade80)}
.hip-bar .val{font-size:13px;font-weight:700;font-family:monospace;min-width:30px;text-align:right}

/* Sensor array */
.sens{display:grid;grid-template-columns:repeat(9,1fr);gap:5px}
.sc{background:var(--in);border-radius:8px;padding:6px 3px;text-align:center;border:2px solid transparent;transition:border-color .15s}
.sc.on{border-color:#facc15}.sc.dead{opacity:.3}
.bw{position:relative;height:70px;background:#020617;border-radius:5px;margin:4px auto;width:70%;overflow:hidden}
.bw i{position:absolute;left:0;right:0;bottom:0;background:linear-gradient(#e2e8f0,#64748b);transition:height .1s}
.bw u{position:absolute;left:0;right:0;border-top:2px dashed #f87171}
.rv{font-size:10px;color:var(--mut)}.nv{font-size:12px;font-weight:700}.sn{font-size:11px;color:var(--ac)}

/* Canvas */
canvas{width:100%;height:130px;background:var(--in);border-radius:10px;margin-top:8px}

/* Controls */
.ctrl-row{display:flex;gap:10px;align-items:center;flex-wrap:wrap;margin-bottom:8px}
button{font:inherit;border-radius:9px;border:0;padding:10px 16px;cursor:pointer;font-weight:700;transition:all .15s;font-size:13px}
.btn-start{background:linear-gradient(135deg,#1a7f37,#3fb950);color:#fff;flex:1;min-width:140px}
.btn-start:hover{box-shadow:0 4px 14px rgba(63,185,80,.4);transform:translateY(-1px)}
.btn-start.running{background:linear-gradient(135deg,#b91c1c,#f85149);animation:pulse 1.5s infinite}
@keyframes pulse{0%,100%{box-shadow:0 0 0 0 rgba(248,81,73,.4)}50%{box-shadow:0 0 0 6px rgba(248,81,73,.0)}}
.btn-s{background:#334155;color:#c9d1d9}.btn-s:hover{background:#475569}
.btn-cal{background:linear-gradient(135deg,#2563eb,#60a5fa);color:#fff}
.step-in{width:55px;background:var(--in);border:1px solid var(--line);border-radius:8px;color:#e8eefc;padding:6px 8px;font-size:13px;text-align:center;outline:none}
.step-in:focus{border-color:var(--ac)}

/* Sliders */
.ctl{margin-bottom:10px}.ch{display:flex;justify-content:space-between;font-size:12px}.ch b{color:var(--ac);font-family:monospace}
input[type=range]{width:100%;-webkit-appearance:none;appearance:none;height:4px;border-radius:4px;background:#1e293b;outline:none;cursor:pointer;margin:4px 0}
input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:14px;height:14px;border-radius:50%;background:var(--ac);cursor:pointer;border:2px solid var(--bg)}
.toggle-row{display:flex;gap:16px;flex-wrap:wrap;margin-top:8px}
label{font-size:12px;display:flex;gap:5px;align-items:center;cursor:pointer}

/* Speed row */
.speed-row{display:flex;align-items:center;gap:8px;font-size:11px;color:var(--mut)}
.speed-val{font-size:11px;color:#6e7681;text-align:center;margin-top:2px}

/* Cal bar */
.bar{height:8px;background:#020617;border-radius:6px;margin-top:6px;overflow:hidden}.bar i{display:block;height:100%;width:0;background:var(--ac);transition:width .3s}

/* Diagnostics grid */
.dgrid{display:grid;grid-template-columns:repeat(auto-fit,minmax(130px,1fr));gap:8px}
.dbox{background:var(--in);border-radius:8px;padding:8px 10px;border:1px solid var(--line)}
.dbox .dt{font-size:10px;color:var(--mut);text-transform:uppercase;letter-spacing:1px}.dbox .dv{font-size:18px;font-weight:700;font-family:monospace;margin-top:2px}

/* Log */
.log{max-height:120px;overflow-y:auto;font-family:monospace;font-size:11px;color:var(--mut);margin-top:8px;background:var(--in);border-radius:8px;padding:8px}
.log div{padding:1px 0}
</style></head><body><div class="wrap">

<div class="hdr">
<h1>🤖 Line Follow Lab <small>RUNNER-4 · Proportional Steering · 9×TCRT5000</small></h1>
<div class="nav">
<a class="r" href="/">← Robot</a>
<a class="l" href="/line">📏 Line Array</a>
<a class="c" href="/color">🎨 Colour</a>
</div>
</div>
<span id="pk" class="pill">Array…</span><span id="lk" class="pill">WiFi…</span><span id="st" class="pill">Status</span>

<!-- ══ LIVE DASHBOARD ══ -->
<div class="card">
<h2><span class="ico">📡</span> Live Dashboard</h2>
<div class="dash">
<div><div class="mut">LINE ERROR (mm)</div><div id="err" class="big" style="color:#38bdf8">–</div><div class="mut">− left &nbsp;·&nbsp; + right</div></div>
<div><div class="mut">CORRECTION (°)</div><div id="corr" class="big" style="color:#f59e0b">–</div><div class="mut">applied to hips</div></div>
<div><div class="mut">ACTIVE SENSORS</div><div id="act" class="big" style="color:#22c55e">0</div><div class="mut">of 9 above threshold</div></div>
<div><div class="mut">JUNCTION</div><div id="jun" class="pill">–</div><div class="mut" style="margin-top:6px">LOST</div><div id="lost" class="pill">–</div></div>
</div>
<div class="track"><div id="mk"></div></div>
<div class="hip-bars">
<div class="hip-bar"><span class="label">L Hip Δ</span><div class="bar-bg"><div class="bar-fill left" id="hipLbar" style="width:50%"></div></div><span class="val" id="hipLval">20°</span></div>
<div class="hip-bar"><span class="label">R Hip Δ</span><div class="bar-bg"><div class="bar-fill right" id="hipRbar" style="width:50%"></div></div><span class="val" id="hipRval">20°</span></div>
</div>
<canvas id="cv" width="900" height="130"></canvas>
<div class="mut" style="margin-top:4px">Blue = error (±40mm) · Orange = correction (±15°) · last 200 samples</div>
</div>

<!-- ══ SENSOR ARRAY ══ -->
<div class="card">
<h2><span class="ico">📏</span> Sensor Array (S1 left → S9 right)</h2>
<div id="sens" class="sens"></div>
<div class="mut" style="margin-top:6px">Yellow border = active · bar = normalised · dashed red = threshold</div>
</div>

<!-- ══ CONTROLS ══ -->
<div class="card">
<h2><span class="ico">🎮</span> Controls</h2>
<div class="ctrl-row">
<button class="btn-start" id="lfBtn" onclick="toggleLF()">▶ START LINE FOLLOW</button>
<span class="mut">Steps:</span><input type="number" id="lfSteps" class="step-in" min="0" max="500" value="0"><span class="mut" style="font-size:11px">(0 = ∞)</span>
</div>
<div class="speed-row" style="margin-top:8px">
<span>Fast</span>
<input type="range" id="lfSpeedSl" min="100" max="2000" value="500" oninput="$('lfSpeedVal').textContent=this.value+'ms'; sv('speed',this.value)">
<span>Slow</span>
</div>
<div class="speed-val"><span id="lfSpeedVal">500ms</span> per phase (6 phases per cycle)</div>
<div class="speed-row" style="margin-top:6px">
<span>Short</span>
<input type="range" id="lfStrideSl" min="5" max="40" value="20" oninput="$('lfStrideVal').textContent=this.value+'°'; sv('hip',this.value)">
<span>Long</span>
</div>
<div class="speed-val">Stride: <span id="lfStrideVal">20°</span> hip swing</div>
<div class="speed-row" style="margin-top:6px">
<span>0.3×</span>
<input type="range" id="lfSlowSl" min="30" max="300" value="100" step="10" oninput="$('lfSlowVal').textContent=(this.value/100).toFixed(1)+'×'; sv('slow',this.value/100)">
<span>3.0×</span>
</div>
<div class="speed-val">Speed factor: <span id="lfSlowVal">1.0×</span></div>
</div>

<!-- ══ PID TUNING ══ -->
<div class="card">
<h2><span class="ico">🎚️</span> Steering Tuning (PD Controller)</h2>
<div id="sl"></div>
<div class="toggle-row">
<label><input type="checkbox" id="steerInv" checked onchange="sv('sign',this.checked?-1:1)"> Invert steering direction</label>
<label><input type="checkbox" id="perPhase" checked onchange="sv('sample',this.checked?1:0)"> Sample per-phase (faster response)</label>
<label><input type="checkbox" id="stopLost" checked onchange="sv('stopLost',this.checked?1:0)"> Stop when line lost</label>
<label><input type="checkbox" id="slowOnErr" checked onchange="sv('slowErr',this.checked?1:0)"> Slow down on large error</label>
</div>
</div>

<!-- ══ CALIBRATION ══ -->
<div class="card">
<h2><span class="ico">🧪</span> Quick Calibration</h2>
<div class="mut">Slide the array across BLACK floor and WHITE line for 5 seconds, then Save.</div>
<div class="ctrl-row" style="margin-top:8px">
<button class="btn-cal" onclick="cmd('cal_start','/line/cmd')">▶ Start 5s Sweep</button>
<button class="btn-s" onclick="cmd('cal_stop','/line/cmd')">■ Finish Now</button>
<button class="btn-s" onclick="cmd('cal_clear','/line/cmd')">Clear Cal</button>
<button class="btn-s" onclick="cmd('save','/line/cmd')">💾 Save Cal to Flash</button>
</div>
<div class="bar" style="margin-top:8px"><i id="calbar"></i></div>
<div id="calmsg" class="mut" style="margin-top:4px"></div>
</div>

<!-- ══ DIAGNOSTICS ══ -->
<div class="card">
<h2><span class="ico">📊</span> Diagnostics</h2>
<div class="dgrid" id="diag">
<div class="dbox"><div class="dt">Cycles</div><div class="dv" id="dCycles">0</div></div>
<div class="dbox"><div class="dt">Phase</div><div class="dv" id="dPhase">–</div></div>
<div class="dbox"><div class="dt">Avg Error</div><div class="dv" id="dAvgErr">–</div></div>
<div class="dbox"><div class="dt">Max Error</div><div class="dv" id="dMaxErr">–</div></div>
<div class="dbox"><div class="dt">Lost Events</div><div class="dv" id="dLost">0</div></div>
<div class="dbox"><div class="dt">Junctions</div><div class="dv" id="dJun">0</div></div>
</div>
<div class="ctrl-row" style="margin-top:10px">
<button class="btn-s" onclick="cmd('save','/lf/cmd')">💾 Save Config</button>
<button class="btn-s" onclick="cmd('resetdiag','/lf/cmd')">Reset Diag</button>
<button class="btn-s" onclick="if(confirm('Reset all line-follow tuning to defaults?'))cmd('reset','/lf/cmd')">Reset Tuning</button>
</div>
<div id="msg" class="mut" style="margin-top:4px"></div>
</div>

<!-- ══ LOG ══ -->
<div class="card">
<h2 style="margin-bottom:4px"><span class="ico">📝</span> Event Log <button class="btn-s" onclick="$('logbox').innerHTML=''" style="padding:3px 8px;font-size:11px;margin-left:auto">Clear</button></h2>
<div class="log" id="logbox"></div>
</div>

</div>
<script>
const $=id=>document.getElementById(id);
let errHist=[],corrHist=[];
let tm={};
let wasRunning=false;
let dataMisses=0;

// Slider config: [key, label, min, max, step]
const SL=[
  ['kp','Kp — proportional (°/mm)',0,2,.01],
  ['kd','Kd — derivative',0,2,.01],
  ['maxcorr','Max correction (°)',1,40,1],
  ['alpha','Sensor EMA α (during follow)',.1,1,.05],
  ['slowThresh','Slow-down error threshold (mm)',5,40,1],
  ['slowMult','Slow-down speed multiplier',1,3,.1],
  ['fem','Femur lift (°)',5,40,1],
];
$('sl').innerHTML=SL.map(s=>`<div class=ctl><div class=ch><span>${s[1]}</span><b id=v_${s[0]}></b></div><input type=range id=s_${s[0]} min=${s[2]} max=${s[3]} step=${s[4]} oninput="sv('${s[0]}',this.value)"></div>`).join('');

// Sensor bars
$('sens').innerHTML=Array.from({length:9},(_,i)=>`<div class=sc id=sc${i}><div class=rv id=rv${i}>0</div><div class=bw><i id=bar${i}></i><u id=th${i}></u></div><div class=nv id=nv${i}>0.00</div><div class=sn>S${i+1}</div></div>`).join('');

function sv(k,v){
  if($('v_'+k))$('v_'+k).textContent=v;
  clearTimeout(tm[k]);
  tm[k]=setTimeout(()=>fetch('/lf/set?'+k+'='+v),120);
}

function cmd(n,url){
  fetch((url||'/lf/cmd')+'?n='+n).then(r=>r.text()).then(t=>{
    $('msg').textContent=t;
    addLog(t);
    if(n=='reset')loadCfg();
  });
}

async function loadCfg(){
  try{
    const c=await(await fetch('/lf/config')).json();
    for(const s of SL){if($('s_'+s[0]))$('s_'+s[0]).value=c[s[0]];if($('v_'+s[0]))$('v_'+s[0]).textContent=c[s[0]];}
    $('lfSpeedSl').value=c.speed; $('lfSpeedVal').textContent=c.speed+'ms';
    $('lfStrideSl').value=c.hip; $('lfStrideVal').textContent=c.hip+'°';
    $('lfSlowSl').value=Math.round(c.slow*100); $('lfSlowVal').textContent=c.slow.toFixed(1)+'×';
    $('steerInv').checked=c.sign<0;
    $('perPhase').checked=c.sample===1;
    $('stopLost').checked=c.stopLost===1;
    $('slowOnErr').checked=c.slowErr===1;
  }catch(e){}
}

function pill(id,t,cls){$(id).textContent=t;$(id).className='pill '+cls}

function addLog(msg){
  const d=document.createElement('div');
  d.textContent=new Date().toLocaleTimeString()+' » '+msg;
  $('logbox').appendChild(d);
  $('logbox').scrollTop=$('logbox').scrollHeight;
}

// Toggle line follow
async function toggleLF(){
  const btn=$('lfBtn');
  if(btn.classList.contains('running')){
    try{await fetch('/lf/stop')}catch(e){}
    btn.classList.remove('running');
    btn.textContent='▶ START LINE FOLLOW';
    addLog('Stop requested');
  }else{
    const steps=$('lfSteps').value||0;
    const speed=$('lfSpeedSl').value||500;
    try{
      const r=await(await fetch('/lf/start?steps='+steps+'&speed='+speed)).json();
      if(r.ok){
        btn.classList.add('running');
        btn.textContent='■ STOP';
        addLog('Started: steps='+steps+' speed='+speed+'ms');
      }else{
        addLog('Error: '+r.msg);
      }
    }catch(e){addLog('Failed to start: '+e.message);}
  }
}

// Draw error + correction history
function draw(){
  const cv=$('cv'),x=cv.getContext('2d'),W=cv.width,H=cv.height;
  x.clearRect(0,0,W,H);
  // Grid
  x.strokeStyle='#1e293b';x.lineWidth=1;
  for(let i=0;i<5;i++){x.beginPath();x.moveTo(0,H*i/4);x.lineTo(W,H*i/4);x.stroke()}
  x.beginPath();x.moveTo(0,H/2);x.lineTo(W,H/2);x.stroke();
  // Error trace (blue)
  if(errHist.length>1){
    x.strokeStyle='#38bdf8';x.lineWidth=2;x.beginPath();
    errHist.forEach((e,i)=>{const X=i*W/199,Y=H/2-(Math.max(-40,Math.min(40,e))/40)*H/2;i?x.lineTo(X,Y):x.moveTo(X,Y)});
    x.stroke();
  }
  // Correction trace (orange)
  if(corrHist.length>1){
    x.strokeStyle='#f59e0b';x.lineWidth=2;x.beginPath();
    corrHist.forEach((c,i)=>{const X=i*W/199,Y=H/2-(Math.max(-15,Math.min(15,c))/15)*H/2;i?x.lineTo(X,Y):x.moveTo(X,Y)});
    x.stroke();
  }
}

// Render status data
function renderStatus(d){
  // Status pills
  pill('pk',d.cal?'Calibrated':'NO CAL',d.cal?'ok':'bad');
  pill('st',d.running?'FOLLOWING':'Idle',d.running?'ok':'');

  // Dashboard values
  $('err').textContent=d.found?d.err.toFixed(1):'–';
  $('err').style.color=d.found?(Math.abs(d.err)>20?'#ef4444':'#38bdf8'):'#6e7681';
  $('corr').textContent=d.running?d.correction.toFixed(1):'–';
  $('act').textContent=d.act;

  // Track marker
  const pct=d.found?((d.err+40)/80)*100:50;
  $('mk').style.left=Math.max(2,Math.min(98,pct))+'%';
  $('mk').style.opacity=d.found?1:.25;

  // Hip delta bars
  const maxDelta=45;
  $('hipLbar').style.width=(d.hipL/maxDelta*100)+'%';
  $('hipRbar').style.width=(d.hipR/maxDelta*100)+'%';
  $('hipLval').textContent=d.hipL+'°';
  $('hipRval').textContent=d.hipR+'°';

  // Junction / Lost
  if(d.junction)pill('jun',d.jtype,'warn');else pill('jun','none','');
  if(d.safe)pill('lost','SAFE STOP','bad');else if(d.lost)pill('lost','LOST','warn');else pill('lost','OK','ok');

  // Button state
  const btn=$('lfBtn');
  if(d.running&&!btn.classList.contains('running')){btn.classList.add('running');btn.textContent='■ STOP';}
  if(!d.running&&btn.classList.contains('running')){btn.classList.remove('running');btn.textContent='▶ START LINE FOLLOW';}

  // Detect start/stop transitions for logging
  if(d.running&&!wasRunning)addLog('Line follow active');
  if(!d.running&&wasRunning)addLog('Line follow stopped ('+d.cycle+' cycles)');
  wasRunning=d.running;

  // History
  if(d.found){errHist.push(d.err);if(errHist.length>200)errHist.shift();}
  corrHist.push(d.correction);if(corrHist.length>200)corrHist.shift();
  draw();

  // Diagnostics
  $('dCycles').textContent=d.cycle;
  $('dPhase').textContent=d.running?d.phase+'/6':'–';
  $('dAvgErr').textContent=d.errSamples>0?d.avgErr.toFixed(1)+'mm':'–';
  $('dMaxErr').textContent=d.maxErr>0?d.maxErr.toFixed(1)+'mm':'–';
  $('dLost').textContent=d.lostEvents;
  $('dJun').textContent=d.junEvents;
}

// Render line sensor data (for the sensor bar chart)
function renderSensor(d){
  for(let i=0;i<9;i++){
    const n=d.norm[i]/1000;
    $('rv'+i).textContent=d.raw[i];
    $('nv'+i).textContent=n.toFixed(2);
    $('bar'+i).style.height=(n*100)+'%';
    $('th'+i).style.bottom=(d.thr*100)+'%';
    $('sc'+i).className='sc'+((d.mask>>i)&1?' on':'')+((d.okmask>>i)&1?'':' dead');
  }
  $('calbar').style.width=(d.calact?(1-d.calleft/5000)*100:0)+'%';
  $('calmsg').textContent=d.calmsg;
}

// Polling
async function getJSON(url){
  const ctrl=new AbortController(),timer=setTimeout(()=>ctrl.abort(),2500);
  try{const r=await fetch(url,{signal:ctrl.signal,cache:'no-store'});if(!r.ok)throw new Error('HTTP '+r.status);return await r.json()}
  finally{clearTimeout(timer)}
}

async function poll(){
  if(document.hidden){setTimeout(poll,1500);return}
  try{
    const [st,sd]=await Promise.all([getJSON('/lf/status'),getJSON('/line/data')]);
    dataMisses=0;
    pill('lk','Connected','ok');
    renderStatus(st);
    renderSensor(sd);
  }catch(e){
    if(++dataMisses>=3)pill('lk','Disconnected','bad');
  }
  setTimeout(poll,500);
}

loadCfg();poll();
</script></body></html>
)rawliteral";
